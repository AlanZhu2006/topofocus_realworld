from __future__ import annotations

import hashlib
import json
import time

from fastapi.testclient import TestClient

from focus_hub.api import create_app
from focus_hub.registry import RobotPolicy
from focus_hub.settings import Settings


ROBOTS = ("robot-0", "robot-1")


def test_v2_concurrent_batch_poll_and_feedback(tmp_path, observation_factory):
    settings = Settings(
        policies={
            robot_id: RobotPolicy("calib-test-v1", allow_goal=True)
            for robot_id in ROBOTS
        },
        robot_tokens={robot_id: f"token-{robot_id}" for robot_id in ROBOTS},
        admin_token="admin-secret",
        spool_dir=tmp_path / "spool",
        state_dir=tmp_path / "state",
        min_free_bytes=0,
    )
    app = create_app(settings)
    client = TestClient(app)
    now = time.time_ns()
    digests: dict[str, str] = {}
    for index, robot_id in enumerate(ROBOTS):
        digest = hashlib.sha256(f"payload-{robot_id}".encode()).hexdigest()
        observation = observation_factory(
            robot_id=robot_id,
            sequence=index + 10,
            now_ns=now,
            mapping_only=False,
            health_ready=True,
        )
        app.state.registry.accept_observation(observation, digest, now_ns=now)
        digests[robot_id] = digest

    assert client.get(
        "/v2/admin/robots/robot-0/runtime-readiness"
    ).status_code == 401
    readiness = client.get(
        "/v2/admin/robots/robot-0/runtime-readiness",
        headers={"X-Admin-Token": "admin-secret"},
    )
    assert readiness.status_code == 200
    assert readiness.json()["ready_for_goal"] is True
    assert readiness.json()["health_source"] == "observation"

    empty = client.get(
        "/v2/robots/robot-0/decisions/latest",
        headers={"X-Robot-Token": "token-robot-0"},
    )
    assert empty.status_code == 204

    inputs = {
        robot_id: {
            "sequence": app.state.registry.snapshot(robot_id).last_sequence,
            "capture_time_ns": (
                app.state.registry.snapshot(robot_id).last_observation.capture_time_ns
            ),
            "payload_sha256": digests[robot_id],
        }
        for robot_id in ROBOTS
    }
    issued_at_ns = time.time_ns()
    decisions = []
    for index, robot_id in enumerate(ROBOTS):
        decisions.append({
            "robot_id": robot_id,
            "scene_id": "scene-1",
            "episode_id": "scene-1-trial-1",
            "round_index": 0,
            "source_step": 0,
            "decision_batch_id": "batch-1",
            "leg_id": f"leg-{robot_id}",
            "decision_id": f"decision-{robot_id}",
            "lease_sequence": 0,
            "mode": "GOAL",
            "coordination": {
                "execution_epoch": 0,
                "active_robot_ids": list(ROBOTS),
            },
            "goal_category": "chair",
            "input_observations": inputs,
            "map_provenance": {
                "map_version": 0,
                "map_snapshot_sha256": ("c" if index == 0 else "d") * 64,
                "map_format_version": "focus-hub-central-map-v3",
                "frame_id": "shared_world",
                "resolution_m": 0.05,
                "transform_version": "calib-test-v1",
                "shared_frame_calibration_id": "shared-board-v1",
            },
            "issued_at_ns": issued_at_ns,
            "expires_at_ns": issued_at_ns + 8_000_000_000,
            "target": {
                "kind": "FRONTIER_POINT",
                "frontier_id": f"frontier-{index}",
                "source_goal_dilation_cells": 10,
                "pose": {
                    "frame_id": "shared_world",
                    "x": index + 1.0,
                    "y": index + 2.0,
                    "z": 0.0,
                    "yaw_rad": 0.25,
                },
            },
            "reason": "concurrent API test",
        })
    batch = {
        "protocol_version": "2.0",
        "schema_version": "focus-decision-batch-v2",
        "decisions": decisions,
    }

    assert client.post("/v2/admin/decision-batches", json=batch).status_code == 401
    published = client.post(
        "/v2/admin/decision-batches",
        json=batch,
        headers={"X-Admin-Token": "admin-secret"},
    )
    assert published.status_code == 202, published.text
    assert published.json()["decision_ids"] == [
        "decision-robot-0",
        "decision-robot-1",
    ]

    for robot_id in ROBOTS:
        response = client.get(
            f"/v2/robots/{robot_id}/decisions/latest",
            headers={"X-Robot-Token": f"token-{robot_id}"},
        )
        assert response.status_code == 200
        assert response.json()["mode"] == "GOAL"
        assert set(response.json()["coordination"]["active_robot_ids"]) == set(ROBOTS)

    event_time_ns = time.time_ns()
    event = {
        "robot_id": "robot-0",
        "scene_id": "scene-1",
        "episode_id": "scene-1-trial-1",
        "decision_batch_id": "batch-1",
        "leg_id": "leg-robot-0",
        "decision_id": "decision-robot-0",
        "lease_sequence": 0,
        "event_id": "event-robot-0-0",
        "status": "NAVIGATING",
        "reason_code": "LOCAL_PLANNER_ACTIVE",
        "observed_at_ns": event_time_ns,
        "local_pose": {
            "frame_id": "robot-0/map",
            "x": 0,
            "y": 0,
            "yaw_rad": 0,
        },
        "path_length_m_from_episode_start": 0.1,
        "velocity_zero_confirmed": False,
    }
    first = client.post(
        "/v2/robots/robot-0/navigation-events",
        json=event,
        headers={"X-Robot-Token": "token-robot-0"},
    )
    assert first.status_code == 202, first.text
    assert first.json()["status"] == "accepted"
    duplicate = client.post(
        "/v2/robots/robot-0/navigation-events",
        json=event,
        headers={"X-Robot-Token": "token-robot-0"},
    )
    assert duplicate.status_code == 202
    assert duplicate.json()["status"] == "duplicate"

    assert client.get(
        "/v2/admin/robots/robot-0/navigation-state"
    ).status_code == 401
    state = client.get(
        "/v2/admin/robots/robot-0/navigation-state",
        headers={"X-Admin-Token": "admin-secret"},
    )
    assert state.status_code == 200
    assert state.json()["latest_decision"]["decision_id"] == "decision-robot-0"
    assert state.json()["latest_event"]["status"] == "NAVIGATING"
    assert state.json()["latest_event_received_at_ns"] > 0

    records = [
        json.loads(line)
        for line in (tmp_path / "state/decision_events.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    assert [record["event"] for record in records] == [
        "v2_publish_batch",
        "v2_navigation_event",
    ]
