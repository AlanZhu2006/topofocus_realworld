from __future__ import annotations

import json

from fastapi.testclient import TestClient

from focus_hub.api import create_app
from focus_hub.registry import HubRegistry, RobotPolicy
from focus_hub.settings import Settings


def make_settings(tmp_path) -> Settings:
    return Settings(
        policies={"robot-0": RobotPolicy("calib-test-v1", allow_goal=False)},
        robot_tokens={"robot-0": "robot-secret"},
        admin_token="admin-secret",
        spool_dir=tmp_path / "spool",
        state_dir=tmp_path / "state",
        min_free_bytes=0,
    )


def upload(client: TestClient, metadata) -> None:
    response = client.post(
        "/v1/robots/robot-0/observations",
        headers={"X-Robot-Token": "robot-secret"},
        data={"metadata_json": metadata.model_dump_json()},
        files={"rgb": ("rgb.jpg", b"rgb", "image/jpeg"),
               "depth": ("depth.png", b"depth", "image/png")},
    )
    assert response.status_code == 200


def test_sequence_state_survives_hub_restart(tmp_path, observation_factory):
    settings = make_settings(tmp_path)
    upload(TestClient(create_app(settings)), observation_factory(sequence=7))

    # A fresh app instance over the same state dir must remember sequence 7.
    restarted = TestClient(create_app(settings))
    latest = restarted.get(
        "/v1/robots/robot-0/observations/latest",
        headers={"X-Robot-Token": "robot-secret"},
    )
    assert latest.status_code == 200
    assert latest.json()["last_sequence"] == 7

    # An older sequence must still be rejected after the restart.
    stale = restarted.post(
        "/v1/robots/robot-0/observations",
        headers={"X-Robot-Token": "robot-secret"},
        data={"metadata_json": observation_factory(sequence=3).model_dump_json()},
        files={"rgb": ("rgb.jpg", b"rgb", "image/jpeg"),
               "depth": ("depth.png", b"depth", "image/png")},
    )
    assert stale.status_code == 409


def test_registry_without_state_path_does_not_persist(tmp_path):
    registry = HubRegistry({"robot-0": RobotPolicy("v1")})
    assert registry.snapshot("robot-0").last_sequence == -1
    assert not list(tmp_path.iterdir())


def test_map_version_advance_endpoint_and_decision_log(tmp_path, observation_factory):
    settings = make_settings(tmp_path)
    client = TestClient(create_app(settings))
    upload(client, observation_factory(sequence=0))

    assert client.post("/v1/admin/map_version/advance").status_code == 401
    response = client.post(
        "/v1/admin/map_version/advance", headers={"X-Admin-Token": "admin-secret"})
    assert response.status_code == 200
    assert response.json()["map_version"] == 1
    response = client.post(
        "/v1/admin/map_version/advance", headers={"X-Admin-Token": "admin-secret"})
    assert response.json()["map_version"] == 2

    # Version survives restart.
    restarted = TestClient(create_app(settings))
    latest = restarted.get(
        "/v1/robots/robot-0/observations/latest",
        headers={"X-Robot-Token": "robot-secret"},
    )
    assert latest.json()["map_version"] == 2

    events = [json.loads(line) for line in
              (tmp_path / "state" / "decision_events.jsonl").read_text().splitlines()]
    assert [e["event"] for e in events].count("map_version_advanced") == 2


def test_ack_and_publish_are_durably_logged(tmp_path, observation_factory):
    settings = make_settings(tmp_path)
    client = TestClient(create_app(settings))
    upload(client, observation_factory(sequence=0))

    import time

    now_ns = time.time_ns()
    decision = {
        "protocol_version": "1.0",
        "robot_id": "robot-0",
        "decision_id": "log-test-1",
        "mode": "HOLD",
        "map_version": 0,
        "transform_version": "calib-test-v1",
        "issued_at_ns": now_ns,
        "expires_at_ns": now_ns + 10_000_000_000,
        "target": None,
        "frontier_id": None,
        "reason": "logging test",
    }
    assert client.post(
        "/v1/admin/decisions", json=decision,
        headers={"X-Admin-Token": "admin-secret"},
    ).status_code == 202

    ack = {
        "protocol_version": "1.0",
        "robot_id": "robot-0",
        "decision_id": "log-test-1",
        "status": "ACCEPTED",
        "timestamp_ns": time.time_ns(),
        "detail": "HOLD",
    }
    assert client.post(
        "/v1/robots/robot-0/decisions/log-test-1/ack", json=ack,
        headers={"X-Robot-Token": "robot-secret"},
    ).status_code == 202

    events = [json.loads(line) for line in
              (tmp_path / "state" / "decision_events.jsonl").read_text().splitlines()]
    kinds = [e["event"] for e in events]
    assert "publish" in kinds and "ack" in kinds
    publish = next(e for e in events if e["event"] == "publish")
    assert publish["decision_id"] == "log-test-1" and publish["mode"] == "HOLD"
    acked = next(e for e in events if e["event"] == "ack")
    assert acked["status"] == "ACCEPTED"
