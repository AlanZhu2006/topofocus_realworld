from __future__ import annotations

import base64
import hashlib
import time

import cv2
import numpy as np
import pytest
from pydantic import ValidationError

from focus_hub.transport_v2 import (
    DecisionBatchV2,
    HighLevelDecisionV2,
    NavigationEventV2,
)


ROBOTS = ("robot-0", "robot-1")


def input_observations() -> dict[str, dict[str, object]]:
    return {
        robot_id: {
            "sequence": index + 4,
            "capture_time_ns": 1_000_000_000 + index,
            "payload_sha256": ("a" if index == 0 else "b") * 64,
        }
        for index, robot_id in enumerate(ROBOTS)
    }


def make_decision(
    robot_id: str,
    *,
    active_robot_ids: tuple[str, ...] = ROBOTS,
    mode: str = "GOAL",
    decision_id: str | None = None,
    leg_id: str | None = None,
    lease_sequence: int = 0,
    issued_at_ns: int = 2_000_000_000,
    expires_at_ns: int = 10_000_000_000,
    target: dict[str, object] | None = None,
) -> HighLevelDecisionV2:
    index = ROBOTS.index(robot_id)
    if target is None and mode == "GOAL":
        target = {
            "kind": "FRONTIER_POINT",
            "frontier_id": f"frontier-{index}",
            "source_goal_dilation_cells": 10,
            "pose": {
                "frame_id": "shared_world",
                "x": float(index + 1),
                "y": float(index + 2),
                "z": 0.0,
                "yaw_rad": 0.5,
            },
        }
    return HighLevelDecisionV2.model_validate({
        "robot_id": robot_id,
        "scene_id": "scene-1",
        "episode_id": "scene-1-trial-1",
        "round_index": 1,
        "source_step": 24,
        "decision_batch_id": "batch-1",
        "leg_id": leg_id or f"leg-{robot_id}",
        "decision_id": decision_id or f"decision-{robot_id}-{lease_sequence}",
        "lease_sequence": lease_sequence,
        "mode": mode,
        "coordination": {
            "execution_epoch": 1,
            "active_robot_ids": list(active_robot_ids),
        },
        "goal_category": "chair",
        "input_observations": input_observations(),
        "map_provenance": {
            "map_version": 3,
            "map_snapshot_sha256": ("c" if index == 0 else "d") * 64,
            "map_format_version": "focus-hub-central-map-v3",
            "frame_id": "shared_world",
            "resolution_m": 0.05,
            "transform_version": f"calib-{robot_id}",
            "shared_frame_calibration_id": "shared-board-v1",
        },
        "issued_at_ns": issued_at_ns,
        "expires_at_ns": expires_at_ns,
        "target": target if mode == "GOAL" else None,
        "reason": "test",
    })


def make_batch(*, active_robot_ids: tuple[str, ...] = ROBOTS) -> DecisionBatchV2:
    decisions = tuple(
        make_decision(
            robot_id,
            active_robot_ids=active_robot_ids,
            mode="GOAL" if robot_id in active_robot_ids else "HOLD",
        )
        for robot_id in ROBOTS
    )
    return DecisionBatchV2(decisions=decisions)


def test_atomic_batch_allows_two_concurrent_goals():
    batch = make_batch()
    assert [decision.mode.value for decision in batch.decisions] == ["GOAL", "GOAL"]
    assert batch.decisions[0].coordination.active_robot_ids == ROBOTS


def test_batch_requires_active_set_to_equal_goal_robots():
    decisions = (
        make_decision("robot-0", active_robot_ids=ROBOTS),
        make_decision("robot-1", active_robot_ids=("robot-0",), mode="HOLD"),
    )
    with pytest.raises(ValidationError, match="coordination"):
        DecisionBatchV2(decisions=decisions)


def test_goal_lease_and_source_clock_are_bounded():
    with pytest.raises(ValidationError, match="10 seconds"):
        make_decision("robot-0", expires_at_ns=12_000_000_001)
    payload = make_decision("robot-0").model_dump(mode="json")
    payload["source_step"] = 25
    with pytest.raises(ValidationError, match="0,24,49"):
        HighLevelDecisionV2.model_validate(payload)


def test_semantic_region_png_identity_is_strict():
    mask = np.zeros((3, 4), dtype=np.uint8)
    mask[1, 2] = 255
    ok, encoded = cv2.imencode(".png", mask)
    assert ok
    png = encoded.tobytes()
    target = {
        "kind": "SEMANTIC_REGION",
        "category": "chair",
        "source_robot_id": "robot-0",
        "evidence_status": "model_inference_map_projected_unverified",
        "source_goal_dilation_cells": 10,
        "region": {
            "frame_id": "shared_world",
            "origin_xy_m": [-1.0, -2.0],
            "resolution_m": 0.05,
            "height": 3,
            "width": 4,
            "row_axis": "+y",
            "column_axis": "+x",
            "encoding": "png_u8_0_255_base64",
            "component_size_cells": 1,
            "payload_size_bytes": len(png),
            "payload_sha256": hashlib.sha256(png).hexdigest(),
            "payload_base64": base64.b64encode(png).decode("ascii"),
        },
        "display_centroid": {
            "frame_id": "shared_world",
            "x": -0.875,
            "y": -1.925,
            "authority": "display_only",
        },
    }
    decision = make_decision("robot-0", target=target)
    assert decision.target is not None
    assert decision.target.kind == "SEMANTIC_REGION"

    bad = decision.model_dump(mode="json")
    bad["target"]["region"]["payload_sha256"] = "f" * 64
    with pytest.raises(ValidationError, match="SHA-256"):
        HighLevelDecisionV2.model_validate(bad)


def test_arrival_requires_zero_velocity_confirmation():
    now = time.time_ns()
    payload = {
        "robot_id": "robot-0",
        "scene_id": "scene-1",
        "episode_id": "scene-1-trial-1",
        "decision_batch_id": "batch-1",
        "leg_id": "leg-robot-0",
        "decision_id": "decision-robot-0-0",
        "lease_sequence": 0,
        "event_id": "event-1",
        "status": "ARRIVED",
        "reason_code": "LOCAL_PLANNER_ARRIVED",
        "observed_at_ns": now,
        "local_pose": {"frame_id": "robot-0/map", "x": 1, "y": 2, "yaw_rad": 0},
        "path_length_m_from_episode_start": 2.5,
        "velocity_zero_confirmed": False,
    }
    with pytest.raises(ValidationError, match="zero-velocity"):
        NavigationEventV2.model_validate(payload)
