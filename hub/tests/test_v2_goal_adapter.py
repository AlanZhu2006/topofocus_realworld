from __future__ import annotations

import base64
import hashlib
import json
import math

import cv2
import numpy as np
import pytest

from conftest import IDENTITY
from focus_hub.transport_v2 import HighLevelDecisionV2
from focus_hub.v2_goal_adapter import (
    V2AdapterAction,
    V2GoalAdapter,
    V2GoalAdapterConfig,
)


def make_target_decision(
    *,
    robot_id: str = "robot-0",
    target: dict[str, object] | None = None,
    mode: str = "GOAL",
    transform_version: str = "calib-test-v1",
    calibration_id: str = "shared-board-v1",
    now_ns: int = 10_000_000_000,
) -> HighLevelDecisionV2:
    if target is None and mode == "GOAL":
        target = {
            "kind": "FRONTIER_POINT",
            "frontier_id": "frontier-0",
            "source_goal_dilation_cells": 10,
            "pose": {
                "frame_id": "shared_world",
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "yaw_rad": 0.5,
            },
        }
    return HighLevelDecisionV2.model_validate({
        "robot_id": robot_id,
        "scene_id": "scene-1",
        "episode_id": "scene-1-trial-1",
        "round_index": 0,
        "source_step": 0,
        "decision_batch_id": "batch-1",
        "leg_id": f"leg-{robot_id}",
        "decision_id": f"decision-{robot_id}-{mode.lower()}",
        "lease_sequence": 0,
        "mode": mode,
        "coordination": {
            "execution_epoch": 0,
            "active_robot_ids": [robot_id] if mode == "GOAL" else [],
        },
        "goal_category": "chair",
        "input_observations": {
            "robot-0": {
                "sequence": 1,
                "capture_time_ns": 9_900_000_000,
                "payload_sha256": "a" * 64,
            },
            "robot-1": {
                "sequence": 2,
                "capture_time_ns": 9_900_000_001,
                "payload_sha256": "b" * 64,
            },
        },
        "map_provenance": {
            "map_version": 1,
            "map_snapshot_sha256": "c" * 64,
            "map_format_version": "focus-hub-central-map-v3",
            "frame_id": "shared_world",
            "resolution_m": 0.05,
            "transform_version": transform_version,
            "shared_frame_calibration_id": calibration_id,
        },
        "issued_at_ns": now_ns,
        "expires_at_ns": now_ns + 8_000_000_000,
        "target": target if mode == "GOAL" else None,
        "reason": "adapter test",
    })


def make_adapter(
    *,
    robot_id: str = "robot-0",
    output_kind: str = "tinynav_poi",
    shared_T_robot_map=IDENTITY,
    allow_unreachable_semantic_projection: bool = False,
) -> V2GoalAdapter:
    return V2GoalAdapter(V2GoalAdapterConfig(
        robot_id=robot_id,
        transform_version="calib-test-v1",
        shared_frame_calibration_id="shared-board-v1",
        shared_T_robot_map=shared_T_robot_map,
        output_kind=output_kind,
        local_frame_id=f"{robot_id}/map",
        allow_unreachable_semantic_projection=(
            allow_unreachable_semantic_projection
        ),
    ))


def semantic_target(*, component_size_cells: int = 1) -> dict[str, object]:
    mask = np.zeros((25, 25), dtype=np.uint8)
    mask[12, 12] = 255
    ok, encoded = cv2.imencode(".png", mask)
    assert ok
    png = encoded.tobytes()
    return {
        "kind": "SEMANTIC_REGION",
        "category": "chair",
        "source_robot_id": "robot-0",
        "evidence_status": "model_inference_map_projected_unverified",
        "source_goal_dilation_cells": 10,
        "region": {
            "frame_id": "shared_world",
            "origin_xy_m": [-0.625, -0.625],
            "resolution_m": 0.05,
            "height": 25,
            "width": 25,
            "row_axis": "+y",
            "column_axis": "+x",
            "encoding": "png_u8_0_255_base64",
            "component_size_cells": component_size_cells,
            "payload_size_bytes": len(png),
            "payload_sha256": hashlib.sha256(png).hexdigest(),
            "payload_base64": base64.b64encode(png).decode("ascii"),
        },
        "display_centroid": {
            "frame_id": "shared_world",
            "x": 0.0,
            "y": 0.0,
            "authority": "display_only",
        },
    }


def test_tinynav_frontier_reduces_to_poi_without_sending(observation_factory):
    adapter = make_adapter()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = adapter.evaluate(
        make_target_decision(),
        now_ns=10_000_000_001,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == V2AdapterAction.GOAL
    payload = json.loads(result.command_preview)
    assert payload["0"]["position"] == [1.0, 2.0, 0.0]
    assert payload["0"]["source"] == "focus_hub_v2"
    assert payload["0"]["arrival_radius_m"] == pytest.approx(0.5)


def test_water_position_and_yaw_use_the_full_inverse_transform(observation_factory):
    # shared_T_robot_map is +90 degrees. Local (1,0,yaw=0) becomes shared
    # (0,1,yaw=pi/2), so the inverse must recover both position and yaw.
    shared_T_robot_map = (
        0.0, -1.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )
    target = {
        "kind": "FRONTIER_POINT",
        "frontier_id": "frontier-rotation",
        "source_goal_dilation_cells": 10,
        "pose": {
            "frame_id": "shared_world",
            "x": 0.0,
            "y": 1.0,
            "z": 0.0,
            "yaw_rad": math.pi / 2,
        },
    }
    adapter = make_adapter(
        robot_id="robot-1",
        output_kind="water_move",
        shared_T_robot_map=shared_T_robot_map,
    )
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = adapter.evaluate(
        make_target_decision(robot_id="robot-1", target=target),
        now_ns=10_000_000_001,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == V2AdapterAction.GOAL
    assert result.local_goal.x == pytest.approx(1.0)
    assert result.local_goal.y == pytest.approx(0.0)
    assert result.local_goal.yaw_rad == pytest.approx(0.0)
    assert result.command_preview.startswith("/api/move?location=1.0000,0.0000,0.0000")


def test_semantic_region_requires_local_reachability_and_preserves_hash(
    observation_factory,
):
    adapter = make_adapter()
    health = observation_factory(mapping_only=False, health_ready=True).health
    decision = make_target_decision(target=semantic_target())
    rejected = adapter.evaluate(
        decision,
        now_ns=10_000_000_001,
        health=health,
        current_position_robot_map=(-1.0, 0.0, 0.0),
    )
    assert rejected.action == V2AdapterAction.HOLD
    assert rejected.reason_code == "UNREACHABLE"

    accepted = adapter.evaluate(
        decision,
        now_ns=10_000_000_002,
        health=health,
        current_position_robot_map=(-1.0, 0.0, 0.0),
        is_local_goal_reachable=lambda x, y: x < -0.1,
    )
    assert accepted.action == V2AdapterAction.GOAL
    assert accepted.local_goal.target_kind == "SEMANTIC_REGION"
    assert accepted.local_goal.source_region_sha256 == decision.target.region.payload_sha256
    assert accepted.local_goal.arrival_radius_m == pytest.approx(0.5)
    assert accepted.local_goal.x < -0.1


def test_semantic_component_count_mismatch_fails_closed(observation_factory):
    adapter = make_adapter()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = adapter.evaluate(
        make_target_decision(target=semantic_target(component_size_cells=2)),
        now_ns=10_000_000_001,
        health=health,
        current_position_robot_map=(0, 0, 0),
        is_local_goal_reachable=lambda _x, _y: True,
    )
    assert result.action == V2AdapterAction.HOLD
    assert result.reason_code == "REGION_ARTIFACT_INVALID"


def test_online_semantic_projection_preserves_goal_for_local_replanning(
    observation_factory,
):
    adapter = make_adapter(allow_unreachable_semantic_projection=True)
    health = observation_factory(mapping_only=False, health_ready=True).health
    decision = make_target_decision(target=semantic_target())

    result = adapter.evaluate(
        decision,
        now_ns=10_000_000_001,
        health=health,
        current_position_robot_map=(-1.0, 0.0, 0.0),
        is_local_goal_reachable=lambda _x, _y: False,
    )

    assert result.action == V2AdapterAction.GOAL
    assert result.local_goal.target_kind == "SEMANTIC_REGION"
    payload = json.loads(result.command_preview)
    assert payload["0"]["target_kind"] == "SEMANTIC_REGION"


def test_adapter_has_no_transport_or_actuator_client():
    import focus_hub.v2_goal_adapter as module

    assert "socket" not in dir(module)
    assert "httpx" not in dir(module)
    assert "requests" not in dir(module)
