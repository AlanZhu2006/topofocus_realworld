from __future__ import annotations

import base64
import hashlib

import cv2
import numpy as np

from focus_hub.transport_v2 import DecisionBatchV2
from focus_hub.v2_goal_continuity import apply_frontier_goal_continuity

from test_v2_registry import make_batch, ready_registries


def with_round_and_targets(
    batch: DecisionBatchV2,
    *,
    round_index: int,
    source_step: int,
    targets: dict[str, tuple[float, float]],
) -> DecisionBatchV2:
    raw = batch.model_dump(mode="json")
    raw["decisions"][0]["decision_batch_id"] = f"batch-{round_index}"
    raw["decisions"][1]["decision_batch_id"] = f"batch-{round_index}"
    for decision in raw["decisions"]:
        robot_id = decision["robot_id"]
        decision["round_index"] = round_index
        decision["source_step"] = source_step
        decision["leg_id"] = f"leg-{robot_id}-r{round_index}"
        decision["decision_id"] = f"decision-{robot_id}-r{round_index}"
        decision["target"]["pose"]["x"] = targets[robot_id][0]
        decision["target"]["pose"]["y"] = targets[robot_id][1]
    return DecisionBatchV2.model_validate(raw)


def with_semantic_target(
    batch: DecisionBatchV2,
    *,
    robot_id: str,
) -> DecisionBatchV2:
    mask = np.full((2, 2), 255, dtype=np.uint8)
    ok, encoded = cv2.imencode(".png", mask)
    assert ok
    png = encoded.tobytes()
    raw = batch.model_dump(mode="json")
    for decision in raw["decisions"]:
        if decision["robot_id"] != robot_id:
            continue
        decision["target"] = {
            "kind": "SEMANTIC_REGION",
            "category": decision["goal_category"],
            "source_robot_id": robot_id,
            "evidence_status": "model_inference_map_projected_unverified",
            "source_goal_dilation_cells": 10,
            "region": {
                "frame_id": "shared_world",
                "origin_xy_m": [0.0, 0.0],
                "resolution_m": 0.05,
                "height": 2,
                "width": 2,
                "row_axis": "+y",
                "column_axis": "+x",
                "encoding": "png_u8_0_255_base64",
                "component_size_cells": 4,
                "payload_size_bytes": len(png),
                "payload_sha256": hashlib.sha256(png).hexdigest(),
                "payload_base64": base64.b64encode(png).decode("ascii"),
            },
            "display_centroid": {
                "frame_id": "shared_world",
                "x": 0.05,
                "y": 0.05,
                "authority": "display_only",
            },
        }
    return DecisionBatchV2.model_validate(raw)


def test_progressing_unfinished_frontiers_are_retained_for_both_robots(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    base = make_batch(observations, digests, now=now)
    previous = with_round_and_targets(
        base,
        round_index=0,
        source_step=0,
        targets={"robot-0": (2.0, 0.0), "robot-1": (2.0, 3.0)},
    )
    current = with_round_and_targets(
        base,
        round_index=1,
        source_step=24,
        targets={"robot-0": (-2.0, 0.0), "robot-1": (-2.0, 3.0)},
    )

    guarded, report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        previous_shared_positions={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.0, 3.0),
        },
        current_shared_positions={
            "robot-0": (0.2, 0.0),
            "robot-1": (0.2, 3.0),
        },
        minimum_progress_m=0.05,
    )

    assert report["status"] == "progressing_frontiers_retained"
    assert report["retained_robot_ids"] == ["robot-0", "robot-1"]
    for decision in guarded.decisions:
        assert decision.target is not None
        assert decision.target.pose.x == 2.0
        assert decision.target.frontier_id.startswith("continuity-r0-")
        assert decision.decision_batch_id == "batch-1"
        assert decision.round_index == 1


def test_stalled_completed_and_small_update_legs_are_not_retained(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    base = make_batch(observations, digests, now=now)
    previous = with_round_and_targets(
        base,
        round_index=0,
        source_step=0,
        targets={"robot-0": (2.0, 0.0), "robot-1": (2.0, 3.0)},
    )
    current = with_round_and_targets(
        base,
        round_index=1,
        source_step=24,
        targets={"robot-0": (-2.0, 0.0), "robot-1": (2.4, 3.0)},
    )

    guarded, report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        previous_shared_positions={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.0, 3.0),
        },
        current_shared_positions={
            "robot-0": (0.01, 0.0),
            "robot-1": (0.2, 3.0),
        },
        minimum_progress_m=0.05,
    )

    assert guarded == current
    assert report["retained_robot_ids"] == []
    assert report["checks"]["robot-0"]["reason"] == (
        "previous_leg_not_making_minimum_progress"
    )
    assert report["checks"]["robot-1"]["reason"] == (
        "source_target_update_is_small"
    )

    arrived, arrived_report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        previous_shared_positions={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.0, 3.0),
        },
        current_shared_positions={
            "robot-0": (1.6, 0.0),
            "robot-1": (1.6, 3.0),
        },
        minimum_progress_m=0.05,
    )
    assert arrived == current
    assert arrived_report["checks"]["robot-0"]["reason"] == (
        "previous_frontier_arrival_disk_reached"
    )


def test_semantic_target_always_preempts_frontier_continuity(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    base = make_batch(observations, digests, now=now)
    previous = with_round_and_targets(
        base,
        round_index=0,
        source_step=0,
        targets={"robot-0": (2.0, 0.0), "robot-1": (2.0, 3.0)},
    )
    current = with_semantic_target(
        with_round_and_targets(
            base,
            round_index=1,
            source_step=24,
            targets={
                "robot-0": (-2.0, 0.0),
                "robot-1": (-2.0, 3.0),
            },
        ),
        robot_id="robot-0",
    )

    guarded, report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        previous_shared_positions={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.0, 3.0),
        },
        current_shared_positions={
            "robot-0": (0.2, 0.0),
            "robot-1": (0.2, 3.0),
        },
        minimum_progress_m=0.05,
    )

    robot_0 = next(
        item for item in guarded.decisions if item.robot_id == "robot-0"
    )
    assert robot_0.target is not None
    assert robot_0.target.kind == "SEMANTIC_REGION"
    assert report["checks"]["robot-0"]["reason"] == (
        "semantic_target_preempts_frontier_continuity"
    )
    assert report["checks"]["robot-1"]["retained"] is True
