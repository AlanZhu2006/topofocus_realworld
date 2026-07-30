from __future__ import annotations

import base64
import hashlib

import cv2
import numpy as np
import pytest

from focus_hub.transport_v2 import DecisionBatchV2
from focus_hub.v2_goal_continuity import (
    SOURCE_CONTINUITY_RETAIN_DISTANCE_M,
    SOURCE_FRONTIER_COMPLETION_DISTANCE_M,
    apply_frontier_goal_continuity,
    source_continuity_memory_batch,
)

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


def test_distant_unfinished_frontiers_are_retained_for_both_robots(
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
        current_shared_positions={
            "robot-0": (0.2, 0.0),
            "robot-1": (0.2, 3.0),
        },
    )

    assert report["status"] == "distant_previous_frontiers_retained"
    assert report["retained_robot_ids"] == ["robot-0", "robot-1"]
    for decision in guarded.decisions:
        assert decision.target is not None
        assert decision.target.pose.x == 2.0
        assert decision.target.frontier_id.startswith("continuity-r0-")
        assert decision.decision_batch_id == "batch-1"
        assert decision.round_index == 1


def test_continuity_memory_restores_source_target_before_projection(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    execution = with_round_and_targets(
        make_batch(observations, digests, now=now),
        round_index=0,
        source_step=0,
        targets={
            "robot-0": (1.0, 0.0),
            "robot-1": (1.0, 3.0),
        },
    )
    lineage = {}
    for decision in execution.decisions:
        assert decision.target is not None
        source_y = 0.0 if decision.robot_id == "robot-0" else 3.0
        lineage[decision.robot_id] = {
            "source_frontier_id": f"source-{decision.robot_id}",
            "source_target_xy_m": [2.0, source_y],
            "selection_source": "guard_input",
            "execution_mode": "GOAL",
            "execution_frontier_id": decision.target.frontier_id,
            "execution_target_xy_m": [
                decision.target.pose.x,
                decision.target.pose.y,
            ],
        }

    memory, report = source_continuity_memory_batch(
        execution,
        clearance_report={"execution_lineage": lineage},
    )

    for decision in memory.decisions:
        assert decision.target is not None
        assert decision.target.pose.x == 2.0
        assert decision.target.frontier_id == (
            f"source-{decision.robot_id}"
        )
        assert report["robots"][decision.robot_id][
            "projection_removed_from_memory"
        ] is True
    for decision in execution.decisions:
        assert decision.target is not None
        assert decision.target.pose.x == 1.0


def test_unfinished_previous_goal_is_retained_through_source_switch_band(
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
        targets={"robot-0": (-2.0, 0.0), "robot-1": (2.0, 3.0)},
    )

    guarded, report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        current_shared_positions={
            "robot-0": (0.9, 0.0),
            "robot-1": (0.2, 3.0),
        },
    )

    assert guarded != current
    assert report["retained_robot_ids"] == ["robot-0"]
    assert report["physical_completion_retained_robot_ids"] == [
        "robot-0"
    ]
    assert report["checks"]["robot-0"]["reason"] == (
        "unfinished_previous_frontier_outside_source_"
        "10_cell_arrival_disk_retained"
    )
    assert report["checks"]["robot-1"]["reason"] == (
        "source_target_already_continuous"
    )

    arrived, arrived_report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        current_shared_positions={
            "robot-0": (1.6, 0.0),
            "robot-1": (1.6, 3.0),
        },
    )
    assert arrived == current
    assert arrived_report["checks"]["robot-0"]["reason"] == (
        "previous_goal_inside_source_10_cell_arrival_disk"
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
        current_shared_positions={
            "robot-0": (0.2, 0.0),
            "robot-1": (0.2, 3.0),
        },
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


def test_source_continuity_threshold_is_twenty_five_map_cells(
    observation_factory,
):
    assert SOURCE_CONTINUITY_RETAIN_DISTANCE_M == 1.25
    assert SOURCE_FRONTIER_COMPLETION_DISTANCE_M == 0.5
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    base = make_batch(observations, digests, now=now)
    previous = with_round_and_targets(
        base,
        round_index=0,
        source_step=0,
        targets={"robot-0": (4.0, 0.0), "robot-1": (4.0, 3.0)},
    )
    current = with_round_and_targets(
        base,
        round_index=1,
        source_step=24,
        targets={"robot-0": (-2.0, 0.0), "robot-1": (-2.0, 3.0)},
    )

    below, report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        current_shared_positions={
            "robot-0": (2.76, 0.0),
            "robot-1": (2.75, 3.0),
        },
        minimum_remaining_distance_m=(
            SOURCE_CONTINUITY_RETAIN_DISTANCE_M
        ),
    )

    assert below.decisions[0] != current.decisions[0]
    assert report["checks"]["robot-0"]["reason"] == (
        "unfinished_previous_frontier_outside_source_"
        "10_cell_arrival_disk_retained"
    )
    assert report["checks"]["robot-1"]["retained"] is True
    assert report["physical_completion_retained_robot_ids"] == [
        "robot-0"
    ]
    assert report["source_rule_retained_robot_ids"] == ["robot-1"]


def test_archived_robot1_history_switch_does_not_reverse_unfinished_leg(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    base = make_batch(observations, digests, now=now)
    previous = with_round_and_targets(
        base,
        round_index=15,
        source_step=374,
        targets={
            "robot-0": (4.0, 4.0),
            # Observed Scene 03 Formal 02 history-1.
            "robot-1": (-1.0850726802376052, 0.990669750033291),
        },
    )
    current = with_round_and_targets(
        base,
        round_index=16,
        source_step=399,
        targets={
            "robot-0": (4.0, 4.0),
            # The next source decision selected history-6 behind the current
            # motion direction before history-1 had actually completed.
            "robot-1": (-0.33507268023760517, 2.4906697500332893),
        },
    )

    guarded, report = apply_frontier_goal_continuity(
        current,
        previous_batch=previous,
        current_shared_positions={
            "robot-0": (4.0, 4.0),
            "robot-1": (-0.43, 1.67),
        },
    )

    robot_1 = next(
        item for item in guarded.decisions
        if item.robot_id == "robot-1"
    )
    assert robot_1.target is not None
    assert robot_1.target.pose.x == pytest.approx(
        -1.0850726802376052
    )
    assert robot_1.target.pose.y == pytest.approx(0.990669750033291)
    assert report["checks"]["robot-1"]["retention_authority"] == (
        "realworld_unfinished_leg_completion_adapter"
    )


def test_explicitly_rejected_previous_leg_is_never_retained(
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
        targets={"robot-0": (4.0, 0.0), "robot-1": (4.0, 3.0)},
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
        current_shared_positions={
            "robot-0": (1.5, 0.0),
            "robot-1": (1.5, 3.0),
        },
        minimum_remaining_distance_m=(
            SOURCE_CONTINUITY_RETAIN_DISTANCE_M
        ),
        previous_rejected_robot_ids=frozenset({"robot-0"}),
    )

    assert guarded.decisions[0] == current.decisions[0]
    assert report["checks"]["robot-0"]["reason"] == (
        "previous_frontier_leg_explicitly_rejected"
    )
    assert report["checks"]["robot-1"]["retained"] is True
