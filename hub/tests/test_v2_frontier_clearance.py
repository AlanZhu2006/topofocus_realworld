from __future__ import annotations

import numpy as np

from focus_hub.map_snapshot import MapSnapshot
from focus_hub.transport_v2 import DecisionBatchV2
from focus_hub.v2_frontier_clearance import apply_frontier_clearance_guard

from test_v2_registry import make_batch, ready_registries


def with_targets(
    batch: DecisionBatchV2,
    targets: dict[str, tuple[float, float]],
) -> DecisionBatchV2:
    raw = batch.model_dump(mode="json")
    for decision in raw["decisions"]:
        x_m, y_m = targets[decision["robot_id"]]
        decision["target"]["pose"]["x"] = x_m
        decision["target"]["pose"]["y"] = y_m
    return DecisionBatchV2.model_validate(raw)


def snapshot_with_one_wall_trap() -> MapSnapshot:
    grid = np.zeros((2, 100, 100), dtype=np.float32)
    grid[1] = 1.0
    # Keep the selected cell nominally free while surrounding it with a
    # sub-footprint pocket. No 0.34 m-clear centre exists in its 0.50 m
    # source arrival disk.
    grid[0, 8:33, 8:33] = 1.0
    grid[0, 19:22, 19:22] = 0.0
    return MapSnapshot(
        grid=grid,
        origin_xy_m=(0.0, 0.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version="multi-robot-source-derived",
        shared_frame_calibration_id="shared-board-v1",
        map_format_version="focus-hub-central-map-v3",
    )


def test_wall_adjacent_frontier_is_held_without_suppressing_clear_peer(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.5, 3.5),
            "robot-1": (1.025, 1.025),
        },
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_with_one_wall_trap(),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
    )

    assert report["status"] == "unsafe_frontiers_suppressed"
    assert report["blocked_robot_ids"] == ["robot-1"]
    assert report["effective_active_robot_ids"] == ["robot-0"]
    assert report["checks"]["robot-1"]["safe_approach_cell_count"] == 0
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "HOLD"]
    assert guarded.decisions[0].target == candidate.decisions[0].target
    assert guarded.decisions[1].target is None


def test_clear_frontiers_preserve_candidate_batch(observation_factory):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (3.0, 3.0), "robot-1": (4.0, 4.0)},
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_with_one_wall_trap(),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
    )

    assert report["status"] == "frontiers_clear"
    assert report["blocked_robot_ids"] == []
    assert guarded == candidate


def test_rejected_frontiers_use_one_clear_source_remaining_frontier(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (1.025, 1.025),
            "robot-1": (1.075, 1.075),
        },
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_with_one_wall_trap(),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers=[
            {
                "frontier_id": "A",
                "x_m": 3.0,
                "y_m": 3.0,
                "size_cells": 50,
            }
        ],
        robot_xy_by_robot={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.5, 0.0),
        },
    )

    assert report["status"] == "unsafe_frontiers_reallocated"
    assert report["selected_frontier_rejected_robot_ids"] == [
        "robot-0",
        "robot-1",
    ]
    assert report["blocked_robot_ids"] == ["robot-1"]
    assert report["effective_active_robot_ids"] == ["robot-0"]
    assert report["fallback_assignments"] == [
        {
            "robot_id": "robot-0",
            "rejected_frontier_id": "frontier-0",
            "fallback_frontier_id": "A",
            "source_rank": 0,
        }
    ]
    assert report["fallback_checks"]["robot-0"][0]["passed"] is True
    assert report["fallback_checks"]["robot-0"][0][
        "required_clearance_m"
    ] == 0.35
    assert report["fallback_checks"]["robot-1"] == []
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "HOLD"]
    target = guarded.decisions[0].target
    assert target is not None
    assert target.kind == "FRONTIER_POINT"
    assert target.frontier_id == "A"
    assert target.pose.x == 3.0
    assert target.pose.y == 3.0
    assert guarded.decisions[1].target is None


def test_rejected_frontiers_still_hold_when_fallback_is_unsafe(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (1.025, 1.025),
            "robot-1": (3.0, 3.0),
        },
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_with_one_wall_trap(),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers=[
            {"frontier_id": "A", "x_m": 1.075, "y_m": 1.075}
        ],
        robot_xy_by_robot={"robot-0": (0.0, 0.0)},
    )

    assert report["status"] == "unsafe_frontiers_suppressed"
    assert report["blocked_robot_ids"] == ["robot-0"]
    assert report["fallback_assignments"] == []
    assert report["fallback_checks"]["robot-0"][0]["passed"] is False
    assert [item.mode.value for item in guarded.decisions] == ["HOLD", "GOAL"]


def test_clear_frontiers_ignore_fallback_candidates(observation_factory):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (3.0, 3.0), "robot-1": (4.0, 4.0)},
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_with_one_wall_trap(),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers=[
            {"frontier_id": "A", "x_m": 5.0, "y_m": 5.0}
        ],
        robot_xy_by_robot={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.0, 0.0),
        },
    )

    assert report["status"] == "frontiers_clear"
    assert report["fallback_assignments"] == []
    assert guarded == candidate
