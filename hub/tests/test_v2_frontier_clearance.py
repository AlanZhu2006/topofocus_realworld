from __future__ import annotations

import math

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


def snapshot_from_free_mask(
    free: np.ndarray,
    *,
    transform_version: str,
) -> MapSnapshot:
    grid = np.zeros((2, *free.shape), dtype=np.float32)
    grid[1] = free.astype(np.float32)
    return MapSnapshot(
        grid=grid,
        origin_xy_m=(0.0, 0.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version=transform_version,
        shared_frame_calibration_id="shared-board-v1",
        map_format_version="focus-hub-central-map-v3",
    )


def test_wall_adjacent_frontier_is_held_without_suppressing_clear_peer(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
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


def test_arrival_disk_intersects_safe_cell_footprint_not_only_its_center(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (0.95, 0.95),
            "robot-1": (3.0, 3.0),
        },
    )
    free = np.zeros((100, 100), dtype=bool)
    free[20:80, 20:80] = True
    snapshot = snapshot_from_free_mask(
        free,
        transform_version="multi-robot-source-derived",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
    )

    check = report["checks"]["robot-0"]
    assert check["approach_distance_method"] == ("point_to_grid_cell_footprint")
    assert check["nearest_safe_approach_distance_m"] < 0.5
    assert check["safe_approach_cell_count"] > 0
    assert check["passed"] is True
    assert guarded == candidate


def test_already_arrived_frontier_is_reallocated_before_republish(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.5, 3.5),
            "robot-1": (2.6, 2.5),
        },
    )
    free = np.ones((100, 100), dtype=bool)
    execution = snapshot_from_free_mask(
        free,
        transform_version="robot-execution-v1",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers=[
            {"frontier_id": "A", "x_m": 4.0, "y_m": 4.0},
        ],
        robot_xy_by_robot={
            "robot-0": (2.0, 2.0),
            "robot-1": (2.5, 2.5),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
    )

    check = report["checks"]["robot-1"]
    assert check["direct_approach_available"] is True
    assert check["frontier_arrival_already_satisfied"] is True
    assert check["minimum_direct_travel_m"] == 0.0
    assert check["direct_approach_has_useful_travel"] is False
    assert check["direct_approach_passed"] is False
    assert report["fallback_assignments"] == [
        {
            "robot_id": "robot-1",
            "rejected_frontier_id": "frontier-1",
            "fallback_frontier_id": "A",
            "source_rank": 0,
        }
    ]
    assert guarded.decisions[1].target is not None
    assert guarded.decisions[1].target.frontier_id == "A"


def test_per_robot_execution_map_rejects_disconnected_global_frontier(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.75, 3.75),
            "robot-1": (2.5, 2.5),
        },
    )
    fused_free = np.ones((100, 100), dtype=bool)
    robot_0_free = np.zeros((100, 100), dtype=bool)
    robot_0_free[5:35, 5:35] = True
    robot_0_free[60:90, 60:90] = True
    robot_1_free = np.ones((100, 100), dtype=bool)

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_from_free_mask(
            fused_free,
            transform_version="multi-robot-source-derived",
        ),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (0.75, 0.75),
            "robot-1": (0.75, 0.75),
        },
        execution_snapshots_by_robot={
            "robot-0": snapshot_from_free_mask(
                robot_0_free,
                transform_version="robot-0-transform-v1",
            ),
            "robot-1": snapshot_from_free_mask(
                robot_1_free,
                transform_version="robot-1-transform-v1",
            ),
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
    )

    assert report["status"] == "unsafe_frontiers_suppressed"
    assert report["blocked_robot_ids"] == ["robot-0"]
    assert report["effective_active_robot_ids"] == ["robot-1"]
    robot_0_check = report["checks"]["robot-0"]
    assert robot_0_check["target_known_free"] is True
    assert robot_0_check["target_reachable_known_free"] is False
    assert robot_0_check["safe_approach_cell_count"] == 0
    assert robot_0_check["reachability_filter_applied"] is True
    assert report["checks"]["robot-1"]["passed"] is True
    assert (
        report["execution_map_contracts"]["robot-0"]["transform_version"]
        == "robot-0-transform-v1"
    )
    assert [item.mode.value for item in guarded.decisions] == ["HOLD", "GOAL"]


def test_clear_frontiers_preserve_candidate_batch(observation_factory):
    observations, _registry, digests, now = ready_registries(observation_factory)
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


def test_far_frontier_is_segmented_below_robot_local_admission_limit(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (12.0, 1.0), "robot-1": (2.0, 2.0)},
    )
    execution = snapshot_from_free_mask(
        np.ones((300, 300), dtype=bool),
        transform_version="robot-execution-v1",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (1.0, 1.0),
            "robot-1": (1.0, 1.0),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
        bounded_approach_projection_by_robot={
            "robot-0": True,
            "robot-1": True,
        },
        projection_path_clearance_by_robot_m={
            "robot-0": 0.05,
            "robot-1": 0.30,
        },
        maximum_execution_distance_by_robot_m={
            "robot-0": 7.5,
            "robot-1": 7.5,
        },
    )

    execution_target = guarded.decisions[0].target
    assert execution_target is not None
    execution_distance = math.hypot(
        execution_target.pose.x - 1.0,
        execution_target.pose.y - 1.0,
    )
    check = report["checks"]["robot-0"]
    assert check["execution_distance_projection_required"] is True
    assert check["direct_target_within_execution_limit"] is False
    assert check["pass_mode"] == "distance_bounded_safe_partial_progress"
    assert execution_distance <= 7.5 + 1e-12
    assert execution_distance > 7.0
    assert report["execution_lineage"]["robot-0"]["source_target_xy_m"] == [
        12.0,
        1.0,
    ]
    assert report["execution_lineage"]["robot-0"]["execution_target_xy_m"] != [
        12.0,
        1.0,
    ]


def test_failure_memory_rejection_uses_robot_specific_source_ranked_fallback(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (3.0, 3.0), "robot-1": (4.0, 4.0)},
    )
    execution = snapshot_from_free_mask(
        np.ones((100, 100), dtype=bool),
        transform_version="robot-execution-v1",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers_by_robot={
            "robot-0": [
                {
                    "frontier_id": "D",
                    "x_m": 3.5,
                    "y_m": 2.5,
                    "source_rank": 1,
                    "source_probability": 0.3,
                },
                {
                    "frontier_id": "C",
                    "x_m": 2.5,
                    "y_m": 3.5,
                    "source_rank": 2,
                    "source_probability": 0.2,
                },
            ],
            "robot-1": [],
        },
        pre_rejected_robot_ids=frozenset({"robot-0"}),
        robot_xy_by_robot={
            "robot-0": (1.0, 1.0),
            "robot-1": (1.0, 1.0),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
    )

    robot_0 = guarded.decisions[0]
    assert robot_0.target is not None
    assert robot_0.target.frontier_id == "D"
    assert report["checks"]["robot-0"]["failure_memory_rejected"] is True
    assert report["fallback_assignments"] == [
        {
            "robot_id": "robot-0",
            "rejected_frontier_id": "frontier-0",
            "fallback_frontier_id": "D",
            "source_rank": 1,
        }
    ]
    lineage = report["execution_lineage"]["robot-0"]
    assert lineage["source_frontier_id"] == "D"
    assert lineage["execution_frontier_id"] == "D"
    assert lineage["source_target_xy_m"] == [3.5, 2.5]


def test_start_seed_snap_is_independent_of_clearance_for_both_robots(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (3.0, 3.0), "robot-1": (4.0, 4.0)},
    )
    free = np.zeros((100, 100), dtype=bool)
    free[20:80, 20:80] = True
    execution = snapshot_from_free_mask(
        free,
        transform_version="robot-execution-v1",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (0.60, 2.0),
            "robot-1": (2.0, 0.60),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
    )

    assert guarded == candidate
    assert report["status"] == "frontiers_clear"
    for robot_id, clearance, snap_radius in (
        ("robot-0", 0.35, 0.75),
        ("robot-1", 0.34, 1.0),
    ):
        check = report["checks"][robot_id]
        assert check["start_seed_distance_m"] > clearance
        assert check["maximum_start_seed_distance_m"] == snap_radius
        assert check["start_seed_within_limit"] is True
        assert check["reachable_known_free_cell_count"] > 0
        assert check["passed"] is True


def narrow_corridor_snapshot(*, transform_version: str) -> MapSnapshot:
    free = np.zeros((100, 100), dtype=bool)
    free[20:80, 20:80] = True
    free[49:52, 5:20] = True
    return snapshot_from_free_mask(
        free,
        transform_version=transform_version,
    )


def test_nearby_unsafe_frontier_projects_to_start_connected_safe_cell(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.0, 3.0),
            "robot-1": (0.575, 2.525),
        },
    )
    execution = narrow_corridor_snapshot(transform_version="robot-execution-v1")

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (3.0, 2.0),
            "robot-1": (3.0, 2.525),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
        bounded_approach_projection_by_robot={
            "robot-0": True,
            "robot-1": True,
        },
    )

    assert report["status"] == "frontiers_projected_to_safe_approaches"
    check = report["checks"]["robot-1"]
    assert check["direct_approach_passed"] is False
    assert check["projected_approach_passed"] is True
    assert check["pass_mode"] == "bounded_safe_approach_projection"
    assert check["projection_excess_beyond_arrival_m"] < 0.34
    assert check["reachable_footprint_clear_cell_count"] > 0
    assert report["selected_frontier_projected_robot_ids"] == ["robot-1"]
    assert report["blocked_robot_ids"] == []
    assert len(report["approach_projections"]) == 1
    original = candidate.decisions[1].target
    projected = guarded.decisions[1].target
    assert original is not None
    assert projected is not None
    assert projected.frontier_id == original.frontier_id
    assert projected.pose.x > original.pose.x
    assert projected.pose.x == check["projected_target_xy_m"][0]
    assert projected.pose.y == check["projected_target_xy_m"][1]


def test_start_connected_safe_partial_progress_can_exceed_one_clearance(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.0, 3.0),
            "robot-1": (0.475, 2.525),
        },
    )
    execution = narrow_corridor_snapshot(transform_version="robot-execution-v1")

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (3.0, 2.0),
            "robot-1": (3.0, 2.525),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
        bounded_approach_projection_by_robot={
            "robot-0": True,
            "robot-1": True,
        },
    )

    check = report["checks"]["robot-1"]
    assert check["direct_approach_passed"] is False
    assert check["projected_approach_passed"] is True
    assert check["projection_excess_beyond_arrival_m"] > 0.34
    assert check["pass_mode"] == "start_connected_safe_partial_progress"
    assert check["projected_source_progress_m"] > 0.25
    assert report["blocked_robot_ids"] == []
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "GOAL"]


def test_unknown_boundary_frontier_projects_to_start_connected_safe_cell(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.0, 3.0),
            # Immediately outside the known-free corridor: this is the normal
            # representation of a free/unknown frontier centroid.
            "robot-1": (0.225, 2.525),
        },
    )
    execution = narrow_corridor_snapshot(transform_version="robot-execution-v1")

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (3.0, 2.0),
            "robot-1": (3.0, 2.525),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
        bounded_approach_projection_by_robot={
            "robot-0": True,
            "robot-1": True,
        },
    )

    check = report["checks"]["robot-1"]
    assert check["target_in_bounds"] is True
    assert check["target_known_free"] is False
    assert check["target_reachable_known_free"] is False
    assert check["direct_approach_passed"] is False
    assert check["projected_approach_passed"] is True
    assert check["pass_mode"] == "start_connected_safe_partial_progress"
    assert check["reachable_footprint_clear_cell_count"] > 0
    assert check["projected_source_progress_m"] > 0.25
    assert report["blocked_robot_ids"] == []
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "GOAL"]


def test_projection_uses_router_clearance_and_keeps_endpoint_footprint_clear(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            # Unknown target beyond the main room. The robot starts in the
            # narrow observed stem and must reach a full-clearance endpoint.
            "robot-0": (4.525, 2.525),
            "robot-1": (3.0, 3.0),
        },
    )
    execution = narrow_corridor_snapshot(transform_version="robot-execution-v1")

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (0.275, 2.525),
            "robot-1": (3.0, 2.0),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
        bounded_approach_projection_by_robot={
            "robot-0": True,
            "robot-1": True,
        },
        projection_path_clearance_by_robot_m={
            "robot-0": 0.05,
            "robot-1": 0.30,
        },
    )

    check = report["checks"]["robot-0"]
    assert check["target_known_free"] is False
    assert check["projection_path_clearance_m"] == 0.05
    assert check["required_clearance_m"] == 0.35
    assert check["projection_start_seed_within_limit"] is True
    assert check["reachable_footprint_clear_cell_count"] > 0
    assert check["projected_approach_passed"] is True
    assert check["pass_mode"] == "bounded_safe_approach_projection"
    assert report["blocked_robot_ids"] == []
    assert guarded.decisions[0].mode.value == "GOAL"


def test_safe_partial_progress_holds_when_robot_is_already_at_boundary(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.0, 3.0),
            "robot-1": (0.475, 2.525),
        },
    )
    execution = narrow_corridor_snapshot(transform_version="robot-execution-v1")

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        robot_xy_by_robot={
            "robot-0": (3.0, 2.0),
            "robot-1": (1.40, 2.525),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
        bounded_approach_projection_by_robot={
            "robot-0": True,
            "robot-1": True,
        },
    )

    check = report["checks"]["robot-1"]
    assert check["projected_minimum_travel_m"] < 0.10
    assert check["projected_approach_passed"] is False
    assert report["blocked_robot_ids"] == ["robot-1"]
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "HOLD"]


def test_rejected_frontiers_use_one_clear_source_remaining_frontier(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
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
    assert report["fallback_checks"]["robot-0"][0]["required_clearance_m"] == 0.35
    assert report["fallback_checks"]["robot-1"][0]["passed"] is True
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
    observations, _registry, digests, now = ready_registries(observation_factory)
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
        fallback_frontiers=[{"frontier_id": "A", "x_m": 1.075, "y_m": 1.075}],
        robot_xy_by_robot={"robot-0": (0.0, 0.0)},
    )

    assert report["status"] == "unsafe_frontiers_suppressed"
    assert report["blocked_robot_ids"] == ["robot-0"]
    assert report["fallback_assignments"] == []
    assert report["fallback_checks"]["robot-0"][0]["passed"] is False
    assert [item.mode.value for item in guarded.decisions] == ["HOLD", "GOAL"]


def test_fallback_matching_maximizes_safe_active_robots(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (1.0, 1.0),
            "robot-1": (1.0, 1.0),
        },
    )
    robot_0_free = np.ones((100, 100), dtype=bool)
    robot_0_free[10:30, 10:30] = False
    robot_1_free = np.zeros((100, 100), dtype=bool)
    robot_1_free[35:70, 35:70] = True
    robot_0_map = snapshot_from_free_mask(
        robot_0_free,
        transform_version="robot-0-transform-v1",
    )
    robot_1_map = snapshot_from_free_mask(
        robot_1_free,
        transform_version="robot-1-transform-v1",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        robot_0_map,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers=[
            {"frontier_id": "A", "x_m": 3.0, "y_m": 3.0},
            {"frontier_id": "C", "x_m": 4.0, "y_m": 4.0},
        ],
        robot_xy_by_robot={
            "robot-0": (2.5, 2.5),
            "robot-1": (2.5, 2.5),
        },
        execution_snapshots_by_robot={
            "robot-0": robot_0_map,
            "robot-1": robot_1_map,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
    )

    assert report["blocked_robot_ids"] == []
    assert report["fallback_assignments"] == [
        {
            "robot_id": "robot-0",
            "rejected_frontier_id": "frontier-0",
            "fallback_frontier_id": "C",
            "source_rank": 1,
        },
        {
            "robot_id": "robot-1",
            "rejected_frontier_id": "frontier-1",
            "fallback_frontier_id": "A",
            "source_rank": 0,
        },
    ]
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "GOAL"]
    assert [
        item.target.frontier_id if item.target is not None else None
        for item in guarded.decisions
    ] == ["C", "A"]


def test_fallback_matching_uses_execution_priority_before_source_rank(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (3.5, 3.5),
            "robot-1": (3.0, 3.0),
        },
    )
    free = np.ones((100, 100), dtype=bool)
    execution = snapshot_from_free_mask(
        free,
        transform_version="robot-execution-v1",
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        execution,
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers_by_robot={
            "robot-1": [
                {
                    "frontier_id": "rear",
                    "x_m": 1.5,
                    "y_m": 1.5,
                    "source_rank": 0,
                    "execution_priority_rank": 1,
                },
                {
                    "frontier_id": "forward",
                    "x_m": 4.0,
                    "y_m": 3.5,
                    "source_rank": 2,
                    "execution_priority_rank": 0,
                },
            ]
        },
        pre_rejected_robot_ids=frozenset({"robot-1"}),
        robot_xy_by_robot={
            "robot-0": (0.75, 0.75),
            "robot-1": (0.75, 0.75),
        },
        execution_snapshots_by_robot={
            "robot-0": execution,
            "robot-1": execution,
        },
        start_seed_snap_radius_by_robot_m={
            "robot-0": 0.75,
            "robot-1": 1.0,
        },
    )

    assert report["fallback_assignments"] == [
        {
            "robot_id": "robot-1",
            "rejected_frontier_id": "frontier-1",
            "fallback_frontier_id": "forward",
            "source_rank": 2,
            "execution_priority_rank": 0,
        }
    ]
    assert guarded.decisions[1].target is not None
    assert guarded.decisions[1].target.frontier_id == "forward"


def test_clear_frontiers_ignore_fallback_candidates(observation_factory):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = with_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (3.0, 3.0), "robot-1": (4.0, 4.0)},
    )

    guarded, report = apply_frontier_clearance_guard(
        candidate,
        snapshot_with_one_wall_trap(),
        clearance_by_robot_m={"robot-0": 0.35, "robot-1": 0.34},
        fallback_frontiers=[{"frontier_id": "A", "x_m": 5.0, "y_m": 5.0}],
        robot_xy_by_robot={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.0, 0.0),
        },
    )

    assert report["status"] == "frontiers_clear"
    assert report["fallback_assignments"] == []
    assert guarded == candidate
