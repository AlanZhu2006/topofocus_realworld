"""Physical approach-clearance guard for source-derived frontier targets.

The immutable HPC source remains authoritative for frontier extraction and
VLM selection.  A real robot, however, cannot place its centre on every
free/unknown boundary centroid.  This execution adapter preserves the
selected candidate in the pre-guard artifact, but removes motion authority
when the frozen fused map has no known-free, footprint-clear approach cell
inside the source's arrival radius.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import hashlib
import math

import numpy as np
from scipy import ndimage

from .map_snapshot import MapSnapshot
from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2


FRONTIER_CLEARANCE_SCHEMA_VERSION = "focus-v2-frontier-clearance-guard-v6"
MINIMUM_PROJECTED_TRAVEL_M = 0.10
MINIMUM_SOURCE_PROGRESS_M = 0.25


def _bounded_id(value: str) -> str:
    if len(value) <= 128:
        return value
    suffix = hashlib.sha256(value.encode("utf-8")).hexdigest()[:16]
    return f"{value[:111]}-{suffix}"


def _point_to_cell_footprint_distances(
    snapshot: MapSnapshot,
    rows: np.ndarray,
    columns: np.ndarray,
    *,
    x_m: float,
    y_m: float,
) -> np.ndarray:
    """Return point-to-cell-square distances for the requested grid cells."""

    center_x = (
        snapshot.origin_xy_m[0]
        + (columns.astype(np.float64) + 0.5) * snapshot.resolution_m
    )
    center_y = (
        snapshot.origin_xy_m[1]
        + (rows.astype(np.float64) + 0.5) * snapshot.resolution_m
    )
    half_cell = snapshot.resolution_m / 2.0
    dx = np.maximum(np.abs(center_x - x_m) - half_cell, 0.0)
    dy = np.maximum(np.abs(center_y - y_m) - half_cell, 0.0)
    return np.hypot(dx, dy)


def _reachable_known_free(
    snapshot: MapSnapshot,
    known_free: np.ndarray,
    *,
    robot_xy_m: tuple[float, float] | None,
    maximum_seed_distance_m: float,
    connectivity: int = 8,
) -> tuple[np.ndarray, dict[str, object]]:
    """Return the robot-local known-free component containing its measured pose."""

    if robot_xy_m is None:
        return known_free, {
            "reachability_filter_applied": False,
            "maximum_start_seed_distance_m": None,
            "start_seed_cell_rc": None,
            "start_seed_distance_m": None,
            "start_seed_within_limit": None,
            "reachable_known_free_cell_count": int(np.count_nonzero(known_free)),
        }
    if connectivity not in {4, 8}:
        raise ValueError("reachability connectivity must be 4 or 8")
    if (
        len(robot_xy_m) != 2
        or not all(math.isfinite(float(value)) for value in robot_xy_m)
        or not math.isfinite(maximum_seed_distance_m)
        or maximum_seed_distance_m <= 0.0
    ):
        raise ValueError("robot reachability seed inputs must be finite")

    rows, columns = np.nonzero(known_free)
    if not rows.size:
        return np.zeros_like(known_free), {
            "reachability_filter_applied": True,
            "maximum_start_seed_distance_m": maximum_seed_distance_m,
            "start_seed_cell_rc": None,
            "start_seed_distance_m": None,
            "start_seed_within_limit": False,
            "reachable_known_free_cell_count": 0,
        }
    distances = _point_to_cell_footprint_distances(
        snapshot,
        rows,
        columns,
        x_m=float(robot_xy_m[0]),
        y_m=float(robot_xy_m[1]),
    )
    seed_index = int(np.argmin(distances))
    seed_distance_m = float(distances[seed_index])
    seed = (int(rows[seed_index]), int(columns[seed_index]))
    if seed_distance_m > maximum_seed_distance_m + 1e-12:
        return np.zeros_like(known_free), {
            "reachability_filter_applied": True,
            "maximum_start_seed_distance_m": maximum_seed_distance_m,
            "start_seed_cell_rc": list(seed),
            "start_seed_distance_m": seed_distance_m,
            "start_seed_within_limit": False,
            "reachable_known_free_cell_count": 0,
        }

    labels, _count = ndimage.label(
        known_free,
        structure=(
            np.ones((3, 3), dtype=bool)
            if connectivity == 8
            else np.asarray(
                [
                    [False, True, False],
                    [True, True, True],
                    [False, True, False],
                ],
                dtype=bool,
            )
        ),
    )
    label_id = int(labels[seed])
    reachable = labels == label_id
    return reachable, {
        "reachability_filter_applied": True,
        "maximum_start_seed_distance_m": maximum_seed_distance_m,
        "start_seed_cell_rc": list(seed),
        "start_seed_distance_m": seed_distance_m,
        "start_seed_within_limit": True,
        "reachable_known_free_cell_count": int(np.count_nonzero(reachable)),
    }


def _frontier_report(
    snapshot: MapSnapshot,
    decision: HighLevelDecisionV2,
    *,
    clearance_m: float,
    robot_xy_m: tuple[float, float] | None = None,
    maximum_seed_distance_m: float | None = None,
    allow_bounded_approach_projection: bool = False,
    projection_path_clearance_m: float | None = None,
) -> dict[str, object]:
    target = decision.target
    if target is None or target.kind != "FRONTIER_POINT":
        raise ValueError("frontier report requires a frontier decision")
    if not math.isfinite(clearance_m) or clearance_m <= 0.0:
        raise ValueError("frontier clearance must be positive and finite")
    if projection_path_clearance_m is None:
        projection_path_clearance_m = clearance_m
    if (
        not math.isfinite(projection_path_clearance_m)
        or projection_path_clearance_m <= 0.0
        or projection_path_clearance_m > clearance_m
    ):
        raise ValueError(
            "projection path clearance must be positive and no greater than "
            "the endpoint footprint clearance"
        )
    if robot_xy_m is not None and (
        maximum_seed_distance_m is None
        or not math.isfinite(maximum_seed_distance_m)
        or maximum_seed_distance_m <= 0.0
    ):
        raise ValueError(
            "frontier reachability requires an independent positive seed "
            "snap radius"
        )
    if snapshot.frame_id != "shared_world":
        raise ValueError("frontier clearance requires a shared_world map")
    if (
        snapshot.shared_frame_calibration_id
        != decision.map_provenance.shared_frame_calibration_id
    ):
        raise ValueError(
            "frontier decision/map shared-frame calibration mismatch"
        )
    if (
        snapshot.map_format_version
        != decision.map_provenance.map_format_version
    ):
        raise ValueError("frontier decision/map format mismatch")
    if not math.isclose(
        snapshot.resolution_m,
        decision.map_provenance.resolution_m,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("frontier decision/map resolution mismatch")

    obstacle = np.asarray(snapshot.grid[0] > 0.5, dtype=bool)
    explored = np.asarray(snapshot.grid[1] > 0.5, dtype=bool)
    known_free = explored & ~obstacle
    clearance_field_m = (
        ndimage.distance_transform_edt(known_free) * snapshot.resolution_m
    )
    reachable, reachability = _reachable_known_free(
        snapshot,
        known_free,
        robot_xy_m=robot_xy_m,
        maximum_seed_distance_m=(
            clearance_m
            if maximum_seed_distance_m is None
            else maximum_seed_distance_m
        ),
    )
    safe = (
        known_free
        & reachable
        & (clearance_field_m + 1e-12 >= clearance_m)
    )

    rows, columns = np.nonzero(safe)
    target_x = float(target.pose.x)
    target_y = float(target.pose.y)
    arrival_radius_m = (
        float(target.source_goal_dilation_cells)
        * decision.map_provenance.resolution_m
    )
    if rows.size:
        safe_distances = _point_to_cell_footprint_distances(
            snapshot,
            rows,
            columns,
            x_m=target_x,
            y_m=target_y,
        )
        inside = safe_distances <= arrival_radius_m + 1e-12
        approach_count = int(np.count_nonzero(inside))
        nearest_safe_m = float(np.min(safe_distances))
        maximum_approach_clearance_m = (
            float(np.max(clearance_field_m[rows[inside], columns[inside]]))
            if approach_count
            else None
        )
    else:
        approach_count = 0
        nearest_safe_m = None
        maximum_approach_clearance_m = None

    column = math.floor(
        (target_x - snapshot.origin_xy_m[0]) / snapshot.resolution_m
    )
    row = math.floor(
        (target_y - snapshot.origin_xy_m[1]) / snapshot.resolution_m
    )
    in_bounds = (
        0 <= row < known_free.shape[0]
        and 0 <= column < known_free.shape[1]
    )
    target_known_free = bool(known_free[row, column]) if in_bounds else False
    target_reachable_known_free = (
        bool(reachable[row, column]) if in_bounds else False
    )
    target_clearance_m = (
        float(clearance_field_m[row, column]) if in_bounds else None
    )
    projection_reachability = {
        "projection_start_seed_cell_rc": None,
        "projection_start_seed_distance_m": None,
        "projection_start_seed_within_limit": None,
        "reachable_footprint_clear_cell_count": None,
    }
    projected_target_xy_m: list[float] | None = None
    projected_approach_cell_rc: list[int] | None = None
    projected_approach_cell_xy_m: list[float] | None = None
    projection_distance_m: float | None = None
    projection_excess_beyond_arrival_m: float | None = None
    projected_minimum_travel_m: float | None = None
    projected_source_progress_m: float | None = None
    projection_within_clearance_extension: bool | None = None
    if (
        allow_bounded_approach_projection
        and approach_count == 0
        and robot_xy_m is not None
        and in_bounds
    ):
        # A source frontier normally lies on a known-free/unknown boundary, so
        # its centroid is not required to be a known-free cell.  Reachability
        # follows the same independently configured graph clearance as the
        # robot-local router, while the projected endpoint must still satisfy
        # the full measured footprint clearance.  The local planner and depth
        # stop retain final authority over every intermediate motion.
        path_traversable = (
            known_free
            & (
                clearance_field_m + 1e-12
                >= projection_path_clearance_m
            )
        )
        reachable_path, path_reachability = _reachable_known_free(
            snapshot,
            path_traversable,
            robot_xy_m=robot_xy_m,
            maximum_seed_distance_m=float(maximum_seed_distance_m),
            connectivity=4,
        )
        reachable_safe = (
            reachable_path
            & (clearance_field_m + 1e-12 >= clearance_m)
        )
        projection_reachability = {
            "projection_start_seed_cell_rc": path_reachability[
                "start_seed_cell_rc"
            ],
            "projection_start_seed_distance_m": path_reachability[
                "start_seed_distance_m"
            ],
            "projection_start_seed_within_limit": path_reachability[
                "start_seed_within_limit"
            ],
            "reachable_footprint_clear_cell_count": int(
                np.count_nonzero(reachable_safe)
            ),
        }
        safe_rows, safe_columns = np.nonzero(reachable_safe)
        if safe_rows.size:
            safe_x = (
                snapshot.origin_xy_m[0]
                + (safe_columns.astype(np.float64) + 0.5)
                * snapshot.resolution_m
            )
            safe_y = (
                snapshot.origin_xy_m[1]
                + (safe_rows.astype(np.float64) + 0.5)
                * snapshot.resolution_m
            )
            center_distances = np.hypot(
                safe_x - target_x,
                safe_y - target_y,
            )
            selected = int(np.argmin(center_distances))
            candidate_x = float(safe_x[selected])
            candidate_y = float(safe_y[selected])
            projection_distance_m = float(center_distances[selected])
            projection_excess_beyond_arrival_m = max(
                0.0,
                projection_distance_m - arrival_radius_m,
            )
            projected_minimum_travel_m = max(
                0.0,
                math.hypot(
                    candidate_x - float(robot_xy_m[0]),
                    candidate_y - float(robot_xy_m[1]),
                )
                - arrival_radius_m,
            )
            projected_source_progress_m = (
                math.hypot(
                    target_x - float(robot_xy_m[0]),
                    target_y - float(robot_xy_m[1]),
                )
                - projection_distance_m
            )
            projection_within_clearance_extension = bool(
                projection_excess_beyond_arrival_m
                <= clearance_m + 1e-12
            )
            if (
                projected_minimum_travel_m
                >= MINIMUM_PROJECTED_TRAVEL_M - 1e-12
                and projected_source_progress_m
                >= MINIMUM_SOURCE_PROGRESS_M - 1e-12
            ):
                projected_target_xy_m = [candidate_x, candidate_y]
                projected_approach_cell_rc = [
                    int(safe_rows[selected]),
                    int(safe_columns[selected]),
                ]
                projected_approach_cell_xy_m = [candidate_x, candidate_y]

    robot_to_frontier_distance_m = (
        None
        if robot_xy_m is None
        else math.hypot(
            target_x - float(robot_xy_m[0]),
            target_y - float(robot_xy_m[1]),
        )
    )
    minimum_direct_travel_m = (
        None
        if robot_to_frontier_distance_m is None
        else max(0.0, robot_to_frontier_distance_m - arrival_radius_m)
    )
    frontier_arrival_already_satisfied = (
        None
        if robot_to_frontier_distance_m is None
        else robot_to_frontier_distance_m <= arrival_radius_m + 1e-12
    )
    direct_approach_available = approach_count > 0
    direct_approach_has_useful_travel = (
        True
        if minimum_direct_travel_m is None
        else minimum_direct_travel_m
        >= MINIMUM_PROJECTED_TRAVEL_M - 1e-12
    )
    # A physical frontier is an exploration target, not a stationary scan
    # command.  Once the frozen base pose is already inside its arrival disk
    # (or would travel less than the existing 10 cm useful-progress floor),
    # re-publishing the same source frontier can create endless immediate
    # ARRIVED rounds without exposing new map area.  Mark it rejected here so
    # the unchanged source-ranked fallback path can choose another independently
    # clearance-checked frontier.  Calls without a measured base pose retain
    # the legacy pure-map behavior.
    direct_approach_passed = (
        direct_approach_available and direct_approach_has_useful_travel
    )
    projected_approach_passed = projected_target_xy_m is not None
    return {
        "robot_id": decision.robot_id,
        "frontier_id": target.frontier_id,
        "target_xy_m": [target_x, target_y],
        "target_cell_rc": [row, column],
        "target_in_bounds": in_bounds,
        "target_known_free": target_known_free,
        "target_reachable_known_free": target_reachable_known_free,
        "target_clearance_m": target_clearance_m,
        "required_clearance_m": clearance_m,
        "arrival_radius_m": arrival_radius_m,
        "approach_distance_method": "point_to_grid_cell_footprint",
        "safe_approach_cell_count": approach_count,
        "nearest_safe_approach_distance_m": nearest_safe_m,
        "maximum_approach_clearance_m": maximum_approach_clearance_m,
        "direct_approach_available": direct_approach_available,
        "robot_to_frontier_distance_m": robot_to_frontier_distance_m,
        "minimum_direct_travel_m": minimum_direct_travel_m,
        "minimum_required_travel_m": MINIMUM_PROJECTED_TRAVEL_M,
        "frontier_arrival_already_satisfied": (
            frontier_arrival_already_satisfied
        ),
        "direct_approach_has_useful_travel": (
            direct_approach_has_useful_travel
        ),
        "direct_approach_passed": direct_approach_passed,
        "bounded_approach_projection_enabled": (
            allow_bounded_approach_projection
        ),
        "projection_path_clearance_m": projection_path_clearance_m,
        "projected_approach_passed": projected_approach_passed,
        "projected_target_xy_m": projected_target_xy_m,
        "projected_approach_cell_rc": projected_approach_cell_rc,
        "projected_approach_cell_xy_m": projected_approach_cell_xy_m,
        "projection_distance_m": projection_distance_m,
        "projection_excess_beyond_arrival_m": (
            projection_excess_beyond_arrival_m
        ),
        "projection_within_clearance_extension": (
            projection_within_clearance_extension
        ),
        "maximum_projection_excess_m": clearance_m,
        "projected_minimum_travel_m": projected_minimum_travel_m,
        "minimum_projected_travel_m": MINIMUM_PROJECTED_TRAVEL_M,
        "projected_source_progress_m": projected_source_progress_m,
        "minimum_source_progress_m": MINIMUM_SOURCE_PROGRESS_M,
        "execution_map_transform_version": snapshot.transform_version,
        **reachability,
        **projection_reachability,
        "pass_mode": (
            "direct_arrival_disk"
            if direct_approach_passed
            else (
                (
                    "bounded_safe_approach_projection"
                    if projection_within_clearance_extension
                    else "start_connected_safe_partial_progress"
                )
                if projected_approach_passed
                else "rejected"
            )
        ),
        "passed": direct_approach_passed or projected_approach_passed,
    }


def _projected_approach_decision(
    previous: HighLevelDecisionV2,
    *,
    report: Mapping[str, object],
    robot_xy_m: tuple[float, float],
) -> HighLevelDecisionV2:
    projected = report.get("projected_target_xy_m")
    if (
        not isinstance(projected, list)
        or len(projected) != 2
        or not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in projected
        )
    ):
        raise ValueError("frontier report has no valid projected approach")
    x_m, y_m = float(projected[0]), float(projected[1])
    raw = previous.model_dump(mode="json")
    raw["target"]["pose"]["x"] = x_m
    raw["target"]["pose"]["y"] = y_m
    raw["target"]["pose"]["yaw_rad"] = math.atan2(
        y_m - robot_xy_m[1],
        x_m - robot_xy_m[0],
    )
    raw["leg_id"] = _bounded_id(
        f"{previous.leg_id}-bounded-safe-approach"
    )
    raw["decision_id"] = _bounded_id(f"{raw['leg_id']}-lease-0")
    raw["reason"] = (
        "source frontier retained; real-world execution target projected to "
        "the closest start-connected footprint-clear cell that makes bounded "
        "progress toward the original source target"
    )
    return HighLevelDecisionV2.model_validate(raw)


def _fallback_decision(
    previous: HighLevelDecisionV2,
    *,
    frontier: Mapping[str, object],
    robot_xy_m: tuple[float, float],
) -> HighLevelDecisionV2:
    frontier_id = str(frontier["frontier_id"])
    x_m = float(frontier["x_m"])
    y_m = float(frontier["y_m"])
    raw = previous.model_dump(mode="json")
    raw["target"] = {
        "kind": "FRONTIER_POINT",
        "frontier_id": frontier_id,
        "source_goal_dilation_cells": 10,
        "pose": {
            "frame_id": "shared_world",
            "x": x_m,
            "y": y_m,
            "z": 0.0,
            "yaw_rad": math.atan2(
                y_m - robot_xy_m[1],
                x_m - robot_xy_m[0],
            ),
        },
    }
    raw["leg_id"] = _bounded_id(
        f"{previous.leg_id}-frontier-fallback-{frontier_id}"
    )
    raw["decision_id"] = _bounded_id(f"{raw['leg_id']}-lease-0")
    raw["reason"] = (
        "source-ranked remaining frontier passed the unchanged real-world "
        "footprint-clearance guard after the VLM-selected frontier was rejected"
    )
    return HighLevelDecisionV2.model_validate(raw)


def _normalize_fallback_frontiers(
    fallback_frontiers: Sequence[Mapping[str, object]] | None,
    *,
    selected_frontier_ids: set[str],
) -> list[dict[str, object]]:
    normalized: list[dict[str, object]] = []
    seen: set[str] = set()
    if fallback_frontiers is None:
        source_frontiers: Sequence[Mapping[str, object]] = ()
    elif isinstance(fallback_frontiers, (str, bytes)) or not isinstance(
        fallback_frontiers, Sequence
    ):
        raise ValueError("remaining frontiers must be a sequence")
    else:
        source_frontiers = fallback_frontiers
    for rank, frontier in enumerate(source_frontiers):
        if not isinstance(frontier, Mapping):
            raise ValueError("remaining frontier must be an object")
        frontier_id = frontier.get("frontier_id")
        x_m = frontier.get("x_m")
        y_m = frontier.get("y_m")
        if (
            not isinstance(frontier_id, str)
            or not frontier_id
            or len(frontier_id) > 128
        ):
            raise ValueError("remaining frontier has an invalid frontier_id")
        if (
            isinstance(x_m, bool)
            or isinstance(y_m, bool)
            or not isinstance(x_m, (int, float))
            or not isinstance(y_m, (int, float))
            or not math.isfinite(float(x_m))
            or not math.isfinite(float(y_m))
        ):
            raise ValueError(
                f"remaining frontier {frontier_id!r} has invalid coordinates"
            )
        if frontier_id in seen:
            raise ValueError(
                f"remaining frontier list repeats {frontier_id!r}"
            )
        if frontier_id in selected_frontier_ids:
            raise ValueError(
                f"remaining frontier {frontier_id!r} was already selected"
            )
        seen.add(frontier_id)
        source_rank = frontier.get("source_rank", rank)
        if (
            isinstance(source_rank, bool)
            or not isinstance(source_rank, int)
            or source_rank < 0
        ):
            raise ValueError(
                f"remaining frontier {frontier_id!r} has invalid source rank"
            )
        record: dict[str, object] = {
            "source_rank": source_rank,
            "frontier_id": frontier_id,
            "x_m": float(x_m),
            "y_m": float(y_m),
        }
        for optional in (
            "source_probability",
            "history_score",
            "source_candidate_kind",
            "source_order",
        ):
            value = frontier.get(optional)
            if isinstance(value, (str, int, float)) or value is None:
                record[optional] = value
        normalized.append(record)
    return normalized


def _rewrite_batch(
    batch: DecisionBatchV2,
    *,
    held_robot_ids: set[str],
    replacements: Mapping[str, HighLevelDecisionV2],
) -> DecisionBatchV2:
    original_active = tuple(
        batch.decisions[0].coordination.active_robot_ids
        if batch.decisions
        else ()
    )
    effective = [
        robot_id
        for robot_id in original_active
        if robot_id not in held_robot_ids
    ]
    decisions: list[HighLevelDecisionV2] = []
    for previous in batch.decisions:
        replacement = replacements.get(previous.robot_id)
        raw = (replacement or previous).model_dump(mode="json")
        raw["coordination"]["active_robot_ids"] = effective
        if previous.robot_id in held_robot_ids:
            raw["mode"] = "HOLD"
            raw["target"] = None
            raw["lease_sequence"] = 0
            raw["leg_id"] = _bounded_id(
                f"{previous.leg_id}-frontier-clearance-hold"
            )
            raw["decision_id"] = _bounded_id(f"{raw['leg_id']}-lease-0")
            raw["reason"] = (
                "real-world frontier-clearance guard rejected a source-derived "
                "frontier with no footprint-clear approach cell"
            )
        decisions.append(HighLevelDecisionV2.model_validate(raw))
    return DecisionBatchV2(decisions=tuple(decisions))


def apply_frontier_clearance_guard(
    batch: DecisionBatchV2,
    snapshot: MapSnapshot,
    *,
    clearance_by_robot_m: Mapping[str, float],
    fallback_frontiers: Sequence[Mapping[str, object]] | None = None,
    fallback_frontiers_by_robot: Mapping[
        str, Sequence[Mapping[str, object]]
    ]
    | None = None,
    pre_rejected_robot_ids: frozenset[str] = frozenset(),
    robot_xy_by_robot: Mapping[str, tuple[float, float]] | None = None,
    execution_snapshots_by_robot: Mapping[str, MapSnapshot] | None = None,
    start_seed_snap_radius_by_robot_m: Mapping[str, float] | None = None,
    bounded_approach_projection_by_robot: Mapping[str, bool] | None = None,
    projection_path_clearance_by_robot_m: Mapping[str, float] | None = None,
) -> tuple[DecisionBatchV2, dict[str, object]]:
    """Reallocate rejected frontier legs once, then hold any still unsafe.

    When per-robot frozen snapshots are supplied, a target must have a
    footprint-clear approach cell in that robot's own known-free component.
    The fused snapshot remains the source/VLM coordinate authority only.
    """

    active_ids = list(
        batch.decisions[0].coordination.active_robot_ids
        if batch.decisions
        else ()
    )
    if not pre_rejected_robot_ids.issubset(set(active_ids)):
        raise ValueError(
            "pre-rejected frontier robot is outside the active batch"
        )
    if fallback_frontiers_by_robot is not None:
        unknown = set(fallback_frontiers_by_robot).difference(active_ids)
        if unknown:
            raise ValueError(
                "per-robot fallback candidates contain inactive robots: "
                f"{sorted(unknown)}"
            )
    checks: dict[str, dict[str, object]] = {}
    rejected: set[str] = set()
    selected_frontier_ids: set[str] = set()
    positions = robot_xy_by_robot or {}
    use_robot_execution_maps = execution_snapshots_by_robot is not None
    seed_snap_radii = start_seed_snap_radius_by_robot_m or {}
    projection_policy = bounded_approach_projection_by_robot or {}
    projection_path_clearances = (
        projection_path_clearance_by_robot_m or {}
    )

    def execution_snapshot(robot_id: str) -> MapSnapshot:
        if execution_snapshots_by_robot is None:
            return snapshot
        if robot_id not in execution_snapshots_by_robot:
            raise ValueError(
                f"missing frozen execution map for {robot_id}"
            )
        return execution_snapshots_by_robot[robot_id]

    def reachability_position(
        robot_id: str,
    ) -> tuple[float, float] | None:
        if not use_robot_execution_maps:
            return None
        raw = positions.get(robot_id)
        if (
            not isinstance(raw, (tuple, list))
            or len(raw) != 2
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in raw
            )
        ):
            raise ValueError(
                f"missing finite frozen shared pose for {robot_id}"
            )
        return (float(raw[0]), float(raw[1]))

    def start_seed_snap_radius(robot_id: str) -> float | None:
        if not use_robot_execution_maps:
            return None
        raw = seed_snap_radii.get(robot_id)
        if (
            isinstance(raw, bool)
            or not isinstance(raw, (int, float))
            or not math.isfinite(float(raw))
            or float(raw) <= 0.0
        ):
            raise ValueError(
                f"missing positive start seed snap radius for {robot_id}"
            )
        return float(raw)

    def projection_path_clearance(robot_id: str) -> float:
        endpoint_clearance = float(clearance_by_robot_m[robot_id])
        raw = projection_path_clearances.get(
            robot_id, endpoint_clearance
        )
        if (
            isinstance(raw, bool)
            or not isinstance(raw, (int, float))
            or not math.isfinite(float(raw))
            or not 0.0 < float(raw) <= endpoint_clearance
        ):
            raise ValueError(
                "projection path clearance must be positive and no greater "
                f"than endpoint clearance for {robot_id}"
            )
        return float(raw)

    replacements: dict[str, HighLevelDecisionV2] = {}
    approach_projections: list[dict[str, object]] = []
    selected_projected: set[str] = set()

    def maybe_project(
        decision: HighLevelDecisionV2,
        report: Mapping[str, object],
        *,
        source: str,
    ) -> HighLevelDecisionV2:
        if report.get("pass_mode") not in {
            "bounded_safe_approach_projection",
            "start_connected_safe_partial_progress",
        }:
            return decision
        robot_xy = reachability_position(decision.robot_id)
        if robot_xy is None:
            raise ValueError("approach projection requires a frozen robot pose")
        projected = _projected_approach_decision(
            decision,
            report=report,
            robot_xy_m=robot_xy,
        )
        approach_projections.append(
            {
                "robot_id": decision.robot_id,
                "frontier_id": report["frontier_id"],
                "source": source,
                "original_target_xy_m": report["target_xy_m"],
                "execution_target_xy_m": report["projected_target_xy_m"],
                "projection_distance_m": report["projection_distance_m"],
                "projection_excess_beyond_arrival_m": report[
                    "projection_excess_beyond_arrival_m"
                ],
                "source_progress_m": report[
                    "projected_source_progress_m"
                ],
                "pass_mode": report["pass_mode"],
            }
        )
        return projected

    for decision in batch.decisions:
        if decision.robot_id not in active_ids:
            continue
        target = decision.target
        if target is None or target.kind != "FRONTIER_POINT":
            continue
        selected_frontier_ids.add(target.frontier_id)
        if decision.robot_id not in clearance_by_robot_m:
            raise ValueError(
                f"missing frontier clearance for {decision.robot_id}"
            )
        check = _frontier_report(
            execution_snapshot(decision.robot_id),
            decision,
            clearance_m=float(clearance_by_robot_m[decision.robot_id]),
            robot_xy_m=reachability_position(decision.robot_id),
            maximum_seed_distance_m=start_seed_snap_radius(
                decision.robot_id
            ),
            allow_bounded_approach_projection=bool(
                projection_policy.get(decision.robot_id, False)
            ),
            projection_path_clearance_m=projection_path_clearance(
                decision.robot_id
            ),
        )
        checks[decision.robot_id] = check
        if decision.robot_id in pre_rejected_robot_ids:
            check["map_guard_passed_before_failure_memory"] = bool(
                check["passed"]
            )
            check["failure_memory_rejected"] = True
            check["pass_mode"] = "rejected_by_navigation_failure_memory"
            check["passed"] = False
            rejected.add(decision.robot_id)
        elif check["passed"] is True:
            projected = maybe_project(
                decision,
                check,
                source="vlm_selected_frontier",
            )
            if projected is not decision:
                replacements[decision.robot_id] = projected
                selected_projected.add(decision.robot_id)
        else:
            rejected.add(decision.robot_id)

    candidates = _normalize_fallback_frontiers(
        fallback_frontiers,
        selected_frontier_ids=selected_frontier_ids,
    )
    candidates_by_robot: dict[str, list[dict[str, object]]] = {}
    for robot_id in active_ids:
        candidates_by_robot[robot_id] = (
            candidates
            if fallback_frontiers_by_robot is None
            else _normalize_fallback_frontiers(
                fallback_frontiers_by_robot.get(robot_id, ()),
                selected_frontier_ids=selected_frontier_ids,
            )
        )
    fallback_checks: dict[str, list[dict[str, object]]] = {}
    assignments: list[dict[str, object]] = []
    fallback_options: dict[
        str,
        list[
            tuple[
                dict[str, object],
                HighLevelDecisionV2,
                dict[str, object],
            ]
        ],
    ] = {}
    rejected_decisions = [
        decision
        for decision in batch.decisions
        if decision.robot_id in rejected
    ]
    for decision in batch.decisions:
        if decision.robot_id not in rejected:
            continue
        robot_xy = positions.get(decision.robot_id)
        if (
            not isinstance(robot_xy, (tuple, list))
            or len(robot_xy) != 2
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in robot_xy
            )
        ):
            fallback_checks[decision.robot_id] = []
            fallback_options[decision.robot_id] = []
            continue
        robot_reports: list[dict[str, object]] = []
        robot_options: list[
            tuple[
                dict[str, object],
                HighLevelDecisionV2,
                dict[str, object],
            ]
        ] = []
        for candidate in candidates_by_robot[decision.robot_id]:
            replacement = _fallback_decision(
                decision,
                frontier=candidate,
                robot_xy_m=(float(robot_xy[0]), float(robot_xy[1])),
            )
            report = _frontier_report(
                execution_snapshot(decision.robot_id),
                replacement,
                clearance_m=float(
                    clearance_by_robot_m[decision.robot_id]
                ),
                robot_xy_m=reachability_position(decision.robot_id),
                maximum_seed_distance_m=start_seed_snap_radius(
                    decision.robot_id
                ),
                allow_bounded_approach_projection=bool(
                    projection_policy.get(decision.robot_id, False)
                ),
                projection_path_clearance_m=projection_path_clearance(
                    decision.robot_id
                ),
            )
            report["source_rank"] = candidate["source_rank"]
            robot_reports.append(report)
            if report["passed"] is True:
                robot_options.append(
                    (candidate, replacement, report)
                )
        fallback_checks[decision.robot_id] = robot_reports
        fallback_options[decision.robot_id] = robot_options

    # A source-ranked greedy pass can consume the only safe frontier of the
    # next robot even when the first robot has another valid fallback.  Choose
    # the deterministic maximum-cardinality matching first, then minimize the
    # total source rank and finally preserve robot order as a tie-break.  The
    # candidate set is tiny (two robots in the real-world contract), and every
    # selected edge has independently passed the unchanged physical guard.
    best_matching: dict[
        str,
        tuple[
            dict[str, object],
            HighLevelDecisionV2,
            dict[str, object],
        ],
    ] = {}
    best_key: tuple[object, ...] | None = None

    def search_matching(
        index: int,
        used_frontiers: set[str],
        current: dict[
            str,
            tuple[
                dict[str, object],
                HighLevelDecisionV2,
                dict[str, object],
            ],
        ],
    ) -> None:
        nonlocal best_matching, best_key
        if index >= len(rejected_decisions):
            all_ranks = [
                int(candidate["source_rank"])
                for options in candidates_by_robot.values()
                for candidate in options
            ]
            sentinel = (max(all_ranks) + 1) if all_ranks else 1
            ranks = tuple(
                int(current[decision.robot_id][0]["source_rank"])
                if decision.robot_id in current
                else sentinel
                for decision in rejected_decisions
            )
            key: tuple[object, ...] = (
                -len(current),
                sum(
                    int(option[0]["source_rank"])
                    for option in current.values()
                ),
                ranks,
            )
            if best_key is None or key < best_key:
                best_key = key
                best_matching = dict(current)
            return

        decision = rejected_decisions[index]
        search_matching(index + 1, used_frontiers, current)
        for option in fallback_options.get(decision.robot_id, []):
            frontier_id = str(option[0]["frontier_id"])
            if frontier_id in used_frontiers:
                continue
            used_frontiers.add(frontier_id)
            current[decision.robot_id] = option
            search_matching(index + 1, used_frontiers, current)
            del current[decision.robot_id]
            used_frontiers.remove(frontier_id)

    search_matching(0, set(), {})
    for decision in rejected_decisions:
        option = best_matching.get(decision.robot_id)
        if option is None:
            continue
        candidate, replacement, report = option
        replacement = maybe_project(
            replacement,
            report,
            source="source_ranked_fallback_frontier",
        )
        replacements[decision.robot_id] = replacement
        assignments.append(
            {
                "robot_id": decision.robot_id,
                "rejected_frontier_id": checks[
                    decision.robot_id
                ]["frontier_id"],
                "fallback_frontier_id": candidate["frontier_id"],
                "source_rank": candidate["source_rank"],
            }
        )

    held = rejected - replacements.keys()
    guarded = (
        batch
        if not rejected and not replacements
        else _rewrite_batch(
            batch,
            held_robot_ids=held,
            replacements=replacements,
        )
    )
    effective_ids = list(
        guarded.decisions[0].coordination.active_robot_ids
        if guarded.decisions
        else ()
    )
    guarded_by_robot = {
        decision.robot_id: decision for decision in guarded.decisions
    }
    assignment_by_robot = {
        str(item["robot_id"]): item for item in assignments
    }
    projection_by_robot = {
        str(item["robot_id"]): item for item in approach_projections
    }
    execution_lineage: dict[str, dict[str, object]] = {}
    for original in batch.decisions:
        original_target = original.target
        if (
            original_target is None
            or original_target.kind != "FRONTIER_POINT"
        ):
            continue
        executed = guarded_by_robot[original.robot_id]
        executed_target = executed.target
        source_frontier_id = original_target.frontier_id
        source_target_xy_m: list[float] = [
            float(original_target.pose.x),
            float(original_target.pose.y),
        ]
        selection_source = "guard_input"
        assignment = assignment_by_robot.get(original.robot_id)
        if assignment is not None:
            fallback_id = str(assignment["fallback_frontier_id"])
            fallback_report = next(
                (
                    item
                    for item in fallback_checks.get(
                        original.robot_id, []
                    )
                    if item.get("frontier_id") == fallback_id
                ),
                None,
            )
            if fallback_report is not None:
                source_frontier_id = fallback_id
                source_target_xy_m = list(
                    fallback_report["target_xy_m"]
                )
                selection_source = "source_ranked_fallback"
        projection = projection_by_robot.get(original.robot_id)
        execution_lineage[original.robot_id] = {
            "input_frontier_id": original_target.frontier_id,
            "input_target_xy_m": [
                float(original_target.pose.x),
                float(original_target.pose.y),
            ],
            "source_frontier_id": source_frontier_id,
            "source_target_xy_m": source_target_xy_m,
            "selection_source": selection_source,
            "projected": projection is not None,
            "projection": projection,
            "execution_mode": executed.mode.value,
            "execution_frontier_id": (
                None
                if executed_target is None
                or executed_target.kind != "FRONTIER_POINT"
                else executed_target.frontier_id
            ),
            "execution_target_xy_m": (
                None
                if executed_target is None
                or executed_target.kind != "FRONTIER_POINT"
                else [
                    float(executed_target.pose.x),
                    float(executed_target.pose.y),
                ]
            ),
        }
    return guarded, {
        "schema_version": FRONTIER_CLEARANCE_SCHEMA_VERSION,
        "status": (
            "frontiers_clear"
            if not rejected and not selected_projected
            else (
                "frontiers_projected_to_safe_approaches"
                if not rejected
                else (
                    "unsafe_frontiers_reallocated"
                    if assignments
                    else (
                        "all_active_frontiers_blocked"
                        if not effective_ids
                        else "unsafe_frontiers_suppressed"
                    )
                )
            )
        ),
        "method": (
            "known-free distance transform over each robot's frozen shared-frame "
            "map; require one cell in that robot's reachable known-free component "
            "whose footprint intersects the source frontier arrival disk; rejected "
            "selections try source-ranked remaining frontiers once"
            if use_robot_execution_maps
            else
            "known-free distance transform over the frozen fused map; require "
            "one footprint-clear cell whose footprint intersects the source "
            "frontier arrival disk; rejected selections try source-ranked "
            "remaining frontiers once"
        ),
        "classification": (
            "source-derived real-world physical execution guard over frozen "
            "robot-local maps registered in the shared frame"
            if use_robot_execution_maps
            else
            "source-derived real-world physical execution guard over the "
            "frozen shared-frame map"
        ),
        "original_active_robot_ids": active_ids,
        "effective_active_robot_ids": effective_ids,
        "selected_frontier_rejected_robot_ids": sorted(rejected),
        "selected_frontier_projected_robot_ids": sorted(selected_projected),
        "blocked_robot_ids": sorted(held),
        "checks": checks,
        "fallback_candidates": candidates,
        "fallback_candidates_by_robot": candidates_by_robot,
        "fallback_checks": fallback_checks,
        "fallback_assignments": assignments,
        "approach_projections": approach_projections,
        "pre_rejected_robot_ids": sorted(pre_rejected_robot_ids),
        "execution_lineage": execution_lineage,
        "start_seed_snap_radius_by_robot_m": {
            robot_id: start_seed_snap_radius(robot_id)
            for robot_id in active_ids
        },
        "projection_path_clearance_by_robot_m": {
            robot_id: projection_path_clearance(robot_id)
            for robot_id in active_ids
        },
        "map_contract": {
            "frame_id": snapshot.frame_id,
            "resolution_m": snapshot.resolution_m,
            "transform_version": snapshot.transform_version,
            "shared_frame_calibration_id": (
                snapshot.shared_frame_calibration_id
            ),
            "map_format_version": snapshot.map_format_version,
        },
        "execution_map_contracts": {
            robot_id: {
                "frame_id": execution_snapshot(robot_id).frame_id,
                "resolution_m": execution_snapshot(robot_id).resolution_m,
                "transform_version": execution_snapshot(robot_id).transform_version,
                "shared_frame_calibration_id": (
                    execution_snapshot(robot_id).shared_frame_calibration_id
                ),
                "map_format_version": (
                    execution_snapshot(robot_id).map_format_version
                ),
            }
            for robot_id in active_ids
        },
        "source_fidelity": (
            "the exact VLM-selected target remains unchanged in its preserved "
            "candidate artifact; this guard accepts either that target or a "
            "separately recorded prior-round continuity target as input; an "
            "execution target may be projected only to "
            "the closest full-footprint-clear endpoint connected through the "
            "robot-local router's configured known-free graph clearance and "
            "producing minimum bounded progress toward the same source "
            "frontier; any "
            "execution fallback is an unused frontier from the same frozen "
            "source manifest, ordered by that robot's preserved source score "
            "when supplied, and must independently pass the same physical-"
            "clearance guard"
        ),
    }
