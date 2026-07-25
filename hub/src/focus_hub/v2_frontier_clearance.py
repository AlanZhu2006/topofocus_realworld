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


FRONTIER_CLEARANCE_SCHEMA_VERSION = "focus-v2-frontier-clearance-guard-v2"


def _bounded_id(value: str) -> str:
    if len(value) <= 128:
        return value
    suffix = hashlib.sha256(value.encode("utf-8")).hexdigest()[:16]
    return f"{value[:111]}-{suffix}"


def _frontier_report(
    snapshot: MapSnapshot,
    decision: HighLevelDecisionV2,
    *,
    clearance_m: float,
) -> dict[str, object]:
    target = decision.target
    if target is None or target.kind != "FRONTIER_POINT":
        raise ValueError("frontier report requires a frontier decision")
    if not math.isfinite(clearance_m) or clearance_m <= 0.0:
        raise ValueError("frontier clearance must be positive and finite")
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
    safe = known_free & (clearance_field_m + 1e-12 >= clearance_m)

    rows, columns = np.nonzero(safe)
    target_x = float(target.pose.x)
    target_y = float(target.pose.y)
    arrival_radius_m = (
        float(target.source_goal_dilation_cells)
        * decision.map_provenance.resolution_m
    )
    if rows.size:
        safe_x = (
            snapshot.origin_xy_m[0]
            + (columns.astype(np.float64) + 0.5) * snapshot.resolution_m
        )
        safe_y = (
            snapshot.origin_xy_m[1]
            + (rows.astype(np.float64) + 0.5) * snapshot.resolution_m
        )
        safe_distances = np.hypot(safe_x - target_x, safe_y - target_y)
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
    target_clearance_m = (
        float(clearance_field_m[row, column]) if in_bounds else None
    )
    return {
        "robot_id": decision.robot_id,
        "frontier_id": target.frontier_id,
        "target_xy_m": [target_x, target_y],
        "target_cell_rc": [row, column],
        "target_in_bounds": in_bounds,
        "target_known_free": target_known_free,
        "target_clearance_m": target_clearance_m,
        "required_clearance_m": clearance_m,
        "arrival_radius_m": arrival_radius_m,
        "safe_approach_cell_count": approach_count,
        "nearest_safe_approach_distance_m": nearest_safe_m,
        "maximum_approach_clearance_m": maximum_approach_clearance_m,
        "passed": approach_count > 0,
    }


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
        normalized.append(
            {
                "source_rank": rank,
                "frontier_id": frontier_id,
                "x_m": float(x_m),
                "y_m": float(y_m),
            }
        )
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
    robot_xy_by_robot: Mapping[str, tuple[float, float]] | None = None,
) -> tuple[DecisionBatchV2, dict[str, object]]:
    """Reallocate rejected frontier legs once, then hold any still unsafe."""

    active_ids = list(
        batch.decisions[0].coordination.active_robot_ids
        if batch.decisions
        else ()
    )
    checks: dict[str, dict[str, object]] = {}
    rejected: set[str] = set()
    selected_frontier_ids: set[str] = set()
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
            snapshot,
            decision,
            clearance_m=float(clearance_by_robot_m[decision.robot_id]),
        )
        checks[decision.robot_id] = check
        if check["passed"] is not True:
            rejected.add(decision.robot_id)

    candidates = _normalize_fallback_frontiers(
        fallback_frontiers,
        selected_frontier_ids=selected_frontier_ids,
    )
    positions = robot_xy_by_robot or {}
    available = list(candidates)
    fallback_checks: dict[str, list[dict[str, object]]] = {}
    replacements: dict[str, HighLevelDecisionV2] = {}
    assignments: list[dict[str, object]] = []
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
            continue
        robot_reports: list[dict[str, object]] = []
        for candidate in tuple(available):
            replacement = _fallback_decision(
                decision,
                frontier=candidate,
                robot_xy_m=(float(robot_xy[0]), float(robot_xy[1])),
            )
            report = _frontier_report(
                snapshot,
                replacement,
                clearance_m=float(
                    clearance_by_robot_m[decision.robot_id]
                ),
            )
            report["source_rank"] = candidate["source_rank"]
            robot_reports.append(report)
            if report["passed"] is True:
                replacements[decision.robot_id] = replacement
                available.remove(candidate)
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
                break
        fallback_checks[decision.robot_id] = robot_reports

    held = rejected - replacements.keys()
    guarded = (
        batch
        if not rejected
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
    return guarded, {
        "schema_version": FRONTIER_CLEARANCE_SCHEMA_VERSION,
        "status": (
            "frontiers_clear"
            if not rejected
            else (
                "unsafe_frontiers_reallocated"
                if replacements
                else (
                    "all_active_frontiers_blocked"
                    if not effective_ids
                    else "unsafe_frontiers_suppressed"
                )
            )
        ),
        "method": (
            "known-free distance transform over the frozen fused map; require "
            "one footprint-clear cell inside the source frontier arrival disk; "
            "rejected selections try source-ranked remaining frontiers once"
        ),
        "classification": (
            "source-derived real-world physical execution guard over the "
            "frozen shared-frame map"
        ),
        "original_active_robot_ids": active_ids,
        "effective_active_robot_ids": effective_ids,
        "selected_frontier_rejected_robot_ids": sorted(rejected),
        "blocked_robot_ids": sorted(held),
        "checks": checks,
        "fallback_candidates": candidates,
        "fallback_checks": fallback_checks,
        "fallback_assignments": assignments,
        "map_contract": {
            "frame_id": snapshot.frame_id,
            "resolution_m": snapshot.resolution_m,
            "transform_version": snapshot.transform_version,
            "shared_frame_calibration_id": (
                snapshot.shared_frame_calibration_id
            ),
            "map_format_version": snapshot.map_format_version,
        },
        "source_fidelity": (
            "the exact VLM-selected target remains unchanged in the preserved "
            "candidate batch; any execution fallback is an unused frontier "
            "from the same frozen source manifest and must independently pass "
            "the unchanged physical-clearance guard"
        ),
    }
