"""Shared frontier-goal continuity guard for physical execution.

The immutable source still produces a fresh VLM allocation at every source
decision boundary.  Its ``main.py`` logical-analysis block retains the
previous frontier while the robot remains at least 25 source map cells from
that previous goal.  This is a *remaining-distance* rule, not an inter-round
progress threshold.  The following source rule independently replaces a goal
when the robot moved no more than 2.5 cells between boundaries; that separate
stationary rule is implemented by :mod:`focus_hub.v2_source_replan`.

The simulator changes its high-level selection once the previous goal is
inside 25 cells.  A physical local planner may still be driving toward that
goal at this boundary.  The deployment adapter therefore keeps the exact
previous target until it also enters the source 10-cell arrival disk, reports
ARRIVED, or is explicitly rejected.  This prevents a fresh history point
behind the robot from reversing an unfinished leg.

Semantic targets and explicitly rejected previous legs always pass through.
"""
from __future__ import annotations

import hashlib
import math
from typing import Mapping

from .transport_v2 import (
    DecisionBatchV2,
    HighLevelDecisionV2,
)


GOAL_CONTINUITY_SCHEMA_VERSION = "focus-v2-goal-continuity-guard-v4"
SOURCE_CONTINUITY_RETAIN_DISTANCE_CELLS = 25.0
SOURCE_MAP_RESOLUTION_M = 0.05
SOURCE_CONTINUITY_RETAIN_DISTANCE_M = (
    SOURCE_CONTINUITY_RETAIN_DISTANCE_CELLS * SOURCE_MAP_RESOLUTION_M
)
SOURCE_FRONTIER_COMPLETION_DISTANCE_CELLS = 10.0
SOURCE_FRONTIER_COMPLETION_DISTANCE_M = (
    SOURCE_FRONTIER_COMPLETION_DISTANCE_CELLS * SOURCE_MAP_RESOLUTION_M
)
Point2 = tuple[float, float]


def _bounded_id(value: str) -> str:
    if len(value) <= 128:
        return value
    suffix = hashlib.sha256(value.encode("utf-8")).hexdigest()[:16]
    return f"{value[:111]}-{suffix}"


def _finite_point(value: object) -> Point2 | None:
    if (
        not isinstance(value, (tuple, list))
        or len(value) != 2
        or isinstance(value[0], bool)
        or isinstance(value[1], bool)
        or not isinstance(value[0], (int, float))
        or not isinstance(value[1], (int, float))
    ):
        return None
    point = (float(value[0]), float(value[1]))
    return point if all(math.isfinite(item) for item in point) else None


def _frontier_xy(decision: HighLevelDecisionV2) -> Point2 | None:
    target = decision.target
    if target is None or target.kind != "FRONTIER_POINT":
        return None
    return (float(target.pose.x), float(target.pose.y))


def _retained_decision(
    current: HighLevelDecisionV2,
    previous: HighLevelDecisionV2,
    *,
    current_xy: Point2,
) -> HighLevelDecisionV2:
    if previous.target is None or previous.target.kind != "FRONTIER_POINT":
        raise ValueError("frontier continuity requires a previous frontier")
    raw = current.model_dump(mode="json")
    raw["target"] = previous.target.model_dump(mode="json")
    raw["target"]["frontier_id"] = _bounded_id(
        "continuity-"
        f"r{previous.round_index}-"
        f"{previous.target.frontier_id}"
    )
    raw["target"]["pose"]["yaw_rad"] = math.atan2(
        previous.target.pose.y - current_xy[1],
        previous.target.pose.x - current_xy[0],
    )
    raw["reason"] = (
        "shared real-world continuity guard retained the previous safe "
        "unfinished frontier until its source arrival disk or an explicit "
        "local terminal result"
    )
    return HighLevelDecisionV2.model_validate(raw)


def source_continuity_memory_batch(
    clearance_batch: DecisionBatchV2,
    *,
    clearance_report: Mapping[str, object],
) -> tuple[DecisionBatchV2, dict[str, object]]:
    """Restore pre-projection source targets for the next source boundary.

    The physical clearance guard may project a frontier onto a reachable
    arrival disk.  Source ``pre_g_points`` remembers the selected frontier,
    not that robot-specific execution projection.  Source-ranked fallback
    frontiers are also preserved through the guard's explicit lineage.
    This batch is memory-only and must never be published as a command.
    """

    raw_lineage = clearance_report.get("execution_lineage")
    if not isinstance(raw_lineage, Mapping):
        raise ValueError("clearance report lacks execution lineage")
    replacements: dict[str, HighLevelDecisionV2] = {}
    records: dict[str, dict[str, object]] = {}
    for decision in clearance_batch.decisions:
        target = decision.target
        if (
            decision.mode.value != "GOAL"
            or target is None
            or target.kind != "FRONTIER_POINT"
        ):
            records[decision.robot_id] = {
                "status": "not_an_executed_frontier",
                "mode": decision.mode.value,
                "target_kind": None if target is None else target.kind,
            }
            continue
        lineage = raw_lineage.get(decision.robot_id)
        if not isinstance(lineage, Mapping):
            raise ValueError(
                f"{decision.robot_id} frontier lacks clearance lineage"
            )
        source_id = lineage.get("source_frontier_id")
        execution_id = lineage.get("execution_frontier_id")
        source_xy = _finite_point(lineage.get("source_target_xy_m"))
        execution_xy = _finite_point(lineage.get("execution_target_xy_m"))
        decision_xy = _frontier_xy(decision)
        if (
            not isinstance(source_id, str)
            or not source_id
            or execution_id != target.frontier_id
            or source_xy is None
            or execution_xy is None
            or decision_xy is None
            or math.dist(execution_xy, decision_xy) > 1e-9
            or lineage.get("execution_mode") != "GOAL"
        ):
            raise ValueError(
                f"{decision.robot_id} clearance lineage differs from its "
                "execution target"
            )
        raw = decision.model_dump(mode="json")
        raw["target"]["frontier_id"] = source_id
        raw["target"]["pose"]["x"] = source_xy[0]
        raw["target"]["pose"]["y"] = source_xy[1]
        restored = HighLevelDecisionV2.model_validate(raw)
        replacements[decision.robot_id] = restored
        records[decision.robot_id] = {
            "status": "source_target_restored_for_memory",
            "source_frontier_id": source_id,
            "source_target_xy_m": list(source_xy),
            "execution_frontier_id": target.frontier_id,
            "execution_target_xy_m": list(execution_xy),
            "projection_removed_from_memory": (
                math.dist(source_xy, execution_xy) > 1e-9
            ),
            "selection_source": lineage.get("selection_source"),
        }
    memory_batch = DecisionBatchV2(
        decisions=tuple(
            replacements.get(decision.robot_id, decision)
            for decision in clearance_batch.decisions
        )
    )
    return memory_batch, {
        "schema_version": "focus-v2-source-continuity-memory-v1",
        "classification": (
            "source-derived memory-only batch; never physical authority"
        ),
        "policy": (
            "remember each source-selected frontier before robot-specific "
            "clearance projection; retain semantic and HOLD decisions "
            "unchanged"
        ),
        "robots": records,
        "batch": memory_batch.model_dump(mode="json"),
    }


def apply_frontier_goal_continuity(
    batch: DecisionBatchV2,
    *,
    previous_batch: DecisionBatchV2 | None,
    current_shared_positions: Mapping[str, object],
    minimum_remaining_distance_m: float = (
        SOURCE_CONTINUITY_RETAIN_DISTANCE_M
    ),
    minimum_completion_distance_m: float = (
        SOURCE_FRONTIER_COMPLETION_DISTANCE_M
    ),
    previous_rejected_robot_ids: frozenset[str] = frozenset(),
    previous_execution_lineage: Mapping[str, object] | None = None,
) -> tuple[DecisionBatchV2, dict[str, object]]:
    """Retain an unfinished source frontier until its arrival disk.

    The returned target is still checked by the current round's per-robot
    footprint/reachability guard before publication.  This function never
    retains a frontier over a semantic-region target.  The source's 25-cell
    switch condition remains explicit in the report; the narrower 10-cell
    physical completion condition only stabilizes real execution.  When the
    previous source point was not itself published because the clearance
    guard projected it, that memory-only point cannot override a different
    fresh source/VLM selection at the next boundary.
    """

    if (
        not math.isfinite(minimum_remaining_distance_m)
        or minimum_remaining_distance_m <= 0.0
    ):
        raise ValueError(
            "minimum remaining distance must be positive and finite"
        )
    if (
        not math.isfinite(minimum_completion_distance_m)
        or minimum_completion_distance_m <= 0.0
        or minimum_completion_distance_m
        >= minimum_remaining_distance_m
    ):
        raise ValueError(
            "completion distance must be positive and smaller than the "
            "source continuity distance"
        )
    if not previous_rejected_robot_ids.issubset(
        {decision.robot_id for decision in batch.decisions}
    ):
        raise ValueError("previous rejected robot is outside the current batch")
    if previous_execution_lineage is not None and not isinstance(
        previous_execution_lineage, Mapping
    ):
        raise ValueError("previous execution lineage must be an object")

    current_by_robot = {
        decision.robot_id: decision for decision in batch.decisions
    }
    previous_by_robot = (
        {
            decision.robot_id: decision
            for decision in previous_batch.decisions
        }
        if previous_batch is not None
        else {}
    )
    if previous_batch is not None:
        current_first = batch.decisions[0]
        previous_first = previous_batch.decisions[0]
        if (
            current_first.scene_id != previous_first.scene_id
            or current_first.episode_id != previous_first.episode_id
            or current_first.goal_category != previous_first.goal_category
            or current_first.round_index != previous_first.round_index + 1
        ):
            raise ValueError(
                "previous continuity batch is not the adjacent source round"
            )

    replacements: dict[str, HighLevelDecisionV2] = {}
    source_rule_retained: list[str] = []
    physical_completion_retained: list[str] = []
    projected_previous_legs_released: list[str] = []
    checks: dict[str, dict[str, object]] = {}
    for robot_id, current in current_by_robot.items():
        previous = previous_by_robot.get(robot_id)
        current_target_xy = _frontier_xy(current)
        previous_target_xy = (
            _frontier_xy(previous) if previous is not None else None
        )
        current_xy = _finite_point(current_shared_positions.get(robot_id))
        check: dict[str, object] = {
            "robot_id": robot_id,
            "retained": False,
            "reason": "no_previous_source_boundary_leg",
        }
        checks[robot_id] = check
        if previous is None:
            continue
        if robot_id in previous_rejected_robot_ids:
            check["reason"] = "previous_frontier_leg_explicitly_rejected"
            continue
        if current.mode.value != "GOAL":
            check["reason"] = "current_robot_not_active"
            continue
        if current.target is not None and current.target.kind == "SEMANTIC_REGION":
            check["reason"] = "semantic_target_preempts_frontier_continuity"
            continue
        if (
            previous.mode.value != "GOAL"
            or current_target_xy is None
            or previous_target_xy is None
        ):
            check["reason"] = "frontier_leg_not_comparable"
            continue
        if current_xy is None:
            check["reason"] = "shared_pose_unavailable"
            continue

        previous_execution_xy: Point2 | None = None
        previous_projection_distance_m: float | None = None
        if previous_execution_lineage is not None:
            raw_lineage = previous_execution_lineage.get(robot_id)
            if raw_lineage is not None:
                if not isinstance(raw_lineage, Mapping):
                    raise ValueError(
                        f"previous execution lineage for {robot_id} is malformed"
                    )
                lineage_source_xy = _finite_point(
                    raw_lineage.get("source_target_xy_m")
                )
                previous_execution_xy = _finite_point(
                    raw_lineage.get("execution_target_xy_m")
                )
                if (
                    lineage_source_xy is None
                    or previous_execution_xy is None
                    or math.dist(lineage_source_xy, previous_target_xy) > 1e-9
                    or raw_lineage.get("execution_mode") != "GOAL"
                ):
                    raise ValueError(
                        f"previous execution lineage for {robot_id} differs "
                        "from its source continuity target"
                    )
                previous_projection_distance_m = math.dist(
                    lineage_source_xy,
                    previous_execution_xy,
                )

        distance_to_previous_target_m = math.dist(
            current_xy, previous_target_xy
        )
        check.update(
            {
                "current_position_xy_m": list(current_xy),
                "previous_target_xy_m": list(previous_target_xy),
                "candidate_target_xy_m": list(current_target_xy),
                "distance_to_previous_target_m": (
                    distance_to_previous_target_m
                ),
                "minimum_remaining_distance_m": (
                    minimum_remaining_distance_m
                ),
                "minimum_completion_distance_m": (
                    minimum_completion_distance_m
                ),
                "previous_frontier_id": previous.target.frontier_id,
                "candidate_frontier_id": current.target.frontier_id,
            }
        )
        if math.dist(current_target_xy, previous_target_xy) <= 1e-9:
            check["reason"] = "source_target_already_continuous"
            continue
        if (
            previous_projection_distance_m is not None
            and previous_projection_distance_m > 1e-9
        ):
            projected_previous_legs_released.append(robot_id)
            check.update(
                {
                    "reason": (
                        "previous_source_frontier_was_projected_fresh_"
                        "source_target_accepted"
                    ),
                    "previous_execution_target_xy_m": list(
                        previous_execution_xy
                    ),
                    "previous_projection_distance_m": (
                        previous_projection_distance_m
                    ),
                    "retention_authority": (
                        "released_projected_memory_only_source_target"
                    ),
                }
            )
            continue
        if (
            distance_to_previous_target_m
            <= minimum_completion_distance_m + 1e-12
        ):
            check["reason"] = (
                "previous_goal_inside_source_10_cell_arrival_disk"
            )
            continue

        retained = _retained_decision(
            current,
            previous,
            current_xy=current_xy,
        )
        replacements[robot_id] = retained
        retained_by_source_rule = bool(
            distance_to_previous_target_m
            >= minimum_remaining_distance_m - 1e-12
        )
        if retained_by_source_rule:
            source_rule_retained.append(robot_id)
        else:
            physical_completion_retained.append(robot_id)
        check.update(
            {
                "retained": True,
                "reason": (
                    "previous_frontier_beyond_source_25_cell_boundary_retained"
                    if retained_by_source_rule
                    else (
                        "unfinished_previous_frontier_outside_source_"
                        "10_cell_arrival_disk_retained"
                    )
                ),
                "retention_authority": (
                    "source_25_cell_continuity_rule"
                    if retained_by_source_rule
                    else "realworld_unfinished_leg_completion_adapter"
                ),
                "execution_frontier_id": retained.target.frontier_id,
                "execution_target_xy_m": list(previous_target_xy),
            }
        )

    guarded = batch
    if replacements:
        guarded = DecisionBatchV2(
            decisions=tuple(
                replacements.get(decision.robot_id, decision)
                for decision in batch.decisions
            )
        )
    retained_robot_ids = sorted(replacements)
    return guarded, {
        "schema_version": GOAL_CONTINUITY_SCHEMA_VERSION,
        "status": (
            "distant_previous_frontiers_retained"
            if retained_robot_ids
            else (
                "initial_round_no_continuity_candidate"
                if previous_batch is None
                else "source_targets_accepted"
            )
        ),
        "classification": (
            "source-derived real-world execution guard over adjacent frozen "
            "shared-frame robot poses and the prior published frontier leg"
        ),
        "policy": (
            "apply identically to both robots: preserve the source 25-cell "
            "continuity rule, then keep the exact accepted frontier through "
            "the real-world handoff band until the base enters its source "
            "10-cell arrival disk; the independent 2.5-cell stationary rule "
            "runs afterward in source order; semantic targets and rejected "
            "prior legs pass through"
        ),
        "retained_robot_ids": retained_robot_ids,
        "source_rule_retained_robot_ids": sorted(source_rule_retained),
        "physical_completion_retained_robot_ids": sorted(
            physical_completion_retained
        ),
        "projected_previous_legs_released_robot_ids": sorted(
            projected_previous_legs_released
        ),
        "minimum_remaining_distance_m": minimum_remaining_distance_m,
        "minimum_completion_distance_m": (
            minimum_completion_distance_m
        ),
        "source_retain_distance_cells": (
            SOURCE_CONTINUITY_RETAIN_DISTANCE_CELLS
        ),
        "source_frontier_completion_distance_cells": (
            SOURCE_FRONTIER_COMPLETION_DISTANCE_CELLS
        ),
        "source_map_resolution_m": SOURCE_MAP_RESOLUTION_M,
        "previous_rejected_robot_ids": sorted(
            previous_rejected_robot_ids
        ),
        "checks": checks,
        "source_fidelity": (
            "the current VLM candidate batch is preserved separately; every "
            "retained execution target is an exact prior-round source-derived "
            "frontier coordinate; the source 1.25 m threshold measures "
            "current-position-to-previous-goal distance exactly as "
            "source/Focus_realworld/main.py:1833, not inter-round motion; "
            "physical execution continues the same accepted leg between "
            "that switch boundary and the source 0.50 m arrival disk; the "
            "rule never lets a memory-only raw source point override a "
            "different fresh source/VLM choice when the prior physical leg "
            "was instead projected to another coordinate; the "
            "retained target must pass the current robot-local frozen-map "
            "clearance guard before publication"
        ),
    }
