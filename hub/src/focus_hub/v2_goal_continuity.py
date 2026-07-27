"""Shared frontier-goal continuity guard for physical execution.

The immutable source still produces a fresh VLM allocation at every source
decision boundary.  This real-world adapter prevents either robot from
abandoning a safe, unfinished frontier leg while it is making measured
progress.  Semantic targets, completed legs, stalled legs, and small source
target updates always pass through unchanged.
"""
from __future__ import annotations

import hashlib
import math
from typing import Mapping

from .transport_v2 import (
    DecisionBatchV2,
    HighLevelDecisionV2,
    V2_MAP_RESOLUTION_M,
)


GOAL_CONTINUITY_SCHEMA_VERSION = "focus-v2-goal-continuity-guard-v1"
DEFAULT_MINIMUM_TARGET_SWITCH_M = 0.75
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
        "unfinished frontier while this robot was making measured progress"
    )
    return HighLevelDecisionV2.model_validate(raw)


def apply_frontier_goal_continuity(
    batch: DecisionBatchV2,
    *,
    previous_batch: DecisionBatchV2 | None,
    previous_shared_positions: Mapping[str, object],
    current_shared_positions: Mapping[str, object],
    minimum_progress_m: float,
    minimum_target_switch_m: float = DEFAULT_MINIMUM_TARGET_SWITCH_M,
) -> tuple[DecisionBatchV2, dict[str, object]]:
    """Retain a progressing unfinished frontier leg for either platform.

    The returned target is still checked by the current round's per-robot
    footprint/reachability guard before publication.  This function never
    retains a frontier over a semantic-region target.
    """

    if not math.isfinite(minimum_progress_m) or minimum_progress_m <= 0.0:
        raise ValueError("minimum progress must be positive and finite")
    if (
        not math.isfinite(minimum_target_switch_m)
        or minimum_target_switch_m <= 0.0
    ):
        raise ValueError("minimum target switch must be positive and finite")

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
    checks: dict[str, dict[str, object]] = {}
    for robot_id, current in current_by_robot.items():
        previous = previous_by_robot.get(robot_id)
        current_target_xy = _frontier_xy(current)
        previous_target_xy = (
            _frontier_xy(previous) if previous is not None else None
        )
        current_xy = _finite_point(current_shared_positions.get(robot_id))
        previous_xy = _finite_point(previous_shared_positions.get(robot_id))
        check: dict[str, object] = {
            "robot_id": robot_id,
            "retained": False,
            "reason": "no_previous_source_boundary_leg",
        }
        checks[robot_id] = check
        if previous is None:
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
        if current_xy is None or previous_xy is None:
            check["reason"] = "shared_pose_unavailable"
            continue

        displacement_m = math.dist(previous_xy, current_xy)
        distance_to_previous_target_m = math.dist(
            current_xy, previous_target_xy
        )
        previous_arrival_radius_m = (
            previous.target.source_goal_dilation_cells
            * V2_MAP_RESOLUTION_M
        )
        target_switch_m = math.dist(
            current_target_xy, previous_target_xy
        )
        check.update(
            {
                "previous_position_xy_m": list(previous_xy),
                "current_position_xy_m": list(current_xy),
                "measured_displacement_m": displacement_m,
                "minimum_progress_m": minimum_progress_m,
                "previous_target_xy_m": list(previous_target_xy),
                "candidate_target_xy_m": list(current_target_xy),
                "distance_to_previous_target_m": (
                    distance_to_previous_target_m
                ),
                "previous_arrival_radius_m": previous_arrival_radius_m,
                "target_switch_m": target_switch_m,
                "minimum_target_switch_m": minimum_target_switch_m,
                "previous_frontier_id": previous.target.frontier_id,
                "candidate_frontier_id": current.target.frontier_id,
            }
        )
        if displacement_m < minimum_progress_m:
            check["reason"] = "previous_leg_not_making_minimum_progress"
            continue
        if distance_to_previous_target_m <= previous_arrival_radius_m:
            check["reason"] = "previous_frontier_arrival_disk_reached"
            continue
        if target_switch_m < minimum_target_switch_m:
            check["reason"] = "source_target_update_is_small"
            continue

        retained = _retained_decision(
            current,
            previous,
            current_xy=current_xy,
        )
        replacements[robot_id] = retained
        check.update(
            {
                "retained": True,
                "reason": "progressing_unfinished_frontier_retained",
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
            "progressing_frontiers_retained"
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
            "apply identically to both robots: retain a materially changed "
            "frontier only while the prior leg is active, outside its arrival "
            "disk, and making minimum shared-frame progress; semantic targets "
            "and stalled, completed, or small-update legs pass through"
        ),
        "retained_robot_ids": retained_robot_ids,
        "minimum_progress_m": minimum_progress_m,
        "minimum_target_switch_m": minimum_target_switch_m,
        "checks": checks,
        "source_fidelity": (
            "the current VLM candidate batch is preserved separately; every "
            "retained execution target is an exact prior-round source-derived "
            "frontier coordinate and must pass the current robot-local frozen-"
            "map clearance guard before publication"
        ),
    }
