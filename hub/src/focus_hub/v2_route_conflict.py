"""Conservative shared-frame route-conflict guard for physical execution.

The immutable source still selects one target per agent.  This module is a
real-world execution adapter: it compares buffered straight-line corridors
from the frozen robot poses to those targets and serializes an unsafe pair
before either high-level GOAL is published.  A pair that is already above its
footprint-derived current-pose floor may proceed when neither complete route
segment ever brings it closer than the observed starting separation.  It does
not claim to know the robot-local planner paths.
"""
from __future__ import annotations

import hashlib
import math
from typing import Mapping

from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2


ROUTE_CONFLICT_SCHEMA_VERSION = "focus-v2-route-conflict-guard-v3"
Point2 = tuple[float, float]


def _bounded_id(value: str) -> str:
    if len(value) <= 128:
        return value
    suffix = hashlib.sha256(value.encode("utf-8")).hexdigest()[:16]
    return f"{value[:111]}-{suffix}"


def _cross(a: Point2, b: Point2, c: Point2) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (
        b[1] - a[1]
    ) * (c[0] - a[0])


def _point_segment_distance(point: Point2, start: Point2, end: Point2) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_sq
    fraction = min(1.0, max(0.0, fraction))
    nearest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def _segments_intersect(
    first_start: Point2,
    first_end: Point2,
    second_start: Point2,
    second_end: Point2,
) -> bool:
    epsilon = 1e-9
    orientations = (
        _cross(first_start, first_end, second_start),
        _cross(first_start, first_end, second_end),
        _cross(second_start, second_end, first_start),
        _cross(second_start, second_end, first_end),
    )
    if (
        orientations[0] * orientations[1] < -epsilon
        and orientations[2] * orientations[3] < -epsilon
    ):
        return True
    if abs(orientations[0]) <= epsilon and _point_segment_distance(
        second_start, first_start, first_end
    ) <= epsilon:
        return True
    if abs(orientations[1]) <= epsilon and _point_segment_distance(
        second_end, first_start, first_end
    ) <= epsilon:
        return True
    if abs(orientations[2]) <= epsilon and _point_segment_distance(
        first_start, second_start, second_end
    ) <= epsilon:
        return True
    return abs(orientations[3]) <= epsilon and _point_segment_distance(
        first_end, second_start, second_end
    ) <= epsilon


def segment_distance(
    first_start: Point2,
    first_end: Point2,
    second_start: Point2,
    second_end: Point2,
) -> float:
    """Return the Euclidean separation of two closed 2-D segments."""

    if _segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance(second_start, first_start, first_end),
        _point_segment_distance(second_end, first_start, first_end),
    )


def _target_xy(decision: HighLevelDecisionV2) -> Point2 | None:
    target = decision.target
    if target is None:
        return None
    if target.kind == "FRONTIER_POINT":
        return (float(target.pose.x), float(target.pose.y))
    return (
        float(target.display_centroid.x),
        float(target.display_centroid.y),
    )


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


def _serialized_batch(
    batch: DecisionBatchV2,
    *,
    leader: str | None,
) -> DecisionBatchV2:
    decisions: list[HighLevelDecisionV2] = []
    effective = [] if leader is None else [leader]
    for previous in batch.decisions:
        raw = previous.model_dump(mode="json")
        raw["coordination"]["active_robot_ids"] = effective
        if previous.robot_id != leader:
            raw["mode"] = "HOLD"
            raw["target"] = None
            raw["lease_sequence"] = 0
            raw["leg_id"] = _bounded_id(
                f"{previous.leg_id}-route-conflict-hold"
            )
            raw["decision_id"] = _bounded_id(f"{raw['leg_id']}-lease-0")
            raw["reason"] = (
                "real-world route-conflict guard serialized the concurrent "
                "source-derived allocation"
            )
        decisions.append(HighLevelDecisionV2.model_validate(raw))
    return DecisionBatchV2(decisions=tuple(decisions))


def apply_route_conflict_guard(
    batch: DecisionBatchV2,
    *,
    shared_start_xy: Mapping[str, object],
    minimum_separation_m: float,
    minimum_current_separation_m: float | None = None,
    priority_index: int = 0,
    goal_evidence_by_robot: Mapping[str, object] | None = None,
) -> tuple[DecisionBatchV2, dict[str, object]]:
    """Serialize a pair whose conservative shared-frame corridors overlap.

    A missing shared pose is also serialized because concurrent clearance
    cannot then be established. Semantic-region legs take priority. Within
    the eligible set, stronger current goal-category detector evidence wins;
    absent or tied evidence falls back to ``priority_index`` rotation.
    """

    if (
        not math.isfinite(minimum_separation_m)
        or minimum_separation_m <= 0.0
    ):
        raise ValueError("minimum route separation must be positive and finite")
    if minimum_current_separation_m is None:
        minimum_current_separation_m = minimum_separation_m
    if (
        not math.isfinite(minimum_current_separation_m)
        or minimum_current_separation_m <= 0.0
        or minimum_current_separation_m > minimum_separation_m
    ):
        raise ValueError(
            "minimum current separation must be positive, finite, and no "
            "greater than the route separation"
        )
    if priority_index < 0:
        raise ValueError("priority_index must be non-negative")
    normalized_evidence: dict[str, float] = {}
    for robot_id, raw_score in (goal_evidence_by_robot or {}).items():
        if (
            not isinstance(robot_id, str)
            or not robot_id
            or isinstance(raw_score, bool)
            or not isinstance(raw_score, (int, float))
            or not math.isfinite(float(raw_score))
            or not 0.0 <= float(raw_score) <= 1.0
        ):
            raise ValueError("goal evidence must map robot IDs to [0, 1]")
        normalized_evidence[robot_id] = float(raw_score)

    active = [
        decision
        for decision in batch.decisions
        if decision.mode.value == "GOAL"
    ]
    active_ids = [decision.robot_id for decision in active]
    routes: dict[str, dict[str, object]] = {}
    missing: list[str] = []
    for decision in active:
        start = _finite_point(shared_start_xy.get(decision.robot_id))
        goal = _target_xy(decision)
        if start is None or goal is None:
            missing.append(decision.robot_id)
            continue
        routes[decision.robot_id] = {
            "start_xy_m": list(start),
            "goal_xy_m": list(goal),
            "target_kind": decision.target.kind if decision.target else None,
        }

    pairwise: list[dict[str, object]] = []
    conflict = bool(missing)
    minimum_observed: float | None = None
    for first_index, first in enumerate(active):
        for second in active[first_index + 1 :]:
            if first.robot_id not in routes or second.robot_id not in routes:
                continue
            first_route = routes[first.robot_id]
            second_route = routes[second.robot_id]
            distance = segment_distance(
                tuple(first_route["start_xy_m"]),
                tuple(first_route["goal_xy_m"]),
                tuple(second_route["start_xy_m"]),
                tuple(second_route["goal_xy_m"]),
            )
            start_distance = math.dist(
                first_route["start_xy_m"],
                second_route["start_xy_m"],
            )
            minimum_observed = (
                distance
                if minimum_observed is None
                else min(minimum_observed, distance)
            )
            route_introduces_closer_approach = (
                distance < start_distance - 1e-9
            )
            current_separation_clear = (
                start_distance >= minimum_current_separation_m - 1e-12
            )
            initial_proximity_only = bool(
                distance < minimum_separation_m
                and current_separation_clear
                and not route_introduces_closer_approach
            )
            pair_conflict = bool(
                distance < minimum_separation_m
                and not initial_proximity_only
            )
            conflict = conflict or pair_conflict
            pairwise.append(
                {
                    "robot_ids": [first.robot_id, second.robot_id],
                    "start_separation_m": start_distance,
                    "straight_segment_separation_m": distance,
                    "route_introduces_closer_approach": (
                        route_introduces_closer_approach
                    ),
                    "current_separation_clear": current_separation_clear,
                    "initial_proximity_only": initial_proximity_only,
                    "conflict": pair_conflict,
                }
            )

    guarded = batch
    selected: str | None = None
    suppressed: list[str] = []
    priority_source: str | None = None
    if len(active) > 1 and conflict:
        eligible = [
            decision for decision in active if decision.robot_id in routes
        ]
        semantic = [
            decision
            for decision in eligible
            if decision.target is not None
            and decision.target.kind == "SEMANTIC_REGION"
        ]
        candidates = semantic or eligible
        if candidates:
            scored = [
                (
                    normalized_evidence[decision.robot_id],
                    decision,
                )
                for decision in candidates
                if decision.robot_id in normalized_evidence
            ]
            if scored:
                maximum_score = max(score for score, _decision in scored)
                tied = [
                    decision
                    for score, decision in scored
                    if math.isclose(
                        score,
                        maximum_score,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                ]
                selected = tied[priority_index % len(tied)].robot_id
                priority_source = "current_goal_detector_evidence"
            else:
                selected = candidates[
                    priority_index % len(candidates)
                ].robot_id
                priority_source = "rotating_source_robot_order"
        suppressed = [
            robot_id for robot_id in active_ids if robot_id != selected
        ]
        guarded = _serialized_batch(batch, leader=selected)

    if len(active) <= 1:
        status = "single_or_no_active_route"
    elif missing and selected is None:
        status = "blocked_missing_all_shared_poses"
    elif missing:
        status = "serialized_missing_shared_pose"
    elif conflict:
        status = "serialized_route_corridor_conflict"
    elif any(item["initial_proximity_only"] for item in pairwise):
        status = "concurrent_routes_separating_from_clear_start"
    else:
        status = "concurrent_corridors_clear"
    effective = list(
        guarded.decisions[0].coordination.active_robot_ids
        if guarded.decisions
        else ()
    )
    report: dict[str, object] = {
        "schema_version": ROUTE_CONFLICT_SCHEMA_VERSION,
        "status": status,
        "method": (
            "closed straight shared-frame start-to-target segment separation "
            "with a footprint-derived current-pose floor; robot-local detours "
            "are not certified"
        ),
        "classification": (
            "source-derived conservative execution guard from observed "
            "frozen shared-frame poses and source-derived VLM targets"
        ),
        "minimum_required_separation_m": minimum_separation_m,
        "minimum_current_separation_m": minimum_current_separation_m,
        "minimum_predicted_separation_m": minimum_observed,
        "original_active_robot_ids": active_ids,
        "effective_active_robot_ids": effective,
        "serialized_leader_robot_id": selected,
        "serialized_leader_priority_source": priority_source,
        "goal_evidence_by_robot": dict(sorted(normalized_evidence.items())),
        "suppressed_robot_ids": suppressed,
        "missing_shared_pose_robot_ids": missing,
        "routes": routes,
        "pairwise": pairwise,
        "source_fidelity": (
            "the VLM candidate and any upstream continuity override are "
            "preserved separately; this guard does not retarget a robot and "
            "current detector evidence only chooses which already-guarded "
            "robot moves first when physical concurrency authority must be "
            "reduced; a pair already above the footprint-derived current "
            "separation floor is not serialized when the complete pair of "
            "straight route segments never comes closer than that observed "
            "starting separation"
        ),
    }
    return guarded, report
