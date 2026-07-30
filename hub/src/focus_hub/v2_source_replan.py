"""Source-derived cross-round replanning for physical frontier execution.

The immutable source keeps a per-agent ``collision_map``.  A forward action
that produces less than ``collision_threshold`` movement marks cells in
front of that agent, and the next FMM query plans over the updated
traversible map.  A physical TinyNav leg already performs its own local
collision/replan budget before emitting an explicit path rejection, but the
Hub previously discarded that evidence at the next VLM boundary.

This module carries the equivalent *high-level* evidence across boundaries:

* raw VLM selections and score vectors remain untouched;
* an explicitly rejected physical approach is remembered by robot, scene,
  calibration, position and direction;
* the same nearby approach is not immediately republished;
* alternatives retain the source robot's score order (or source history
  score order) and still need the unchanged physical clearance guard; and
* memory ceases to apply after material robot relocation, allowing a
  different approach to the same frontier.

It performs no network or robot I/O.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
from typing import Mapping, Sequence

from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2


SOURCE_REPLAN_SCHEMA_VERSION = "focus-v2-source-replan-v2"
SOURCE_COLLISION_THRESHOLD_M = 0.10
SOURCE_STAGNANT_REPLAN_CELLS = 2.5
SOURCE_MAP_RESOLUTION_M = 0.05
SOURCE_STAGNANT_REPLAN_M = (
    SOURCE_STAGNANT_REPLAN_CELLS * SOURCE_MAP_RESOLUTION_M
)
SOURCE_FRONTIER_ARRIVAL_RADIUS_M = 10 * SOURCE_MAP_RESOLUTION_M

DEFAULT_TARGET_MATCH_RADIUS_M = 0.75
DEFAULT_ORIGIN_MATCH_RADIUS_M = 1.25
DEFAULT_SAME_SECTOR_ORIGIN_RADIUS_M = 0.75
DEFAULT_SAME_SECTOR_ANGLE_DEG = 20.0
DEFAULT_MAX_ENTRIES_PER_ROBOT = 16
DEFAULT_BACKTRACK_DIRECTION_UPDATE_M = 0.75
DEFAULT_BACKTRACK_MIN_TARGET_DISTANCE_M = 2.0
DEFAULT_BACKTRACK_ANGLE_DEG = 90.0

SPATIAL_FRONTIER_FAILURE_REASONS = frozenset(
    {
        "LOCAL_GOAL_UNREACHABLE",
        "LOCAL_PATH_REVERSE_REQUIRED",
        "LOCAL_PLANNER_TURN_STALLED",
        "LOCAL_PLANNER_NO_PROGRESS",
        "LOCAL_PLANNER_PATH_STALE",
        "UNREACHABLE",
        "CROSS_ROUND_SOURCE_STALL",
    }
)

Point2 = tuple[float, float]


def _finite_xy(value: object, *, field_name: str) -> Point2:
    if (
        not isinstance(value, (tuple, list))
        or len(value) != 2
        or any(
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(float(item))
            for item in value
        )
    ):
        raise ValueError(f"{field_name} must contain two finite numbers")
    return (float(value[0]), float(value[1]))


def _frontier_xy(decision: HighLevelDecisionV2) -> Point2 | None:
    target = decision.target
    if target is None or target.kind != "FRONTIER_POINT":
        return None
    return (float(target.pose.x), float(target.pose.y))


def _bearing_deg(origin: Point2, target: Point2) -> float | None:
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    if math.hypot(dx, dy) < SOURCE_COLLISION_THRESHOLD_M:
        return None
    return math.degrees(math.atan2(dy, dx))


def _angle_difference_deg(left: float, right: float) -> float:
    return abs((left - right + 180.0) % 360.0 - 180.0)


def _backtrack_check(
    *,
    robot_xy: Point2,
    target_xy: Point2,
    progress_vector: Point2 | None,
    minimum_target_distance_m: float,
    backtrack_angle_deg: float,
) -> dict[str, object]:
    target_vector = (
        target_xy[0] - robot_xy[0],
        target_xy[1] - robot_xy[1],
    )
    target_distance_m = math.hypot(*target_vector)
    if progress_vector is None:
        return {
            "severe_backtrack": False,
            "reason": "no_observed_progress_direction",
            "target_distance_m": target_distance_m,
            "angle_deg": None,
        }
    progress_norm = math.hypot(*progress_vector)
    if progress_norm <= 1e-12:
        return {
            "severe_backtrack": False,
            "reason": "zero_observed_progress_direction",
            "target_distance_m": target_distance_m,
            "angle_deg": None,
        }
    if target_distance_m <= 1e-12:
        return {
            "severe_backtrack": False,
            "reason": "target_at_current_position",
            "target_distance_m": target_distance_m,
            "angle_deg": None,
        }
    cosine = (
        progress_vector[0] * target_vector[0]
        + progress_vector[1] * target_vector[1]
    ) / (progress_norm * target_distance_m)
    angle_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, cosine)))
    )
    severe = bool(
        target_distance_m
        >= minimum_target_distance_m - 1e-12
        and angle_deg >= backtrack_angle_deg - 1e-12
    )
    return {
        "severe_backtrack": severe,
        "reason": (
            "long_target_reverses_observed_progress"
            if severe
            else "target_within_direction_guard"
        ),
        "target_distance_m": target_distance_m,
        "angle_deg": angle_deg,
    }


def _event_summary(event: Mapping[str, object] | None) -> dict[str, object]:
    if event is None:
        return {}
    summary: dict[str, object] = {}
    for key in (
        "event_id",
        "status",
        "reason_code",
        "decision_id",
        "leg_id",
        "lease_sequence",
        "event_time_ns",
        "observed_at_ns",
    ):
        value = event.get(key)
        if isinstance(value, (str, int, float, bool)) or value is None:
            summary[key] = value
    return summary


@dataclass
class NavigationFailureMemory:
    """Bounded robot-local memory of rejected physical frontier approaches."""

    scene_id: str
    episode_id: str
    shared_frame_calibration_id: str
    target_match_radius_m: float = DEFAULT_TARGET_MATCH_RADIUS_M
    origin_match_radius_m: float = DEFAULT_ORIGIN_MATCH_RADIUS_M
    same_sector_origin_radius_m: float = (
        DEFAULT_SAME_SECTOR_ORIGIN_RADIUS_M
    )
    same_sector_angle_deg: float = DEFAULT_SAME_SECTOR_ANGLE_DEG
    max_entries_per_robot: int = DEFAULT_MAX_ENTRIES_PER_ROBOT
    entries: list[dict[str, object]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.scene_id or not self.episode_id:
            raise ValueError("failure memory scene/episode identity is empty")
        if not self.shared_frame_calibration_id:
            raise ValueError("failure memory calibration identity is empty")
        for value, name in (
            (self.target_match_radius_m, "target match radius"),
            (self.origin_match_radius_m, "origin match radius"),
            (
                self.same_sector_origin_radius_m,
                "same-sector origin radius",
            ),
            (self.same_sector_angle_deg, "same-sector angle"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.same_sector_origin_radius_m > self.origin_match_radius_m:
            raise ValueError(
                "same-sector origin radius cannot exceed origin match radius"
            )
        if self.same_sector_angle_deg > 90.0:
            raise ValueError("same-sector angle cannot exceed 90 degrees")
        if self.max_entries_per_robot < 1:
            raise ValueError("failure-memory capacity must be positive")

    def _validate_decision_identity(
        self, decision: HighLevelDecisionV2
    ) -> None:
        if (
            decision.scene_id != self.scene_id
            or decision.episode_id != self.episode_id
            or decision.map_provenance.shared_frame_calibration_id
            != self.shared_frame_calibration_id
        ):
            raise ValueError(
                "failure decision identity differs from memory identity"
            )

    def matching_entries(
        self,
        *,
        robot_id: str,
        target_xy_m: object,
        robot_xy_m: object,
    ) -> list[dict[str, object]]:
        """Return active spatial matches with an explicit match explanation."""

        target = _finite_xy(target_xy_m, field_name="candidate target")
        origin = _finite_xy(robot_xy_m, field_name="current robot position")
        matches: list[dict[str, object]] = []
        current_bearing = _bearing_deg(origin, target)
        for entry in self.entries:
            if entry.get("robot_id") != robot_id:
                continue
            failed_target = _finite_xy(
                entry.get("source_target_xy_m"),
                field_name="remembered source target",
            )
            failed_origin = _finite_xy(
                entry.get("failure_robot_xy_m"),
                field_name="remembered failure origin",
            )
            origin_distance = math.dist(origin, failed_origin)
            if origin_distance > self.origin_match_radius_m + 1e-12:
                continue
            target_distance = math.dist(target, failed_target)
            match_kind: str | None = None
            angle_difference: float | None = None
            if target_distance <= self.target_match_radius_m + 1e-12:
                match_kind = "near_same_source_target"
            elif (
                origin_distance
                <= self.same_sector_origin_radius_m + 1e-12
                and current_bearing is not None
            ):
                raw_heading = entry.get("failure_heading_deg")
                failed_bearing = (
                    float(raw_heading)
                    if (
                        isinstance(raw_heading, (int, float))
                        and not isinstance(raw_heading, bool)
                        and math.isfinite(float(raw_heading))
                    )
                    else _bearing_deg(failed_origin, failed_target)
                )
                if failed_bearing is not None:
                    angle_difference = _angle_difference_deg(
                        current_bearing,
                        failed_bearing,
                    )
                    if (
                        angle_difference
                        <= self.same_sector_angle_deg + 1e-12
                    ):
                        match_kind = "same_blocked_approach_sector"
            if match_kind is None:
                continue
            matches.append(
                {
                    "entry_id": entry["entry_id"],
                    "reason_codes": entry["reason_codes"],
                    "occurrence_count": entry["occurrence_count"],
                    "match_kind": match_kind,
                    "target_distance_m": target_distance,
                    "origin_distance_m": origin_distance,
                    "angle_difference_deg": angle_difference,
                    "blocked_bearing_source": (
                        "observed_failure_base_heading"
                        if entry.get("failure_heading_deg") is not None
                        else "failure_origin_to_source_target_bearing"
                    ),
                }
            )
        return matches

    def record_frontier_failure(
        self,
        decision: HighLevelDecisionV2,
        *,
        reason_code: str,
        failure_robot_xy_m: object,
        recorded_at_ns: int,
        source_target_xy_m: object | None = None,
        event: Mapping[str, object] | None = None,
        failure_heading_deg: float | None = None,
        pose_classification: str = (
            "source_derived_frozen_round_start_pose_proxy"
        ),
    ) -> dict[str, object]:
        """Record one explicit failure, merging a repeated nearby approach."""

        self._validate_decision_identity(decision)
        if reason_code not in SPATIAL_FRONTIER_FAILURE_REASONS:
            return {
                "status": "ignored_non_spatial_failure",
                "robot_id": decision.robot_id,
                "reason_code": reason_code,
            }
        execution_target = _frontier_xy(decision)
        if execution_target is None:
            return {
                "status": "ignored_non_frontier_failure",
                "robot_id": decision.robot_id,
                "reason_code": reason_code,
            }
        if isinstance(recorded_at_ns, bool) or recorded_at_ns <= 0:
            raise ValueError("failure record time must be positive")
        failure_origin = _finite_xy(
            failure_robot_xy_m,
            field_name="failure robot position",
        )
        source_target = (
            execution_target
            if source_target_xy_m is None
            else _finite_xy(
                source_target_xy_m,
                field_name="source target",
            )
        )
        if failure_heading_deg is not None and (
            isinstance(failure_heading_deg, bool)
            or not isinstance(failure_heading_deg, (int, float))
            or not math.isfinite(float(failure_heading_deg))
        ):
            raise ValueError("failure heading must be finite when supplied")
        normalized_heading = (
            None
            if failure_heading_deg is None
            else (
                (float(failure_heading_deg) + 180.0) % 360.0
                - 180.0
            )
        )
        evidence_classification = (
            "source_derived_from_observed_shared_boundary_poses"
            if reason_code == "CROSS_ROUND_SOURCE_STALL"
            else "observed_robot_local_rejection_event"
        )

        existing: dict[str, object] | None = None
        for entry in reversed(self.entries):
            if entry.get("robot_id") != decision.robot_id:
                continue
            entry_target = _finite_xy(
                entry.get("source_target_xy_m"),
                field_name="remembered source target",
            )
            entry_origin = _finite_xy(
                entry.get("failure_robot_xy_m"),
                field_name="remembered failure origin",
            )
            if (
                math.dist(source_target, entry_target)
                <= self.target_match_radius_m + 1e-12
                and math.dist(failure_origin, entry_origin)
                <= self.origin_match_radius_m + 1e-12
            ):
                existing = entry
                break

        if existing is not None:
            reasons = list(existing["reason_codes"])
            if reason_code not in reasons:
                reasons.append(reason_code)
            evidence_classifications = list(
                existing.get("evidence_classifications", [])
            )
            if evidence_classification not in evidence_classifications:
                evidence_classifications.append(evidence_classification)
            existing.update(
                {
                    "last_round_index": decision.round_index,
                    "last_source_step": decision.source_step,
                    "last_recorded_at_ns": recorded_at_ns,
                    "frontier_id": decision.target.frontier_id,
                    "source_target_xy_m": list(source_target),
                    "execution_target_xy_m": list(execution_target),
                    "failure_robot_xy_m": list(failure_origin),
                    "failure_heading_deg": (
                        normalized_heading
                        if normalized_heading is not None
                        else existing.get("failure_heading_deg")
                    ),
                    "reason_codes": reasons,
                    "evidence_classifications": evidence_classifications,
                    "occurrence_count": int(
                        existing["occurrence_count"]
                    )
                    + 1,
                    "last_event": _event_summary(event),
                    "pose_classification": pose_classification,
                }
            )
            return {
                "status": "merged_repeated_failure",
                "robot_id": decision.robot_id,
                "entry_id": existing["entry_id"],
                "occurrence_count": existing["occurrence_count"],
            }

        identity = (
            f"{self.scene_id}|{self.episode_id}|{decision.robot_id}|"
            f"{decision.round_index}|{decision.leg_id}|{recorded_at_ns}"
        )
        entry_id = "navfail-" + hashlib.sha256(
            identity.encode("utf-8")
        ).hexdigest()[:16]
        entry = {
            "entry_id": entry_id,
            "robot_id": decision.robot_id,
            "scene_id": self.scene_id,
            "episode_id": self.episode_id,
            "shared_frame_calibration_id": (
                self.shared_frame_calibration_id
            ),
            "first_round_index": decision.round_index,
            "last_round_index": decision.round_index,
            "first_source_step": decision.source_step,
            "last_source_step": decision.source_step,
            "first_recorded_at_ns": recorded_at_ns,
            "last_recorded_at_ns": recorded_at_ns,
            "frontier_id": decision.target.frontier_id,
            "source_target_xy_m": list(source_target),
            "execution_target_xy_m": list(execution_target),
            "failure_robot_xy_m": list(failure_origin),
            "failure_heading_deg": normalized_heading,
            "reason_codes": [reason_code],
            "evidence_classifications": [evidence_classification],
            "occurrence_count": 1,
            "last_event": _event_summary(event),
            "pose_classification": pose_classification,
        }
        self.entries.append(entry)
        robot_entries = [
            item
            for item in self.entries
            if item.get("robot_id") == decision.robot_id
        ]
        while len(robot_entries) > self.max_entries_per_robot:
            oldest = robot_entries.pop(0)
            self.entries.remove(oldest)
        return {
            "status": "recorded_new_failure",
            "robot_id": decision.robot_id,
            "entry_id": entry_id,
            "occurrence_count": 1,
        }

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": SOURCE_REPLAN_SCHEMA_VERSION,
            "scene_id": self.scene_id,
            "episode_id": self.episode_id,
            "shared_frame_calibration_id": (
                self.shared_frame_calibration_id
            ),
            "source_contract": {
                "collision_threshold_m": SOURCE_COLLISION_THRESHOLD_M,
                "stationary_replan_cells": SOURCE_STAGNANT_REPLAN_CELLS,
                "source_map_resolution_m": SOURCE_MAP_RESOLUTION_M,
                "stationary_replan_m": SOURCE_STAGNANT_REPLAN_M,
                "source_paths": [
                    "source/Focus_realworld/agents/vlm_agents.py",
                    "source/Focus_realworld/main.py",
                ],
            },
            "policy": {
                "target_match_radius_m": self.target_match_radius_m,
                "origin_match_radius_m": self.origin_match_radius_m,
                "same_sector_origin_radius_m": (
                    self.same_sector_origin_radius_m
                ),
                "same_sector_angle_deg": self.same_sector_angle_deg,
                "max_entries_per_robot": self.max_entries_per_robot,
                "scope": (
                    "robot-local within one scene, episode and shared-frame "
                    "calibration; a materially relocated robot may approach "
                    "the same source target from a new side"
                ),
            },
            "entries": [dict(entry) for entry in self.entries],
            "classification": (
                "mixed provenance: observed robot-local rejection events "
                "and source-derived stationary-boundary evidence; inspect "
                "each entry's evidence_classifications, reason_codes and "
                "pose_classification"
            ),
        }


def _candidate_xy(candidate: Mapping[str, object]) -> Point2:
    return _finite_xy(
        (candidate.get("x_m"), candidate.get("y_m")),
        field_name="source fallback frontier",
    )


def _ranked_robot_candidates(
    robot_result: Mapping[str, object],
) -> tuple[list[dict[str, object]], str]:
    selection = robot_result.get("final_shadow_selection")
    if not isinstance(selection, Mapping):
        return [], "source_robot_has_no_selection"
    kind = selection.get("kind")
    if kind == "semantic_goal":
        return [], "semantic_goal_has_no_exploration_fallback"
    if kind == "frontier":
        raw_candidates = robot_result.get("candidate_frontiers")
        raw_scores = robot_result.get("choice_probabilities")
        if (
            not isinstance(raw_candidates, Sequence)
            or isinstance(raw_candidates, (str, bytes))
            or not isinstance(raw_scores, Mapping)
        ):
            raise ValueError("frontier fallback provenance is malformed")
        candidates: list[dict[str, object]] = []
        for source_order, raw in enumerate(raw_candidates):
            if not isinstance(raw, Mapping):
                raise ValueError("source frontier candidate is malformed")
            frontier_id = raw.get("frontier_id")
            if not isinstance(frontier_id, str) or not frontier_id:
                raise ValueError("source frontier candidate ID is malformed")
            x_m, y_m = _candidate_xy(raw)
            score = raw_scores.get(frontier_id)
            if (
                isinstance(score, bool)
                or not isinstance(score, (int, float))
                or not math.isfinite(float(score))
                or float(score) < 0.0
            ):
                raise ValueError("source frontier score is malformed")
            candidates.append(
                {
                    "frontier_id": frontier_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "source_probability": float(score),
                    "source_candidate_kind": "frontier",
                    "source_order": source_order,
                }
            )
        candidates.sort(
            key=lambda item: (
                -float(item["source_probability"]),
                int(item["source_order"]),
            )
        )
        return candidates, "source_vlm_probability_descending"
    if kind == "history":
        raw_candidates = robot_result.get("candidate_history_nodes")
        if (
            not isinstance(raw_candidates, Sequence)
            or isinstance(raw_candidates, (str, bytes))
        ):
            return [], "legacy_manifest_lacks_history_candidates"
        candidates = []
        for source_order, raw in enumerate(raw_candidates):
            if not isinstance(raw, Mapping):
                raise ValueError("source history candidate is malformed")
            frontier_id = raw.get("frontier_id")
            score = raw.get("history_score")
            if (
                not isinstance(frontier_id, str)
                or not frontier_id
                or isinstance(score, bool)
                or not isinstance(score, (int, float))
                or not math.isfinite(float(score))
            ):
                raise ValueError("source history candidate identity is malformed")
            x_m, y_m = _candidate_xy(raw)
            candidates.append(
                {
                    "frontier_id": frontier_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "source_probability": None,
                    "history_score": float(score),
                    "source_candidate_kind": "history",
                    "source_order": source_order,
                }
            )
        candidates.sort(
            key=lambda item: (
                -float(item["history_score"]),
                int(item["source_order"]),
            )
        )
        return candidates, "source_history_score_descending_first_max"
    raise ValueError(f"unsupported source selection kind: {kind!r}")


def _ranked_history_candidates(
    robot_result: Mapping[str, object],
) -> list[dict[str, object]]:
    """Return the source's own history candidates in its preserved order."""

    raw_candidates = robot_result.get("candidate_history_nodes")
    if (
        not isinstance(raw_candidates, Sequence)
        or isinstance(raw_candidates, (str, bytes))
    ):
        return []
    candidates: list[dict[str, object]] = []
    for source_order, raw in enumerate(raw_candidates):
        if not isinstance(raw, Mapping):
            raise ValueError("source history candidate is malformed")
        frontier_id = raw.get("frontier_id")
        score = raw.get("history_score")
        if (
            not isinstance(frontier_id, str)
            or not frontier_id
            or isinstance(score, bool)
            or not isinstance(score, (int, float))
            or not math.isfinite(float(score))
        ):
            raise ValueError("source history candidate identity is malformed")
        x_m, y_m = _candidate_xy(raw)
        candidates.append(
            {
                "frontier_id": frontier_id,
                "x_m": x_m,
                "y_m": y_m,
                "source_probability": None,
                "history_score": float(score),
                "source_candidate_kind": "history",
                "source_order": source_order,
            }
        )
    candidates.sort(
        key=lambda item: (
            -float(item["history_score"]),
            int(item["source_order"]),
        )
    )
    return candidates


def evaluate_source_replan(
    batch: DecisionBatchV2,
    *,
    shadow_manifest: Mapping[str, object],
    memory: NavigationFailureMemory,
    robot_xy_by_robot: Mapping[str, object],
    source_stationary_robot_ids: frozenset[str] = frozenset(),
    progress_vector_by_robot: Mapping[str, object] | None = None,
    backtrack_min_target_distance_m: float = (
        DEFAULT_BACKTRACK_MIN_TARGET_DISTANCE_M
    ),
    backtrack_angle_deg: float = DEFAULT_BACKTRACK_ANGLE_DEG,
) -> tuple[
    frozenset[str],
    dict[str, list[dict[str, object]]],
    dict[str, object],
]:
    """Apply failure memory and build per-robot source-ranked alternatives.

    The first return value tells the physical clearance guard which current
    frontier inputs must be treated as rejected.  The second contains
    unoccupied, memory-clear alternatives in each robot's own source score
    order.  A long frontier in the rear hemisphere of the last observed
    progress direction is redirected only when the same frozen source batch
    contains a non-reversing alternative; necessary backtracking therefore
    remains possible.  No target is published here.
    """

    decisions = {item.robot_id: item for item in batch.decisions}
    active = set(
        batch.decisions[0].coordination.active_robot_ids
        if batch.decisions
        else ()
    )
    if not source_stationary_robot_ids.issubset(active):
        raise ValueError(
            "source-stationary robot is outside the active batch"
        )
    if set(decisions) != set(robot_xy_by_robot):
        raise ValueError("source replan requires one position per robot")
    if progress_vector_by_robot is None:
        progress_vector_by_robot = {}
    unknown_progress = set(progress_vector_by_robot).difference(decisions)
    if unknown_progress:
        raise ValueError(
            "progress direction contains robots outside the decision batch"
        )
    if (
        not math.isfinite(backtrack_min_target_distance_m)
        or backtrack_min_target_distance_m <= 0.0
        or not math.isfinite(backtrack_angle_deg)
        or not 90.0 <= backtrack_angle_deg <= 180.0
    ):
        raise ValueError("backtrack guard thresholds are invalid")
    for decision in batch.decisions:
        memory._validate_decision_identity(decision)

    raw_results = shadow_manifest.get("robots")
    if (
        not isinstance(raw_results, Sequence)
        or isinstance(raw_results, (str, bytes))
    ):
        raise ValueError("shadow manifest robot results are malformed")
    results: dict[str, Mapping[str, object]] = {}
    for raw in raw_results:
        if not isinstance(raw, Mapping):
            raise ValueError("shadow manifest robot result is malformed")
        robot_id = raw.get("robot_id")
        if not isinstance(robot_id, str) or robot_id in results:
            raise ValueError("shadow manifest robot IDs are malformed")
        results[robot_id] = raw
    if set(results) != set(decisions):
        raise ValueError("shadow result IDs differ from the decision batch")

    occupied_ids: set[str] = set()
    occupied_points: dict[str, Point2] = {}
    for robot_id in active:
        decision = decisions[robot_id]
        point = _frontier_xy(decision)
        if point is None:
            continue
        occupied_ids.add(decision.target.frontier_id)
        occupied_points[robot_id] = point

    rejected: set[str] = set()
    fallback_by_robot: dict[str, list[dict[str, object]]] = {}
    checks: dict[str, dict[str, object]] = {}
    for robot_id, decision in decisions.items():
        robot_xy = _finite_xy(
            robot_xy_by_robot[robot_id],
            field_name=f"{robot_id} position",
        )
        raw_progress_vector = progress_vector_by_robot.get(robot_id)
        progress_vector = (
            None
            if raw_progress_vector is None
            else _finite_xy(
                raw_progress_vector,
                field_name=f"{robot_id} observed progress vector",
            )
        )
        target_xy = _frontier_xy(decision)
        current_backtrack = (
            {
                "severe_backtrack": False,
                "reason": "inactive_or_non_frontier_target",
                "target_distance_m": None,
                "angle_deg": None,
            }
            if robot_id not in active or target_xy is None
            else _backtrack_check(
                robot_xy=robot_xy,
                target_xy=target_xy,
                progress_vector=progress_vector,
                minimum_target_distance_m=(
                    backtrack_min_target_distance_m
                ),
                backtrack_angle_deg=backtrack_angle_deg,
            )
        )
        current_matches = (
            []
            if robot_id not in active or target_xy is None
            else memory.matching_entries(
                robot_id=robot_id,
                target_xy_m=target_xy,
                robot_xy_m=robot_xy,
            )
        )
        source_stationary_replan = bool(
            robot_id in source_stationary_robot_ids
            and target_xy is not None
        )
        current_frontier_arrival_already_satisfied = bool(
            robot_id in active
            and target_xy is not None
            and math.dist(robot_xy, target_xy)
            <= SOURCE_FRONTIER_ARRIVAL_RADIUS_M + 1e-12
        )
        if (
            current_matches
            or source_stationary_replan
            or current_frontier_arrival_already_satisfied
        ):
            rejected.add(robot_id)

        ranked, ranking_source = _ranked_robot_candidates(
            results[robot_id]
        )
        if (
            current_frontier_arrival_already_satisfied
            and isinstance(
                results[robot_id].get("final_shadow_selection"), Mapping
            )
            and results[robot_id]["final_shadow_selection"].get("kind")
            == "frontier"
        ):
            # Frontier labels A-D are regenerated every source round.  The
            # same physical boundary can therefore return under a new label
            # after the robot has already entered its source 10-cell arrival
            # disk.  Compare in shared XY, then expose only the source's own
            # frozen history nodes as last-resort exploration alternatives.
            # The downstream robot-local connectivity and footprint guard
            # must still approve any such history target before publication.
            existing_ids = {
                str(candidate["frontier_id"]) for candidate in ranked
            }
            history = [
                candidate
                for candidate in _ranked_history_candidates(results[robot_id])
                if str(candidate["frontier_id"]) not in existing_ids
            ]
            if history:
                ranked.extend(history)
                ranking_source = (
                    f"{ranking_source}_then_source_history_score_descending"
                )
        accepted_candidates: list[dict[str, object]] = []
        excluded: list[dict[str, object]] = []
        for source_rank, candidate in enumerate(ranked):
            frontier_id = str(candidate["frontier_id"])
            point = (float(candidate["x_m"]), float(candidate["y_m"]))
            reason: str | None = None
            memory_matches: list[dict[str, object]] = []
            if frontier_id in occupied_ids:
                reason = "already_assigned_in_guard_input"
            elif any(
                other_id != robot_id
                and math.dist(point, other_point)
                <= SOURCE_MAP_RESOLUTION_M + 1e-12
                for other_id, other_point in occupied_points.items()
            ):
                reason = "duplicates_peer_guard_input_coordinate"
            else:
                memory_matches = memory.matching_entries(
                    robot_id=robot_id,
                    target_xy_m=point,
                    robot_xy_m=robot_xy,
                )
                if memory_matches:
                    reason = "navigation_failure_memory_match"
            record = dict(candidate)
            record["source_rank"] = source_rank
            record["backtrack_check"] = _backtrack_check(
                robot_xy=robot_xy,
                target_xy=point,
                progress_vector=progress_vector,
                minimum_target_distance_m=(
                    backtrack_min_target_distance_m
                ),
                backtrack_angle_deg=backtrack_angle_deg,
            )
            if reason is None:
                accepted_candidates.append(record)
            else:
                excluded.append(
                    {
                        **record,
                        "excluded_reason": reason,
                        "memory_matches": memory_matches,
                    }
                )
        non_backtracking_candidates = [
            item
            for item in accepted_candidates
            if not bool(item["backtrack_check"]["severe_backtrack"])
        ]
        backtracking_candidates = [
            item
            for item in accepted_candidates
            if bool(item["backtrack_check"]["severe_backtrack"])
        ]
        backtrack_redirected = bool(
            robot_id in active
            and current_backtrack["severe_backtrack"]
            and non_backtracking_candidates
        )
        if backtrack_redirected:
            # Preserve source score order within each group.  A non-reversing
            # source candidate is attempted first; reverse candidates remain
            # available only if the physical clearance guard rejects every
            # non-reversing option, so exploration completeness is retained.
            accepted_candidates = (
                non_backtracking_candidates + backtracking_candidates
            )
            rejected.add(robot_id)
        # A no-allocation/HOLD robot is intentionally absent from the
        # coordination active set.  Preserve its source result in ``checks``
        # below, but do not offer physical fallback targets for it to the
        # downstream clearance guard.
        if robot_id in active:
            fallback_by_robot[robot_id] = accepted_candidates
        checks[robot_id] = {
            "robot_id": robot_id,
            "active": robot_id in active,
            "current_target_kind": (
                None if decision.target is None else decision.target.kind
            ),
            "current_frontier_id": (
                None
                if decision.target is None
                or decision.target.kind != "FRONTIER_POINT"
                else decision.target.frontier_id
            ),
            "current_target_xy_m": (
                None if target_xy is None else list(target_xy)
            ),
            "current_target_rejected": bool(
                current_matches
                or source_stationary_replan
                or backtrack_redirected
                or current_frontier_arrival_already_satisfied
            ),
            "current_frontier_arrival_already_satisfied": (
                current_frontier_arrival_already_satisfied
            ),
            "source_frontier_arrival_radius_m": (
                SOURCE_FRONTIER_ARRIVAL_RADIUS_M
            ),
            "source_stationary_replan": source_stationary_replan,
            "current_memory_matches": current_matches,
            "observed_progress_vector_xy_m": (
                None if progress_vector is None else list(progress_vector)
            ),
            "current_backtrack_check": current_backtrack,
            "backtrack_redirected": backtrack_redirected,
            "non_backtracking_fallback_count": len(
                non_backtracking_candidates
            ),
            "backtracking_fallback_count": len(backtracking_candidates),
            "fallback_ranking_source": ranking_source,
            "accepted_fallback_candidates": accepted_candidates,
            "excluded_fallback_candidates": excluded,
        }

    report = {
        "schema_version": SOURCE_REPLAN_SCHEMA_VERSION,
        "status": (
            "execution_candidates_redirected"
            if rejected
            else "source_candidates_accepted"
        ),
        "classification": (
            "source-derived real-world execution adapter over an unchanged "
            "frozen VLM candidate batch"
        ),
        "current_rejected_robot_ids": sorted(rejected),
        "source_stationary_robot_ids": sorted(
            source_stationary_robot_ids
        ),
        "backtrack_policy": {
            "minimum_target_distance_m": (
                backtrack_min_target_distance_m
            ),
            "minimum_reversal_angle_deg": backtrack_angle_deg,
            "fallback_policy": (
                "redirect only when a non-reversing source-ranked candidate "
                "exists; retain reverse candidates after non-reversing "
                "candidates for clearance fallback"
            ),
        },
        "checks": checks,
        "memory_snapshot": memory.to_dict(),
        "source_fidelity": (
            "raw VLM selections and score vectors remain in the shadow and "
            "candidate artifacts; only physical execution is redirected "
            "after observed local path failure, the source 2.5-cell "
            "stationary rule, or spatially entering the source 10-cell "
            "frontier arrival disk even when A-D labels are regenerated; "
            "a long target "
            "in the prior-progress rear hemisphere is grouped behind "
            "non-reversing candidates while source score order is preserved "
            "inside each group "
            "only when the same frozen source batch provides one, while "
            "necessary backtracking remains available"
        ),
    }
    return frozenset(rejected), fallback_by_robot, report


def source_target_from_clearance_lineage(
    clearance_report: Mapping[str, object],
    decision: HighLevelDecisionV2,
) -> Point2 | None:
    """Resolve the pre-projection source coordinate for failure provenance."""

    lineage_raw = clearance_report.get("execution_lineage")
    if not isinstance(lineage_raw, Mapping):
        return _frontier_xy(decision)
    lineage = lineage_raw.get(decision.robot_id)
    if not isinstance(lineage, Mapping):
        return _frontier_xy(decision)
    execution_frontier_id = lineage.get("execution_frontier_id")
    target = decision.target
    if (
        target is None
        or target.kind != "FRONTIER_POINT"
        or execution_frontier_id != target.frontier_id
    ):
        return _frontier_xy(decision)
    raw = lineage.get("source_target_xy_m")
    try:
        return _finite_xy(raw, field_name="clearance source target")
    except ValueError:
        return _frontier_xy(decision)
