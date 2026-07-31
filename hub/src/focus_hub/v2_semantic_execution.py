"""Independent execution confirmation for source semantic goals.

The immutable source intentionally applies ``Find_Goal`` when any positive
target-category cell exists in the persistent semantic map.  That rule is
preserved in every frozen shadow manifest.  A real robot, however, must not
turn one small, stale model region into physical authority without evidence
from the current camera frame.

This module is an execution adapter only:

* it never changes the source semantic map or frozen VLM result;
* it requires both a non-speckle source-map component and corroboration from
  the independent current-frame detector; persistent component area alone is
  never execution authority because source-compatible max fusion can retain
  and spatially accumulate a stale false positive;
* an unconfirmed semantic override falls back to the source's already frozen
  exploration selection, or HOLD when no such selection exists; and
* all decisions and evidence remain explicit in a versioned report.

It performs no network or robot I/O.
"""
from __future__ import annotations

import math
from typing import Mapping, Sequence


SEMANTIC_EXECUTION_GUARD_SCHEMA_VERSION = (
    "focus-v2-semantic-execution-guard-v2"
)
DEFAULT_MINIMUM_COMPONENT_CELLS = 3
DEFAULT_MINIMUM_DETECTOR_CONFIDENCE = 0.50

# The detector vocabulary is COCO/YOLO while the source ObjectNav vocabulary
# follows HM3D.  These are direct category equivalents only.
GOAL_DETECTOR_CLASSES: dict[str, tuple[str, ...]] = {
    "chair": ("chair",),
    "sofa": ("couch",),
    "plant": ("potted plant",),
    "bed": ("bed",),
    "toilet": ("toilet",),
    "tv": ("tv",),
}


def _robot_results(
    manifest: Mapping[str, object],
) -> dict[str, Mapping[str, object]]:
    raw = manifest.get("robots")
    if (
        not isinstance(raw, Sequence)
        or isinstance(raw, (str, bytes))
    ):
        raise ValueError("semantic guard manifest robots are malformed")
    results: dict[str, Mapping[str, object]] = {}
    for item in raw:
        if not isinstance(item, Mapping):
            raise ValueError("semantic guard robot result is malformed")
        robot_id = item.get("robot_id")
        if (
            not isinstance(robot_id, str)
            or not robot_id
            or robot_id in results
        ):
            raise ValueError("semantic guard robot identity is malformed")
        results[robot_id] = item
    if not results:
        raise ValueError("semantic guard manifest contains no robots")
    return results


def _current_detector_confidence(
    detections: object,
    *,
    detector_classes: tuple[str, ...],
) -> tuple[float | None, dict[str, float]]:
    if not isinstance(detections, Mapping):
        raise ValueError("semantic guard detections are malformed")
    matches: dict[str, float] = {}
    for detector_class in detector_classes:
        value = detections.get(detector_class)
        if value is None:
            continue
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or not 0.0 <= float(value) <= 1.0
        ):
            raise ValueError(
                f"semantic guard detection {detector_class!r} is invalid"
            )
        matches[detector_class] = float(value)
    return (
        None if not matches else max(matches.values()),
        matches,
    )


def evaluate_semantic_execution_guard(
    manifest: Mapping[str, object],
    *,
    minimum_component_cells: int = DEFAULT_MINIMUM_COMPONENT_CELLS,
    minimum_detector_confidence: float = (
        DEFAULT_MINIMUM_DETECTOR_CONFIDENCE
    ),
) -> tuple[
    dict[str, Mapping[str, object] | None],
    dict[str, object],
]:
    """Return constrained execution overrides and their evidence report.

    The override mapping contains only robots whose source semantic goal was
    not independently confirmed.  Its value is the exact frozen exploration
    selection that preceded the source semantic override, or ``None`` for an
    explicit HOLD.  Callers must still validate that exact binding before
    constructing a command-capable batch.
    """

    if (
        isinstance(minimum_component_cells, bool)
        or not isinstance(minimum_component_cells, int)
        or minimum_component_cells < 1
    ):
        raise ValueError("minimum semantic component cells must be positive")
    if (
        not math.isfinite(minimum_detector_confidence)
        or not 0.0 < minimum_detector_confidence <= 1.0
    ):
        raise ValueError(
            "minimum semantic detector confidence must be in (0, 1]"
        )

    goal_category = manifest.get("goal_category")
    if (
        not isinstance(goal_category, str)
        or goal_category not in GOAL_DETECTOR_CLASSES
    ):
        raise ValueError(
            "semantic guard goal category lacks a direct detector mapping"
        )
    results = _robot_results(manifest)
    overrides: dict[str, Mapping[str, object] | None] = {}
    checks: dict[str, dict[str, object]] = {}
    confirmed: list[str] = []
    rejected: list[str] = []
    fallback: list[str] = []
    held: list[str] = []

    for robot_id, result in results.items():
        selection = result.get("final_shadow_selection")
        if selection is None:
            checks[robot_id] = {
                "status": "not_a_semantic_candidate",
                "source_selection_kind": None,
            }
            continue
        if not isinstance(selection, Mapping):
            raise ValueError(
                f"{robot_id} semantic guard selection is malformed"
            )
        if selection.get("kind") != "semantic_goal":
            checks[robot_id] = {
                "status": "not_a_semantic_candidate",
                "source_selection_kind": selection.get("kind"),
            }
            continue
        if selection.get("category") != goal_category:
            raise ValueError(
                f"{robot_id} semantic category differs from scene goal"
            )
        raw_size = selection.get("size_cells")
        if (
            isinstance(raw_size, bool)
            or not isinstance(raw_size, int)
            or raw_size < 1
        ):
            raise ValueError(
                f"{robot_id} semantic component size is malformed"
            )
        detector_confidence, matching_detections = (
            _current_detector_confidence(
                result.get("detections", {}),
                detector_classes=GOAL_DETECTOR_CLASSES[goal_category],
            )
        )
        component_pass = raw_size >= minimum_component_cells
        detector_pass = bool(
            detector_confidence is not None
            and detector_confidence
            >= minimum_detector_confidence - 1e-12
        )
        current_frame_consensus_pass = component_pass and detector_pass
        confirmed_for_execution = bool(current_frame_consensus_pass)
        confirmation_mode = (
            "source_component_with_current_frame_detector"
            if current_frame_consensus_pass
            else None
        )
        check: dict[str, object] = {
            "status": (
                "confirmed_for_execution"
                if confirmed_for_execution
                else "source_semantic_unconfirmed"
            ),
            "source_selection_kind": "semantic_goal",
            "goal_category": goal_category,
            "component_size_cells": raw_size,
            "minimum_component_cells": minimum_component_cells,
            "component_size_pass": component_pass,
            "component_size_is_diagnostic_only": True,
            "detector_classes": list(
                GOAL_DETECTOR_CLASSES[goal_category]
            ),
            "matching_current_frame_detections": matching_detections,
            "maximum_current_frame_detector_confidence": (
                detector_confidence
            ),
            "minimum_detector_confidence": (
                minimum_detector_confidence
            ),
            "current_frame_detector_pass": detector_pass,
            "current_frame_detector_consensus_pass": (
                current_frame_consensus_pass
            ),
            "confirmation_mode": confirmation_mode,
            "source_semantic_evidence_status": selection.get(
                "evidence_status"
            ),
            "frozen_observation_binding": {
                "source_sequence": result.get("source_sequence"),
                "source_capture_time_ns": result.get(
                    "source_capture_time_ns"
                ),
                "map_snapshot_sha256": result.get(
                    "map_snapshot_sha256"
                ),
                "classification": (
                    "same frozen RGB observation used by source Stage-1 "
                    "YOLO and the source VLM round"
                ),
            },
        }
        checks[robot_id] = check
        if confirmed_for_execution:
            confirmed.append(robot_id)
            continue

        rejected.append(robot_id)
        exploration = result.get(
            "exploration_selection_before_target_override"
        )
        if exploration is None:
            overrides[robot_id] = None
            held.append(robot_id)
            check.update(
                {
                    "execution_action": "HOLD",
                    "fallback_selection_kind": None,
                    "fallback_target_id": None,
                }
            )
            continue
        if (
            not isinstance(exploration, Mapping)
            or exploration.get("kind") not in {"frontier", "history"}
        ):
            raise ValueError(
                f"{robot_id} semantic fallback is not a frozen "
                "frontier/history selection"
            )
        overrides[robot_id] = exploration
        fallback.append(robot_id)
        check.update(
            {
                "execution_action": (
                    "use_frozen_exploration_selection"
                ),
                "fallback_selection_kind": exploration.get("kind"),
                "fallback_target_id": exploration.get("target_id"),
            }
        )

    report = {
        "schema_version": SEMANTIC_EXECUTION_GUARD_SCHEMA_VERSION,
        "status": (
            "unconfirmed_semantic_candidates_redirected"
            if rejected
            else "all_semantic_candidates_confirmed_or_absent"
        ),
        "goal_category": goal_category,
        "confirmed_robot_ids": sorted(confirmed),
        "rejected_robot_ids": sorted(rejected),
        "fallback_robot_ids": sorted(fallback),
        "held_robot_ids": sorted(held),
        "policy": {
            "minimum_component_cells": minimum_component_cells,
            "minimum_current_frame_detector_confidence": (
                minimum_detector_confidence
            ),
            "confirmation_rule": (
                "source component size >= minimum_component_cells AND "
                "same-frame detector confidence >= threshold; persistent "
                "component area alone is never execution authority"
            ),
            "unconfirmed_semantic_action": (
                "use the exact frozen exploration selection before the "
                "semantic override; HOLD if absent"
            ),
            "semantic_map_reinforcement": False,
        },
        "checks": checks,
        "classification": (
            "source-derived real-world execution guard over preserved "
            "model-inference evidence; no source result or semantic map "
            "was modified"
        ),
    }
    return overrides, report
