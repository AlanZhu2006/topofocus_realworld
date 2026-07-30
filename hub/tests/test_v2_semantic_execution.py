from __future__ import annotations

import pytest

from focus_hub.v2_semantic_execution import (
    evaluate_semantic_execution_guard,
)


def semantic_selection(*, size_cells: int) -> dict[str, object]:
    return {
        "kind": "semantic_goal",
        "target_id": "target-plant",
        "category": "plant",
        "size_cells": size_cells,
        "evidence_status": "model_inference_map_projected_unverified",
    }


def frontier_selection(target_id: str = "D") -> dict[str, object]:
    return {
        "kind": "frontier",
        "target_id": target_id,
        "frontier_id": target_id,
        "x_m": 4.68,
        "y_m": 10.76,
    }


def manifest(
    *,
    size_cells: int,
    detections: dict[str, float],
    exploration: dict[str, object] | None = None,
) -> dict[str, object]:
    semantic = semantic_selection(size_cells=size_cells)
    return {
        "goal_category": "plant",
        "robots": [
            {
                "robot_id": "robot-0",
                "detections": detections,
                "exploration_selection_before_target_override": (
                    frontier_selection() if exploration is None else exploration
                ),
                "final_shadow_selection": semantic,
            },
            {
                "robot_id": "robot-1",
                "detections": {"chair": 0.8},
                "exploration_selection_before_target_override": (
                    frontier_selection("B")
                ),
                "final_shadow_selection": frontier_selection("B"),
            },
        ],
    }


@pytest.mark.parametrize("size_cells", [2, 7])
def test_archived_robot0_false_regions_fall_back_without_detector_agreement(
    size_cells,
):
    # Scene 03 observed two Robot 0 false candidates: a two-cell region and a
    # seven-cell region.  Both frozen current frames contained no independent
    # potted-plant detection.
    payload = manifest(size_cells=size_cells, detections={})

    overrides, report = evaluate_semantic_execution_guard(payload)

    assert overrides["robot-0"] == frontier_selection()
    check = report["checks"]["robot-0"]
    assert check["status"] == "source_semantic_unconfirmed"
    assert check["current_frame_detector_pass"] is False
    assert check["execution_action"] == (
        "use_frozen_exploration_selection"
    )
    assert report["rejected_robot_ids"] == ["robot-0"]
    assert report["confirmed_robot_ids"] == []


def test_archived_successful_plant_consensus_keeps_source_semantic_goal():
    # The final successful run recorded a five-cell source semantic component
    # and potted-plant confidence 0.8516193628311157 in the same frozen frame.
    payload = manifest(
        size_cells=5,
        detections={"potted plant": 0.8516193628311157},
    )

    overrides, report = evaluate_semantic_execution_guard(payload)

    assert overrides == {}
    assert report["confirmed_robot_ids"] == ["robot-0"]
    check = report["checks"]["robot-0"]
    assert check["status"] == "confirmed_for_execution"
    assert check["component_size_pass"] is True
    assert check["current_frame_detector_pass"] is True
    assert check["confirmation_mode"] == (
        "compact_component_with_current_frame_detector"
    )


def test_archived_large_source_component_does_not_require_yolo_override():
    # Scene 03 Formal 03 reached the real plant from a 176-cell source
    # component even though the exact frozen YOLO frame did not detect it.
    # Keep the source semantic result authoritative when its spatial support
    # is already strong; YOLO is only the compact-component corroborator.
    payload = manifest(size_cells=176, detections={"chair": 0.3145})

    overrides, report = evaluate_semantic_execution_guard(payload)

    assert overrides == {}
    check = report["checks"]["robot-0"]
    assert check["strong_component_pass"] is True
    assert check["current_frame_detector_pass"] is False
    assert check["confirmation_mode"] == "strong_source_component"


def test_semantic_speckle_is_rejected_even_with_detector_agreement():
    payload = manifest(
        size_cells=2,
        detections={"potted plant": 0.9},
    )

    overrides, report = evaluate_semantic_execution_guard(payload)

    assert "robot-0" in overrides
    check = report["checks"]["robot-0"]
    assert check["component_size_pass"] is False
    assert check["current_frame_detector_pass"] is True


def test_unconfirmed_semantic_without_frozen_exploration_holds():
    payload = manifest(size_cells=7, detections={})
    payload["robots"][0][
        "exploration_selection_before_target_override"
    ] = None

    overrides, report = evaluate_semantic_execution_guard(payload)

    assert overrides == {"robot-0": None}
    assert report["held_robot_ids"] == ["robot-0"]
    assert report["checks"]["robot-0"]["execution_action"] == "HOLD"


def test_malformed_detector_confidence_fails_closed():
    payload = manifest(
        size_cells=5,
        detections={"potted plant": float("nan")},
    )

    with pytest.raises(ValueError, match="detection"):
        evaluate_semantic_execution_guard(payload)
