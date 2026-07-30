from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "tools" / "live_vlm_shadow.py"
)
SPEC = importlib.util.spec_from_file_location(
    "live_vlm_shadow_contract", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_robot_context_retains_artifact_provenance_field():
    assert "artifacts" in MODULE.RobotContext.__dataclass_fields__
    assert "T_shared_base" in MODULE.RobotContext.__dataclass_fields__


def test_reviewed_source_contract_matches_immutable_workspace_inputs():
    records = MODULE.observe_reviewed_source_artifacts(MODULE.WORKSPACE)

    assert len(records) == 9
    assert records[0]["source_path"] == "source/Focus_realworld/main.py"
    assert records[0]["size_bytes"] == 103808
    assert records[0]["sha256"].startswith("0d241151a9d1")


def test_strict_vlm_contract_accepts_only_error_free_cascade():
    MODULE.require_complete_cascade_result(
        SimpleNamespace(errors=[]), robot_id="robot-0"
    )

    with pytest.raises(RuntimeError, match="robot-1.*Judgment"):
        MODULE.require_complete_cascade_result(
            SimpleNamespace(errors=["Judgment request timed out"]),
            robot_id="robot-1",
        )


def test_initial_shared_frontier_exhaustion_is_an_explicit_hold():
    reason = MODULE.initial_allocation_hold_reason(
        scene_state=None,
        candidate_frontiers=[],
        semantic_goal=None,
    )

    assert reason is not None
    assert "explicit HOLD" in reason


def test_frontier_hold_does_not_override_replan_or_semantic_goal():
    frontier = SimpleNamespace(frontier_id="A")

    assert MODULE.initial_allocation_hold_reason(
        scene_state=SimpleNamespace(),
        candidate_frontiers=[],
        semantic_goal=None,
    ) is None
    assert MODULE.initial_allocation_hold_reason(
        scene_state=None,
        candidate_frontiers=[frontier],
        semantic_goal=None,
    ) is None
    assert MODULE.initial_allocation_hold_reason(
        scene_state=None,
        candidate_frontiers=[],
        semantic_goal={"kind": "semantic_goal"},
    ) is None


def yolo_summary() -> dict[str, object]:
    return {
        "last_observation_sequence": 17,
        "semantic_mapping": {
            "yolo_reinforcement": {
                "enabled": True,
                "method": "yolov10_image_detections_for_perception_vlm_only",
                "status": "model_inference_unverified_stage1_only",
                "map_reinforcement_enabled": False,
                "inference_policy": "every_current_observation_for_stage1",
                "frames_inferred": 3,
                "last_sequence": 17,
                "last_error": None,
                "last_detections": [
                    {"class_name": "chair", "confidence": 0.7},
                ],
                "model_provenance": {
                    "source_path": "/models/yolov10m.pt",
                    "size_bytes": 123,
                    "sha256": "a" * 64,
                },
            }
        },
    }


def test_yolo_stage1_is_bound_to_exact_latest_rgb_and_model():
    sequence, detections, provenance = MODULE.validated_yolo_source(
        yolo_summary()
    )

    assert sequence == 17
    assert detections == {"chair": 0.7}
    assert provenance["sha256"] == "a" * 64


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("last_error", "CUDA failure", "latest YOLO inference failed"),
        ("last_sequence", 16, "latest observation"),
        ("model_provenance", None, "model provenance"),
    ],
)
def test_yolo_stage1_rejects_incomplete_or_misaligned_evidence(
    field, value, message
):
    summary = yolo_summary()
    summary["semantic_mapping"]["yolo_reinforcement"][field] = value

    with pytest.raises(RuntimeError, match=message):
        MODULE.validated_yolo_source(summary)


def semantic_context(
    robot_id: str,
    *,
    backend: str = "segformer_b0_ade20k_to_mp3d40",
    fusion_mode: str = "multi_view",
    yolo_map_reinforcement: bool = False,
):
    return SimpleNamespace(
        spec=SimpleNamespace(robot_id=robot_id),
        map_summary={
            "semantic_fusion_mode": fusion_mode,
            "semantic_mapping": {
                "pixel_segmenter": {
                    "backend": backend,
                    "status": "ready",
                },
                "yolo_reinforcement": {
                    "map_reinforcement_enabled": yolo_map_reinforcement,
                },
            },
        },
    )


def test_semantic_input_contract_binds_uniform_robot_maps():
    contract = MODULE.semantic_input_contract(
        [semantic_context("robot-0"), semantic_context("robot-1")]
    )

    assert contract["uniform_across_robots"] is True
    assert (
        contract["pixel_segmenter_backend"]
        == "segformer_b0_ade20k_to_mp3d40"
    )
    assert contract["source_maskrcnn_override_available_in_hub"] is False
    assert contract["yolo_map_reinforcement_enabled"] is False
    assert set(contract["robots"]) == {"robot-0", "robot-1"}


def test_semantic_input_contract_marks_exact_source_composite_available():
    contract = MODULE.semantic_input_contract(
        [
            semantic_context(
                "robot-0",
                backend="source_rednet_detectron2_hm3d15",
                fusion_mode="max",
            ),
            semantic_context(
                "robot-1",
                backend="source_rednet_detectron2_hm3d15",
                fusion_mode="max",
            ),
        ]
    )

    assert contract["source_maskrcnn_override_available_in_hub"] is True
    assert contract["pixel_segmenter_backend"] == (
        "source_rednet_detectron2_hm3d15"
    )
    assert "executable-source" in contract["pixel_model_classification"]


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("backend", "rednet_mp3d40", "different pixel semantic backends"),
        ("fusion_mode", "max", "different semantic fusion modes"),
        (
            "yolo_map_reinforcement",
            True,
            "different YOLO map-reinforcement policies",
        ),
    ],
)
def test_semantic_input_contract_rejects_mixed_robot_maps(
    field, value, message
):
    options = {field: value}

    with pytest.raises(ValueError, match=message):
        MODULE.semantic_input_contract(
            [
                semantic_context("robot-0"),
                semantic_context("robot-1", **options),
            ]
        )


def test_source_visited_paths_uses_observed_base_trajectory():
    contexts = [
        SimpleNamespace(
            spec=SimpleNamespace(robot_id="robot-0"),
            robot_trajectory_xy_m=(
                (0.1, 0.1),
                (0.1, 0.1),
                (0.25, 0.15),
            ),
            robot_trajectory_provenance={
                "temporal_alignment": "map_atomic",
            },
        )
    ]

    paths, report = MODULE.source_visited_paths(
        contexts,
        origin_xy_m=(0.0, 0.0),
        resolution_m=0.05,
        shape_hw=(20, 20),
    )

    assert paths == [[(2, 2), (2, 5)]]
    assert report["robot-0"]["observed_world_point_count"] == 3
    assert report["robot-0"]["source_cell_path_count"] == 2
    assert report["robot-0"]["classification"].startswith(
        "observed base trajectory"
    )
    assert report["robot-0"]["trajectory_provenance"][
        "temporal_alignment"
    ] == "map_atomic"


def test_frozen_robot_trajectory_prefers_atomic_map_generation(tmp_path):
    map_path = tmp_path / "central_map.npz"
    np.savez_compressed(
        map_path,
        robot_trajectory_xy_m=np.asarray(
            [[0.1, 0.2], [0.3, 0.4]],
            dtype=np.float64,
        ),
        robot_trajectory_last_observation_sequence=np.asarray(
            17,
            dtype=np.int64,
        ),
        robot_trajectory_pose_source=np.asarray(
            "shared_T_camera @ inverse(base_T_camera)"
        ),
    )
    summary = {
        "last_observation_sequence": 17,
        "robot_trajectory_snapshot": {
            "container": "central_map.npz",
            "field": "robot_trajectory_xy_m",
            "point_count": 2,
            "last_observation_sequence": 17,
            "pose_source": (
                "shared_T_camera @ inverse(base_T_camera)"
            ),
        },
    }
    # A newer independently written live status must not leak into the map
    # generation used by the VLM image.
    status = {
        "last_observation_sequence": 19,
        "robot_trajectory_xy_m": [[9.0, 9.0]],
    }

    points, provenance = MODULE.frozen_robot_trajectory(
        map_path,
        summary,
        status,
    )

    assert points == ((0.1, 0.2), (0.3, 0.4))
    assert provenance["temporal_alignment"] == "map_atomic"
    assert provenance["last_observation_sequence"] == 17


def test_legacy_frozen_trajectory_records_non_atomic_alignment(tmp_path):
    map_path = tmp_path / "central_map.npz"
    np.savez_compressed(map_path, grid=np.zeros((2, 2, 2)))

    points, provenance = MODULE.frozen_robot_trajectory(
        map_path,
        {"last_observation_sequence": 17},
        {
            "last_observation_sequence": 19,
            "robot_trajectory_xy_m": [[0.1, 0.2]],
        },
    )

    assert points == ((0.1, 0.2),)
    assert provenance["temporal_alignment"] == (
        "recorded_but_not_map_atomic"
    )
    assert provenance["map_last_observation_sequence"] == 17
    assert provenance["live_status_last_observation_sequence"] == 19
