from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

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


def test_strict_vlm_contract_accepts_only_error_free_cascade():
    MODULE.require_complete_cascade_result(
        SimpleNamespace(errors=[]), robot_id="robot-0"
    )

    with pytest.raises(RuntimeError, match="robot-1.*Judgment"):
        MODULE.require_complete_cascade_result(
            SimpleNamespace(errors=["Judgment request timed out"]),
            robot_id="robot-1",
        )


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
