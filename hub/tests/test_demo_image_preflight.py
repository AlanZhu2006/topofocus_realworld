from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from focus_hub.demo_image_preflight import (
    ImagePreflightManifest,
    validate_historical_inputs,
)


def artifact_record(workspace: Path, path: Path, payload: bytes) -> dict[str, object]:
    absolute = workspace / path
    absolute.parent.mkdir(parents=True, exist_ok=True)
    absolute.write_bytes(payload)
    return {
        "path": str(path),
        "size_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "classification": "observed_local_runtime",
    }


def make_manifest(tmp_path, observation_factory) -> ImagePreflightManifest:
    cases: list[dict[str, object]] = []
    for case_index in range(4):
        observations: list[dict[str, object]] = []
        for robot_index, robot_id in enumerate(("robot-0", "robot-1")):
            sequence = case_index * 10 + robot_index
            observation = observation_factory(
                robot_id=robot_id,
                sequence=sequence,
                now_ns=10_000_000_000 + case_index * 1_000_000,
            )
            directory = Path("runtime") / robot_id / f"{sequence:020d}"
            metadata = observation.model_dump_json().encode()
            observations.append(
                {
                    "robot_id": robot_id,
                    "sequence": sequence,
                    "capture_time_ns": observation.capture_time_ns,
                    "camera_frame": observation.pose.shared_T_camera.child_frame,
                    "transform_version": observation.pose.transform_version,
                    "wire_object_goal_category": observation.object_goal.category,
                    "metadata": artifact_record(
                        tmp_path, directory / "metadata.json", metadata
                    ),
                    "rgb": artifact_record(tmp_path, directory / "rgb.jpg", b"rgb"),
                    "depth": artifact_record(
                        tmp_path, directory / "depth.png", b"depth"
                    ),
                }
            )
        cases.append(
            {
                "case_id": f"case_{case_index}",
                "target_category": "chair",
                "purpose": "synthetic manifest validation fixture",
                "target_assignment": "operator_selected_replay_target",
                "observations": observations,
            }
        )
    return ImagePreflightManifest.model_validate(
        {
            "preflight_id": "test_preflight",
            "classification": (
                "observed_historical_inputs_plus_operator_selected_replay_targets"
            ),
            "official_navigation_metrics_eligible": False,
            "ineligibility_reason": "static images have no navigation outcome",
            "repetitions_per_case": 5,
            "maximum_capture_skew_s": 1.0,
            "cases": cases,
        }
    )


def test_historical_manifest_validates_bytes_and_wire_identity(
    tmp_path, observation_factory
):
    manifest = make_manifest(tmp_path, observation_factory)
    report = validate_historical_inputs(manifest, workspace=tmp_path)

    assert report["status"] == "historical_inputs_verified"
    assert report["official_navigation_metrics_eligible"] is False
    assert report["case_count"] == 4
    assert report["planned_case_repetitions"] == 20
    assert report["planned_agent_image_calls"] == 40
    assert report["artifact_count"] == 24


def test_historical_manifest_rejects_artifact_hash_drift(
    tmp_path, observation_factory
):
    manifest = make_manifest(tmp_path, observation_factory)
    rgb_path = tmp_path / manifest.cases[0].observations[0].rgb.path
    rgb_path.write_bytes(b"bad")

    with pytest.raises(ValueError, match="hash drift"):
        validate_historical_inputs(manifest, workspace=tmp_path)


def test_historical_manifest_rejects_non_hpc_target(tmp_path, observation_factory):
    manifest = make_manifest(tmp_path, observation_factory)
    payload = manifest.model_dump()
    payload["cases"][0]["target_category"] = "table"

    with pytest.raises(ValueError, match="unsupported HPC ObjectNav target"):
        ImagePreflightManifest.model_validate(payload)
