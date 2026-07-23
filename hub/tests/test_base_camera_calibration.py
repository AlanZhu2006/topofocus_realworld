from __future__ import annotations

import json

import pytest

from focus_hub.base_camera_calibration import load_base_camera_calibration


IDENTITY = [
    1, 0, 0, 0.2,
    0, 1, 0, 0.0,
    0, 0, 1, 0.4,
    0, 0, 0, 1,
]


def artifact(*, status="operator_measured_physical_mount"):
    return {
        "schema_version": "focus-base-camera-calibration-v1",
        "robot_id": "robot-0",
        "base_T_camera": {
            "parent_frame": "base_link",
            "child_frame": "camera",
            "matrix": IDENTITY,
        },
        "measurement": {"status": status, "note": "physical ruler"},
        "passed": True,
    }


def test_measured_base_camera_artifact_loads_with_file_provenance(tmp_path):
    path = tmp_path / "mount.json"
    path.write_text(json.dumps(artifact()))

    loaded = load_base_camera_calibration(
        path, expected_robot_id="robot-0", expected_camera_frame="camera"
    )

    assert loaded.matrix == tuple(float(value) for value in IDENTITY)
    assert loaded.source_size_bytes == path.stat().st_size
    assert len(loaded.source_sha256) == 64
    assert loaded.wire_transform()["parent_frame"] == "base_link"


def test_unverified_placeholder_is_rejected(tmp_path):
    path = tmp_path / "mount.json"
    path.write_text(json.dumps(artifact(status="nominal_guess")))

    with pytest.raises(ValueError, match="not classified as measured"):
        load_base_camera_calibration(
            path, expected_robot_id="robot-0", expected_camera_frame="camera"
        )
