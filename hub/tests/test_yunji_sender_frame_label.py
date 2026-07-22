import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


def load_sender_module():
    path = Path(__file__).resolve().parents[1] / "robot_overlay" / "yunji_sender.py"
    spec = importlib.util.spec_from_file_location("focus_test_yunji_sender", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_camera_frame_tracks_physical_camera_source():
    sender = load_sender_module()

    assert sender.camera_frame_for_source("rosbridge", "d455") == (
        "camera_front_up_depth_optical_frame"
    )
    assert sender.camera_frame_for_source("local-realsense", "d405") == (
        "d405_color_optical_frame"
    )
    assert sender.camera_frame_for_source("local-realsense", "d455") == (
        "d455_color_optical_frame"
    )


@pytest.mark.parametrize(
    ("source", "model"),
    [("unknown", "d455"), ("local-realsense", "unknown")],
)
def test_camera_frame_rejects_unknown_source_or_model(source, model):
    sender = load_sender_module()

    with pytest.raises(ValueError):
        sender.camera_frame_for_source(source, model)


def test_load_camera_extrinsic_validates_model_and_rigid_matrix(tmp_path):
    sender = load_sender_module()
    path = tmp_path / "extrinsic.json"
    path.write_text(
        json.dumps(
            {
                "camera_model": "d455",
                "base_link_from_camera": {
                    "matrix": sender.MEASURED_T_BASE_LINK_CAMERA_D455.reshape(
                        -1
                    ).tolist()
                },
            }
        )
    )

    loaded = sender.load_camera_extrinsic(str(path), "d455")

    assert loaded is not None
    assert loaded.shape == (4, 4)
    with pytest.raises(ValueError, match="model mismatch"):
        sender.load_camera_extrinsic(str(path), "d405")


def test_load_camera_extrinsic_rejects_non_rigid_rotation(tmp_path):
    sender = load_sender_module()
    matrix = sender.MEASURED_T_BASE_LINK_CAMERA_D455.copy()
    matrix[0, 0] = 2.0
    path = tmp_path / "bad.json"
    path.write_text(
        json.dumps(
            {
                "camera_model": "d455",
                "base_link_from_camera": {"matrix": matrix.reshape(-1).tolist()},
            }
        )
    )

    with pytest.raises(ValueError, match="orthonormal"):
        sender.load_camera_extrinsic(str(path), "d455")


def test_sender_help_is_renderable():
    path = Path(__file__).resolve().parents[1] / "robot_overlay" / "yunji_sender.py"

    result = subprocess.run(
        [sys.executable, str(path), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--camera-extrinsic-file" in result.stdout
