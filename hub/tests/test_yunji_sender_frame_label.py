import importlib.util
from pathlib import Path

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
