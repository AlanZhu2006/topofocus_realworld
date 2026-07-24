from __future__ import annotations

import importlib.util
from pathlib import Path


PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_overlay"
    / "ros_image_frame_alias.py"
)
SPEC = importlib.util.spec_from_file_location("ros_image_frame_alias", PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid(**overrides):
    values = {
        "frame_id": "camera_infra1_optical_frame",
        "width": 848,
        "height": 480,
        "encoding": "mono8",
        "expected_frame": "camera_infra1_optical_frame",
        "expected_width": 848,
        "expected_height": 480,
        "expected_encoding": "mono8",
    }
    values.update(overrides)
    return MODULE.validate_image_contract(**values)


def test_approved_tinynav_left_infra_alias_is_accepted() -> None:
    assert valid() is None


def test_unapproved_frame_is_rejected() -> None:
    assert "frame_id" in valid(frame_id="camera_color_optical_frame")


def test_geometry_or_encoding_change_is_rejected() -> None:
    assert "dimensions" in valid(width=640)
    assert "encoding" in valid(encoding="rgb8")


def test_explicit_usb2_fallback_profile_is_accepted() -> None:
    assert valid(
        width=640,
        alternate_dimensions=((640, 480),),
    ) is None
    assert "dimensions" in valid(
        width=320,
        alternate_dimensions=((640, 480),),
    )


def test_image_size_parser_rejects_malformed_or_nonpositive_values() -> None:
    assert MODULE.parse_dimensions("640x480") == (640, 480)
    for raw in ("640", "axb", "0x480", "640x-1"):
        try:
            MODULE.parse_dimensions(raw)
        except Exception:
            pass
        else:
            raise AssertionError(f"expected {raw!r} to be rejected")
