from __future__ import annotations

import importlib.util
from pathlib import Path


PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_overlay"
    / "ros_continuous_depth_geometry_rgb.py"
)
SPEC = importlib.util.spec_from_file_location(
    "ros_continuous_depth_geometry_rgb",
    PATH,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid(**overrides):
    stamp_ns = 1_000_000_000_000
    values = {
        "frame_id": "camera",
        "width": 848,
        "height": 480,
        "encoding": "32FC1",
        "is_bigendian": False,
        "step": 848 * 4,
        "data_length": 848 * 480 * 4,
        "stamp_ns": stamp_ns,
        "now_ns": stamp_ns + 100_000_000,
        "last_published_stamp_ns": stamp_ns - 1,
        "expected_frame": "camera",
        "approved_dimensions": ((848, 480), (640, 480)),
        "max_capture_age_s": 2.0,
        "max_future_skew_s": 0.25,
    }
    values.update(overrides)
    return MODULE.validate_continuous_depth_contract(**values)


def test_observed_continuous_depth_contract_is_accepted() -> None:
    assert valid() is None
    assert valid(
        width=640,
        step=640 * 4,
        data_length=640 * 480 * 4,
    ) is None


def test_geometry_profile_changes_fail_closed() -> None:
    assert "frame_id" in valid(frame_id="camera_infra1_optical_frame")
    assert "dimensions" in valid(width=320, step=320 * 4)
    assert "encoding" in valid(encoding="16UC1")
    assert "big-endian" in valid(is_bigendian=True)
    assert "step" in valid(step=1)
    assert "data_length" in valid(data_length=1)


def test_capture_must_be_fresh_and_monotonic() -> None:
    stamp_ns = 1_000_000_000_000
    assert "not newer" in valid(last_published_stamp_ns=stamp_ns)
    assert "capture age" in valid(now_ns=stamp_ns + 2_000_000_001)
    assert "future" in valid(now_ns=stamp_ns - 250_000_001)
    assert "missing" in valid(stamp_ns=0, last_published_stamp_ns=0)


def test_image_size_parser_rejects_bad_values() -> None:
    assert MODULE.parse_dimensions("848x480") == (848, 480)
    for raw in ("848", "axb", "0x480", "848x-1"):
        try:
            MODULE.parse_dimensions(raw)
        except Exception:
            pass
        else:
            raise AssertionError(f"expected {raw!r} to be rejected")
