from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


OVERLAY = Path(__file__).parents[1] / "robot_overlay"


def load_module():
    path = OVERLAY / "tinynav_relocalized_odometry.py"
    spec = importlib.util.spec_from_file_location(
        "test_tinynav_relocalized_odometry_overlay_module", path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stamp_and_pose_helpers_preserve_ros_contract() -> None:
    module = load_module()
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=4, nanosec=5)
        ),
        pose=SimpleNamespace(pose=pose),
    )

    assert module.stamp_ns(message) == 4_000_000_005
    assert module.odometry_matrix(message) == pytest.approx(
        (
            1.0, 0.0, 0.0, 1.0,
            0.0, 1.0, 0.0, 2.0,
            0.0, 0.0, 1.0, 3.0,
            0.0, 0.0, 0.0, 1.0,
        )
    )


def test_overlay_declares_no_raw_pose_fallback() -> None:
    source = (
        OVERLAY / "tinynav_relocalized_odometry.py"
    ).read_text(encoding="utf-8")

    assert '"raw_pose_fallback_enabled": False' in source
    assert "map_T_camera=inverse(tracking_T_map)" in source
    assert "output.pose.covariance = list(covariance)" in source
    assert "output.twist = message.twist" in source
