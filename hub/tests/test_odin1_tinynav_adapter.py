from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


OVERLAY = Path(__file__).resolve().parents[1] / "robot_overlay"


def load_adapter():
    path = OVERLAY / "odin1_tinynav_adapter.py"
    sys.path.insert(0, str(OVERLAY))
    try:
        spec = importlib.util.spec_from_file_location(
            "test_odin1_tinynav_adapter", path
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(OVERLAY))


@pytest.mark.parametrize("yaw", [0.0, 0.4, -1.2, math.pi])
def test_matrix_to_quaternion_preserves_rotation(yaw: float) -> None:
    adapter = load_adapter()
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    qx, qy, qz, qw = adapter.matrix_to_quaternion(rotation)
    recovered_yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )

    assert math.cos(recovered_yaw) == pytest.approx(math.cos(yaw))
    assert math.sin(recovered_yaw) == pytest.approx(math.sin(yaw))
    assert qx * qx + qy * qy + qz * qz + qw * qw == pytest.approx(1.0)


def test_adapter_rejects_non_rotation_matrix() -> None:
    adapter = load_adapter()
    with pytest.raises(ValueError, match="orthonormal"):
        adapter.matrix_to_quaternion(np.diag([2.0, 1.0, 1.0]))


def test_adapter_has_no_motion_output() -> None:
    source = (OVERLAY / "odin1_tinynav_adapter.py").read_text(encoding="utf-8")
    assert "/cmd_vel" not in source
    assert "/api/" not in source
    assert "/slam/depth" in source
    assert "/focus/odin1/cloud_world" in source
