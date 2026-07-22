from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from focus_hub.calibration import (
    IDENTITY,
    apply_shared_frame_transform,
    compute_gravity_preserving_alignment,
    compute_shared_frame_transform,
    gravity_tilt_deg,
)
from focus_hub.geometry import compose_rigid


def load_board_calibration_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "tools"
        / "calibrate_camera_offset_via_board.py"
    )
    spec = importlib.util.spec_from_file_location("focus_test_board_calibration", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def se3(yaw_rad: float, x: float, y: float, z: float) -> tuple[float, ...]:
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    m = np.array(
        [
            [c, -s, 0.0, x],
            [s, c, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    return tuple(m.reshape(-1).tolist())


def assert_se3_close(a: tuple[float, ...], b: tuple[float, ...]) -> None:
    np.testing.assert_allclose(np.array(a), np.array(b), atol=1e-9)


def test_identity_sync_yields_identity_transform():
    transform = compute_shared_frame_transform(IDENTITY, IDENTITY)
    assert_se3_close(transform, IDENTITY)


def test_coincident_sync_recovers_reference_pose_when_applied_to_other_sync_pose():
    reference_at_sync = se3(0.3, 1.0, -2.0, 0.0)
    other_at_sync = se3(-1.2, 5.0, 0.5, 0.0)
    transform = compute_shared_frame_transform(reference_at_sync, other_at_sync)
    recovered = apply_shared_frame_transform(transform, other_at_sync)
    assert_se3_close(recovered, reference_at_sync)


def test_transform_maps_later_other_poses_consistently():
    reference_at_sync = se3(0.0, 0.0, 0.0, 0.0)
    other_at_sync = se3(np.pi / 2, 3.0, 0.0, 0.0)
    transform = compute_shared_frame_transform(reference_at_sync, other_at_sync)

    other_later = se3(np.pi / 2, 3.0, 1.0, 0.0)
    mapped = apply_shared_frame_transform(transform, other_later)
    # other moved +1m along its own y after sync; since sync rotation was
    # +90deg, that motion should appear as +1m along shared_world's x, and
    # the calibrated rotation should cancel out (other's sync heading maps
    # to shared_world's zero heading).
    assert mapped[3] == pytest.approx(1.0, abs=1e-9)
    assert mapped[7] == pytest.approx(0.0, abs=1e-9)


def test_measured_offset_is_honored_when_robots_are_not_coincident():
    reference_at_sync = se3(0.0, 0.0, 0.0, 0.0)
    other_at_sync = se3(0.0, 10.0, 10.0, 0.0)
    offset = se3(
        0.0, 0.5, 0.0, 0.0
    )  # other parked 0.5m ahead of reference, same heading

    transform = compute_shared_frame_transform(reference_at_sync, other_at_sync, offset)
    recovered = apply_shared_frame_transform(transform, other_at_sync)
    expected = compose_rigid(reference_at_sync, offset)
    assert_se3_close(recovered, expected)


def test_rejects_malformed_matrices():
    with pytest.raises(ValueError):
        compute_shared_frame_transform((1.0,) * 15, IDENTITY)


def test_gravity_preserving_alignment_maps_landmark_origin_without_tilting_z():
    reference = np.eye(4)
    reference[:3, 3] = [1.0, -2.0, 0.3]
    other = np.eye(4)
    # Deliberately include roll/pitch in the unconstrained alignment.
    yaw, pitch, roll = 0.7, 0.2, -0.1
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    other[:3, :3] = rz @ ry @ rx
    other[:3, 3] = [4.0, 5.0, -0.4]

    transform = compute_gravity_preserving_alignment(
        reference.reshape(-1), other.reshape(-1)
    )
    mapped = np.asarray(transform).reshape(4, 4) @ other

    np.testing.assert_allclose(mapped[:3, 3], reference[:3, 3], atol=1e-9)
    assert gravity_tilt_deg(transform) == pytest.approx(0.0, abs=1e-9)


def test_symmetric_grid_order_is_canonicalized_to_upper_left_endpoint():
    board = load_board_calibration_module()
    centers = np.array([[[20.0, 30.0]], [[15.0, 20.0]], [[10.0, 10.0]]])

    canonical, reversed_order = board.canonicalize_grid_centers(centers)

    assert reversed_order is True
    np.testing.assert_array_equal(canonical[0], [[10.0, 10.0]])
    np.testing.assert_array_equal(canonical[-1], [[20.0, 30.0]])
