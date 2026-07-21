from __future__ import annotations

import numpy as np
import pytest

from focus_hub.calibration import (
    IDENTITY,
    apply_shared_frame_transform,
    compute_shared_frame_transform,
)
from focus_hub.geometry import compose_rigid


def se3(yaw_rad: float, x: float, y: float, z: float) -> tuple[float, ...]:
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    m = np.array([
        [c, -s, 0.0, x],
        [s, c, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ])
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
    offset = se3(0.0, 0.5, 0.0, 0.0)  # other parked 0.5m ahead of reference, same heading

    transform = compute_shared_frame_transform(reference_at_sync, other_at_sync, offset)
    recovered = apply_shared_frame_transform(transform, other_at_sync)
    expected = compose_rigid(reference_at_sync, offset)
    assert_se3_close(recovered, expected)


def test_rejects_malformed_matrices():
    with pytest.raises(ValueError):
        compute_shared_frame_transform((1.0,) * 15, IDENTITY)
