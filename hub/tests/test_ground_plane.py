from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from focus_hub.ground_plane import (
    GroundPlaneConfig,
    estimate_startup_ground,
    fit_ground_candidate,
    plane_angle_deg,
    plane_height_at,
    plane_normal,
)


def plane_points(z: float, *, seed: int, count: int = 1200) -> np.ndarray:
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-1.5, 1.5, size=(count, 2))
    ground = np.column_stack((xy, np.full(count, z)))
    obstacles = np.column_stack(
        (rng.uniform(-1.0, 1.0, size=(250, 2)), rng.uniform(z + 0.2, z + 0.4, 250))
    )
    return np.vstack((ground, obstacles))


def test_ground_candidate_recovers_horizontal_plane_with_outliers():
    result = fit_ground_candidate(
        plane_points(0.08, seed=1),
        np.array([0.2, -0.1, 0.60]),
        GroundPlaneConfig(),
    )

    assert result.accepted
    assert result.ground_z_m == pytest.approx(0.08, abs=0.01)
    assert result.inlier_points > 1000
    assert result.tilt_deg == pytest.approx(0.0, abs=0.2)
    assert result.plane_coefficients is not None
    np.testing.assert_allclose(result.plane_coefficients, [0.0, 0.0, 0.08], atol=0.01)


def test_startup_ground_requires_three_consistent_frames(monkeypatch):
    frames = [
        SimpleNamespace(T_shared_camera=np.eye(4), points=plane_points(z, seed=index))
        for index, z in enumerate((0.08, 0.09, 0.07), start=1)
    ]
    for frame in frames:
        frame.T_shared_camera[2, 3] = 0.60
    monkeypatch.setattr(
        "focus_hub.ground_plane.depth_points_world",
        lambda frame, _K, _config: frame.points,
    )

    result = estimate_startup_ground(frames, np.eye(3))

    assert result.accepted
    assert result.ground_z_m == pytest.approx(0.08, abs=0.01)
    assert len(result.candidates) == 3
    assert result.plane_coefficients is not None
    np.testing.assert_allclose(result.plane_coefficients, [0.0, 0.0, 0.08], atol=0.01)


def test_startup_ground_rejects_inconsistent_planes(monkeypatch):
    frames = [
        SimpleNamespace(T_shared_camera=np.eye(4), points=plane_points(z, seed=index))
        for index, z in enumerate((0.02, 0.10, 0.18), start=1)
    ]
    for frame in frames:
        frame.T_shared_camera[2, 3] = 0.65
    monkeypatch.setattr(
        "focus_hub.ground_plane.depth_points_world",
        lambda frame, _K, _config: frame.points,
    )

    result = estimate_startup_ground(frames, np.eye(3))

    assert not result.accepted
    assert result.reason == "inconsistent_candidates"


def test_ground_candidate_preserves_tilted_plane_coefficients():
    rng = np.random.default_rng(12)
    xy = rng.uniform(-1.5, 1.5, size=(1600, 2))
    z = 0.10 * xy[:, 0] - 0.04 * xy[:, 1] + 0.03
    points = np.column_stack((xy, z))

    result = fit_ground_candidate(
        points,
        np.array([0.2, -0.1, 0.60]),
        GroundPlaneConfig(),
    )

    assert result.accepted
    assert result.plane_coefficients is not None
    np.testing.assert_allclose(
        result.plane_coefficients, [0.10, -0.04, 0.03], atol=1e-6
    )
    assert plane_height_at(result.plane_coefficients, [2.0, 1.0]) == pytest.approx(0.19)
    normal = plane_normal(result.plane_coefficients)
    assert np.linalg.norm(normal) == pytest.approx(1.0)
    assert plane_angle_deg(
        result.plane_coefficients, result.plane_coefficients
    ) == pytest.approx(0.0)
