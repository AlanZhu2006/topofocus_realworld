import numpy as np
import pytest

from semantic_mapping.ground_estimator import (
    GroundEstimatorConfig,
    GroundHeightEstimator,
)


def plane_points(
    *,
    count: int,
    coefficients: tuple[float, float, float],
    noise_std: float = 0.0,
    seed: int = 1,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-1.5, 1.5, size=(count, 2))
    a, b, c = coefficients
    z = a * xy[:, 0] + b * xy[:, 1] + c
    z += rng.normal(0.0, noise_std, size=count)
    return np.column_stack((xy, z))


def test_ransac_recovers_ground_with_obstacle_outliers() -> None:
    ground = plane_points(
        count=1000,
        coefficients=(0.01, -0.02, 0.08),
        noise_std=0.005,
    )
    rng = np.random.default_rng(2)
    obstacles = np.column_stack(
        (
            rng.uniform(-1.0, 1.0, size=(250, 2)),
            rng.uniform(0.16, 0.24, size=250),
        )
    )
    estimator = GroundHeightEstimator(
        0.0,
        GroundEstimatorConfig(
            ema_alpha=1.0,
            max_update_step_m=0.2,
            max_candidate_jump_m=0.2,
        ),
    )

    result = estimator.update(
        np.vstack((ground, obstacles)), np.array([0.5, -0.25, 0.65])
    )

    assert result.accepted
    assert result.candidate_ground_z == pytest.approx(0.09, abs=0.01)
    assert result.ground_z == pytest.approx(0.0)
    assert result.inlier_points > 800
    assert result.tilt_deg == pytest.approx(1.28, abs=0.3)


def test_filter_limits_each_ground_height_update() -> None:
    points = plane_points(count=800, coefficients=(0.0, 0.0, 0.10))
    estimator = GroundHeightEstimator(
        0.0,
        GroundEstimatorConfig(
            candidate_window_size=3,
            ema_alpha=1.0,
            max_update_step_m=0.02,
            max_candidate_jump_m=0.2,
        ),
    )

    first = estimator.update(points, np.array([0.0, 0.0, 0.6]))
    second = estimator.update(points, np.array([0.0, 0.0, 0.6]))

    assert first.ground_z == pytest.approx(0.0)
    assert second.ground_z == pytest.approx(0.02)


def test_candidate_median_rejects_a_single_height_outlier() -> None:
    estimator = GroundHeightEstimator(
        0.0,
        GroundEstimatorConfig(
            candidate_window_size=5,
            ema_alpha=1.0,
            max_update_step_m=0.2,
            max_candidate_jump_m=0.2,
        ),
    )
    camera = np.array([0.0, 0.0, 0.6])

    outlier = estimator.update(
        plane_points(count=800, coefficients=(0.0, 0.0, 0.14)), camera
    )
    first_ground = estimator.update(
        plane_points(count=800, coefficients=(0.0, 0.0, 0.01)), camera
    )
    second_ground = estimator.update(
        plane_points(count=800, coefficients=(0.0, 0.0, 0.01)), camera
    )

    assert outlier.ground_z == pytest.approx(0.0)
    assert first_ground.ground_z == pytest.approx(0.0)
    assert second_ground.ground_z == pytest.approx(0.01)


def test_ground_fit_rejects_steep_plane_and_keeps_fallback() -> None:
    points = plane_points(count=800, coefficients=(0.5, 0.0, 0.0))
    estimator = GroundHeightEstimator(
        0.0,
        GroundEstimatorConfig(
            search_min_z_relative=-1.0,
            search_max_z_relative=1.0,
            max_tilt_deg=10.0,
        ),
    )

    result = estimator.update(points, np.array([0.0, 0.0, 0.6]))

    assert not result.accepted
    assert result.reason == "no_valid_plane"
    assert result.ground_z == 0.0


def test_ground_fit_rejects_insufficient_candidates() -> None:
    estimator = GroundHeightEstimator(0.3, GroundEstimatorConfig())
    result = estimator.update(
        np.zeros((20, 3), dtype=np.float32), np.array([0.0, 0.0, 0.8])
    )

    assert not result.accepted
    assert result.reason == "insufficient_candidates"
    assert result.ground_z == 0.3


def test_bootstrap_captures_initial_ground_outside_tracking_band() -> None:
    points = plane_points(
        count=1000, coefficients=(0.01, -0.01, -0.40), noise_std=0.004
    )
    estimator = GroundHeightEstimator(
        0.0,
        GroundEstimatorConfig(
            bootstrap_enabled=True,
            bootstrap_required_candidates=3,
            bootstrap_consensus_tolerance_m=0.03,
            bootstrap_search_min_z_relative=-1.0,
            bootstrap_search_max_z_relative=0.5,
        ),
    )
    camera = np.array([0.2, -0.1, 0.0])

    first = estimator.update(points, camera)
    second = estimator.update(points, camera)
    accepted = estimator.update(points, camera)

    assert first.reason == "bootstrap_pending"
    assert second.reason == "bootstrap_pending"
    assert accepted.accepted
    assert accepted.reason == "bootstrap_accepted"
    assert accepted.mode == "bootstrap"
    assert accepted.ground_z == pytest.approx(-0.397, abs=0.01)


def test_bootstrap_rejects_plane_not_below_camera() -> None:
    estimator = GroundHeightEstimator(
        0.0,
        GroundEstimatorConfig(
            bootstrap_enabled=True,
            bootstrap_search_min_z_relative=-1.0,
            bootstrap_search_max_z_relative=0.5,
        ),
    )

    result = estimator.update(
        plane_points(count=800, coefficients=(0.0, 0.0, 0.10)),
        np.array([0.0, 0.0, 0.0]),
    )

    assert not result.accepted
    assert result.reason == "bootstrap_camera_height_out_of_range"
    assert result.mode == "bootstrap"
