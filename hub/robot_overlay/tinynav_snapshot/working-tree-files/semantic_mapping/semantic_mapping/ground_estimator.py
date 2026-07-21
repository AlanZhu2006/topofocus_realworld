"""Robust local ground-height estimation for a z-up map frame."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class GroundEstimatorConfig:
    """Filtering, RANSAC, and temporal smoothing parameters."""

    horizontal_radius_m: float = 2.0
    search_min_z_relative: float = -0.25
    search_max_z_relative: float = 0.25
    max_points: int = 4000
    ransac_iterations: int = 128
    inlier_threshold_m: float = 0.03
    min_candidate_points: int = 200
    min_inlier_points: int = 300
    min_inlier_ratio: float = 0.15
    max_tilt_deg: float = 12.0
    max_candidate_jump_m: float = 0.15
    candidate_window_size: int = 9
    ema_alpha: float = 0.05
    max_update_step_m: float = 0.01
    random_seed: int = 0
    # Bootstrap is intentionally opt-in for the pure estimator. The ROS node
    # enables it because a newly-created TinyNav map may not have ground_z=0.
    bootstrap_enabled: bool = False
    bootstrap_search_min_z_relative: float = -1.0
    bootstrap_search_max_z_relative: float = 0.50
    bootstrap_min_camera_height_m: float = 0.15
    bootstrap_max_camera_height_m: float = 1.00
    bootstrap_required_candidates: int = 3
    bootstrap_consensus_tolerance_m: float = 0.04

    def __post_init__(self) -> None:
        finite_values = (
            self.horizontal_radius_m,
            self.search_min_z_relative,
            self.search_max_z_relative,
            self.inlier_threshold_m,
            self.min_inlier_ratio,
            self.max_tilt_deg,
            self.max_candidate_jump_m,
            self.ema_alpha,
            self.max_update_step_m,
            self.bootstrap_search_min_z_relative,
            self.bootstrap_search_max_z_relative,
            self.bootstrap_min_camera_height_m,
            self.bootstrap_max_camera_height_m,
            self.bootstrap_consensus_tolerance_m,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("Ground estimator values must be finite")
        if self.horizontal_radius_m <= 0.0:
            raise ValueError("horizontal_radius_m must be positive")
        if self.search_min_z_relative >= self.search_max_z_relative:
            raise ValueError("Ground search band must have positive height")
        if self.max_points < 3 or self.ransac_iterations <= 0:
            raise ValueError("Ground RANSAC sample limits are invalid")
        if self.inlier_threshold_m <= 0.0:
            raise ValueError("inlier_threshold_m must be positive")
        if self.min_candidate_points < 3 or self.min_inlier_points < 3:
            raise ValueError("Ground point thresholds must be at least three")
        if self.min_inlier_points > self.max_points:
            raise ValueError("min_inlier_points cannot exceed max_points")
        if not 0.0 < self.min_inlier_ratio <= 1.0:
            raise ValueError("min_inlier_ratio must be in (0, 1]")
        if not 0.0 <= self.max_tilt_deg < 90.0:
            raise ValueError("max_tilt_deg must be in [0, 90)")
        if self.max_candidate_jump_m <= 0.0:
            raise ValueError("max_candidate_jump_m must be positive")
        if self.candidate_window_size < 3 or self.candidate_window_size % 2 == 0:
            raise ValueError("candidate_window_size must be an odd value >= 3")
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        if self.max_update_step_m <= 0.0:
            raise ValueError("max_update_step_m must be positive")
        if self.bootstrap_search_min_z_relative >= self.bootstrap_search_max_z_relative:
            raise ValueError("Bootstrap ground search band must have positive height")
        if self.bootstrap_min_camera_height_m < 0.0:
            raise ValueError("bootstrap_min_camera_height_m must be non-negative")
        if self.bootstrap_max_camera_height_m <= self.bootstrap_min_camera_height_m:
            raise ValueError(
                "bootstrap_max_camera_height_m must exceed "
                "bootstrap_min_camera_height_m"
            )
        if self.bootstrap_required_candidates <= 0:
            raise ValueError("bootstrap_required_candidates must be positive")
        if self.bootstrap_consensus_tolerance_m <= 0.0:
            raise ValueError("bootstrap_consensus_tolerance_m must be positive")


@dataclass(frozen=True)
class GroundEstimate:
    """Result of one update, including diagnostics for rejected fits."""

    accepted: bool
    ground_z: float
    candidate_ground_z: float | None
    consensus_ground_z: float | None
    candidate_points: int
    inlier_points: int
    inlier_ratio: float
    tilt_deg: float | None
    reason: str
    mode: str = "tracking"


class GroundHeightEstimator:
    """Maintain a filtered scalar ground height from local surface points."""

    def __init__(
        self, initial_ground_z: float, config: GroundEstimatorConfig
    ) -> None:
        if not math.isfinite(initial_ground_z):
            raise ValueError("initial_ground_z must be finite")
        self.config = config
        self.ground_z = float(initial_ground_z)
        self.candidate_history: deque[float] = deque(
            [self.ground_z] * self.config.candidate_window_size,
            maxlen=self.config.candidate_window_size,
        )
        self.bootstrap_history: deque[float] = deque(
            maxlen=self.config.bootstrap_required_candidates
        )
        self.bootstrap_completed = not self.config.bootstrap_enabled
        self.accepted_updates = 0
        self.rejected_updates = 0

    def update(
        self,
        points_xyz: NDArray,
        camera_position_xyz: NDArray,
    ) -> GroundEstimate:
        """Fit a near-horizontal plane and update ground height at the camera."""
        points = np.asarray(points_xyz, dtype=np.float64)
        camera = np.asarray(camera_position_xyz, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"points_xyz must have shape (N, 3), got {points.shape}")
        if camera.shape != (3,) or not np.all(np.isfinite(camera)):
            raise ValueError("camera_position_xyz must contain three finite values")

        finite = np.all(np.isfinite(points), axis=1)
        relative_xy = points[:, :2] - camera[:2]
        within_radius = np.einsum("ij,ij->i", relative_xy, relative_xy) <= (
            self.config.horizontal_radius_m**2
        )
        mode = self.mode
        if mode == "bootstrap":
            search_min_z = self.config.bootstrap_search_min_z_relative
            search_max_z = self.config.bootstrap_search_max_z_relative
        else:
            search_min_z = self.config.search_min_z_relative
            search_max_z = self.config.search_max_z_relative
        z_relative = points[:, 2] - self.ground_z
        within_height = (
            (z_relative >= search_min_z) & (z_relative <= search_max_z)
        )
        candidates = points[finite & within_radius & within_height]
        candidate_count = int(candidates.shape[0])
        if candidate_count < self.config.min_candidate_points:
            return self._reject(
                "insufficient_candidates", candidate_count=candidate_count
            )

        if candidate_count > self.config.max_points:
            selection = np.linspace(
                0,
                candidate_count - 1,
                num=self.config.max_points,
                dtype=np.int64,
            )
            candidates = candidates[selection]
            candidate_count = int(candidates.shape[0])

        coefficients, inliers = self._fit_plane(candidates)
        if coefficients is None or inliers is None:
            return self._reject("no_valid_plane", candidate_count=candidate_count)

        inlier_count = int(np.count_nonzero(inliers))
        inlier_ratio = inlier_count / candidate_count
        if (
            inlier_count < self.config.min_inlier_points
            or inlier_ratio < self.config.min_inlier_ratio
        ):
            return self._reject(
                "insufficient_inliers",
                candidate_count=candidate_count,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
            )

        design = np.column_stack(
            (candidates[inliers, 0], candidates[inliers, 1], np.ones(inlier_count))
        )
        coefficients, _, rank, _ = np.linalg.lstsq(
            design, candidates[inliers, 2], rcond=None
        )
        if rank < 3:
            return self._reject(
                "degenerate_refinement",
                candidate_count=candidate_count,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
            )

        slope = float(np.hypot(coefficients[0], coefficients[1]))
        tilt_deg = math.degrees(math.atan(slope))
        if tilt_deg > self.config.max_tilt_deg:
            return self._reject(
                "tilt_exceeds_limit",
                candidate_count=candidate_count,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
                tilt_deg=tilt_deg,
            )

        candidate_ground_z = float(
            coefficients[0] * camera[0]
            + coefficients[1] * camera[1]
            + coefficients[2]
        )
        if mode == "bootstrap":
            return self._update_bootstrap(
                candidate_ground_z,
                camera_z=float(camera[2]),
                candidate_count=candidate_count,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
                tilt_deg=tilt_deg,
            )
        if (
            not math.isfinite(candidate_ground_z)
            or abs(candidate_ground_z - self.ground_z)
            > self.config.max_candidate_jump_m
        ):
            return self._reject(
                "candidate_jump_exceeds_limit",
                candidate_count=candidate_count,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
                tilt_deg=tilt_deg,
                candidate_ground_z=candidate_ground_z,
            )

        self.candidate_history.append(candidate_ground_z)
        consensus_ground_z = float(np.median(self.candidate_history))
        filtered_delta = self.config.ema_alpha * (
            consensus_ground_z - self.ground_z
        )
        bounded_delta = float(
            np.clip(
                filtered_delta,
                -self.config.max_update_step_m,
                self.config.max_update_step_m,
            )
        )
        self.ground_z += bounded_delta
        self.accepted_updates += 1
        return GroundEstimate(
            accepted=True,
            ground_z=self.ground_z,
            candidate_ground_z=candidate_ground_z,
            consensus_ground_z=consensus_ground_z,
            candidate_points=candidate_count,
            inlier_points=inlier_count,
            inlier_ratio=inlier_ratio,
            tilt_deg=tilt_deg,
            reason="accepted",
            mode=mode,
        )

    def _update_bootstrap(
        self,
        candidate_ground_z: float,
        *,
        camera_z: float,
        candidate_count: int,
        inlier_count: int,
        inlier_ratio: float,
        tilt_deg: float,
    ) -> GroundEstimate:
        """Accept a broad initial fit only after repeated camera-below evidence."""
        camera_height = camera_z - candidate_ground_z
        if (
            not math.isfinite(candidate_ground_z)
            or camera_height < self.config.bootstrap_min_camera_height_m
            or camera_height > self.config.bootstrap_max_camera_height_m
        ):
            return self._reject(
                "bootstrap_camera_height_out_of_range",
                candidate_count=candidate_count,
                inlier_count=inlier_count,
                inlier_ratio=inlier_ratio,
                tilt_deg=tilt_deg,
                candidate_ground_z=candidate_ground_z,
            )

        self.bootstrap_history.append(candidate_ground_z)
        consensus_ground_z = float(np.median(self.bootstrap_history))
        if len(self.bootstrap_history) < self.config.bootstrap_required_candidates:
            return GroundEstimate(
                accepted=False,
                ground_z=self.ground_z,
                candidate_ground_z=candidate_ground_z,
                consensus_ground_z=consensus_ground_z,
                candidate_points=candidate_count,
                inlier_points=inlier_count,
                inlier_ratio=inlier_ratio,
                tilt_deg=tilt_deg,
                reason="bootstrap_pending",
                mode="bootstrap",
            )

        spread = float(
            np.max(np.abs(np.asarray(self.bootstrap_history) - consensus_ground_z))
        )
        if spread > self.config.bootstrap_consensus_tolerance_m:
            self.bootstrap_history.clear()
            self.bootstrap_history.append(candidate_ground_z)
            return GroundEstimate(
                accepted=False,
                ground_z=self.ground_z,
                candidate_ground_z=candidate_ground_z,
                consensus_ground_z=consensus_ground_z,
                candidate_points=candidate_count,
                inlier_points=inlier_count,
                inlier_ratio=inlier_ratio,
                tilt_deg=tilt_deg,
                reason="bootstrap_inconsistent",
                mode="bootstrap",
            )

        self.ground_z = consensus_ground_z
        self.candidate_history = deque(
            [self.ground_z] * self.config.candidate_window_size,
            maxlen=self.config.candidate_window_size,
        )
        self.bootstrap_completed = True
        self.accepted_updates += 1
        return GroundEstimate(
            accepted=True,
            ground_z=self.ground_z,
            candidate_ground_z=candidate_ground_z,
            consensus_ground_z=consensus_ground_z,
            candidate_points=candidate_count,
            inlier_points=inlier_count,
            inlier_ratio=inlier_ratio,
            tilt_deg=tilt_deg,
            reason="bootstrap_accepted",
            mode="bootstrap",
        )

    def _fit_plane(
        self, candidates: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64] | None, NDArray[np.bool_] | None]:
        rng = np.random.default_rng(self.config.random_seed)
        best_coefficients: NDArray[np.float64] | None = None
        best_inliers: NDArray[np.bool_] | None = None
        best_count = -1
        best_error = math.inf
        design_all = np.column_stack(
            (candidates[:, 0], candidates[:, 1], np.ones(candidates.shape[0]))
        )

        for _ in range(self.config.ransac_iterations):
            sample_indices = rng.choice(candidates.shape[0], size=3, replace=False)
            sample_design = design_all[sample_indices]
            coefficients, _, rank, _ = np.linalg.lstsq(
                sample_design, candidates[sample_indices, 2], rcond=None
            )
            if rank < 3:
                continue
            slope = float(np.hypot(coefficients[0], coefficients[1]))
            if math.degrees(math.atan(slope)) > self.config.max_tilt_deg:
                continue
            residuals = np.abs(candidates[:, 2] - design_all @ coefficients)
            inliers = residuals <= self.config.inlier_threshold_m
            count = int(np.count_nonzero(inliers))
            median_error = (
                math.inf if count == 0 else float(np.median(residuals[inliers]))
            )
            if count > best_count or (count == best_count and median_error < best_error):
                best_coefficients = coefficients
                best_inliers = inliers
                best_count = count
                best_error = median_error

        return best_coefficients, best_inliers

    @property
    def mode(self) -> str:
        """Report the current estimator stage for diagnostics."""
        return "bootstrap" if not self.bootstrap_completed else "tracking"

    def _reject(
        self,
        reason: str,
        *,
        candidate_count: int,
        inlier_count: int = 0,
        inlier_ratio: float = 0.0,
        tilt_deg: float | None = None,
        candidate_ground_z: float | None = None,
    ) -> GroundEstimate:
        self.rejected_updates += 1
        return GroundEstimate(
            accepted=False,
            ground_z=self.ground_z,
            candidate_ground_z=candidate_ground_z,
            consensus_ground_z=None,
            candidate_points=candidate_count,
            inlier_points=inlier_count,
            inlier_ratio=inlier_ratio,
            tilt_deg=tilt_deg,
            reason=reason,
            mode=self.mode,
        )
