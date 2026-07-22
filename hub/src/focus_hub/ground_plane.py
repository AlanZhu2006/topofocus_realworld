"""Deterministic startup ground-plane estimation for real RGB-D maps.

The RANSAC policy and defaults are source-derived from the audited TinyNav
experiment preserved at
``hub/robot_overlay/tinynav_snapshot/working-tree-files/semantic_mapping/``
(``ground_estimator.py``).  This smaller Hub implementation only performs the
startup bootstrap needed before a fixed 2-D map is created; it has no ROS or
robot-control dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GroundPlaneConfig:
    horizontal_radius_m: float = 2.0
    min_camera_height_m: float = 0.15
    max_camera_height_m: float = 1.00
    depth_stride: int = 2
    min_range_m: float = 0.3
    max_range_m: float = 5.0
    max_points_per_frame: int = 4000
    ransac_iterations: int = 128
    inlier_threshold_m: float = 0.03
    min_candidate_points: int = 200
    min_inlier_points: int = 300
    min_inlier_ratio: float = 0.15
    max_tilt_deg: float = 12.0
    required_frames: int = 3
    consensus_tolerance_m: float = 0.04
    random_seed: int = 0

    def __post_init__(self) -> None:
        positive = (
            self.horizontal_radius_m,
            self.min_camera_height_m,
            self.max_camera_height_m,
            self.min_range_m,
            self.max_range_m,
            self.inlier_threshold_m,
            self.min_inlier_ratio,
            self.consensus_tolerance_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("ground-plane thresholds must be finite and positive")
        if self.max_camera_height_m <= self.min_camera_height_m:
            raise ValueError("maximum camera height must exceed minimum")
        if self.max_range_m <= self.min_range_m:
            raise ValueError("maximum depth range must exceed minimum")
        if self.depth_stride <= 0 or self.max_points_per_frame < 3:
            raise ValueError("ground-plane sampling limits are invalid")
        if self.ransac_iterations <= 0 or self.required_frames < 2:
            raise ValueError("ground-plane consensus limits are invalid")
        if self.min_candidate_points < 3 or self.min_inlier_points < 3:
            raise ValueError("ground-plane point thresholds must be at least three")
        if self.min_inlier_points > self.max_points_per_frame:
            raise ValueError("minimum inliers cannot exceed sampled points")
        if not 0.0 < self.min_inlier_ratio <= 1.0:
            raise ValueError("minimum inlier ratio must be in (0, 1]")
        if not 0.0 <= self.max_tilt_deg < 90.0:
            raise ValueError("maximum tilt must be in [0, 90) degrees")


@dataclass(frozen=True)
class GroundCandidate:
    accepted: bool
    ground_z_m: float | None
    reason: str
    candidate_points: int
    inlier_points: int
    inlier_ratio: float
    tilt_deg: float | None
    # Plane is expressed in the mapper's world frame as z = ax + by + c.
    # Keeping the full plane is essential: reducing a tilted plane to one
    # scalar z makes visible floor pixels look like obstacles away from the
    # camera XY at which that scalar was evaluated.
    plane_coefficients: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class GroundConsensus:
    accepted: bool
    ground_z_m: float | None
    reason: str
    candidates: tuple[GroundCandidate, ...]
    plane_coefficients: tuple[float, float, float] | None = None


def plane_normal(
    plane_coefficients: tuple[float, float, float] | np.ndarray,
) -> np.ndarray:
    """Return the unit upward normal for ``z = ax + by + c``."""
    coefficients = np.asarray(plane_coefficients, dtype=np.float64)
    if coefficients.shape != (3,) or not np.all(np.isfinite(coefficients)):
        raise ValueError("plane_coefficients must contain three finite values")
    normal = np.array([-coefficients[0], -coefficients[1], 1.0])
    return normal / np.linalg.norm(normal)


def plane_height_at(
    plane_coefficients: tuple[float, float, float] | np.ndarray,
    xy: tuple[float, float] | np.ndarray,
) -> float:
    """Evaluate ``z = ax + by + c`` at one finite world-frame XY."""
    coefficients = np.asarray(plane_coefficients, dtype=np.float64)
    point_xy = np.asarray(xy, dtype=np.float64)
    if coefficients.shape != (3,) or not np.all(np.isfinite(coefficients)):
        raise ValueError("plane_coefficients must contain three finite values")
    if point_xy.shape != (2,) or not np.all(np.isfinite(point_xy)):
        raise ValueError("xy must contain two finite values")
    return float(
        coefficients[0] * point_xy[0] + coefficients[1] * point_xy[1] + coefficients[2]
    )


def plane_angle_deg(
    first: tuple[float, float, float] | np.ndarray,
    second: tuple[float, float, float] | np.ndarray,
) -> float:
    """Smallest angle between two upward plane normals, in degrees."""
    cosine = float(
        np.clip(np.dot(plane_normal(first), plane_normal(second)), -1.0, 1.0)
    )
    return math.degrees(math.acos(cosine))


def depth_points_world(frame, K: np.ndarray, config: GroundPlaneConfig) -> np.ndarray:
    """Deterministically back-project a strided depth image into world XYZ."""
    depth_full = np.asarray(frame.depth_m)
    if depth_full.ndim != 2:
        raise ValueError(f"depth image must be 2-D, got {depth_full.shape}")
    stride = config.depth_stride
    depth = depth_full[::stride, ::stride].astype(np.float64)
    vs, us = np.meshgrid(
        np.arange(0, depth_full.shape[0], stride, dtype=np.float64),
        np.arange(0, depth_full.shape[1], stride, dtype=np.float64),
        indexing="ij",
    )
    intrinsics = np.asarray(K, dtype=np.float64)
    if intrinsics.shape != (3, 3) or not np.all(np.isfinite(intrinsics)):
        raise ValueError("K must be a finite 3x3 matrix")
    valid = (depth >= config.min_range_m) & (depth <= config.max_range_m)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)
    z = depth[valid]
    x = (us[valid] - intrinsics[0, 2]) / intrinsics[0, 0] * z
    y = (vs[valid] - intrinsics[1, 2]) / intrinsics[1, 1] * z
    camera_points = np.stack((x, y, z), axis=-1)
    pose = np.asarray(frame.T_shared_camera, dtype=np.float64)
    if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
        raise ValueError("T_shared_camera must be a finite 4x4 matrix")
    return camera_points @ pose[:3, :3].T + pose[:3, 3]


def fit_ground_candidate(
    points_world: np.ndarray,
    camera_position: np.ndarray,
    config: GroundPlaneConfig,
) -> GroundCandidate:
    """Fit one near-horizontal local plane and evaluate it at the camera XY."""
    points = np.asarray(points_world, dtype=np.float64)
    camera = np.asarray(camera_position, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points_world must have shape (N,3), got {points.shape}")
    if camera.shape != (3,) or not np.all(np.isfinite(camera)):
        raise ValueError("camera_position must contain three finite values")

    finite = np.all(np.isfinite(points), axis=1)
    relative_xy = points[:, :2] - camera[:2]
    local = np.einsum("ij,ij->i", relative_xy, relative_xy) <= (
        config.horizontal_radius_m**2
    )
    below_camera = (points[:, 2] >= camera[2] - config.max_camera_height_m) & (
        points[:, 2] <= camera[2] - config.min_camera_height_m
    )
    candidates = points[finite & local & below_camera]
    candidate_count = int(candidates.shape[0])
    if candidate_count < config.min_candidate_points:
        return GroundCandidate(
            False, None, "insufficient_candidates", candidate_count, 0, 0.0, None, None
        )
    if candidate_count > config.max_points_per_frame:
        selection = np.linspace(
            0,
            candidate_count - 1,
            num=config.max_points_per_frame,
            dtype=np.int64,
        )
        candidates = candidates[selection]
        candidate_count = int(candidates.shape[0])

    design = np.column_stack(
        (candidates[:, 0], candidates[:, 1], np.ones(candidate_count))
    )
    rng = np.random.default_rng(config.random_seed)
    best_inliers: np.ndarray | None = None
    best_count = -1
    best_error = math.inf
    for _ in range(config.ransac_iterations):
        sample = rng.choice(candidate_count, size=3, replace=False)
        coefficients, _, rank, _ = np.linalg.lstsq(
            design[sample], candidates[sample, 2], rcond=None
        )
        if rank < 3:
            continue
        tilt = math.degrees(math.atan(float(np.hypot(*coefficients[:2]))))
        if tilt > config.max_tilt_deg:
            continue
        residuals = np.abs(candidates[:, 2] - design @ coefficients)
        inliers = residuals <= config.inlier_threshold_m
        count = int(np.count_nonzero(inliers))
        error = math.inf if count == 0 else float(np.median(residuals[inliers]))
        if count > best_count or (count == best_count and error < best_error):
            best_inliers = inliers
            best_count = count
            best_error = error
    if best_inliers is None:
        return GroundCandidate(
            False, None, "no_valid_plane", candidate_count, 0, 0.0, None, None
        )

    inlier_count = int(np.count_nonzero(best_inliers))
    inlier_ratio = inlier_count / candidate_count
    if (
        inlier_count < config.min_inlier_points
        or inlier_ratio < config.min_inlier_ratio
    ):
        return GroundCandidate(
            False,
            None,
            "insufficient_inliers",
            candidate_count,
            inlier_count,
            inlier_ratio,
            None,
            None,
        )
    refined, _, rank, _ = np.linalg.lstsq(
        design[best_inliers], candidates[best_inliers, 2], rcond=None
    )
    if rank < 3:
        return GroundCandidate(
            False,
            None,
            "degenerate_refinement",
            candidate_count,
            inlier_count,
            inlier_ratio,
            None,
            None,
        )
    tilt = math.degrees(math.atan(float(np.hypot(*refined[:2]))))
    ground_z = float(refined[0] * camera[0] + refined[1] * camera[1] + refined[2])
    camera_height = float(camera[2] - ground_z)
    if (
        not math.isfinite(ground_z)
        or tilt > config.max_tilt_deg
        or camera_height < config.min_camera_height_m
        or camera_height > config.max_camera_height_m
    ):
        return GroundCandidate(
            False,
            ground_z,
            "plane_outside_limits",
            candidate_count,
            inlier_count,
            inlier_ratio,
            tilt,
            tuple(float(value) for value in refined),
        )
    return GroundCandidate(
        True,
        ground_z,
        "accepted",
        candidate_count,
        inlier_count,
        inlier_ratio,
        tilt,
        tuple(float(value) for value in refined),
    )


def estimate_startup_ground(
    frames: list,
    K: np.ndarray,
    config: GroundPlaneConfig | None = None,
) -> GroundConsensus:
    """Require a consistent accepted plane from the latest startup frames."""
    cfg = config or GroundPlaneConfig()
    if len(frames) < cfg.required_frames:
        return GroundConsensus(False, None, "insufficient_frames", (), None)
    selected = frames[-cfg.required_frames :]
    candidates = tuple(
        fit_ground_candidate(
            depth_points_world(frame, K, cfg),
            np.asarray(frame.T_shared_camera, dtype=np.float64)[:3, 3],
            cfg,
        )
        for frame in selected
    )
    if not all(candidate.accepted for candidate in candidates):
        return GroundConsensus(False, None, "candidate_rejected", candidates, None)
    heights = np.asarray(
        [candidate.ground_z_m for candidate in candidates], dtype=np.float64
    )
    consensus = float(np.median(heights))
    if float(np.max(np.abs(heights - consensus))) > cfg.consensus_tolerance_m:
        return GroundConsensus(False, None, "inconsistent_candidates", candidates, None)
    coefficients = np.median(
        np.asarray(
            [candidate.plane_coefficients for candidate in candidates], dtype=np.float64
        ),
        axis=0,
    )
    return GroundConsensus(
        True,
        consensus,
        "accepted",
        candidates,
        tuple(float(value) for value in coefficients),
    )
