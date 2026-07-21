"""Pure RGB-D geometry using ROS optical-frame coordinates.

Input depth is assumed to be registered to the image described by the supplied
intrinsics. Camera points follow REP-103 optical axes: X right, Y down, Z
forward. Axis changes are applied only through explicit SE(3) transforms.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.floating]
UInt8Array = NDArray[np.uint8]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for an image with fixed dimensions."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera width and height must be positive")
        values = np.asarray([self.fx, self.fy, self.cx, self.cy], dtype=np.float64)
        if not np.all(np.isfinite(values)):
            raise ValueError("Camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Camera focal lengths must be positive")

    @property
    def matrix(self) -> NDArray[np.float64]:
        """Return the 3x3 pinhole calibration matrix."""
        return np.asarray(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class BackprojectionResult:
    """Valid sampled points and their source pixels/colors."""

    points_camera: NDArray[np.float32]
    pixels_uv: NDArray[np.int32]
    depth_m: NDArray[np.float32]
    colors_rgb: UInt8Array | None


def depth_image_to_meters(depth: NDArray, encoding: str) -> NDArray[np.float32]:
    """Convert a ROS depth image array to meters without mutating the input."""
    depth_array = np.asarray(depth)
    if depth_array.ndim != 2:
        raise ValueError(f"Depth image must be 2D, got shape {depth_array.shape}")

    normalized_encoding = encoding.upper()
    if normalized_encoding in {"16UC1", "MONO16"}:
        return depth_array.astype(np.float32) * np.float32(0.001)
    if normalized_encoding == "32FC1":
        return depth_array.astype(np.float32, copy=True)
    raise ValueError(
        f"Unsupported depth encoding {encoding!r}; expected 16UC1, mono16, or 32FC1"
    )


def _depth_validity_mask(
    depth_m: NDArray[np.float32],
    min_depth_m: float,
    max_depth_m: float,
    edge_filter: bool,
    edge_threshold_m: float,
) -> NDArray[np.bool_]:
    if not np.isfinite(min_depth_m) or not np.isfinite(max_depth_m):
        raise ValueError("Depth limits must be finite")
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("Expected 0 <= min_depth_m < max_depth_m")
    if edge_threshold_m <= 0.0:
        raise ValueError("edge_threshold_m must be positive")

    valid = (
        np.isfinite(depth_m)
        & (depth_m > 0.0)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    if not edge_filter:
        return valid

    edge = np.zeros(depth_m.shape, dtype=bool)
    horizontal_pair = valid[:, 1:] & valid[:, :-1]
    horizontal_jump = horizontal_pair & (
        np.abs(depth_m[:, 1:] - depth_m[:, :-1]) > edge_threshold_m
    )
    edge[:, 1:] |= horizontal_jump
    edge[:, :-1] |= horizontal_jump

    vertical_pair = valid[1:, :] & valid[:-1, :]
    vertical_jump = vertical_pair & (
        np.abs(depth_m[1:, :] - depth_m[:-1, :]) > edge_threshold_m
    )
    edge[1:, :] |= vertical_jump
    edge[:-1, :] |= vertical_jump
    return valid & ~edge


def backproject_depth(
    depth_m: FloatArray,
    intrinsics: CameraIntrinsics,
    *,
    rgb_image: NDArray | None = None,
    stride: int = 2,
    min_depth_m: float = 0.25,
    max_depth_m: float = 5.0,
    edge_filter: bool = True,
    edge_threshold_m: float = 0.10,
) -> BackprojectionResult:
    """Backproject valid aligned depth samples into the camera optical frame."""
    depth = np.asarray(depth_m, dtype=np.float32)
    expected_shape = (intrinsics.height, intrinsics.width)
    if depth.shape != expected_shape:
        raise ValueError(
            f"Depth shape {depth.shape} does not match CameraInfo {expected_shape}"
        )
    if stride <= 0:
        raise ValueError("stride must be a positive integer")

    colors: UInt8Array | None = None
    if rgb_image is not None:
        image = np.asarray(rgb_image)
        if image.ndim != 3 or image.shape[:2] != expected_shape or image.shape[2] < 3:
            raise ValueError(
                "RGB image must have shape "
                f"({intrinsics.height}, {intrinsics.width}, >=3), got {image.shape}"
            )
    else:
        image = None

    valid = _depth_validity_mask(
        depth,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        edge_filter=edge_filter,
        edge_threshold_m=edge_threshold_m,
    )
    rows = np.arange(0, intrinsics.height, stride, dtype=np.int32)
    cols = np.arange(0, intrinsics.width, stride, dtype=np.int32)
    u_grid, v_grid = np.meshgrid(cols, rows)
    sampled_valid = valid[np.ix_(rows, cols)]

    if not np.any(sampled_valid):
        return BackprojectionResult(
            points_camera=np.empty((0, 3), dtype=np.float32),
            pixels_uv=np.empty((0, 2), dtype=np.int32),
            depth_m=np.empty((0,), dtype=np.float32),
            colors_rgb=None if image is None else np.empty((0, 3), dtype=np.uint8),
        )

    u = u_grid[sampled_valid]
    v = v_grid[sampled_valid]
    z = depth[v, u]
    x = (
        (u.astype(np.float32) - np.float32(intrinsics.cx))
        * z
        / np.float32(intrinsics.fx)
    )
    y = (
        (v.astype(np.float32) - np.float32(intrinsics.cy))
        * z
        / np.float32(intrinsics.fy)
    )
    points = np.column_stack((x, y, z)).astype(np.float32, copy=False)
    pixels = np.column_stack((u, v)).astype(np.int32, copy=False)
    if image is not None:
        colors = image[v, u, :3].astype(np.uint8, copy=True)

    return BackprojectionResult(
        points_camera=points,
        pixels_uv=pixels,
        depth_m=z.astype(np.float32, copy=False),
        colors_rgb=colors,
    )


def transform_points(points: FloatArray, transform: FloatArray) -> NDArray[np.float32]:
    """Apply `p_target = T_target_source * p_source` to an Nx3 point array."""
    point_array = np.asarray(points, dtype=np.float32)
    matrix = np.asarray(transform, dtype=np.float64)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError(f"Points must have shape (N, 3), got {point_array.shape}")
    if matrix.shape != (4, 4):
        raise ValueError(f"Transform must have shape (4, 4), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("Transform must contain finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError("Transform must have homogeneous final row [0, 0, 0, 1]")
    if point_array.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    transformed = point_array @ matrix[:3, :3].T + matrix[:3, 3]
    return transformed.astype(np.float32, copy=False)
