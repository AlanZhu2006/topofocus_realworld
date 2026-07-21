"""Voxel indexing and Amanatides-Woo ray traversal.

Coordinates are expressed in one Cartesian target frame. Voxel indices use
floor division relative to a fixed map origin, so negative coordinates and
positive coordinates follow the same convention.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


VoxelIndex = tuple[int, int, int]


@dataclass(frozen=True)
class BatchRaycastResult:
    """Unique frame-level DDA results and input validation counts."""

    free_voxels: NDArray[np.int64]
    occupied_voxels: NDArray[np.int64]
    valid_rays: int
    rejected_rays: int
    traversed_free_cells: int


def _point3(value: Sequence[float] | NDArray, name: str) -> NDArray[np.float64]:
    point = np.asarray(value, dtype=np.float64)
    if point.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {point.shape}")
    if not np.all(np.isfinite(point)):
        raise ValueError(f"{name} must contain finite values")
    return point


def point_to_voxel(
    point: Sequence[float] | NDArray,
    origin: Sequence[float] | NDArray,
    resolution_m: float,
) -> VoxelIndex:
    """Convert a target-frame point to its containing voxel index."""
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("resolution_m must be finite and positive")
    point_array = _point3(point, "point")
    origin_array = _point3(origin, "origin")
    index = np.floor((point_array - origin_array) / resolution_m).astype(np.int64)
    return int(index[0]), int(index[1]), int(index[2])


def voxel_center(
    index: Sequence[int] | NDArray,
    origin: Sequence[float] | NDArray,
    resolution_m: float,
) -> NDArray[np.float64]:
    """Return the target-frame center of a voxel."""
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("resolution_m must be finite and positive")
    index_array = np.asarray(index, dtype=np.int64)
    if index_array.shape != (3,):
        raise ValueError(f"index must have shape (3,), got {index_array.shape}")
    origin_array = _point3(origin, "origin")
    return origin_array + (index_array.astype(np.float64) + 0.5) * resolution_m


def dda_voxel_traversal(
    start: Sequence[float] | NDArray,
    end: Sequence[float] | NDArray,
    origin: Sequence[float] | NDArray,
    resolution_m: float,
    *,
    include_start: bool = True,
    include_end: bool = True,
) -> list[VoxelIndex]:
    """Visit voxels intersected by a line segment in segment order.

    When a ray crosses an exact edge or corner, all tied axes advance in one
    step. This produces a thin traversal without adding cells touched only at
    a zero-volume boundary.
    """
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("resolution_m must be finite and positive")
    start_point = _point3(start, "start")
    end_point = _point3(end, "end")
    map_origin = _point3(origin, "origin")

    start_index = np.asarray(
        point_to_voxel(start_point, map_origin, resolution_m), dtype=np.int64
    )
    end_index = np.asarray(
        point_to_voxel(end_point, map_origin, resolution_m), dtype=np.int64
    )
    current = start_index.copy()
    path: list[VoxelIndex] = [tuple(int(value) for value in current)]

    if np.array_equal(start_index, end_index):
        if not include_start or not include_end:
            return []
        return path

    direction = end_point - start_point
    step = np.sign(direction).astype(np.int64)
    t_max = np.full(3, np.inf, dtype=np.float64)
    t_delta = np.full(3, np.inf, dtype=np.float64)

    for axis in range(3):
        component = float(direction[axis])
        if component > 0.0:
            boundary = map_origin[axis] + (current[axis] + 1) * resolution_m
            t_max[axis] = (boundary - start_point[axis]) / component
            t_delta[axis] = resolution_m / component
        elif component < 0.0:
            boundary = map_origin[axis] + current[axis] * resolution_m
            t_max[axis] = (boundary - start_point[axis]) / component
            t_delta[axis] = -resolution_m / component

    maximum_steps = int(np.abs(end_index - start_index).sum()) + 1
    tie_tolerance = 1e-12
    for _ in range(maximum_steps):
        if np.array_equal(current, end_index):
            break
        next_t = float(np.min(t_max))
        tied_axes = np.flatnonzero(np.abs(t_max - next_t) <= tie_tolerance)
        if tied_axes.size == 0 or not math.isfinite(next_t):
            raise RuntimeError("DDA traversal could not advance toward endpoint")
        current[tied_axes] += step[tied_axes]
        t_max[tied_axes] += t_delta[tied_axes]
        path.append(tuple(int(value) for value in current))
    else:
        raise RuntimeError("DDA traversal exceeded its deterministic step bound")

    if not np.array_equal(current, end_index):
        raise RuntimeError("DDA traversal did not reach endpoint voxel")
    first = 0 if include_start else 1
    last = len(path) if include_end else len(path) - 1
    return path[first:max(first, last)]


def raycast_free_voxels(
    camera_origin: Sequence[float] | NDArray,
    endpoint: Sequence[float] | NDArray,
    map_origin: Sequence[float] | NDArray,
    resolution_m: float,
    truncation_distance_m: float,
) -> tuple[list[VoxelIndex], VoxelIndex]:
    """Return free ray cells and the occupied surface endpoint cell.

    Free traversal ends before the measured surface. The occupied endpoint is
    explicitly removed from the free result even when truncation ends inside
    the same voxel.
    """
    if not math.isfinite(truncation_distance_m) or truncation_distance_m < 0.0:
        raise ValueError("truncation_distance_m must be finite and non-negative")
    start = _point3(camera_origin, "camera_origin")
    surface = _point3(endpoint, "endpoint")
    fixed_origin = _point3(map_origin, "map_origin")
    endpoint_index = point_to_voxel(surface, fixed_origin, resolution_m)

    delta = surface - start
    distance = float(np.linalg.norm(delta))
    if distance <= truncation_distance_m or distance <= 1e-12:
        return [], endpoint_index

    free_end = surface - delta * (truncation_distance_m / distance)
    free_path = dda_voxel_traversal(
        start,
        free_end,
        fixed_origin,
        resolution_m,
        include_start=True,
        include_end=True,
    )
    free_path = [index for index in free_path if index != endpoint_index]
    return free_path, endpoint_index


def batch_raycast_free_voxels(
    camera_origin: Sequence[float] | NDArray,
    endpoints: NDArray,
    map_origin: Sequence[float] | NDArray,
    resolution_m: float,
    truncation_distance_m: float,
) -> BatchRaycastResult:
    """Run the same thin 3D DDA state machine over a batch of rays.

    NumPy advances every active ray together. Returned free and occupied voxel
    arrays are lexicographically sorted and unique; occupied endpoints are
    removed from the free array after all rays have been traversed.
    """
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("resolution_m must be finite and positive")
    if not math.isfinite(truncation_distance_m) or truncation_distance_m < 0.0:
        raise ValueError("truncation_distance_m must be finite and non-negative")
    start = _point3(camera_origin, "camera_origin")
    fixed_origin = _point3(map_origin, "map_origin")
    points = np.asarray(endpoints, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"endpoints must have shape (N, 3), got {points.shape}")
    if points.shape[0] == 0:
        empty = np.empty((0, 3), dtype=np.int64)
        return BatchRaycastResult(empty, empty.copy(), 0, 0, 0)

    finite = np.all(np.isfinite(points), axis=1)
    deltas = points - start
    distances = np.linalg.norm(deltas, axis=1)
    valid = finite & (distances > 1e-12)
    valid_points = points[valid]
    valid_deltas = deltas[valid]
    valid_distances = distances[valid]
    valid_count = int(valid_points.shape[0])
    rejected_count = int(points.shape[0] - valid_count)
    if valid_count == 0:
        empty = np.empty((0, 3), dtype=np.int64)
        return BatchRaycastResult(empty, empty.copy(), 0, rejected_count, 0)

    occupied = np.floor(
        (valid_points - fixed_origin) / resolution_m
    ).astype(np.int64)
    occupied = np.unique(occupied, axis=0)

    carve = valid_distances > truncation_distance_m
    if not np.any(carve):
        empty = np.empty((0, 3), dtype=np.int64)
        return BatchRaycastResult(
            empty, occupied, valid_count, rejected_count, 0
        )
    carve_deltas = valid_deltas[carve]
    carve_distances = valid_distances[carve]
    free_end = valid_points[carve] - carve_deltas * (
        truncation_distance_m / carve_distances
    )[:, np.newaxis]

    start_index = np.floor((start - fixed_origin) / resolution_m).astype(np.int64)
    current = np.tile(start_index, (free_end.shape[0], 1))
    end_indices = np.floor(
        (free_end - fixed_origin) / resolution_m
    ).astype(np.int64)
    directions = free_end - start
    step = np.sign(directions).astype(np.int64)
    t_max = np.full(directions.shape, np.inf, dtype=np.float64)
    t_delta = np.full(directions.shape, np.inf, dtype=np.float64)

    for axis in range(3):
        positive = directions[:, axis] > 0.0
        negative = directions[:, axis] < 0.0
        positive_boundary = (
            fixed_origin[axis] + (current[:, axis] + 1) * resolution_m
        )
        negative_boundary = fixed_origin[axis] + current[:, axis] * resolution_m
        t_max[positive, axis] = (
            positive_boundary[positive] - start[axis]
        ) / directions[positive, axis]
        t_delta[positive, axis] = resolution_m / directions[positive, axis]
        t_max[negative, axis] = (
            negative_boundary[negative] - start[axis]
        ) / directions[negative, axis]
        t_delta[negative, axis] = -resolution_m / directions[negative, axis]

    chunks = [current.copy()]
    step_bounds = np.abs(end_indices - current).sum(axis=1)
    maximum_steps = int(step_bounds.max(initial=0)) + 1
    tie_tolerance = 1e-12
    for _ in range(maximum_steps):
        active = np.any(current != end_indices, axis=1)
        if not np.any(active):
            break
        active_rows = np.flatnonzero(active)
        active_t_max = t_max[active]
        next_t = np.min(active_t_max, axis=1)
        tied = np.abs(active_t_max - next_t[:, np.newaxis]) <= tie_tolerance
        current_active = current[active]
        current_active += step[active] * tied
        current[active] = current_active
        t_max_active = np.where(
            tied, active_t_max + t_delta[active], active_t_max
        )
        t_max[active] = t_max_active
        chunks.append(current[active_rows].copy())
    else:
        raise RuntimeError("Batch DDA traversal exceeded its deterministic step bound")
    if np.any(current != end_indices):
        raise RuntimeError("Batch DDA traversal did not reach every endpoint voxel")

    free = np.unique(np.concatenate(chunks, axis=0), axis=0)
    occupied_rows = np.ascontiguousarray(occupied).view(
        np.dtype((np.void, occupied.dtype.itemsize * 3))
    ).reshape(-1)
    free_rows = np.ascontiguousarray(free).view(
        np.dtype((np.void, free.dtype.itemsize * 3))
    ).reshape(-1)
    free = free[~np.isin(free_rows, occupied_rows)]
    return BatchRaycastResult(
        free_voxels=free,
        occupied_voxels=occupied,
        valid_rays=valid_count,
        rejected_rays=rejected_count,
        traversed_free_cells=int(sum(chunk.shape[0] for chunk in chunks)),
    )
