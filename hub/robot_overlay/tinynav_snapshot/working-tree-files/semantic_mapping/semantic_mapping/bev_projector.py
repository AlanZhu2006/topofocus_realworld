"""Height-aware projection of sparse occupancy voxels into a 2D BEV."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from semantic_mapping.occupancy_voxel_map import (
    OccupancyState,
    SparseOccupancyVoxelMap,
)


@dataclass(frozen=True)
class BEVProjectionConfig:
    """Z-up projection bands relative to a configured ground height."""

    resolution_m: float = 0.05
    ground_z: float = 0.0
    ground_min_z_relative: float = -0.10
    ground_max_z_relative: float = 0.15
    collision_min_z_relative: float = 0.10
    collision_max_z_relative: float = 0.75
    ignore_above_z_relative: float = 1.80
    padding_cells: int = 1
    exclude_ground_band_from_collision: bool = True

    def __post_init__(self) -> None:
        values = (
            self.resolution_m,
            self.ground_z,
            self.ground_min_z_relative,
            self.ground_max_z_relative,
            self.collision_min_z_relative,
            self.collision_max_z_relative,
            self.ignore_above_z_relative,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("BEV projection values must be finite")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.ground_min_z_relative >= self.ground_max_z_relative:
            raise ValueError("Ground band must have positive height")
        if self.collision_min_z_relative >= self.collision_max_z_relative:
            raise ValueError("Collision band must have positive height")
        if self.ignore_above_z_relative < self.collision_max_z_relative:
            raise ValueError("ignore_above_z_relative must include collision band")
        if self.padding_cells < 0:
            raise ValueError("padding_cells must be non-negative")


@dataclass(frozen=True)
class OccupancyBEV:
    """Planner-ready geometry channels with one shared map geometry."""

    occupancy_probability: NDArray[np.float32]
    free_probability: NDArray[np.float32]
    explored: NDArray[np.uint8]
    occupancy_grid: NDArray[np.int8]
    height_min: NDArray[np.float32]
    height_max: NDArray[np.float32]
    origin_xy: NDArray[np.float64]
    resolution_m: float

    @property
    def height(self) -> int:
        return int(self.occupancy_grid.shape[0])

    @property
    def width(self) -> int:
        return int(self.occupancy_grid.shape[1])


def project_occupancy_to_bev(
    voxel_map: SparseOccupancyVoxelMap,
    config: BEVProjectionConfig,
) -> OccupancyBEV:
    """Project observed voxel columns using ground and collision height bands."""
    fixed_origin = voxel_map.origin
    indices, log_odds = voxel_map.projection_arrays()
    if indices.shape[0] == 0:
        empty_float = np.empty((0, 0), dtype=np.float32)
        return OccupancyBEV(
            occupancy_probability=empty_float.copy(),
            free_probability=empty_float.copy(),
            explored=np.empty((0, 0), dtype=np.uint8),
            occupancy_grid=np.empty((0, 0), dtype=np.int8),
            height_min=empty_float.copy(),
            height_max=empty_float.copy(),
            origin_xy=fixed_origin[:2].copy(),
            resolution_m=config.resolution_m,
        )

    centers = fixed_origin + (
        indices.astype(np.float64) + 0.5
    ) * voxel_map.config.resolution_m
    z_relative = centers[:, 2] - config.ground_z
    height_mask = (
        (z_relative >= config.ground_min_z_relative)
        & (z_relative <= config.ignore_above_z_relative)
    )
    if not np.any(height_mask):
        empty_float = np.empty((0, 0), dtype=np.float32)
        return OccupancyBEV(
            occupancy_probability=empty_float.copy(),
            free_probability=empty_float.copy(),
            explored=np.empty((0, 0), dtype=np.uint8),
            occupancy_grid=np.empty((0, 0), dtype=np.int8),
            height_min=empty_float.copy(),
            height_max=empty_float.copy(),
            origin_xy=fixed_origin[:2].copy(),
            resolution_m=config.resolution_m,
        )

    centers = centers[height_mask]
    z_relative = z_relative[height_mask]
    log_odds = log_odds[height_mask]
    cells = np.floor(
        (centers[:, :2] - fixed_origin[:2]) / config.resolution_m
    ).astype(np.int64)
    probabilities = 1.0 / (1.0 + np.exp(-log_odds))
    occupied_voxels = probabilities > voxel_map.config.occupied_threshold

    min_x = int(np.min(cells[:, 0])) - config.padding_cells
    max_x = int(np.max(cells[:, 0])) + config.padding_cells
    min_y = int(np.min(cells[:, 1])) - config.padding_cells
    max_y = int(np.max(cells[:, 1])) + config.padding_cells
    width = max_x - min_x + 1
    height = max_y - min_y + 1

    occupancy = np.full((height, width), -np.inf, dtype=np.float32)
    free = np.full((height, width), -np.inf, dtype=np.float32)
    explored = np.zeros((height, width), dtype=np.uint8)
    height_min = np.full((height, width), np.inf, dtype=np.float32)
    height_max = np.full((height, width), -np.inf, dtype=np.float32)
    flat_cells = (cells[:, 1] - min_y) * width + (cells[:, 0] - min_x)
    explored.reshape(-1)[flat_cells] = 1

    in_ground_band = (
        (z_relative >= config.ground_min_z_relative)
        & (z_relative <= config.ground_max_z_relative)
    )
    in_collision_band = (
        (z_relative >= config.collision_min_z_relative)
        & (z_relative <= config.collision_max_z_relative)
    )
    if config.exclude_ground_band_from_collision:
        in_collision_band &= ~in_ground_band
    ground_updates = in_ground_band & ~in_collision_band

    occupancy_flat = occupancy.reshape(-1)
    free_flat = free.reshape(-1)
    np.maximum.at(
        occupancy_flat,
        flat_cells[in_collision_band],
        probabilities[in_collision_band],
    )
    np.maximum.at(
        free_flat,
        flat_cells[in_collision_band],
        1.0 - probabilities[in_collision_band],
    )
    ground_free = np.where(
        occupied_voxels[ground_updates],
        probabilities[ground_updates],
        1.0 - probabilities[ground_updates],
    )
    ground_occupancy = np.where(
        occupied_voxels[ground_updates],
        1.0 - probabilities[ground_updates],
        probabilities[ground_updates],
    )
    np.maximum.at(
        free_flat, flat_cells[ground_updates], ground_free
    )
    np.maximum.at(
        occupancy_flat, flat_cells[ground_updates], ground_occupancy
    )

    height_min_flat = height_min.reshape(-1)
    height_max_flat = height_max.reshape(-1)
    np.minimum.at(
        height_min_flat,
        flat_cells[occupied_voxels],
        z_relative[occupied_voxels],
    )
    np.maximum.at(
        height_max_flat,
        flat_cells[occupied_voxels],
        z_relative[occupied_voxels],
    )
    occupancy[~np.isfinite(occupancy)] = np.nan
    free[~np.isfinite(free)] = np.nan
    height_min[~np.isfinite(height_min)] = np.nan
    height_max[~np.isfinite(height_max)] = np.nan

    occupancy_grid = np.full((height, width), OccupancyState.UNKNOWN, dtype=np.int8)
    occupied_mask = np.isfinite(occupancy) & (
        occupancy > voxel_map.config.occupied_threshold
    )
    free_mask = (
        ~occupied_mask
        & np.isfinite(free)
        & (free > (1.0 - voxel_map.config.free_threshold))
    )
    occupancy_grid[free_mask] = OccupancyState.FREE
    occupancy_grid[occupied_mask] = OccupancyState.OCCUPIED

    origin_xy = fixed_origin[:2] + np.asarray(
        [min_x, min_y], dtype=np.float64
    ) * config.resolution_m
    return OccupancyBEV(
        occupancy_probability=occupancy,
        free_probability=free,
        explored=explored,
        occupancy_grid=occupancy_grid,
        height_min=height_min,
        height_max=height_max,
        origin_xy=origin_xy,
        resolution_m=config.resolution_m,
    )
