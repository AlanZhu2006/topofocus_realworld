"""Pure geometry helpers for robot-local navigation occupancy grids."""
from __future__ import annotations

from dataclasses import replace
import math
from typing import Any

import numpy as np


def _matrix4(value: Any) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("target_T_base must have shape (4, 4)")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("target_T_base must be finite")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError("target_T_base must be homogeneous")
    return matrix


def clear_current_footprint(
    bev: Any,
    target_T_base: Any,
    *,
    shape: str,
    front_m: float = 0.0,
    rear_m: float = 0.0,
    half_width_m: float = 0.0,
    radius_m: float = 0.0,
) -> tuple[Any, int]:
    """Pad a BEV to the measured base and clear only its current footprint.

    A forward camera can produce a tightly cropped grid which excludes the
    base.  The robot's currently occupied physical footprint is known to be
    collision-free for that robot, so a rolling costmap may clear exactly that
    footprint.  Unknown cells outside it remain unknown.

    The returned object is a dataclass replacement; the source BEV and its
    arrays remain unchanged for provenance-preserving map serialization.
    """

    if shape not in {"rectangle", "circle"}:
        raise ValueError("footprint shape must be rectangle or circle")
    dimensions = (front_m, rear_m, half_width_m, radius_m)
    if not all(math.isfinite(value) and value >= 0.0 for value in dimensions):
        raise ValueError("footprint dimensions must be finite and non-negative")
    if shape == "rectangle" and min(front_m, rear_m, half_width_m) <= 0.0:
        raise ValueError("rectangular footprint dimensions must be positive")
    if shape == "circle" and radius_m <= 0.0:
        raise ValueError("circular footprint radius must be positive")
    resolution = float(bev.resolution_m)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("BEV resolution must be finite and positive")
    if int(bev.width) <= 0 or int(bev.height) <= 0:
        return bev, 0

    transform = _matrix4(target_T_base)
    base_x = float(transform[0, 3])
    base_y = float(transform[1, 3])
    base_x_axis = transform[:2, 0]
    base_y_axis = transform[:2, 1]
    if shape == "rectangle":
        local_corners = np.asarray(
            [
                [front_m, half_width_m],
                [front_m, -half_width_m],
                [-rear_m, half_width_m],
                [-rear_m, -half_width_m],
            ],
            dtype=np.float64,
        )
        world_corners = (
            np.asarray([base_x, base_y])
            + local_corners[:, :1] * base_x_axis
            + local_corners[:, 1:] * base_y_axis
        )
        minimum_xy = np.min(world_corners, axis=0)
        maximum_xy = np.max(world_corners, axis=0)
    else:
        minimum_xy = np.asarray(
            [base_x - radius_m, base_y - radius_m], dtype=np.float64
        )
        maximum_xy = np.asarray(
            [base_x + radius_m, base_y + radius_m], dtype=np.float64
        )

    old_origin = np.asarray(bev.origin_xy, dtype=np.float64)
    minimum_cell = np.floor((minimum_xy - old_origin) / resolution).astype(int)
    maximum_cell = np.floor((maximum_xy - old_origin) / resolution).astype(int)
    pad_left = max(0, -int(minimum_cell[0]))
    pad_right = max(0, int(maximum_cell[0]) - int(bev.width) + 1)
    pad_bottom = max(0, -int(minimum_cell[1]))
    pad_top = max(0, int(maximum_cell[1]) - int(bev.height) + 1)
    padding = ((pad_bottom, pad_top), (pad_left, pad_right))

    occupancy_probability = np.pad(
        bev.occupancy_probability,
        padding,
        mode="constant",
        constant_values=np.nan,
    )
    free_probability = np.pad(
        bev.free_probability,
        padding,
        mode="constant",
        constant_values=np.nan,
    )
    explored = np.pad(
        bev.explored, padding, mode="constant", constant_values=0
    )
    occupancy_grid = np.pad(
        bev.occupancy_grid, padding, mode="constant", constant_values=-1
    )
    height_min = np.pad(
        bev.height_min, padding, mode="constant", constant_values=np.nan
    )
    height_max = np.pad(
        bev.height_max, padding, mode="constant", constant_values=np.nan
    )
    origin_xy = old_origin - np.asarray(
        [pad_left, pad_bottom], dtype=np.float64
    ) * resolution

    rows, columns = np.indices(occupancy_grid.shape)
    world_x = origin_xy[0] + (columns + 0.5) * resolution
    world_y = origin_xy[1] + (rows + 0.5) * resolution
    delta_x = world_x - base_x
    delta_y = world_y - base_y
    local_x = delta_x * base_x_axis[0] + delta_y * base_x_axis[1]
    local_y = delta_x * base_y_axis[0] + delta_y * base_y_axis[1]
    if shape == "rectangle":
        mask = (
            (local_x >= -rear_m)
            & (local_x <= front_m)
            & (np.abs(local_y) <= half_width_m)
        )
    else:
        mask = local_x * local_x + local_y * local_y <= radius_m * radius_m

    cleared_cells = int(np.count_nonzero(mask & (occupancy_grid != 0)))
    occupancy_probability[mask] = np.float32(0.0)
    free_probability[mask] = np.float32(1.0)
    explored[mask] = np.uint8(1)
    occupancy_grid[mask] = np.int8(0)
    height_min[mask] = np.nan
    height_max[mask] = np.nan
    return (
        replace(
            bev,
            occupancy_probability=occupancy_probability,
            free_probability=free_probability,
            explored=explored,
            occupancy_grid=occupancy_grid,
            height_min=height_min,
            height_max=height_max,
            origin_xy=origin_xy,
        ),
        cleared_cells,
    )
