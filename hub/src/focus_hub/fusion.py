"""Shared-map fusion and sequential frontier allocation (source-derived).

Upstream fuses the two agents' grids with element-wise ``torch.max`` and
allocates exploration targets sequentially: agent 0 chooses among annotated
frontier candidates, the chosen candidate is removed, then agent 1 chooses
from the remainder — guaranteeing distinct targets.  These helpers port both
rules for the hub.

Fusion requires the maps to already live on an identical grid (same origin,
resolution and shape) in the *same* shared_world frame.  Establishing that
frame physically (per-robot ``T_shared_world_robot_map`` with independent
verification) is the G4 calibration work; this module only implements the
machinery that runs after it.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frontiers import Frontier


def fuse_grids(grids: list[np.ndarray]) -> np.ndarray:
    """Element-wise max fusion of per-robot channel grids (upstream rule)."""
    if not grids:
        raise ValueError("nothing to fuse")
    shape = grids[0].shape
    for grid in grids[1:]:
        if grid.shape != shape:
            raise ValueError(f"grid shapes differ: {grid.shape} vs {shape}")
    return np.maximum.reduce([np.asarray(g) for g in grids])


def align_and_fuse_grids(
    grids: list[np.ndarray],
    origins_xy_m: list[tuple[float, float]],
    resolution_m: float,
) -> tuple[np.ndarray, tuple[float, float]]:
    """Fuses per-robot grids that share the same *frame and resolution* but
    have different origins/extents -- the real situation two independently
    run `hub_pipeline_daemon.py` instances produce after G4 calibration:
    each robot's poses land in the same shared_world frame (the calibrated
    robot's sender applies the transform before upload), but each daemon
    still independently picks its own map bounding box from wherever its
    own robot happened to start, so the two grids are not `fuse_grids`'s
    "already identical grid" precondition.

    Computes the union bounding box in world coordinates, places each
    robot's own grid into a same-sized canvas at the correct integer pixel
    offset (a plain slice assignment -- no resampling/interpolation needed
    since both grids already share resolution_m and axis alignment, only
    origin differs), then reuses `fuse_grids` for the actual element-wise
    max. Cells outside a given robot's original coverage are left at zero
    in that robot's canvas, matching CentralSemanticMap's own
    "not yet observed" convention, so they don't wrongly override the other
    robot's real data at the same union cell.
    """
    if not grids:
        raise ValueError("nothing to fuse")
    if len(grids) != len(origins_xy_m):
        raise ValueError("grids and origins_xy_m must be the same length")
    channels = grids[0].shape[0]
    for grid in grids[1:]:
        if grid.shape[0] != channels:
            raise ValueError(f"channel counts differ: {grid.shape[0]} vs {channels}")

    min_x = min(origin[0] for origin in origins_xy_m)
    min_y = min(origin[1] for origin in origins_xy_m)
    max_x = max(origin[0] + grid.shape[2] * resolution_m for grid, origin in zip(grids, origins_xy_m))
    max_y = max(origin[1] + grid.shape[1] * resolution_m for grid, origin in zip(grids, origins_xy_m))

    fused_w = int(np.ceil((max_x - min_x) / resolution_m))
    fused_h = int(np.ceil((max_y - min_y) / resolution_m))
    fused_origin_xy_m = (min_x, min_y)

    placed = []
    for grid, origin in zip(grids, origins_xy_m):
        canvas = np.zeros((channels, fused_h, fused_w), dtype=grid.dtype)
        col_off = int(round((origin[0] - min_x) / resolution_m))
        row_off = int(round((origin[1] - min_y) / resolution_m))
        h, w = grid.shape[1], grid.shape[2]
        canvas[:, row_off:row_off + h, col_off:col_off + w] = grid
        placed.append(canvas)

    return fuse_grids(placed), fused_origin_xy_m


@dataclass(frozen=True)
class Allocation:
    robot_id: str
    frontier: Frontier
    source: str
    probabilities: dict[str, float]


def allocate_frontiers_sequential(
    robot_ids: list[str],
    frontiers: list[Frontier],
    choose,
) -> list[Allocation]:
    """Allocate distinct frontiers to robots in order (upstream rule).

    ``choose(robot_id, remaining_frontiers)`` returns an object with
    ``frontier``, ``source`` and ``probabilities`` attributes (the
    ``FrontierChoice`` shape).  Each chosen frontier is removed before the
    next robot chooses; runs out of candidates -> the remaining robots get no
    allocation (callers decide between HOLD or reuse).
    """
    remaining = list(frontiers)
    allocations: list[Allocation] = []
    for robot_id in robot_ids:
        if not remaining:
            break
        choice = choose(robot_id, remaining)
        chosen = choice.frontier
        if all(f.frontier_id != chosen.frontier_id for f in remaining):
            raise ValueError(f"chooser returned a frontier not in the remaining set: {chosen}")
        remaining = [f for f in remaining if f.frontier_id != chosen.frontier_id]
        allocations.append(Allocation(
            robot_id=robot_id,
            frontier=chosen,
            source=choice.source,
            probabilities=dict(choice.probabilities),
        ))
    return allocations
