"""Height-aware projection of confirmed semantic voxels into a 2D BEV."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from semantic_mapping.semantic_voxel_map import SparseSemanticVoxelMap


@dataclass(frozen=True)
class SemanticBEVGrid:
    """Metric 2D grid shared with the independent occupancy BEV when available."""

    origin_xy: tuple[float, float]
    resolution_m: float
    width: int
    height: int

    def __post_init__(self) -> None:
        if len(self.origin_xy) != 2 or not all(
            math.isfinite(value) for value in self.origin_xy
        ):
            raise ValueError("origin_xy must contain two finite values")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be finite and positive")
        if self.width < 0 or self.height < 0:
            raise ValueError("BEV grid dimensions must be non-negative")


@dataclass(frozen=True)
class SemanticBEVProjectionConfig:
    """Z-up semantic and floor-support bands relative to a ground height."""

    resolution_m: float = 0.05
    ground_z: float = 0.0
    ground_min_z_relative: float = -0.10
    ground_max_z_relative: float = 0.15
    semantic_min_z_relative: float = 0.05
    semantic_max_z_relative: float = 1.50
    ignore_above_z_relative: float = 1.80
    padding_cells: int = 1
    min_cell_confidence: float = 0.50

    def __post_init__(self) -> None:
        values = (
            self.resolution_m,
            self.ground_z,
            self.ground_min_z_relative,
            self.ground_max_z_relative,
            self.semantic_min_z_relative,
            self.semantic_max_z_relative,
            self.ignore_above_z_relative,
            self.min_cell_confidence,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Semantic BEV projection values must be finite")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.ground_min_z_relative >= self.ground_max_z_relative:
            raise ValueError("Ground band must have positive height")
        if self.semantic_min_z_relative >= self.semantic_max_z_relative:
            raise ValueError("Semantic band must have positive height")
        if self.semantic_max_z_relative > self.ignore_above_z_relative:
            raise ValueError("ignore_above_z_relative must include semantic band")
        if self.padding_cells < 0:
            raise ValueError("padding_cells must be non-negative")
        if not 0.0 <= self.min_cell_confidence <= 1.0:
            raise ValueError("min_cell_confidence must be in [0, 1]")


@dataclass(frozen=True)
class SemanticBEV:
    """Semantic channels aligned to one metric bird's-eye-view grid."""

    semantic_scores: NDArray[np.float32]
    semantic_label: NDArray[np.uint8]
    semantic_confidence: NDArray[np.float32]
    explored: NDArray[np.uint8]
    height_min: NDArray[np.float32]
    height_max: NDArray[np.float32]
    origin_xy: NDArray[np.float64]
    resolution_m: float
    ground_z: float

    @property
    def height(self) -> int:
        return int(self.semantic_label.shape[0])

    @property
    def width(self) -> int:
        return int(self.semantic_label.shape[1])


def project_semantic_to_bev(
    semantic_map: SparseSemanticVoxelMap,
    config: SemanticBEVProjectionConfig,
    *,
    grid: SemanticBEVGrid | None = None,
    floor_class_id: int | None = None,
) -> SemanticBEV:
    """Project semantic surface evidence without replacing geometry occupancy.

    Normal semantic classes are aggregated only within the semantic height band.
    Confirmed floor voxels in the ground band provide a fallback label for cells
    that contain no object/wall semantic evidence. This keeps floor support from
    overriding an object that occupies the same XY column.
    """
    centers, labels, scores = semantic_map.confirmed_score_arrays()
    class_count = semantic_map.config.class_count
    if centers.shape[0] == 0:
        return _empty_bev(class_count, config, grid)
    if floor_class_id is not None and not 0 <= floor_class_id < class_count:
        raise ValueError("floor_class_id must fit semantic class count")

    z_relative = centers[:, 2].astype(np.float64) - config.ground_z
    in_semantic_band = (
        (z_relative >= config.semantic_min_z_relative)
        & (z_relative <= config.semantic_max_z_relative)
    )
    in_ground_band = (
        (z_relative >= config.ground_min_z_relative)
        & (z_relative <= config.ground_max_z_relative)
    )
    floor_support = (
        np.zeros(centers.shape[0], dtype=np.bool_)
        if floor_class_id is None
        else in_ground_band & (labels == floor_class_id)
    )
    eligible = in_semantic_band | floor_support
    if not np.any(eligible):
        return _empty_bev(class_count, config, grid)

    centers = centers[eligible]
    scores = scores[eligible]
    z_relative = z_relative[eligible]
    semantic_mask = in_semantic_band[eligible]
    floor_mask = floor_support[eligible]
    grid = _resolve_grid(semantic_map, centers, config, grid)
    if grid.width == 0 or grid.height == 0:
        return _empty_bev(class_count, config, grid)

    origin_xy = np.asarray(grid.origin_xy, dtype=np.float64)
    cells = np.floor(
        (centers[:, :2].astype(np.float64) - origin_xy) / grid.resolution_m
    ).astype(np.int64)
    in_grid = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < grid.width)
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < grid.height)
    )
    if not np.any(in_grid):
        return _empty_bev(class_count, config, grid)

    cells = cells[in_grid]
    scores = scores[in_grid]
    z_relative = z_relative[in_grid]
    semantic_mask = semantic_mask[in_grid]
    floor_mask = floor_mask[in_grid]
    flat_cells = cells[:, 1] * grid.width + cells[:, 0]
    cell_count = grid.width * grid.height
    object_scores = np.zeros((cell_count, class_count), dtype=np.float32)
    floor_scores = np.zeros((cell_count, class_count), dtype=np.float32)
    if np.any(semantic_mask):
        np.add.at(object_scores, flat_cells[semantic_mask], scores[semantic_mask])
    if floor_class_id is not None and np.any(floor_mask):
        np.add.at(
            floor_scores[:, floor_class_id],
            flat_cells[floor_mask],
            scores[floor_mask, floor_class_id],
        )
    object_strength = object_scores.sum(axis=1)
    use_floor = (object_strength <= 0.0) & (floor_scores.sum(axis=1) > 0.0)
    accumulated = object_scores
    accumulated[use_floor] = floor_scores[use_floor]
    strength = accumulated.sum(axis=1)
    explored_flat = strength > 0.0
    normalized = np.zeros_like(accumulated)
    normalized[explored_flat] = (
        accumulated[explored_flat] / strength[explored_flat, None]
    )
    labels_flat = np.full(
        cell_count, semantic_map.config.unknown_class_id, dtype=np.uint8
    )
    confidence_flat = np.full(cell_count, np.nan, dtype=np.float32)
    if np.any(explored_flat):
        winning = np.argmax(normalized[explored_flat], axis=1)
        winning_confidence = normalized[explored_flat, winning]
        accepted = winning_confidence >= config.min_cell_confidence
        positions = np.flatnonzero(explored_flat)
        labels_flat[positions[accepted]] = winning[accepted].astype(np.uint8)
        confidence_flat[positions] = winning_confidence.astype(np.float32)

    object_min, object_max = _height_extrema(
        flat_cells, z_relative, semantic_mask, cell_count
    )
    floor_min, floor_max = _height_extrema(
        flat_cells, z_relative, floor_mask, cell_count
    )
    height_min = object_min
    height_max = object_max
    height_min[use_floor] = floor_min[use_floor]
    height_max[use_floor] = floor_max[use_floor]
    height_min[~explored_flat] = np.nan
    height_max[~explored_flat] = np.nan

    shape = (grid.height, grid.width)
    return SemanticBEV(
        semantic_scores=normalized.reshape(grid.height, grid.width, class_count),
        semantic_label=labels_flat.reshape(shape),
        semantic_confidence=confidence_flat.reshape(shape),
        explored=explored_flat.reshape(shape).astype(np.uint8),
        height_min=height_min.reshape(shape),
        height_max=height_max.reshape(shape),
        origin_xy=origin_xy,
        resolution_m=grid.resolution_m,
        ground_z=float(config.ground_z),
    )


def _resolve_grid(
    semantic_map: SparseSemanticVoxelMap,
    centers: NDArray[np.float32],
    config: SemanticBEVProjectionConfig,
    grid: SemanticBEVGrid | None,
) -> SemanticBEVGrid:
    if grid is not None:
        return grid
    map_origin = np.asarray(semantic_map.config.origin_xyz, dtype=np.float64)
    cells = np.floor(
        (centers[:, :2].astype(np.float64) - map_origin[:2]) / config.resolution_m
    ).astype(np.int64)
    min_x = int(np.min(cells[:, 0])) - config.padding_cells
    max_x = int(np.max(cells[:, 0])) + config.padding_cells
    min_y = int(np.min(cells[:, 1])) - config.padding_cells
    max_y = int(np.max(cells[:, 1])) + config.padding_cells
    return SemanticBEVGrid(
        origin_xy=tuple(
            map_origin[:2]
            + np.asarray([min_x, min_y], dtype=np.float64) * config.resolution_m
        ),
        resolution_m=config.resolution_m,
        width=max_x - min_x + 1,
        height=max_y - min_y + 1,
    )


def _height_extrema(
    flat_cells: NDArray[np.int64],
    z_relative: NDArray[np.float64],
    mask: NDArray[np.bool_],
    cell_count: int,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    minimum = np.full(cell_count, np.inf, dtype=np.float32)
    maximum = np.full(cell_count, -np.inf, dtype=np.float32)
    if np.any(mask):
        np.minimum.at(minimum, flat_cells[mask], z_relative[mask])
        np.maximum.at(maximum, flat_cells[mask], z_relative[mask])
    minimum[~np.isfinite(minimum)] = np.nan
    maximum[~np.isfinite(maximum)] = np.nan
    return minimum, maximum


def _empty_bev(
    class_count: int,
    config: SemanticBEVProjectionConfig,
    grid: SemanticBEVGrid | None,
) -> SemanticBEV:
    if grid is None:
        height = 0
        width = 0
        origin_xy = np.zeros(2, dtype=np.float64)
        resolution_m = config.resolution_m
    else:
        height = grid.height
        width = grid.width
        origin_xy = np.asarray(grid.origin_xy, dtype=np.float64)
        resolution_m = grid.resolution_m
    shape = (height, width)
    return SemanticBEV(
        semantic_scores=np.zeros((height, width, class_count), dtype=np.float32),
        semantic_label=np.zeros(shape, dtype=np.uint8),
        semantic_confidence=np.full(shape, np.nan, dtype=np.float32),
        explored=np.zeros(shape, dtype=np.uint8),
        height_min=np.full(shape, np.nan, dtype=np.float32),
        height_max=np.full(shape, np.nan, dtype=np.float32),
        origin_xy=origin_xy,
        resolution_m=resolution_m,
        ground_z=float(config.ground_z),
    )
