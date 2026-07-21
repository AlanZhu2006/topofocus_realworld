"""Pure metrics for interpreting and comparing 2-D Hub maps.

The metrics intentionally avoid claiming navigation accuracy: no ground-truth
floor plan is available for the live robot spools.  They quantify observable
properties such as coverage, obstacle density, fragmentation and cell churn so
parameter sweeps and operator-present moved runs can be compared honestly.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class MapQualityMetrics:
    total_cells: int
    explored_cells: int
    obstacle_cells: int
    semantic_cells: int
    obstacle_explored_ratio: float
    explored_fraction: float
    obstacle_components: int
    largest_obstacle_component_cells: int
    isolated_obstacle_cells: int
    thin_obstacle_cells: int
    explored_boundary_cells: int

    def to_dict(self) -> dict[str, int | float]:
        return asdict(self)


def _neighbor_count(mask: np.ndarray, *, diagonal: bool) -> np.ndarray:
    padded = np.pad(mask.astype(np.uint8), 1)
    height, width = mask.shape
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if diagonal:
        offsets += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    count = np.zeros(mask.shape, dtype=np.uint8)
    for row_delta, col_delta in offsets:
        row_start = 1 + row_delta
        col_start = 1 + col_delta
        count += padded[
            row_start : row_start + height,
            col_start : col_start + width,
        ]
    return count


def compute_map_quality(grid: np.ndarray) -> MapQualityMetrics:
    evidence = np.asarray(grid)
    if evidence.ndim != 3 or evidence.shape[0] < 2:
        raise ValueError(
            f"grid must have shape (channels>=2,H,W), got {evidence.shape}"
        )
    if not np.all(np.isfinite(evidence)):
        raise ValueError("grid contains non-finite values")

    obstacle = evidence[0] > 0.5
    explored = evidence[1] > 0.5
    semantic = (
        evidence[2:].max(axis=0) > 0.1
        if evidence.shape[0] > 2
        else np.zeros_like(obstacle)
    )
    obstacle_count = int(np.count_nonzero(obstacle))
    explored_count = int(np.count_nonzero(explored))
    total = int(obstacle.size)

    if obstacle_count:
        component_count, labels = cv2.connectedComponents(
            obstacle.astype(np.uint8), connectivity=8
        )
        component_sizes = np.bincount(labels.ravel())[1:]
        components = int(component_count - 1)
        largest = int(component_sizes.max()) if component_sizes.size else 0
    else:
        components = 0
        largest = 0

    obstacle_neighbors = _neighbor_count(obstacle, diagonal=True)
    unknown = ~explored & ~obstacle
    unknown_neighbors = _neighbor_count(unknown, diagonal=False)
    return MapQualityMetrics(
        total_cells=total,
        explored_cells=explored_count,
        obstacle_cells=obstacle_count,
        semantic_cells=int(np.count_nonzero(semantic)),
        obstacle_explored_ratio=(
            round(obstacle_count / explored_count, 8) if explored_count else 0.0
        ),
        explored_fraction=round(explored_count / total, 8),
        obstacle_components=components,
        largest_obstacle_component_cells=largest,
        isolated_obstacle_cells=int(
            np.count_nonzero(obstacle & (obstacle_neighbors == 0))
        ),
        thin_obstacle_cells=int(
            np.count_nonzero(obstacle & (obstacle_neighbors <= 2))
        ),
        explored_boundary_cells=int(
            np.count_nonzero(explored & (unknown_neighbors > 0))
        ),
    )


def compare_map_grids(before: np.ndarray, after: np.ndarray) -> dict[str, int | float]:
    first = np.asarray(before)
    second = np.asarray(after)
    if first.shape != second.shape:
        raise ValueError(
            f"map grid shape changed from {first.shape} to {second.shape}"
        )
    if first.ndim != 3 or first.shape[0] < 2:
        raise ValueError("map grids must have shape (channels>=2,H,W)")
    before_obstacle = first[0] > 0.5
    after_obstacle = second[0] > 0.5
    before_explored = first[1] > 0.5
    after_explored = second[1] > 0.5
    changed = np.any(np.abs(second - first) > 1e-6, axis=0)
    return {
        "changed_xy_cells": int(np.count_nonzero(changed)),
        "newly_explored_cells": int(
            np.count_nonzero(after_explored & ~before_explored)
        ),
        "new_obstacle_cells": int(
            np.count_nonzero(after_obstacle & ~before_obstacle)
        ),
        "cleared_obstacle_cells": int(
            np.count_nonzero(before_obstacle & ~after_obstacle)
        ),
        "obstacle_jaccard": round(
            float(np.count_nonzero(before_obstacle & after_obstacle))
            / max(1, int(np.count_nonzero(before_obstacle | after_obstacle))),
            8,
        ),
    }
