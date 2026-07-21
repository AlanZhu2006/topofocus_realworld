"""Deterministic Foxglove/map visualization helpers.

The map itself stores evidence channels, not display pixels.  In particular,
downsampling an already-colorized RGBA image blends unrelated categorical
colors and can create purple/yellow patches which do not correspond to any
semantic class.  This module therefore reduces evidence first and assigns one
exact display color afterwards.

These helpers are read-only and have no robot-control dependency.
"""
from __future__ import annotations

import numpy as np

from .frontiers import _category_palette


UNKNOWN_RGB = (96, 96, 96)
FREE_RGB = (235, 235, 235)
OBSTACLE_RGB = (40, 40, 40)


def downsample_evidence_grid(grid: np.ndarray, factor: int) -> np.ndarray:
    """Max-reduce spatial evidence blocks by an integer factor.

    Max reduction preserves the mapper's evidence semantics and, unlike RGBA
    averaging, cannot invent a display color between two categories.  Partial
    rows/columns are cropped consistently with the relay's previous behavior.
    """
    evidence = np.asarray(grid, dtype=np.float32)
    if evidence.ndim != 3 or evidence.shape[0] < 2:
        raise ValueError(
            f"grid must have shape (channels>=2,H,W), got {evidence.shape}"
        )
    if factor < 1:
        raise ValueError("downsample factor must be at least one")
    if factor == 1:
        return evidence
    _, height, width = evidence.shape
    cropped_height = height - height % factor
    cropped_width = width - width % factor
    if cropped_height == 0 or cropped_width == 0:
        raise ValueError(
            f"downsample factor {factor} exceeds grid shape {(height, width)}"
        )
    cropped = evidence[:, :cropped_height, :cropped_width]
    return cropped.reshape(
        evidence.shape[0],
        cropped_height // factor,
        factor,
        cropped_width // factor,
        factor,
    ).max(axis=(2, 4))


def colorize_geometry_grid(grid: np.ndarray) -> np.ndarray:
    """Render only unknown/free/obstacle geometry as exact RGBA colors."""
    evidence = np.asarray(grid)
    if evidence.ndim != 3 or evidence.shape[0] < 2:
        raise ValueError(
            f"grid must have shape (channels>=2,H,W), got {evidence.shape}"
        )
    obstacle = evidence[0] > 0.5
    explored = evidence[1] > 0.5
    height, width = obstacle.shape
    rgb = np.full((height, width, 3), UNKNOWN_RGB, dtype=np.uint8)
    rgb[explored] = FREE_RGB
    rgb[obstacle] = OBSTACLE_RGB
    alpha = np.full((height, width, 1), 255, dtype=np.uint8)
    alpha[~explored & ~obstacle] = 60
    return np.concatenate((rgb, alpha), axis=-1)


def colorize_semantic_grid(
    grid: np.ndarray, category_names: tuple[str, ...]
) -> np.ndarray:
    """Render geometry plus exact categorical colors for semantic evidence."""
    rgba = colorize_geometry_grid(grid)
    if not category_names:
        return rgba
    evidence = np.asarray(grid)
    categories = evidence[2 : 2 + len(category_names)]
    if categories.shape[0] != len(category_names):
        raise ValueError(
            f"grid has {evidence.shape[0] - 2} semantic channels but "
            f"{len(category_names)} names were supplied"
        )
    has_category = categories.max(axis=0) > 0.1
    best_category = categories.argmax(axis=0)
    palette_bgr = _category_palette(len(category_names))
    rgba[has_category, :3] = palette_bgr[
        best_category[has_category]
    ][:, ::-1]
    rgba[has_category, 3] = 255
    return rgba


def semantic_evidence_cells(grid: np.ndarray, threshold: float = 0.1) -> int:
    """Count XY cells carrying at least one semantic-category observation."""
    evidence = np.asarray(grid)
    if evidence.ndim != 3 or evidence.shape[0] < 2:
        raise ValueError(
            f"grid must have shape (channels>=2,H,W), got {evidence.shape}"
        )
    if evidence.shape[0] == 2:
        return 0
    return int(np.count_nonzero(evidence[2:].max(axis=0) > threshold))
