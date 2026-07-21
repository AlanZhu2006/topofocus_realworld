"""Frontier extraction over the fused central map.

Source-derived from the upstream two-agent loop: frontiers are the boundary
between explored free space and unknown space; candidate targets are the
centroids of the largest connected boundary clusters (the upstream code
annotates at most four candidates for the VLM to choose from).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy import ndimage


@dataclass(frozen=True)
class Frontier:
    frontier_id: str
    row: int
    col: int
    x_m: float
    y_m: float
    size_cells: int


def extract_frontiers(
    grid: np.ndarray,
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    *,
    max_candidates: int = 4,
    min_cluster_cells: int = 20,
) -> list[Frontier]:
    """Return up to max_candidates frontier centroids, largest cluster first."""
    obstacle = grid[0] > 0.5
    explored = grid[1] > 0.5
    free = explored & ~obstacle
    unknown = ~explored

    # A frontier cell is free with at least one unknown 4-neighbour.
    unknown_neighbor = ndimage.binary_dilation(
        unknown, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
    )
    frontier_cells = free & unknown_neighbor
    if not frontier_cells.any():
        return []

    labels, count = ndimage.label(frontier_cells, structure=np.ones((3, 3), dtype=bool))
    sizes = ndimage.sum_labels(np.ones_like(labels), labels, index=range(1, count + 1))
    order = np.argsort(sizes)[::-1]

    frontiers: list[Frontier] = []
    for rank, cluster_index in enumerate(order):
        size = int(sizes[cluster_index])
        if size < min_cluster_cells or len(frontiers) >= max_candidates:
            break
        rows, cols = np.nonzero(labels == cluster_index + 1)
        row, col = int(np.round(rows.mean())), int(np.round(cols.mean()))
        frontiers.append(
            Frontier(
                frontier_id=chr(ord("A") + rank),
                row=row,
                col=col,
                x_m=origin_xy_m[0] + (col + 0.5) * resolution_m,
                y_m=origin_xy_m[1] + (row + 0.5) * resolution_m,
                size_cells=size,
            )
        )
    return frontiers


def render_annotated_bev(
    grid: np.ndarray,
    frontiers: list[Frontier],
    robot_rc: tuple[int, int] | None,
    *,
    scale: int = 2,
) -> np.ndarray:
    """Render the BEV with lettered frontier markers for the VLM (BGR image).

    Row 0 is at the bottom (world +y up); the image is upsampled for legibility.

    Bug fixed 2026-07-19: this used to build the canvas with row 0 at the
    TOP (standard image indexing), draw all markers/text, and only flip the
    whole canvas at the very end. Flipping already-rendered TEXT glyphs
    mirrors them vertically — every frontier letter the VLM has ever been
    shown by this function was upside-down (confirmed visually: "A" rendered
    as an inverted-V). Fixed by flipping the row coordinate BEFORE drawing
    instead of flipping the finished canvas after.
    """
    obstacle = grid[0] > 0.5
    explored = grid[1] > 0.5
    h, w = obstacle.shape
    image = np.full((h, w, 3), 96, dtype=np.uint8)          # unknown: dark grey
    image[explored] = (235, 235, 235)                        # explored free: light
    image[obstacle] = (40, 40, 40)                           # obstacles: near-black
    image = np.flipud(image)                                 # flip background only, before drawing
    image = cv2.resize(image, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

    def to_px(row: int, col: int) -> tuple[int, int]:
        return int((col + 0.5) * scale), int((h - 1 - row + 0.5) * scale)

    if robot_rc is not None:
        cv2.circle(image, to_px(*robot_rc), 4 * scale, (0, 140, 255), -1)  # orange (BGR)

    for frontier in frontiers:
        center = to_px(frontier.row, frontier.col)
        cv2.circle(image, center, 5 * scale, (0, 0, 255), 2)
        cv2.putText(
            image,
            frontier.frontier_id,
            (center[0] + 4 * scale, center[1] - 4 * scale),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5 * scale,
            (0, 0, 255),
            2,
        )
    return image.copy()


def _category_palette(n: int) -> np.ndarray:
    """Deterministic, visually distinct BGR palette, one row per category.

    Upstream (`Decision_Generation_Vis`) uses a fixed hand-picked
    `color_palette` array tied to Habitat's own category indexing; the exact
    color VALUES are not semantically load-bearing (the prompts never
    reference a category's color, only frontier=black/history=green/
    pose=red/prev-goal=blue, which ARE preserved exactly below) — this is a
    reasonable, explicitly-labelled substitute, not a fabricated fidelity
    claim.
    """
    hues = (np.arange(n, dtype=np.float32) * (179.0 / max(1, n))).astype(np.uint8)
    hsv = np.stack([hues, np.full(n, 200, np.uint8), np.full(n, 230, np.uint8)], axis=-1)
    return cv2.cvtColor(hsv.reshape(1, n, 3), cv2.COLOR_HSV2BGR).reshape(n, 3)


def render_semantic_decision_map(
    grid: np.ndarray,
    category_names: tuple[str, ...],
    frontiers: list[Frontier],
    robot_rc: tuple[int, int] | None,
    heading_deg: float | None,
    *,
    history_nodes: list[tuple[int, int]] | None = None,
    pre_goal_rc: tuple[int, int] | None = None,
    scale: int = 2,
) -> np.ndarray:
    """Ported from `Decision_Generation_Vis` (main.py), adapted to this
    project's grid convention (grid[0]=obstacle, grid[1]=explored,
    grid[2:2+len(category_names)]=per-category channels, all in [0,1]).

    Colors match upstream exactly where the prompts reference them: black
    circles+uppercase letters for frontier points, green circles+lowercase
    letters for historical observation points (only drawn when
    ``history_nodes`` is given — this is upstream's ``sem_map``, used for
    the Judgment/FN VLM; omit it to get upstream's ``sem_map_frontier``,
    used for the Decision VLM), a red arrow for the robot's pose+heading,
    a blue dot for the previous goal point. Per-category background
    coloring uses a substitute palette — see `_category_palette`.
    """
    obstacle = grid[0] > 0.5
    explored = grid[1] > 0.5
    cat = grid[2:2 + len(category_names)]
    h, w = obstacle.shape

    image = np.full((h, w, 3), 96, dtype=np.uint8)          # unknown: dark grey
    image[explored] = (235, 235, 235)                        # explored free: light
    image[obstacle] = (40, 40, 40)                            # obstacles: near-black

    if len(category_names) > 0:
        palette = _category_palette(len(category_names))
        has_category = cat.max(axis=0) > 0.1
        best_category = cat.argmax(axis=0)
        image[has_category] = palette[best_category[has_category]]

    # Flip the background only, before any drawing — see render_annotated_bev's
    # docstring for why flipping the finished canvas (with text on it) is wrong.
    image = np.flipud(image)
    image = cv2.resize(image, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

    def to_px(row: int, col: int) -> tuple[int, int]:
        return int((col + 0.5) * scale), int((h - 1 - row + 0.5) * scale)

    for frontier in frontiers:
        center = to_px(frontier.row, frontier.col)
        cv2.circle(image, center, 5 * scale, (0, 0, 0), -1)
        cv2.putText(image, frontier.frontier_id, (center[0] + 5 * scale, center[1] + 5 * scale),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, (0, 0, 0), max(1, scale))

    if history_nodes:
        letters = [chr(ord("a") + i) for i in range(26)] + [chr(ord("A") + i) for i in range(26)]
        for i, (row, col) in enumerate(history_nodes[:52]):
            center = to_px(row, col)
            cv2.circle(image, center, 5 * scale, (0, 255, 0), -1)
            label = letters[i] if i < len(letters) else "?"
            cv2.putText(image, label, (center[0] + 5 * scale, center[1] + 5 * scale),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * scale, (0, 255, 0), max(1, scale))

    if robot_rc is not None:
        center = to_px(*robot_rc)
        # A simple triangular heading arrow — a labelled substitute for
        # upstream's vu.get_contour_points helper (not ported: Habitat-
        # utils-specific), same visual effect (position + heading), not
        # byte-identical polygon vertices.
        theta = np.deg2rad(heading_deg or 0.0)
        size = 8 * scale
        tip = (int(center[0] + size * np.cos(theta)), int(center[1] - size * np.sin(theta)))
        left = (int(center[0] + size * 0.5 * np.cos(theta + 2.6)),
                int(center[1] - size * 0.5 * np.sin(theta + 2.6)))
        right = (int(center[0] + size * 0.5 * np.cos(theta - 2.6)),
                 int(center[1] - size * 0.5 * np.sin(theta - 2.6)))
        cv2.drawContours(image, [np.array([tip, left, right])], 0, (0, 0, 255), -1)

    if pre_goal_rc is not None:
        cv2.circle(image, to_px(*pre_goal_rc), 8 * scale, (255, 0, 0), -1)

    return image.copy()
