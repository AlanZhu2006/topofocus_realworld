"""Frontier extraction over the fused central map.

Source-derived from the upstream two-agent loop: frontiers are the boundary
between explored free space and unknown space; candidate targets are the
centroids of the largest connected boundary clusters (the upstream code
annotates at most four candidates for the VLM to choose from).
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import cv2
import numpy as np
from skimage import measure, morphology


FRONTIER_LABELS = ("A", "B", "C", "D")
SOURCE_FRONTIER_MIN_COMPONENT_CELLS = 5


@dataclass(frozen=True)
class Frontier:
    frontier_id: str
    row: int
    col: int
    x_m: float
    y_m: float
    size_cells: int


def validate_frontier_candidates(
    frontiers: list[Frontier],
    *,
    require_prefix: bool = False,
) -> tuple[str, ...]:
    """Validate the shared A-D identity used by image, prompt and target.

    A per-robot candidate view may be a stable subset after an earlier robot's
    choice was removed (for example ``("A", "C", "D")``).  The initially
    extracted shared set must be the contiguous prefix starting at A.
    """

    if len(frontiers) > len(FRONTIER_LABELS):
        raise ValueError("VLM frontier candidates are limited to A-D")
    labels = tuple(frontier.frontier_id for frontier in frontiers)
    if len(set(labels)) != len(labels):
        raise ValueError("VLM frontier candidate labels must be unique")
    if any(label not in FRONTIER_LABELS for label in labels):
        raise ValueError("VLM frontier candidate labels must be in A-D")
    canonical = tuple(
        label for label in FRONTIER_LABELS if label in set(labels)
    )
    if labels != canonical:
        raise ValueError("VLM frontier candidates must retain canonical A-D order")
    if require_prefix and labels != FRONTIER_LABELS[: len(labels)]:
        raise ValueError("the shared frontier set must be a contiguous A-D prefix")
    for frontier in frontiers:
        if (
            isinstance(frontier.row, bool)
            or isinstance(frontier.col, bool)
            or not isinstance(frontier.row, int)
            or not isinstance(frontier.col, int)
            or frontier.row < 0
            or frontier.col < 0
        ):
            raise ValueError(
                f"frontier {frontier.frontier_id} has an invalid grid cell"
            )
        if (
            not math.isfinite(float(frontier.x_m))
            or not math.isfinite(float(frontier.y_m))
        ):
            raise ValueError(
                f"frontier {frontier.frontier_id} has non-finite coordinates"
            )
        if (
            isinstance(frontier.size_cells, bool)
            or not isinstance(frontier.size_cells, int)
            or frontier.size_cells <= 0
        ):
            raise ValueError(
                f"frontier {frontier.frontier_id} has an invalid cluster size"
            )
    return labels


def extract_frontiers(
    grid: np.ndarray,
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    *,
    max_candidates: int = 4,
    min_cluster_cells: int = SOURCE_FRONTIER_MIN_COMPONENT_CELLS,
) -> list[Frontier]:
    """Run the executable source ``main.py::Frontiers`` geometry.

    The source closes the explored mask, keeps the contour of its largest
    connected region, subtracts a 3x3-dilated obstacle mask, labels the
    resulting boundary with 8-connectivity and sorts accepted components by
    area.  Its loop starts at region-property index 1, so the first
    scan-ordered component is skipped.  That looks accidental, but it is
    observable source behavior and is preserved here.

    Source Habitat maps are square.  The real-world fused grid can be
    rectangular, so only the array allocation/border slicing is generalized;
    every morphological and component-selection operation is unchanged.
    """

    geometry = _source_frontier_geometry(
        grid,
        max_candidates=max_candidates,
        min_cluster_cells=min_cluster_cells,
    )
    frontiers: list[Frontier] = []
    for rank, (row, col, size) in enumerate(geometry["components"]):
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
    validate_frontier_candidates(frontiers, require_prefix=True)
    return frontiers


def _source_frontier_geometry(
    grid: np.ndarray,
    *,
    max_candidates: int,
    min_cluster_cells: int,
) -> dict[str, object]:
    if (
        not isinstance(grid, np.ndarray)
        or grid.ndim != 3
        or grid.shape[0] < 2
        or grid.shape[1] < 1
        or grid.shape[2] < 1
    ):
        raise ValueError("frontier grid must be CxHxW with at least 2x1x1")
    if (
        isinstance(max_candidates, bool)
        or not isinstance(max_candidates, int)
        or not 1 <= max_candidates <= len(FRONTIER_LABELS)
    ):
        raise ValueError("frontier candidate limit must be within 1..4")
    if (
        isinstance(min_cluster_cells, bool)
        or not isinstance(min_cluster_cells, int)
        or min_cluster_cells < 1
    ):
        raise ValueError("frontier component threshold must be positive")

    obstacle = np.asarray(grid[0], dtype=np.float32)
    explored = np.asarray(grid[1], dtype=np.float32)
    if not np.isfinite(obstacle).all() or not np.isfinite(explored).all():
        raise ValueError("frontier obstacle/explored channels must be finite")
    h, w = obstacle.shape
    obstacle_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilated_obstacle = cv2.dilate(obstacle, obstacle_kernel)
    explored_u8 = cv2.inRange(explored, 0.1, 1.0)
    closed_explored = cv2.morphologyEx(
        explored_u8,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), dtype=np.uint8),
    )
    contours, _ = cv2.findContours(
        closed_explored,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_NONE,
    )
    explored_boundary = np.zeros((h, w), dtype=np.float32)
    if contours:
        contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(
            explored_boundary,
            [contour],
            -1,
            1.0,
            1,
        )
    explored_boundary[:2, :] = 0.0
    explored_boundary[-2:, :] = 0.0
    explored_boundary[:, :2] = 0.0
    explored_boundary[:, -2:] = 0.0

    target_edge = explored_boundary - dilated_obstacle
    target_edge[target_edge > 0.8] = 1.0
    target_edge[target_edge != 1.0] = 0.0
    labels, _ = measure.label(
        target_edge,
        connectivity=2,
        return_num=True,
    )
    properties = measure.regionprops(labels)
    # Exact source indexing: ``range(1, len(props))`` intentionally omits
    # properties[0], then maps property index i back to label i+1.
    costs = [
        (index, int(properties[index].area))
        for index in range(1, len(properties))
        if properties[index].area >= min_cluster_cells
    ]
    costs.sort(key=lambda item: item[1], reverse=True)
    selected = costs[:max_candidates]
    selected_edge = np.zeros_like(target_edge, dtype=np.uint8)
    components: list[tuple[int, int, int]] = []
    for property_index, area in selected:
        selected_edge[labels == property_index + 1] = 1
        row_f, col_f = properties[property_index].centroid
        components.append((int(row_f), int(col_f), area))
    return {
        "components": components,
        "selected_edge_mask": selected_edge,
    }


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
    validate_frontier_candidates(frontiers)
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


_SOURCE_COLOR_PALETTE_RGB = np.asarray(
    [
        (1.0, 1.0, 1.0),
        (0.6, 0.6, 0.6),
        (0.95, 0.95, 0.95),
        (0.96, 0.36, 0.26),
        (0.12156862745098039, 0.47058823529411764, 0.7058823529411765),
        (0.9400000000000001, 0.7818, 0.66),
        (0.9400000000000001, 0.8868, 0.66),
        (0.8882000000000001, 0.9400000000000001, 0.66),
        (0.7832000000000001, 0.9400000000000001, 0.66),
        (0.6782000000000001, 0.9400000000000001, 0.66),
        (0.66, 0.9400000000000001, 0.7468000000000001),
        (0.66, 0.9400000000000001, 0.8518000000000001),
        (0.66, 0.9232, 0.9400000000000001),
        (0.66, 0.8182, 0.9400000000000001),
        (0.66, 0.7132, 0.9400000000000001),
        (0.7117999999999999, 0.66, 0.9400000000000001),
        (0.8168, 0.66, 0.9400000000000001),
        (0.9218, 0.66, 0.9400000000000001),
        (0.9400000000000001, 0.66, 0.8531999999999998),
        (0.9400000000000001, 0.66, 0.748199999999999),
    ],
    dtype=np.float64,
)


def _source_palette_bgr(category_count: int) -> np.ndarray:
    required = 5 + category_count
    if category_count < 0 or required > len(_SOURCE_COLOR_PALETTE_RGB):
        raise ValueError("source palette does not cover the category set")
    # PIL's source palette path truncates with ``int(x * 255)``.
    rgb = (_SOURCE_COLOR_PALETTE_RGB[:required] * 255.0).astype(np.uint8)
    return rgb[:, ::-1].copy()


def _category_palette(category_count: int) -> np.ndarray:
    """Return the stable operator-map semantic colors in BGR order.

    This is deliberately separate from the exact source VLM palette above:
    Foxglove/operator maps have an existing display contract, while only the
    rendered image supplied to the source Decision prompt must use the source
    palette.
    """

    if (
        isinstance(category_count, bool)
        or not isinstance(category_count, int)
        or category_count < 0
    ):
        raise ValueError("category count must be a non-negative integer")
    if category_count == 0:
        return np.empty((0, 3), dtype=np.uint8)
    hues = (
        np.arange(category_count, dtype=np.float32)
        * (179.0 / category_count)
    ).astype(np.uint8)
    hsv = np.stack(
        [
            hues,
            np.full(category_count, 200, np.uint8),
            np.full(category_count, 230, np.uint8),
        ],
        axis=-1,
    )
    return cv2.cvtColor(
        hsv.reshape(1, category_count, 3),
        cv2.COLOR_HSV2BGR,
    ).reshape(category_count, 3)


def render_semantic_decision_map(
    grid: np.ndarray,
    category_names: tuple[str, ...],
    frontiers: list[Frontier],
    robot_rc: tuple[int, int] | None,
    heading_deg: float | None,
    *,
    history_nodes: list[tuple[int, int]] | None = None,
    pre_goal_rc: tuple[int, int] | None = None,
    semantic_labels: list[tuple[str, int, int]] | None = None,
    goal_category: str | None = None,
    visited_paths_rc: Sequence[Sequence[tuple[int, int]]] | None = None,
    scale: int | None = None,
    canvas_size_px: int = 480,
) -> np.ndarray:
    """Ported from `Decision_Generation_Vis` (main.py), adapted to this
    project's grid convention (grid[0]=obstacle, grid[1]=explored,
    grid[2:2+len(category_names)]=per-category channels, all in [0,1]).

    The tracked source palette, 480x480 canvas, frontier edge layer, black
    A-D markers, green history markers, red pose arrow, blue previous-goal
    marker and optional target-component highlight are reproduced.  Source
    assumes a square 480-cell Habitat map; real-world grids can be
    rectangular, so coordinates are geometrically scaled onto the same
    480x480 VLM canvas.  Passing ``scale`` retains a small native-grid canvas
    only for deterministic unit/debug rendering.
    """
    validate_frontier_candidates(frontiers)
    # Decision_Generation_Vis uses ``np.rint(channel) == 1`` rather than a
    # generic probability comparison.
    obstacle = np.rint(grid[0]) == 1
    explored = np.rint(grid[1]) == 1
    cat = grid[2:2 + len(category_names)]
    h, w = obstacle.shape
    if cat.shape[0] != len(category_names):
        raise ValueError("semantic decision grid lacks source categories")
    palette = _source_palette_bgr(len(category_names))
    semantic_indices = np.zeros((h, w), dtype=np.uint8)
    semantic_indices[explored] = 2
    semantic_indices[obstacle] = 1
    if category_names:
        # Source renders the semantic argmax wherever a non-void category has
        # any positive map evidence.  The Hub grid omits source's explicit
        # void channel, so all-zero cells are the equivalent no-category mask.
        has_category = cat.max(axis=0) > 0.0
        best_category = cat.argmax(axis=0)
        semantic_indices[has_category] = (
            best_category[has_category] + 5
        ).astype(np.uint8)
    if visited_paths_rc is not None:
        for robot_index, path in enumerate(visited_paths_rc):
            palette_index = 3 + robot_index
            if palette_index >= len(palette):
                raise ValueError("source palette lacks a visited-path color")
            visited = _source_visited_path_mask(path, (h, w))
            semantic_indices[visited] = palette_index
    frontier_geometry = _source_frontier_geometry(
        grid,
        max_candidates=len(FRONTIER_LABELS),
        min_cluster_cells=SOURCE_FRONTIER_MIN_COMPONENT_CELLS,
    )
    semantic_indices[
        np.asarray(frontier_geometry["selected_edge_mask"], dtype=bool)
    ] = 3
    if goal_category is not None:
        if goal_category not in category_names:
            raise ValueError("goal category is outside the semantic grid")
        goal_index = category_names.index(goal_category)
        goal_binary = np.asarray(cat[goal_index] > 0.0, dtype=bool)
        labels, count = measure.label(
            goal_binary,
            connectivity=2,
            return_num=True,
        )
        if count > 0:
            properties = measure.regionprops(labels)
            largest = max(properties, key=lambda item: item.area)
            goal_component = labels == largest.label
            goal_display = morphology.binary_dilation(
                goal_component,
                footprint=morphology.disk(4),
            )
            semantic_indices[goal_display] = 4
    # Source then paints a disk around every A-D component back to explored
    # color before drawing the black marker and letter.
    for frontier in frontiers:
        frontier_seed = np.zeros((h, w), dtype=bool)
        frontier_seed[frontier.row, frontier.col] = True
        frontier_display = morphology.binary_dilation(
            frontier_seed,
            footprint=morphology.disk(4),
        )
        semantic_indices[frontier_display] = 2
    image = palette[semantic_indices]

    # Flip the background only, before any drawing — see render_annotated_bev's
    # docstring for why flipping the finished canvas (with text on it) is wrong.
    image = np.flipud(image)
    if scale is not None:
        if (
            isinstance(scale, bool)
            or not isinstance(scale, int)
            or scale < 1
        ):
            raise ValueError("semantic debug scale must be positive")
        output_w = w * scale
        output_h = h * scale
        marker_scale = scale
    else:
        if (
            isinstance(canvas_size_px, bool)
            or not isinstance(canvas_size_px, int)
            or canvas_size_px < 64
        ):
            raise ValueError("source VLM canvas size must be at least 64 px")
        output_w = canvas_size_px
        output_h = canvas_size_px
        marker_scale = 1
    image = cv2.resize(
        image,
        (output_w, output_h),
        interpolation=cv2.INTER_NEAREST,
    )
    x_scale = output_w / w
    y_scale = output_h / h

    def to_px(row: int, col: int) -> tuple[int, int]:
        # main.py's ``d240`` is exactly ``480-row`` and frontier/history
        # coordinates are drawn without a half-cell offset. Generalize only
        # the width/height scale for rectangular real-world maps.
        return (
            int(col * x_scale),
            int((h - row) * y_scale),
        )

    for frontier in frontiers:
        center = to_px(frontier.row, frontier.col)
        cv2.circle(image, center, 5 * marker_scale, (0, 0, 0), -1)
        cv2.putText(
            image,
            frontier.frontier_id,
            (
                center[0] + 5 * marker_scale,
                center[1] + 5 * marker_scale,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5 * marker_scale,
            (0, 0, 0),
            marker_scale,
        )

    if history_nodes:
        letters = [chr(ord("a") + i) for i in range(26)] + [chr(ord("A") + i) for i in range(26)]
        for i, (row, col) in enumerate(history_nodes[:52]):
            center = to_px(row, col)
            cv2.circle(
                image,
                center,
                5 * marker_scale,
                (0, 255, 0),
                -1,
            )
            label = letters[i] if i < len(letters) else "?"
            cv2.putText(
                image,
                label,
                (
                    center[0] + 5 * marker_scale,
                    center[1] + 5 * marker_scale,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5 * marker_scale,
                (0, 255, 0),
                marker_scale,
            )

    # Upstream writes each semantic category name at the first point of every
    # extracted polygon on both the Judgment and Decision maps.  Without these
    # labels the palette alone would drop source information supplied to the
    # VLM.
    if semantic_labels:
        for category, row, col in semantic_labels:
            if (
                not category
                or not 0 <= row < h
                or not 0 <= col < w
            ):
                raise ValueError("semantic map label is malformed or out of bounds")
            cv2.putText(
                image,
                category,
                to_px(row, col),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5 * marker_scale,
                (0, 0, 0),
                marker_scale,
                cv2.LINE_AA,
            )

    if robot_rc is not None:
        center = to_px(*robot_rc)
        theta = -np.deg2rad(heading_deg or 0.0)
        size = 15 * marker_scale
        arrow = np.asarray(
            [
                center,
                (
                    int(
                        center[0]
                        + size / 1.5 * np.cos(theta + np.pi * 4 / 3)
                    ),
                    int(
                        center[1]
                        + size / 1.5 * np.sin(theta + np.pi * 4 / 3)
                    ),
                ),
                (
                    int(center[0] + size * np.cos(theta)),
                    int(center[1] + size * np.sin(theta)),
                ),
                (
                    int(
                        center[0]
                        + size / 1.5 * np.cos(theta - np.pi * 4 / 3)
                    ),
                    int(
                        center[1]
                        + size / 1.5 * np.sin(theta - np.pi * 4 / 3)
                    ),
                ),
            ],
            dtype=np.int32,
        )
        cv2.drawContours(image, [arrow], 0, (0, 0, 255), -1)

    if pre_goal_rc is not None:
        cv2.circle(
            image,
            to_px(*pre_goal_rc),
            8 * marker_scale,
            (255, 0, 0),
            -1,
        )

    return image.copy()


def _source_visited_path_mask(
    path: Sequence[tuple[int, int]],
    shape_hw: tuple[int, int],
) -> np.ndarray:
    """Rasterize source ``utils.visualization.draw_line`` segments."""

    height, width = shape_hw
    mask = np.zeros((height, width), dtype=bool)
    points: list[tuple[int, int]] = []
    for value in path:
        if (
            not isinstance(value, (tuple, list))
            or len(value) != 2
            or any(
                isinstance(item, bool)
                or not isinstance(item, int)
                for item in value
            )
        ):
            raise ValueError("visited path cell is malformed")
        row, col = int(value[0]), int(value[1])
        if not 0 <= row < height or not 0 <= col < width:
            raise ValueError("visited path cell is outside the decision grid")
        points.append((row, col))
    if not points:
        return mask
    if len(points) == 1:
        points = [points[0], points[0]]
    for start, end in zip(points, points[1:]):
        for index in range(26):
            row = int(
                np.rint(start[0] + (end[0] - start[0]) * index / 25)
            )
            col = int(
                np.rint(start[1] + (end[1] - start[1]) * index / 25)
            )
            # Preserve ``utils.visualization.draw_line`` slicing literally,
            # including its empty negative-start slice at the zero border.
            mask[row - 1 : row + 1, col - 1 : col + 1] = True
    return mask
