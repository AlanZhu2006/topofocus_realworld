"""Deterministic Foxglove/map visualization helpers.

The map itself stores evidence channels, not display pixels.  In particular,
downsampling an already-colorized RGBA image blends unrelated categorical
colors and can create purple/yellow patches which do not correspond to any
semantic class.  This module therefore reduces evidence first and assigns one
exact display color afterwards.

These helpers are read-only and have no robot-control dependency.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .frontiers import _category_palette


UNKNOWN_RGB = (96, 96, 96)
FREE_RGB = (235, 235, 235)
OBSTACLE_RGB = (40, 40, 40)


@dataclass(frozen=True)
class RobotMapOverlay:
    """One robot's read-only pose/trail styling for an operator overview."""

    label: str
    trajectory_xy_m: tuple[tuple[float, float], ...] = ()
    pose_xy_m: tuple[float, float] | None = None
    heading_deg: float | None = None
    trajectory_bgr: tuple[int, int, int] = (255, 110, 30)
    pose_bgr: tuple[int, int, int] = (0, 0, 255)


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


def render_semantic_overview(
    grid: np.ndarray,
    category_names: tuple[str, ...],
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    *,
    robot_overlays: tuple[RobotMapOverlay, ...] = (),
    frontiers: tuple[object, ...] = (),
    semantic_threshold: float = 0.1,
    minimum_component_cells: int = 3,
    close_explored_gaps_cells: int = 3,
    crop_padding_cells: int = 20,
    minimum_output_pixels: int = 600,
    maximum_scale: int = 5,
) -> np.ndarray:
    """Render an example-style top-down semantic image for Foxglove.

    This is an operator visualization only. It does not alter the evidence
    tensor used by the source-derived VLM cascade. The raster combines:

    * near-white unknown and light-gray explored space;
    * current black geometric obstacles;
    * exact palette-colored semantic pixels plus one compact class label per
      sufficiently large connected component;
    * persistent camera trajectories and current pose/heading triangles;
    * optional source-style black lettered frontier candidates.

    Cropping is based on observed map content and overlays, so a small live
    scene remains readable inside a 24-26 m backing grid.
    """

    evidence = np.asarray(grid, dtype=np.float32)
    if evidence.ndim != 3 or evidence.shape[0] < 2:
        raise ValueError(
            f"grid must have shape (channels>=2,H,W), got {evidence.shape}"
        )
    if resolution_m <= 0.0 or not np.isfinite(resolution_m):
        raise ValueError("resolution_m must be finite and positive")
    if minimum_component_cells < 1:
        raise ValueError("minimum_component_cells must be positive")
    if close_explored_gaps_cells < 0:
        raise ValueError("close_explored_gaps_cells must be non-negative")
    if crop_padding_cells < 0:
        raise ValueError("crop_padding_cells must be non-negative")

    height, width = evidence.shape[1:]
    obstacle = evidence[0] > 0.5
    explored = evidence[1] > 0.5
    categories = evidence[2 : 2 + len(category_names)]
    if categories.shape[0] != len(category_names):
        raise ValueError(
            f"grid has {evidence.shape[0] - 2} semantic channels but "
            f"{len(category_names)} names were supplied"
        )

    # The real depth image contains narrow missing-return seams. Ray tracing
    # fills observed free space, but one- or two-cell white slits can remain
    # visually distracting. Close only those small gaps in this operator
    # raster; the map tensor and VLM input remain untouched.
    display_explored = explored
    if close_explored_gaps_cells > 1:
        kernel_size = int(close_explored_gaps_cells)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        display_explored = cv2.morphologyEx(
            explored.astype(np.uint8),
            cv2.MORPH_CLOSE,
            kernel,
        ).astype(bool)

    # The example uses a light paper-like background. Use charcoal rather than
    # pure black for obstacle endpoints, reserving black for lettered frontier
    # markers and keeping unknown space visually distinct.
    canvas = np.full((height, width, 3), (252, 252, 252), dtype=np.uint8)
    canvas[display_explored] = (232, 232, 232)
    canvas[obstacle] = (72, 72, 72)

    palette = _category_palette(len(category_names))
    semantic_masks: list[np.ndarray] = []
    semantic_labels: list[tuple[int, int, str, tuple[int, int, int]]] = []
    for category_index, category_name in enumerate(category_names):
        raw_mask = categories[category_index] > semantic_threshold
        if not np.any(raw_mask):
            semantic_masks.append(raw_mask)
            continue
        component_count, component_ids, stats, centroids = (
            cv2.connectedComponentsWithStats(
                raw_mask.astype(np.uint8), connectivity=8
            )
        )
        display_mask = np.zeros_like(raw_mask)
        accepted_components: list[tuple[int, int]] = []
        for component_id in range(1, component_count):
            cell_count = int(
                stats[component_id, cv2.CC_STAT_AREA]
            )
            if cell_count < minimum_component_cells:
                continue
            display_mask |= component_ids == component_id
            accepted_components.append((cell_count, component_id))
        semantic_masks.append(display_mask)
        if not accepted_components:
            continue
        canvas[display_mask] = palette[category_index]
        _, largest_component = max(accepted_components)
        centroid_col, centroid_row = centroids[largest_component]
        semantic_labels.append(
            (
                int(round(float(centroid_row))),
                int(round(float(centroid_col))),
                category_name,
                tuple(int(value) for value in palette[category_index]),
            )
        )

    semantic_any = (
        np.logical_or.reduce(semantic_masks)
        if semantic_masks
        else np.zeros((height, width), dtype=bool)
    )
    active = display_explored | obstacle | semantic_any

    def world_to_cell(point: tuple[float, float]) -> tuple[int, int]:
        x_m, y_m = point
        col = int(math.floor((float(x_m) - origin_xy_m[0]) / resolution_m))
        row = int(math.floor((float(y_m) - origin_xy_m[1]) / resolution_m))
        return row, col

    overlay_cells: list[tuple[int, int]] = []
    for overlay in robot_overlays:
        overlay_cells.extend(
            world_to_cell(point) for point in overlay.trajectory_xy_m
        )
        if overlay.pose_xy_m is not None:
            overlay_cells.append(world_to_cell(overlay.pose_xy_m))
    for frontier in frontiers:
        overlay_cells.append((int(frontier.row), int(frontier.col)))
    for row, col in overlay_cells:
        if 0 <= row < height and 0 <= col < width:
            active[row, col] = True

    active_rows, active_cols = np.nonzero(active)
    if active_rows.size:
        row_start = max(0, int(active_rows.min()) - crop_padding_cells)
        row_stop = min(height, int(active_rows.max()) + crop_padding_cells + 1)
        col_start = max(0, int(active_cols.min()) - crop_padding_cells)
        col_stop = min(width, int(active_cols.max()) + crop_padding_cells + 1)
    else:
        row_start, row_stop, col_start, col_stop = 0, height, 0, width
    cropped = canvas[row_start:row_stop, col_start:col_stop]
    crop_height, crop_width = cropped.shape[:2]
    scale = max(
        1,
        min(
            maximum_scale,
            int(
                math.ceil(
                    minimum_output_pixels / max(1, max(crop_height, crop_width))
                )
            ),
        ),
    )
    image = cv2.resize(
        np.flipud(cropped),
        (crop_width * scale, crop_height * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    def to_pixel(row: int, col: int) -> tuple[int, int] | None:
        if not (
            row_start <= row < row_stop
            and col_start <= col < col_stop
        ):
            return None
        return (
            int((col - col_start + 0.5) * scale),
            int((crop_height - 1 - (row - row_start) + 0.5) * scale),
        )

    # Semantic labels are compact callouts, not enclosing boxes. Place each
    # callout near its largest component while avoiding earlier labels; this is
    # especially important in fused views where chair/table evidence can be
    # adjacent.
    # Reserve the visible robot/frontier annotations before placing semantic
    # callouts.  The previous renderer placed class labels first and then drew
    # poses on top, which made a correct chair/plant label look corrupted when
    # an agent or frontier occupied the same small crop.
    occupied_label_rects: list[tuple[int, int, int, int]] = []
    for frontier in frontiers:
        center = to_pixel(int(frontier.row), int(frontier.col))
        if center is None:
            continue
        occupied_label_rects.append(
            (
                center[0] - 3 * scale,
                center[1] - 3 * scale,
                center[0] + 12 * scale,
                center[1] + 7 * scale,
            )
        )

    def overlaps_existing(rect: tuple[int, int, int, int]) -> bool:
        left, top, right, bottom = rect
        return any(
            left < other_right
            and right > other_left
            and top < other_bottom
            and bottom > other_top
            for other_left, other_top, other_right, other_bottom
            in occupied_label_rects
        )

    # Reserve every pose marker first, then choose a non-overlapping text
    # origin for each robot.  Doing this in two passes prevents the first
    # robot's label from being placed on top of a nearby second robot pose.
    robot_label_origins: list[tuple[int, int] | None] = [
        None for _overlay in robot_overlays
    ]
    robot_annotation_info: list[
        tuple[
            int,
            tuple[int, int],
            int,
            int,
            int,
            int,
        ]
    ] = []
    for overlay_index, overlay in enumerate(robot_overlays):
        if overlay.pose_xy_m is None:
            continue
        center = to_pixel(*world_to_cell(overlay.pose_xy_m))
        if center is None:
            continue
        pose_size = max(9, 5 * scale)
        font_scale = max(0.45, 0.25 * scale)
        thickness = max(1, scale // 2)
        (text_width, text_height), baseline = cv2.getTextSize(
            overlay.label,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            thickness,
        )
        occupied_label_rects.append(
            (
                center[0] - pose_size,
                center[1] - pose_size,
                center[0] + pose_size,
                center[1] + pose_size,
            )
        )
        robot_annotation_info.append(
            (
                overlay_index,
                center,
                pose_size,
                text_width,
                text_height,
                baseline,
            )
        )

    for (
        overlay_index,
        center,
        pose_size,
        text_width,
        text_height,
        baseline,
    ) in robot_annotation_info:
        gap = max(4, scale)
        candidate_origins = (
            (center[0] + pose_size + gap, center[1] + pose_size),
            (center[0] + pose_size + gap, center[1] - pose_size - gap),
            (center[0] - pose_size - gap - text_width, center[1] + pose_size),
            (
                center[0] - pose_size - gap - text_width,
                center[1] - pose_size - gap,
            ),
            (
                center[0] - text_width // 2,
                center[1] + pose_size + gap + text_height + 1,
            ),
            (
                center[0] - text_width // 2,
                center[1] - pose_size - gap - baseline - 1,
            ),
        )
        selected_origin: tuple[int, int] | None = None
        selected_rect: tuple[int, int, int, int] | None = None
        for origin_x, origin_y in candidate_origins:
            candidate = (
                int(origin_x - 1),
                int(origin_y - text_height - 1),
                int(origin_x + text_width + 1),
                int(origin_y + baseline + 1),
            )
            if (
                candidate[0] < 0
                or candidate[1] < 0
                or candidate[2] >= image.shape[1]
                or candidate[3] >= image.shape[0]
                or overlaps_existing(candidate)
            ):
                continue
            selected_origin = (int(origin_x), int(origin_y))
            selected_rect = candidate
            break
        if selected_origin is None:
            # Extremely small crops can exhaust all six placements. Keep the
            # label visible inside the image; semantic callouts still avoid
            # this clipped fallback rectangle.
            origin_x = int(
                np.clip(
                    center[0] + pose_size + gap,
                    1,
                    max(1, image.shape[1] - text_width - 2),
                )
            )
            origin_y = int(
                np.clip(
                    center[1] + pose_size,
                    text_height + 2,
                    max(text_height + 2, image.shape[0] - baseline - 2),
                )
            )
            selected_origin = (origin_x, origin_y)
            selected_rect = (
                origin_x - 1,
                origin_y - text_height - 1,
                origin_x + text_width + 1,
                origin_y + baseline + 1,
            )
        robot_label_origins[overlay_index] = selected_origin
        occupied_label_rects.append(selected_rect)

    for row, col, category_name, category_bgr in semantic_labels:
        position = to_pixel(row, col)
        if position is None:
            continue
        font_scale = max(0.45, 0.24 * scale)
        thickness = max(1, scale // 2)
        (text_width, text_height), baseline = cv2.getTextSize(
            category_name,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            thickness,
        )
        padding = max(3, scale)
        gap = max(5, 2 * scale)
        box_width = text_width + 2 * padding
        box_height = text_height + baseline + 2 * padding
        candidate_origins = (
            (position[0] + gap, position[1] - box_height - gap),
            (position[0] + gap, position[1] + gap),
            (position[0] - box_width - gap, position[1] - box_height - gap),
            (position[0] - box_width - gap, position[1] + gap),
            (position[0] - box_width // 2, position[1] - box_height - 3 * gap),
            (position[0] - box_width // 2, position[1] + 3 * gap),
        )
        box = None
        for left, top in candidate_origins:
            candidate = (
                int(left),
                int(top),
                int(left + box_width),
                int(top + box_height),
            )
            if (
                candidate[0] < 0
                or candidate[1] < 0
                or candidate[2] >= image.shape[1]
                or candidate[3] >= image.shape[0]
                or overlaps_existing(candidate)
            ):
                continue
            box = candidate
            break
        if box is None:
            left = int(
                np.clip(
                    position[0] + gap,
                    0,
                    max(0, image.shape[1] - box_width - 1),
                )
            )
            top = int(
                np.clip(
                    position[1] - box_height - gap,
                    0,
                    max(0, image.shape[0] - box_height - 1),
                )
            )
            box = (left, top, left + box_width, top + box_height)
        occupied_label_rects.append(box)
        cv2.line(
            image,
            position,
            (
                int(np.clip(position[0], box[0], box[2])),
                int(np.clip(position[1], box[1], box[3])),
            ),
            category_bgr,
            thickness,
            cv2.LINE_AA,
        )
        cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (250, 250, 250), -1)
        cv2.rectangle(
            image,
            (box[0], box[1]),
            (box[2], box[3]),
            category_bgr,
            thickness,
        )
        cv2.putText(
            image,
            category_name,
            (box[0] + padding, box[1] + padding + text_height),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (35, 35, 35),
            thickness,
            cv2.LINE_AA,
        )

    for overlay_index, overlay in enumerate(robot_overlays):
        trail_pixels = []
        for point in overlay.trajectory_xy_m:
            pixel = to_pixel(*world_to_cell(point))
            if pixel is not None:
                trail_pixels.append(pixel)
        if len(trail_pixels) >= 2:
            cv2.polylines(
                image,
                [np.asarray(trail_pixels, dtype=np.int32)],
                False,
                overlay.trajectory_bgr,
                max(2, scale),
                cv2.LINE_AA,
            )
        if overlay.pose_xy_m is None:
            continue
        center = to_pixel(*world_to_cell(overlay.pose_xy_m))
        if center is None:
            continue
        heading_rad = math.radians(overlay.heading_deg or 0.0)
        size = max(9, 5 * scale)
        tip = (
            int(center[0] + size * math.cos(heading_rad)),
            int(center[1] - size * math.sin(heading_rad)),
        )
        left = (
            int(center[0] + size * 0.55 * math.cos(heading_rad + 2.55)),
            int(center[1] - size * 0.55 * math.sin(heading_rad + 2.55)),
        )
        right = (
            int(center[0] + size * 0.55 * math.cos(heading_rad - 2.55)),
            int(center[1] - size * 0.55 * math.sin(heading_rad - 2.55)),
        )
        cv2.fillConvexPoly(
            image,
            np.asarray([tip, left, right], dtype=np.int32),
            overlay.pose_bgr,
            cv2.LINE_AA,
        )
        label_origin = robot_label_origins[overlay_index]
        if label_origin is not None:
            cv2.putText(
                image,
                overlay.label,
                label_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                max(0.45, 0.25 * scale),
                overlay.pose_bgr,
                max(1, scale // 2),
                cv2.LINE_AA,
            )

    for frontier in frontiers:
        center = to_pixel(int(frontier.row), int(frontier.col))
        if center is None:
            continue
        cv2.circle(image, center, max(4, 2 * scale), (0, 0, 0), -1)
        cv2.putText(
            image,
            str(frontier.frontier_id),
            (center[0] + 3 * scale, center[1] + 3 * scale),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.45, 0.25 * scale),
            (0, 0, 0),
            max(1, scale // 2),
            cv2.LINE_AA,
        )

    return np.ascontiguousarray(image)
