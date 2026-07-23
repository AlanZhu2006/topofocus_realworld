from __future__ import annotations

import numpy as np
import pytest

from focus_hub.frontiers import _category_palette
from focus_hub.map_visualization import (
    FREE_RGB,
    OBSTACLE_RGB,
    RobotMapOverlay,
    UNKNOWN_RGB,
    colorize_geometry_grid,
    colorize_semantic_grid,
    downsample_evidence_grid,
    render_semantic_overview,
    semantic_evidence_cells,
)


def test_downsample_reduces_evidence_before_color_assignment():
    grid = np.zeros((4, 4, 4), dtype=np.float32)
    grid[1, :2, :2] = 1.0
    grid[0, 1, 1] = 1.0
    grid[2, 0, 0] = 0.8

    reduced = downsample_evidence_grid(grid, 2)

    assert reduced.shape == (4, 2, 2)
    assert reduced[0, 0, 0] == pytest.approx(1.0)
    assert reduced[1, 0, 0] == pytest.approx(1.0)
    assert reduced[2, 0, 0] == pytest.approx(0.8)


def test_geometry_colors_are_exact_and_unknown_is_translucent():
    grid = np.zeros((2, 1, 3), dtype=np.float32)
    grid[1, 0, 1:] = 1.0
    grid[0, 0, 2] = 1.0

    rgba = colorize_geometry_grid(grid)

    assert tuple(rgba[0, 0]) == (*UNKNOWN_RGB, 60)
    assert tuple(rgba[0, 1]) == (*FREE_RGB, 255)
    assert tuple(rgba[0, 2]) == (*OBSTACLE_RGB, 255)


def test_semantic_color_is_palette_entry_not_rgba_average():
    grid = np.zeros((4, 2, 2), dtype=np.float32)
    grid[1] = 1.0
    grid[3, 0, 0] = 1.0
    reduced = downsample_evidence_grid(grid, 2)

    rgba = colorize_semantic_grid(reduced, ("first", "second"))
    expected_rgb = tuple(_category_palette(2)[1][::-1])

    assert tuple(rgba[0, 0]) == (*expected_rgb, 255)
    assert semantic_evidence_cells(reduced) == 1


def test_invalid_downsample_factor_is_rejected():
    with pytest.raises(ValueError, match="at least one"):
        downsample_evidence_grid(np.zeros((2, 2, 2), dtype=np.float32), 0)


def test_semantic_overview_combines_pixels_trajectory_pose_and_frontier():
    grid = np.zeros((17, 30, 30), dtype=np.float32)
    grid[1, 5:24, 5:24] = 1.0
    grid[0, 20:22, 10:14] = 1.0
    grid[2, 12:15, 16:19] = 1.0
    # A one-cell plant speckle is below the operator-view component gate.
    grid[4, 7, 7] = 1.0
    overlay = RobotMapOverlay(
        label="wsj",
        trajectory_xy_m=((0.35, 0.35), (0.55, 0.55), (0.75, 0.75)),
        pose_xy_m=(0.75, 0.75),
        heading_deg=45.0,
        trajectory_bgr=(251, 101, 31),
        pose_bgr=(3, 7, 249),
    )
    frontier = type(
        "FrontierView", (), {"row": 18, "col": 18, "frontier_id": "A"}
    )()

    image = render_semantic_overview(
        grid,
        (
            "chair", "sofa", "plant", "bed", "toilet", "tv", "bathtub",
            "shower", "fireplace", "appliances", "towel", "sink",
            "chest_of_drawers", "table", "stairs",
        ),
        (0.0, 0.0),
        0.05,
        robot_overlays=(overlay,),
        frontiers=(frontier,),
        minimum_output_pixels=200,
    )

    chair_bgr = _category_palette(15)[0]
    plant_bgr = _category_palette(15)[2]
    assert image.ndim == 3 and image.shape[2] == 3
    assert np.any(np.all(image == chair_bgr, axis=-1))
    assert not np.any(np.all(image == plant_bgr, axis=-1))
    assert np.any(np.all(image == overlay.trajectory_bgr, axis=-1))
    assert np.any(np.all(image == overlay.pose_bgr, axis=-1))
    assert np.any(np.all(image == (0, 0, 0), axis=-1))
