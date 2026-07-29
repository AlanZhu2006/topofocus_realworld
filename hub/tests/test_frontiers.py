from __future__ import annotations

import numpy as np
import pytest

from focus_hub.frontiers import (
    Frontier,
    extract_frontiers,
    render_annotated_bev,
    render_semantic_decision_map,
    validate_frontier_candidates,
)


def _grid(h=60, w=60, n_categories=15):
    grid = np.zeros((2 + n_categories, h, w), dtype=np.float32)
    grid[1, 10:h - 10, 10:w - 10] = 1.0  # explored
    return grid


def _green_pixel_row_span(img: np.ndarray) -> tuple[int, int]:
    mask = (img[:, :, 1] > 200) & (img[:, :, 0] < 100) & (img[:, :, 2] < 100)
    ys, _ = np.nonzero(mask)
    return int(ys.min()), int(ys.max())


def test_render_annotated_bev_row_ordering_not_mirrored():
    """Regression test for a real bug found 2026-07-19: text drawn before a
    final np.flipud() renders upside-down. A marker at a LARGER grid row
    (world +y, "row 0 at bottom" per the docstring) must land at a SMALLER
    pixel y (higher on screen) than a marker at a smaller row.
    """
    grid = _grid()
    h = grid.shape[1]
    scale = 4
    img_low_row = render_annotated_bev(grid, [], robot_rc=(5, 30), scale=scale)
    img_high_row = render_annotated_bev(grid, [], robot_rc=(h - 5, 30), scale=scale)

    def orange_pixel_y(img):
        mask = (img[:, :, 2] > 200) & (img[:, :, 1] > 100) & (img[:, :, 1] < 200) & (img[:, :, 0] < 50)
        ys, _ = np.nonzero(mask)
        return float(ys.mean())

    y_low_row = orange_pixel_y(img_low_row)
    y_high_row = orange_pixel_y(img_high_row)
    # A larger grid row (closer to world-top) must render at a SMALLER pixel y.
    assert y_high_row < y_low_row


def test_render_annotated_bev_frontier_letter_upright():
    """The 'A' glyph must not be a vertically-mirrored blob: real text has
    more ink in its lower half than its upper half for the Hershey-Simplex
    'A' glyph (the crossbar+splayed legs sit below the apex) — a
    vertically-flipped 'A' would show the opposite (heavier top, from the
    flipped crossbar/legs sitting near the top edge of the glyph box).
    """
    grid = _grid()
    frontiers = [Frontier(frontier_id="A", row=30, col=30, x_m=0.0, y_m=0.0, size_cells=10)]
    img = render_annotated_bev(grid, frontiers, robot_rc=None, scale=8)
    red_mask = (img[:, :, 2] > 200) & (img[:, :, 1] < 100) & (img[:, :, 0] < 100)
    ys, xs = np.nonzero(red_mask)
    # Exclude the circle marker itself isn't drawn here (robot_rc=None), so
    # all red pixels are the letter glyph (plus the frontier ring, which is
    # a thin circle outline roughly centered — still symmetric, doesn't bias
    # the top/bottom ink balance the way a mirrored glyph would).
    mid_y = (ys.min() + ys.max()) / 2
    upper = (ys < mid_y).sum()
    lower = (ys >= mid_y).sum()
    assert lower >= upper  # 'A' has more ink toward its base than its apex


def test_render_semantic_decision_map_history_row_ordering_not_mirrored():
    grid = _grid()
    h = grid.shape[1]
    img = render_semantic_decision_map(
        grid, tuple(f"cat{i}" for i in range(15)), [], robot_rc=None, heading_deg=None,
        history_nodes=[(5, 30), (h - 5, 30)], scale=4)
    y_min, y_max = _green_pixel_row_span(img)
    # Two well-separated history dots -> green pixels should span a wide
    # range; more importantly, verify via direct pixel lookup that the
    # LARGER-row node (h-5) lands at the SMALLER pixel y.
    def to_px(row, col, scale=4):
        return int(col * scale), int((h - row) * scale)

    x_lo, y_lo = to_px(5, 30)
    x_hi, y_hi = to_px(h - 5, 30)
    assert y_hi < y_lo
    assert tuple(img[y_lo, x_lo]) == (0, 255, 0)
    assert tuple(img[y_hi, x_hi]) == (0, 255, 0)


def test_semantic_decision_map_uses_source_canvas_palette_and_visited_path():
    grid = _grid()
    image = render_semantic_decision_map(
        grid,
        tuple(f"cat{i}" for i in range(15)),
        [],
        robot_rc=None,
        heading_deg=None,
        visited_paths_rc=[[(20, 20), (30, 20)]],
    )

    assert image.shape == (480, 480, 3)
    x = int((20 + 0.5) * (480 / 60))
    y = int((60 - 1 - 25 + 0.5) * (480 / 60))
    # Source palette index 3 is RGB (0.96, 0.36, 0.26), converted to BGR
    # with int(x*255) truncation.
    assert tuple(image[y, x]) == (66, 91, 244)


def test_source_visited_path_keeps_zero_border_slice_behavior():
    grid = _grid()
    image = render_semantic_decision_map(
        grid,
        tuple(f"cat{i}" for i in range(15)),
        [],
        robot_rc=None,
        heading_deg=None,
        visited_paths_rc=[[(0, 0)]],
        scale=1,
    )

    # Source draw_line writes [-1:1, -1:1], an empty NumPy slice, rather
    # than clipping it into the map.
    assert tuple(image[-1, 0]) == (255, 255, 255)


def test_decision_render_does_not_apply_local_planner_tv_dilation():
    names = (
        "chair",
        "sofa",
        "plant",
        "bed",
        "toilet",
        "tv",
        "bathtub",
        "shower",
        "fireplace",
        "appliances",
        "towel",
        "sink",
        "chest_of_drawers",
        "table",
        "stairs",
    )
    chair_grid = _grid()
    tv_grid = _grid()
    chair_grid[2, 30, 30] = 1.0
    tv_grid[2 + names.index("tv"), 30, 30] = 1.0

    chair = render_semantic_decision_map(
        chair_grid,
        names,
        [],
        robot_rc=None,
        heading_deg=None,
        goal_category="chair",
        scale=1,
    )
    tv = render_semantic_decision_map(
        tv_grid,
        names,
        [],
        robot_rc=None,
        heading_deg=None,
        goal_category="tv",
        scale=1,
    )

    # Decision_Generation_Vis treats every target channel identically.  The
    # 7x7 TV dilation belongs only to vlm_agents.py::act's local FMM mask.
    assert np.array_equal(tv, chair)


def test_extract_frontiers_matches_source_contour_component_pipeline():
    grid = _grid()
    # Split the source explored-contour boundary into multiple components.
    # main.py::Frontiers skips the first scan-ordered component and retains
    # the next source component.
    grid[0, 8:15, 18:23] = 1.0
    grid[0, 45:52, 35:40] = 1.0
    frontiers = extract_frontiers(grid, (0.0, 0.0), 0.05, min_cluster_cells=5)
    assert len(frontiers) >= 1
    assert all(f.frontier_id in "ABCD" for f in frontiers)


def test_extract_frontiers_preserves_source_first_component_skip():
    grid = _grid()

    assert extract_frontiers(grid, (0.0, 0.0), 0.05) == []


def test_frontier_candidate_contract_accepts_stable_subset_only():
    candidates = [
        Frontier("A", 1, 2, 0.1, 0.2, 20),
        Frontier("C", 3, 4, 0.3, 0.4, 10),
    ]
    assert validate_frontier_candidates(candidates) == ("A", "C")
    with pytest.raises(ValueError, match="canonical A-D order"):
        validate_frontier_candidates(list(reversed(candidates)))
    with pytest.raises(ValueError, match="contiguous A-D prefix"):
        validate_frontier_candidates(candidates, require_prefix=True)


def test_semantic_labels_are_drawn_at_world_consistent_map_positions():
    grid = _grid()
    scale = 4
    col = 20

    def black_centroid(row: int) -> tuple[float, float]:
        image = render_semantic_decision_map(
            grid,
            tuple(f"cat{i}" for i in range(15)),
            [],
            robot_rc=None,
            heading_deg=None,
            semantic_labels=[("plant", row, col)],
            scale=scale,
        )
        black = np.all(
            image == np.asarray([0, 0, 0], dtype=np.uint8),
            axis=2,
        )
        ys, xs = np.nonzero(black)
        return float(ys.mean()), float(xs.mean())

    low_row_y, low_row_x = black_centroid(20)
    high_row_y, high_row_x = black_centroid(40)
    assert high_row_y < low_row_y
    assert np.isclose(low_row_y - high_row_y, 20 * scale)
    assert np.isclose(low_row_x, high_row_x)
