from __future__ import annotations

import numpy as np
import pytest

from focus_hub.map_quality import compute_map_quality, compare_map_grids


def test_map_quality_counts_geometry_semantics_and_fragments():
    grid = np.zeros((3, 5, 6), dtype=np.float32)
    grid[1, 1:4, 1:5] = 1.0
    grid[0, 1, 1:4] = 1.0
    grid[0, 3, 4] = 1.0
    grid[2, 2, 2] = 1.0

    metrics = compute_map_quality(grid)

    assert metrics.total_cells == 30
    assert metrics.explored_cells == 12
    assert metrics.obstacle_cells == 4
    assert metrics.semantic_cells == 1
    assert metrics.obstacle_explored_ratio == pytest.approx(1 / 3)
    assert metrics.obstacle_components == 2
    assert metrics.largest_obstacle_component_cells == 3
    assert metrics.isolated_obstacle_cells == 1


def test_map_comparison_reports_reversible_obstacle_changes():
    before = np.zeros((2, 3, 4), dtype=np.float32)
    after = before.copy()
    before[0, 1, 1] = 1.0
    before[1, 1, 1] = 1.0
    after[0, 1, 1] = 0.0
    after[0, 1, 2] = 1.0
    after[1, 1, 1:3] = 1.0

    comparison = compare_map_grids(before, after)

    assert comparison["changed_xy_cells"] == 2
    assert comparison["newly_explored_cells"] == 1
    assert comparison["new_obstacle_cells"] == 1
    assert comparison["cleared_obstacle_cells"] == 1
    assert comparison["obstacle_jaccard"] == 0.0


def test_map_comparison_rejects_different_extents():
    with pytest.raises(ValueError, match="shape changed"):
        compare_map_grids(
            np.zeros((2, 2, 2), dtype=np.float32),
            np.zeros((2, 3, 2), dtype=np.float32),
        )
