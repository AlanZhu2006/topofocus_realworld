from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from focus_hub.navigation_occupancy import clear_current_footprint
from focus_hub.v2_robot_runtime import OccupancyGrid2D


@dataclass(frozen=True)
class FakeBEV:
    occupancy_probability: np.ndarray
    free_probability: np.ndarray
    explored: np.ndarray
    occupancy_grid: np.ndarray
    height_min: np.ndarray
    height_max: np.ndarray
    origin_xy: np.ndarray
    resolution_m: float

    @property
    def height(self) -> int:
        return int(self.occupancy_grid.shape[0])

    @property
    def width(self) -> int:
        return int(self.occupancy_grid.shape[1])


def make_bev() -> FakeBEV:
    grid = np.full((3, 3), -1, dtype=np.int8)
    grid[:, 2] = 0
    return FakeBEV(
        occupancy_probability=np.full((3, 3), np.nan, dtype=np.float32),
        free_probability=np.full((3, 3), np.nan, dtype=np.float32),
        explored=(grid == 0).astype(np.uint8),
        occupancy_grid=grid,
        height_min=np.full((3, 3), np.nan, dtype=np.float32),
        height_max=np.full((3, 3), np.nan, dtype=np.float32),
        origin_xy=np.asarray([0.0, 0.0], dtype=np.float64),
        resolution_m=0.1,
    )


def pose(x: float, y: float) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[0, 3] = x
    result[1, 3] = y
    return result


def test_current_rectangle_pads_cropped_grid_and_connects_base():
    source = make_bev()
    cleared, cleared_cells = clear_current_footprint(
        source,
        pose(0.45, 0.15),
        shape="rectangle",
        front_m=0.15,
        rear_m=0.15,
        half_width_m=0.10,
    )

    assert source.width == 3
    assert np.all(source.occupancy_grid[:, :2] == -1)
    assert cleared.width > source.width
    assert cleared_cells > 0
    grid = OccupancyGrid2D(
        width=cleared.width,
        height=cleared.height,
        resolution_m=cleared.resolution_m,
        origin_x_m=float(cleared.origin_xy[0]),
        origin_y_m=float(cleared.origin_xy[1]),
        data=tuple(int(value) for value in cleared.occupancy_grid.ravel()),
    )
    assert grid.cell(0.45, 0.15) is not None
    component = grid.reachable_component(0.45, 0.15, clearance_cells=0)
    assert len(component) >= 6
    # Padding is not globally converted to free space.
    assert np.any(cleared.occupancy_grid == -1)


def test_current_circle_clears_only_cells_inside_measured_radius():
    source = make_bev()
    cleared, _ = clear_current_footprint(
        source,
        pose(0.45, 0.15),
        shape="circle",
        radius_m=0.11,
    )
    rows, columns = np.indices(cleared.occupancy_grid.shape)
    x = cleared.origin_xy[0] + (columns + 0.5) * cleared.resolution_m
    y = cleared.origin_xy[1] + (rows + 0.5) * cleared.resolution_m
    outside = (x - 0.45) ** 2 + (y - 0.15) ** 2 > 0.11**2
    newly_padded = columns >= source.width
    assert np.all(cleared.occupancy_grid[outside & newly_padded] == -1)


def test_current_footprint_fill_never_erases_observed_obstacle():
    source = make_bev()
    source.occupancy_grid[1, 1] = np.int8(100)
    source.occupancy_probability[1, 1] = np.float32(1.0)
    source.free_probability[1, 1] = np.float32(0.0)
    source.explored[1, 1] = np.uint8(1)

    cleared, _ = clear_current_footprint(
        source,
        pose(0.15, 0.15),
        shape="circle",
        radius_m=0.11,
    )

    assert cleared.occupancy_grid[1, 1] == 100
    assert cleared.occupancy_probability[1, 1] == 1.0
    assert cleared.free_probability[1, 1] == 0.0


def test_navigation_mapper_omits_unbounded_voxel_visualization_work():
    overlay = Path(__file__).resolve().parents[1] / "robot_overlay"
    source = (overlay / "navigation_occupancy_mapper.py").read_text(
        encoding="utf-8"
    )

    assert ".occupied_points(" not in source
    assert ".counts(" not in source
    assert "clear_current_footprint(" in source
