import numpy as np
import pytest

from semantic_mapping.bev_projector import (
    BEVProjectionConfig,
    project_occupancy_to_bev,
)
from semantic_mapping.occupancy_voxel_map import (
    OccupancyState,
    OccupancyVoxelConfig,
    SparseOccupancyVoxelMap,
)


def make_voxel_map() -> SparseOccupancyVoxelMap:
    return SparseOccupancyVoxelMap(
        OccupancyVoxelConfig(resolution_m=0.1, truncation_distance_m=0.05)
    )


def test_height_bands_keep_floor_and_overhead_traversable() -> None:
    voxel_map = make_voxel_map()
    voxel_map.update_occupied((0, 0, 0), 1)  # ground center z=0.05
    voxel_map.update_occupied((1, 0, 2), 1)  # obstacle center z=0.25
    voxel_map.update_occupied((2, 0, 0), 1)  # ground support
    voxel_map.update_occupied((2, 0, 12), 1)  # overhead center z=1.25
    voxel_map.update_occupied((3, 0, 7), 1)  # collision-band top z=0.75
    for timestamp in range(3):
        voxel_map.update_free((4, 0, 4), timestamp)  # observed free z=0.45

    bev = project_occupancy_to_bev(
        voxel_map,
        BEVProjectionConfig(resolution_m=0.1, ground_z=0.0, padding_cells=0),
    )

    assert bev.occupancy_grid.shape == (1, 5)
    assert bev.occupancy_grid[0].tolist() == [
        OccupancyState.FREE,
        OccupancyState.OCCUPIED,
        OccupancyState.FREE,
        OccupancyState.OCCUPIED,
        OccupancyState.FREE,
    ]
    assert np.all(bev.explored == 1)
    assert bev.height_max[0, 2] == pytest.approx(1.25)
    assert bev.height_min[0, 2] == pytest.approx(0.05)


def test_overhead_without_ground_evidence_remains_unknown_not_free() -> None:
    voxel_map = make_voxel_map()
    voxel_map.update_occupied((-2, -1, 10), 1)
    bev = project_occupancy_to_bev(
        voxel_map,
        BEVProjectionConfig(resolution_m=0.1, padding_cells=1),
    )

    assert bev.occupancy_grid.shape == (3, 3)
    assert bev.origin_xy.tolist() == pytest.approx([-0.3, -0.2])
    assert bev.explored[1, 1] == 1
    assert bev.occupancy_grid[1, 1] == OccupancyState.UNKNOWN
    assert np.isnan(bev.occupancy_probability[1, 1])
    assert np.isnan(bev.free_probability[1, 1])


def test_empty_voxel_map_produces_empty_bev() -> None:
    bev = project_occupancy_to_bev(make_voxel_map(), BEVProjectionConfig())
    assert bev.width == 0
    assert bev.height == 0
    assert bev.occupancy_grid.size == 0
