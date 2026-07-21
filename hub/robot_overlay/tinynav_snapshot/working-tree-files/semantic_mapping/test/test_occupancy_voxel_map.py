import numpy as np

from semantic_mapping.occupancy_voxel_map import (
    OccupancyState,
    OccupancyVoxelConfig,
    SparseOccupancyVoxelMap,
)


def make_map() -> SparseOccupancyVoxelMap:
    return SparseOccupancyVoxelMap(
        OccupancyVoxelConfig(
            resolution_m=1.0,
            truncation_distance_m=0.1,
            free_update=-0.4,
            occupied_update=0.85,
        )
    )


def test_log_odds_clamp_and_classification() -> None:
    voxel_map = make_map()
    index = (0, 0, 0)
    assert voxel_map.state(index) == OccupancyState.UNKNOWN

    for timestamp in range(20):
        voxel_map.update_free(index, timestamp)
    assert voxel_map.voxels[index].log_odds == voxel_map.config.min_log_odds
    assert voxel_map.state(index) == OccupancyState.FREE

    for timestamp in range(20, 40):
        voxel_map.update_occupied(index, timestamp)
    assert voxel_map.voxels[index].log_odds == voxel_map.config.max_log_odds
    assert voxel_map.state(index) == OccupancyState.OCCUPIED
    voxel = voxel_map.voxels[index]
    assert voxel.observation_count == 40
    assert voxel.free_observation_count == 20
    assert voxel.occupied_observation_count == 20


def test_frame_integration_carves_free_and_marks_endpoint() -> None:
    voxel_map = make_map()
    endpoints = np.array([[3.2, 0.1, 0.1]], dtype=np.float32)
    stats = voxel_map.integrate_points([0.1, 0.1, 0.1], endpoints, 123)

    assert stats.valid_rays == 1
    assert stats.unique_free_voxels == 3
    assert stats.unique_occupied_voxels == 1
    for index in [(0, 0, 0), (1, 0, 0), (2, 0, 0)]:
        assert voxel_map.voxels[index].free_observation_count == 1
    endpoint = voxel_map.voxels[(3, 0, 0)]
    assert endpoint.occupied_observation_count == 1
    assert endpoint.free_observation_count == 0


def test_duplicate_rays_update_each_voxel_once_per_frame() -> None:
    voxel_map = make_map()
    endpoints = np.repeat([[3.2, 0.1, 0.1]], 20, axis=0)
    stats = voxel_map.integrate_points([0.1, 0.1, 0.1], endpoints, 1)

    assert stats.valid_rays == 20
    assert voxel_map.voxels[(0, 0, 0)].observation_count == 1
    assert voxel_map.voxels[(3, 0, 0)].observation_count == 1


def test_occupied_endpoint_wins_over_another_ray_free_traversal() -> None:
    voxel_map = make_map()
    endpoints = np.array([[1.2, 0.1, 0.1], [3.2, 0.1, 0.1]])
    voxel_map.integrate_points([0.1, 0.1, 0.1], endpoints, 5)

    conflict = voxel_map.voxels[(1, 0, 0)]
    assert conflict.occupied_observation_count == 1
    assert conflict.free_observation_count == 0


def test_array_round_trip_preserves_voxels() -> None:
    voxel_map = make_map()
    voxel_map.integrate_points(
        [0.1, 0.1, 0.1], np.array([[3.2, 0.1, 0.1]]), 99
    )
    restored = SparseOccupancyVoxelMap.from_arrays(
        voxel_map.config, voxel_map.to_arrays()
    )

    assert restored.to_arrays().keys() == voxel_map.to_arrays().keys()
    for name, values in voxel_map.to_arrays().items():
        np.testing.assert_array_equal(restored.to_arrays()[name], values)
