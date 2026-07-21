import numpy as np

from semantic_mapping.raycasting import (
    batch_raycast_free_voxels,
    dda_voxel_traversal,
    point_to_voxel,
    raycast_free_voxels,
    voxel_center,
)


def test_voxel_indexing_handles_origin_boundaries_and_negative_values() -> None:
    origin = np.array([1.0, -2.0, 0.5])
    assert point_to_voxel([1.0, -2.0, 0.5], origin, 0.5) == (0, 0, 0)
    assert point_to_voxel([1.499, -1.501, 0.999], origin, 0.5) == (0, 0, 0)
    assert point_to_voxel([1.5, -1.5, 1.0], origin, 0.5) == (1, 1, 1)
    assert point_to_voxel([0.999, -2.001, 0.499], origin, 0.5) == (-1, -1, -1)
    np.testing.assert_allclose(voxel_center((-1, -1, -1), origin, 0.5), [0.75, -2.25, 0.25])


def test_axis_aligned_dda_visits_cells_in_order() -> None:
    path = dda_voxel_traversal(
        [0.1, 0.1, 0.1], [3.2, 0.1, 0.1], [0.0, 0.0, 0.0], 1.0
    )
    assert path == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def test_diagonal_dda_advances_tied_axes_without_supercover_cells() -> None:
    forward = dda_voxel_traversal(
        [0.1, 0.1, 0.1], [2.1, 2.1, 2.1], [0.0, 0.0, 0.0], 1.0
    )
    reverse = dda_voxel_traversal(
        [2.1, 2.1, 2.1], [-0.1, -0.1, -0.1], [0.0, 0.0, 0.0], 1.0
    )
    assert forward == [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
    assert reverse == [(2, 2, 2), (1, 1, 1), (0, 0, 0), (-1, -1, -1)]


def test_zero_length_dda_and_inclusion_flags() -> None:
    assert dda_voxel_traversal([0.2] * 3, [0.2] * 3, [0.0] * 3, 1.0) == [
        (0, 0, 0)
    ]
    assert dda_voxel_traversal(
        [0.2] * 3,
        [0.2] * 3,
        [0.0] * 3,
        1.0,
        include_end=False,
    ) == []


def test_free_raycast_never_includes_occupied_endpoint() -> None:
    free, occupied = raycast_free_voxels(
        [0.1, 0.1, 0.1],
        [3.2, 0.1, 0.1],
        [0.0, 0.0, 0.0],
        1.0,
        truncation_distance_m=0.2,
    )
    assert occupied == (3, 0, 0)
    assert free == [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
    assert occupied not in free


def test_batch_raycast_matches_union_of_scalar_traversals() -> None:
    camera = np.array([0.1, -0.2, 0.3])
    endpoints = np.array(
        [
            [2.2, -0.2, 0.3],
            [-1.2, 1.4, 0.8],
            [1.6, 1.3, 1.8],
            [np.nan, 0.0, 0.0],
            camera,
        ]
    )
    expected_free: set[tuple[int, int, int]] = set()
    expected_occupied: set[tuple[int, int, int]] = set()
    for endpoint in endpoints[:3]:
        free, occupied = raycast_free_voxels(
            camera, endpoint, [0.0, 0.0, 0.0], 0.5, 0.1
        )
        expected_free.update(free)
        expected_occupied.add(occupied)
    expected_free.difference_update(expected_occupied)

    result = batch_raycast_free_voxels(
        camera, endpoints, [0.0, 0.0, 0.0], 0.5, 0.1
    )
    assert result.valid_rays == 3
    assert result.rejected_rays == 2
    assert {tuple(row) for row in result.free_voxels} == expected_free
    assert {tuple(row) for row in result.occupied_voxels} == expected_occupied
