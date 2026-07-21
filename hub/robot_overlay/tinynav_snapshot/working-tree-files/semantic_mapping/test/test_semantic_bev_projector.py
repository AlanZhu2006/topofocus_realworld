import numpy as np
import pytest

from semantic_mapping.semantic_bev_projector import (
    SemanticBEVGrid,
    SemanticBEVProjectionConfig,
    project_semantic_to_bev,
)
from semantic_mapping.semantic_voxel_map import (
    SemanticVoxelConfig,
    SparseSemanticVoxelMap,
)


def _semantic_map() -> SparseSemanticVoxelMap:
    return SparseSemanticVoxelMap(
        SemanticVoxelConfig(
            resolution_m=0.1,
            class_count=4,
            valid_class_ids=(0, 1, 2, 3),
            dynamic_class_ids=(),
            min_observations=1,
            confirmation_threshold=0.5,
        )
    )


def _integrate(
    semantic_map: SparseSemanticVoxelMap,
    point: tuple[float, float, float],
    label: int,
) -> None:
    semantic_map.integrate_observations(
        np.asarray([point], dtype=np.float32),
        np.asarray([label], dtype=np.uint8),
        np.asarray([1.0], dtype=np.float32),
        1,
    )


def test_height_bands_keep_floor_as_fallback_below_objects() -> None:
    semantic_map = _semantic_map()
    _integrate(semantic_map, (0.01, 0.01, 0.01), 1)  # floor, z=0.05
    _integrate(semantic_map, (0.11, 0.01, 0.21), 2)  # wall, z=0.25
    _integrate(semantic_map, (0.21, 0.01, 0.01), 1)  # floor support
    _integrate(semantic_map, (0.21, 0.01, 0.21), 3)  # chair above floor
    _integrate(semantic_map, (0.31, 0.01, 1.71), 2)  # above semantic band

    bev = project_semantic_to_bev(
        semantic_map,
        SemanticBEVProjectionConfig(
            resolution_m=0.1,
            ground_z=0.0,
            semantic_min_z_relative=0.10,
            semantic_max_z_relative=1.50,
            padding_cells=0,
        ),
        grid=SemanticBEVGrid((0.0, 0.0), 0.1, 4, 1),
        floor_class_id=1,
    )

    assert bev.semantic_label.tolist() == [[1, 2, 3, 0]]
    assert bev.explored.tolist() == [[1, 1, 1, 0]]
    assert bev.semantic_scores.shape == (1, 4, 4)
    assert bev.semantic_scores[0, 2, 3] == pytest.approx(1.0)
    assert bev.semantic_confidence[0, :3].tolist() == pytest.approx([1.0] * 3)
    assert bev.height_min[0, :3].tolist() == pytest.approx([0.05, 0.25, 0.25])
    assert np.isnan(bev.height_max[0, 3])


def test_external_geometry_keeps_semantic_and_occupancy_cells_aligned() -> None:
    semantic_map = _semantic_map()
    _integrate(semantic_map, (0.01, 0.01, 0.21), 2)
    _integrate(semantic_map, (0.21, 0.21, 0.21), 3)

    bev = project_semantic_to_bev(
        semantic_map,
        SemanticBEVProjectionConfig(resolution_m=0.1, padding_cells=0),
        grid=SemanticBEVGrid((-0.10, -0.10), 0.1, 5, 5),
    )

    assert bev.origin_xy.tolist() == pytest.approx([-0.10, -0.10])
    assert bev.semantic_label.shape == (5, 5)
    assert bev.semantic_label[1, 1] == 2
    assert bev.semantic_label[3, 3] == 3


def test_empty_semantics_preserve_external_grid_geometry() -> None:
    bev = project_semantic_to_bev(
        _semantic_map(),
        SemanticBEVProjectionConfig(),
        grid=SemanticBEVGrid((-1.0, 2.0), 0.2, 3, 2),
    )

    assert bev.semantic_label.shape == (2, 3)
    assert np.all(bev.semantic_label == 0)
    assert np.all(bev.explored == 0)
    assert np.all(np.isnan(bev.semantic_confidence))
