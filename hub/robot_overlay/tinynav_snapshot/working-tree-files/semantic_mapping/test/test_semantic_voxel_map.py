import numpy as np
import pytest

from semantic_mapping.semantic_voxel_map import (
    SemanticVoxelConfig,
    SparseSemanticVoxelMap,
)


def _config(**overrides) -> SemanticVoxelConfig:
    values = {
        "resolution_m": 1.0,
        "origin_xyz": (0.0, 0.0, 0.0),
        "class_count": 4,
        "valid_class_ids": (0, 1, 2, 3),
        "unknown_class_id": 0,
        "dynamic_class_ids": (3,),
        "min_observations": 2,
        "confirmation_threshold": 0.6,
    }
    values.update(overrides)
    return SemanticVoxelConfig(**values)


def test_consistent_observations_confirm_after_threshold() -> None:
    semantic_map = SparseSemanticVoxelMap(_config())
    point = np.asarray([[0.2, 0.2, 0.2]], dtype=np.float32)
    for timestamp in (1, 2):
        semantic_map.integrate_observations(
            point, np.asarray([2], dtype=np.uint8), np.asarray([0.8]), timestamp
        )
    assert semantic_map.label_and_confidence((0, 0, 0)) == pytest.approx((2, 1.0))
    assert semantic_map.counts().confirmed == 1


def test_conflicting_confidence_votes_are_stable() -> None:
    semantic_map = SparseSemanticVoxelMap(_config(confirmation_threshold=0.5))
    point = np.asarray([[0.2, 0.2, 0.2]], dtype=np.float32)
    semantic_map.integrate_observations(point, np.asarray([1]), np.asarray([0.9]), 1)
    semantic_map.integrate_observations(point, np.asarray([2]), np.asarray([0.4]), 2)
    label, confidence = semantic_map.label_and_confidence((0, 0, 0))
    assert label == 1
    assert confidence == pytest.approx(0.9 / 1.3)


def test_frame_density_is_normalized_per_voxel() -> None:
    semantic_map = SparseSemanticVoxelMap(_config(min_observations=1))
    points = np.full((10, 3), 0.2, dtype=np.float32)
    semantic_map.integrate_observations(
        points, np.ones(10, dtype=np.uint8), np.ones(10), 1
    )
    voxel = semantic_map.voxels[(0, 0, 0)]
    assert voxel.semantic_scores[1] == pytest.approx(1.0)
    assert voxel.observation_count == 1


def test_unknown_dynamic_and_zero_weight_do_not_allocate() -> None:
    semantic_map = SparseSemanticVoxelMap(_config())
    points = np.asarray([[0.1, 0.1, 0.1]] * 3, dtype=np.float32)
    stats = semantic_map.integrate_observations(
        points,
        np.asarray([0, 3, 1], dtype=np.uint8),
        np.asarray([1.0, 1.0, 0.0], dtype=np.float32),
        1,
    )
    assert len(semantic_map) == 0
    assert stats.skipped_unknown == 1
    assert stats.skipped_dynamic == 1
    assert stats.skipped_zero_weight == 1


def test_array_round_trip_preserves_evidence() -> None:
    config = _config(min_observations=1)
    semantic_map = SparseSemanticVoxelMap(config)
    semantic_map.integrate_observations(
        np.asarray([[-0.1, 0.1, 0.1], [1.1, 0.1, 0.1]], dtype=np.float32),
        np.asarray([1, 2], dtype=np.uint8),
        np.asarray([0.7, 0.6], dtype=np.float32),
        12,
    )
    restored = SparseSemanticVoxelMap.from_arrays(config, semantic_map.to_arrays())
    assert sorted(restored.voxels) == [(-1, 0, 0), (1, 0, 0)]
    np.testing.assert_allclose(
        restored.voxels[(-1, 0, 0)].semantic_scores,
        semantic_map.voxels[(-1, 0, 0)].semantic_scores,
    )
