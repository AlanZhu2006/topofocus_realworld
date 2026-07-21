import numpy as np
import pytest

from semantic_mapping.semantic_fusion import (
    SemanticWeightConfig,
    build_semantic_observations,
    semantic_edge_weights,
)
from semantic_mapping.semantic_schema import SemanticClass, SemanticClassSchema


def _schema() -> SemanticClassSchema:
    return SemanticClassSchema(
        version=1,
        classes=(
            SemanticClass(0, "unknown", (0, 0, 0)),
            SemanticClass(1, "floor", (0, 255, 0)),
            SemanticClass(2, "wall", (128, 128, 128)),
            SemanticClass(3, "dynamic_object", (0, 255, 255), dynamic=True),
        ),
    )


def test_edge_weight_reduces_mask_boundary() -> None:
    labels = np.ones((5, 7), dtype=np.uint8)
    labels[:, 4:] = 2
    config = SemanticWeightConfig(edge_margin_px=2.0, min_edge_weight=0.1)
    weights = semantic_edge_weights(labels, config)
    assert weights[2, 3] == pytest.approx(0.1)
    assert weights[2, 4] == pytest.approx(0.1)
    assert weights[2, 0] > weights[2, 2] > weights[2, 3]


def test_observation_filtering_and_weighting() -> None:
    labels = np.asarray([[1, 0, 3, 2]], dtype=np.uint8)
    confidence = np.asarray([[0.9, 0.9, 0.9, 0.4]], dtype=np.float32)
    points = np.asarray(
        [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0, 0, 1]],
        dtype=np.float32,
    )
    pixels = np.asarray([[0, 0], [1, 0], [2, 0], [3, 0]], dtype=np.int32)
    batch = build_semantic_observations(
        points,
        pixels,
        labels,
        confidence,
        np.zeros(3),
        _schema(),
        SemanticWeightConfig(min_confidence=0.5, depth_decay_m=1.0),
    )
    assert batch.labels.tolist() == [1]
    assert batch.unknown_points == 1
    assert batch.dynamic_points == 1
    assert batch.low_confidence_points == 1
    assert batch.weights[0] < 0.9


def test_rejects_out_of_bounds_pixels() -> None:
    with pytest.raises(ValueError, match="outside semantic images"):
        build_semantic_observations(
            np.zeros((1, 3), dtype=np.float32),
            np.asarray([[2, 0]], dtype=np.int32),
            np.ones((1, 2), dtype=np.uint8),
            np.ones((1, 2), dtype=np.float32),
            np.zeros(3),
            _schema(),
            SemanticWeightConfig(),
        )
