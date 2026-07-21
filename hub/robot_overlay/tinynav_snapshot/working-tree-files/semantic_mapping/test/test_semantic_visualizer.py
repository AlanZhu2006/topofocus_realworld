from pathlib import Path

import numpy as np
import pytest

from semantic_mapping.semantic_schema import SemanticClassSchema
from semantic_mapping.semantic_visualizer import blend_semantic_overlay


CLASSES_PATH = Path(__file__).parents[1] / "config" / "semantic_classes.yaml"


def test_overlay_blends_known_class_and_preserves_unknown() -> None:
    schema = SemanticClassSchema.from_yaml(CLASSES_PATH)
    rgb = np.full((1, 2, 3), 100, dtype=np.uint8)
    labels = np.array([[0, 1]], dtype=np.uint8)
    confidence = np.array([[1.0, 0.5]], dtype=np.float32)

    result = blend_semantic_overlay(rgb, labels, confidence, schema, alpha=0.5)

    np.testing.assert_array_equal(result[0, 0], [100, 100, 100])
    np.testing.assert_array_equal(result[0, 1], [95, 120, 95])


def test_overlay_rejects_invalid_inputs() -> None:
    schema = SemanticClassSchema.from_yaml(CLASSES_PATH)
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    labels = np.zeros((2, 2), dtype=np.uint8)
    confidence = np.ones((2, 2), dtype=np.float32)

    with pytest.raises(ValueError, match="alpha"):
        blend_semantic_overlay(rgb, labels, confidence, schema, alpha=1.1)
    with pytest.raises(ValueError, match="matching RGB"):
        blend_semantic_overlay(rgb, labels[:1], confidence[:1], schema)
