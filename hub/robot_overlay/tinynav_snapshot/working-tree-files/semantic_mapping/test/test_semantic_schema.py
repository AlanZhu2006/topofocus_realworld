from pathlib import Path

import numpy as np
import pytest

from semantic_mapping.semantic_backend import SemanticFrame
from semantic_mapping.semantic_schema import SemanticClassSchema


SCHEMA_PATH = Path(__file__).parents[1] / "config" / "semantic_classes.yaml"


def test_navigation_class_schema_and_colorization() -> None:
    schema = SemanticClassSchema.from_yaml(SCHEMA_PATH)
    labels = np.array([[0, 1, 4, 10]], dtype=np.uint8)

    colors = schema.colorize(labels)

    assert schema.version == 1
    assert schema.class_names[4] == "couch"
    assert schema.dynamic_class_ids == frozenset({10})
    np.testing.assert_array_equal(colors[0, 1], [80, 180, 80])
    np.testing.assert_array_equal(colors[0, 3], [40, 200, 255])


def test_schema_rejects_unknown_label_id() -> None:
    schema = SemanticClassSchema.from_yaml(SCHEMA_PATH)
    with pytest.raises(ValueError, match="unknown IDs"):
        schema.validate_labels(np.array([[255]], dtype=np.uint8))


def test_semantic_frame_validates_shape_dtype_and_confidence() -> None:
    labels = np.array([[1, 2]], dtype=np.uint8)
    frame = SemanticFrame(
        label_image=labels,
        confidence_image=np.array([[0.8, 0.9]], dtype=np.float32),
        class_names={0: "unknown", 1: "floor", 2: "wall"},
        timestamp_ns=123,
        source_timestamp_ns=120,
    )
    assert frame.timestamp_ns == 123

    with pytest.raises(ValueError, match="matching labels"):
        SemanticFrame(
            label_image=labels,
            confidence_image=np.ones((2, 1), dtype=np.float32),
            class_names=frame.class_names,
            timestamp_ns=123,
        )
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        SemanticFrame(
            label_image=labels,
            confidence_image=np.array([[1.1, 0.5]], dtype=np.float32),
            class_names=frame.class_names,
            timestamp_ns=123,
        )
