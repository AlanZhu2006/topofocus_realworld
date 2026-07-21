import numpy as np
import pytest

from semantic_mapping.segformer_processing import (
    SegformerProcessorConfig,
    navigation_semantics_from_logits,
    prepare_segformer_input,
)


def test_prepare_segformer_input_is_rgb_nchw() -> None:
    config = SegformerProcessorConfig(
        input_height=1,
        input_width=1,
        image_mean=(0.0, 0.0, 0.0),
        image_std=(1.0, 1.0, 1.0),
        rescale_factor=1.0 / 255.0,
    )
    rgb = np.array([[[255, 128, 0]]], dtype=np.uint8)

    result = prepare_segformer_input(rgb, config)

    assert result.shape == (1, 3, 1, 1)
    assert result.dtype == np.float32
    np.testing.assert_allclose(result[:, :, 0, 0], [[1.0, 128.0 / 255.0, 0.0]])


def test_logits_map_to_full_size_labels_and_confidence() -> None:
    logits = np.full((1, 4, 1, 2), -4.0, dtype=np.float32)
    logits[0, 1, 0, 0] = 4.0
    logits[0, 3, 0, 1] = 4.0
    lookup = np.array([2, 1, 10, 0], dtype=np.uint8)

    labels, confidence = navigation_semantics_from_logits(
        logits, lookup, (2, 4), min_confidence=0.35
    )

    np.testing.assert_array_equal(labels, [[1, 1, 0, 0], [1, 1, 0, 0]])
    assert confidence.dtype == np.float32
    assert np.all(confidence[:, :2] > 0.99)
    np.testing.assert_array_equal(confidence[:, 2:], 0.0)


def test_logits_reject_class_count_mismatch() -> None:
    with pytest.raises(ValueError, match="class count"):
        navigation_semantics_from_logits(
            np.zeros((1, 2, 1, 1), dtype=np.float32),
            np.zeros(3, dtype=np.uint8),
            (1, 1),
        )
