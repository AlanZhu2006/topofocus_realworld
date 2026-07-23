from __future__ import annotations

import numpy as np
import pytest

from focus_hub.segformer_ade20k import (
    build_source_to_mp3d_lookup,
    collapse_ade20k_prediction,
)


def test_name_mapping_collapses_chair_variants_and_keeps_unmapped_unknown():
    lookup = build_source_to_mp3d_lookup(
        {
            0: "wall",
            1: "chair",
            2: "armchair",
            3: "table",
            4: "plant",
        }
    )

    assert lookup.tolist() == [1, 4, 4, 6, 15]


def test_allowed_categories_fail_closed_and_filter_other_objects():
    lookup = build_source_to_mp3d_lookup(
        {0: "chair", 1: "table", 2: "wall"},
        allowed_categories=("chair",),
    )
    assert lookup.tolist() == [4, 1, 1]

    with pytest.raises(ValueError, match="unsupported"):
        build_source_to_mp3d_lookup(
            {0: "chair"},
            allowed_categories=("not-a-source-category",),
        )


def test_confidence_gate_precedes_nearest_neighbour_restore():
    labels = np.array([[0, 1], [1, 0]], dtype=np.int64)
    confidence = np.array([[0.9, 0.34], [0.8, 0.7]], dtype=np.float32)
    lookup = np.array([4, 6], dtype=np.int16)

    result = collapse_ade20k_prediction(
        labels,
        confidence,
        lookup,
        (4, 4),
        min_confidence=0.35,
    )

    assert result.dtype == np.int16
    assert result.shape == (4, 4)
    np.testing.assert_array_equal(result[:2, :2], np.full((2, 2), 4))
    np.testing.assert_array_equal(result[:2, 2:], np.full((2, 2), 1))
    np.testing.assert_array_equal(result[2:, :2], np.full((2, 2), 6))
    np.testing.assert_array_equal(result[2:, 2:], np.full((2, 2), 4))
