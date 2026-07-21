from pathlib import Path

import numpy as np
import pytest
import yaml

from semantic_mapping.precomputed_mask_backend import PrecomputedMaskBackend
from semantic_mapping.semantic_backend import SemanticFrameUnavailable
from semantic_mapping.semantic_schema import SemanticClassSchema


SCHEMA_PATH = Path(__file__).parents[1] / "config" / "semantic_classes.yaml"


def make_backend(
    directory: Path,
    frames: list[dict[str, object]],
    *,
    max_time_error_ns: int = 20,
) -> PrecomputedMaskBackend:
    manifest = {
        "version": 1,
        "semantic_classes_version": 1,
        "frames": frames,
    }
    (directory / "manifest.yaml").write_text(
        yaml.safe_dump(manifest), encoding="utf-8"
    )
    return PrecomputedMaskBackend(
        directory,
        SemanticClassSchema.from_yaml(SCHEMA_PATH),
        max_time_error_ns=max_time_error_ns,
    )


def test_exact_mask_load_and_default_confidence(tmp_path: Path) -> None:
    labels = np.array([[0, 1], [2, 4]], dtype=np.uint8)
    np.save(tmp_path / "labels.npy", labels)
    backend = make_backend(
        tmp_path, [{"timestamp_ns": 100, "label": "labels.npy"}]
    )

    frame = backend.infer(np.zeros((2, 2, 3), dtype=np.uint8), 100)

    np.testing.assert_array_equal(frame.label_image, labels)
    np.testing.assert_array_equal(
        frame.confidence_image,
        np.array([[0.0, 1.0], [1.0, 1.0]], dtype=np.float32),
    )
    assert frame.timestamp_ns == 100
    assert frame.source_timestamp_ns == 100


def test_nearest_mask_respects_time_error_and_tie_breaks_earlier(
    tmp_path: Path,
) -> None:
    early = np.full((2, 2), 1, dtype=np.uint8)
    late = np.full((2, 2), 2, dtype=np.uint8)
    np.save(tmp_path / "early.npy", early)
    np.save(tmp_path / "late.npy", late)
    backend = make_backend(
        tmp_path,
        [
            {"timestamp_ns": 90, "label": "early.npy"},
            {"timestamp_ns": 110, "label": "late.npy"},
        ],
        max_time_error_ns=10,
    )

    frame = backend.infer(np.zeros((2, 2, 3), dtype=np.uint8), 100)
    assert frame.source_timestamp_ns == 90
    np.testing.assert_array_equal(frame.label_image, early)
    backend.validate_timestamp(100)

    with pytest.raises(SemanticFrameUnavailable, match="limit"):
        backend.infer(np.zeros((2, 2, 3), dtype=np.uint8), 200)
    with pytest.raises(SemanticFrameUnavailable, match="limit"):
        backend.validate_timestamp(200)


def test_explicit_confidence_and_rgb_shape_validation(tmp_path: Path) -> None:
    labels = np.array([[1, 2], [4, 5]], dtype=np.uint8)
    confidence = np.array([[0.5, 0.6], [0.7, 0.8]], dtype=np.float32)
    np.save(tmp_path / "labels.npy", labels)
    np.save(tmp_path / "confidence.npy", confidence)
    backend = make_backend(
        tmp_path,
        [
            {
                "timestamp_ns": 100,
                "label": "labels.npy",
                "confidence": "confidence.npy",
            }
        ],
    )

    frame = backend.infer(np.zeros((2, 2, 3), dtype=np.uint8), 100)
    np.testing.assert_allclose(frame.confidence_image, confidence)
    with pytest.raises(ValueError, match="does not match RGB"):
        backend.infer(np.zeros((3, 2, 3), dtype=np.uint8), 100)


def test_backend_rejects_label_id_outside_schema(tmp_path: Path) -> None:
    np.save(tmp_path / "invalid.npy", np.array([[255]], dtype=np.uint8))
    backend = make_backend(
        tmp_path, [{"timestamp_ns": 1, "label": "invalid.npy"}]
    )

    with pytest.raises(ValueError, match="unknown IDs"):
        backend.infer(np.zeros((1, 1, 3), dtype=np.uint8), 1)
