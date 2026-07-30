from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from focus_hub.source_semantics import (
    SOURCE_YOLO_WEIGHT_SHA256,
    SOURCE_YOLO_WEIGHT_SIZE,
    accumulate_maskrcnn_channels,
    apply_source_maskrcnn_override,
    rednet_labels_to_hm3d_channels,
    verify_artifact,
)


def test_rednet_conversion_preserves_duplicate_source_category_mapping():
    labels = np.array([[4, 16, 1]], dtype=np.int16)

    channels = rednet_labels_to_hm3d_channels(labels)

    assert channels.shape == (1, 3, 15)
    assert channels[0, 0, 0] == 1.0  # chair
    assert channels[0, 1, 11] == 1.0  # sink
    assert channels[0, 1, 14] == 1.0  # stairs: source table repeats ID 16
    assert channels[0, 2].sum() == 0.0


def test_maskrcnn_accumulator_keeps_source_instance_counts():
    classes = np.array([58, 58, 56, 0], dtype=np.int64)
    masks = np.zeros((4, 2, 2), dtype=np.bool_)
    masks[0, 0, 0] = True
    masks[1, 0, 0] = True
    masks[2, 1, 1] = True
    masks[3, :, :] = True  # non-semantic COCO class

    result = accumulate_maskrcnn_channels(classes, masks, (2, 2))

    assert result[0, 0, 2] == 2.0
    assert result[1, 1, 0] == 1.0
    assert result[:, :, 6:].sum() == 0.0


def test_six_class_override_matches_source_equality_and_order():
    rednet = np.zeros((1, 4, 15), dtype=np.float32)
    rednet[0, :, 0] = 1  # chair
    rednet[0, :, 1] = 1  # sofa
    rednet[0, :, 3] = 1  # bed
    rednet[0, :, 6] = 1  # unchanged non-override class
    maskrcnn = np.zeros_like(rednet)
    maskrcnn[0, 0, 0] = 1
    maskrcnn[0, 1, 1] = 2  # non-zero retains source sofa
    maskrcnn[0, 2, 2] = 1
    maskrcnn[0, 3, 2] = 2  # source tests == 1, so do not add
    maskrcnn[0, 2, 3] = 1
    maskrcnn[0, 0, 4] = 1
    maskrcnn[0, 1, 5] = 1

    result = apply_source_maskrcnn_override(rednet, maskrcnn)

    np.testing.assert_array_equal(result[0, :, 0], [1, 0, 0, 0])
    np.testing.assert_array_equal(result[0, :, 1], [0, 1, 0, 0])
    np.testing.assert_array_equal(result[0, :, 2], [0, 0, 1, 0])
    np.testing.assert_array_equal(result[0, :, 3], [0, 0, 1, 0])
    np.testing.assert_array_equal(result[0, :, 4], [1, 0, 0, 0])
    np.testing.assert_array_equal(result[0, :, 5], [0, 1, 0, 0])
    np.testing.assert_array_equal(result[0, :, 6], [1, 1, 1, 1])


def test_artifact_identity_is_fail_closed(tmp_path):
    payload = b"source-model"
    artifact = tmp_path / "weight.bin"
    artifact.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()

    record = verify_artifact(
        artifact,
        expected_size=len(payload),
        expected_sha256=digest,
        label="fixture",
    )

    assert record["status"] == "observed_and_checksum_verified"
    with pytest.raises(ValueError, match="identity mismatch"):
        verify_artifact(
            artifact,
            expected_size=len(payload) + 1,
            expected_sha256=digest,
            label="fixture",
        )


def test_source_yolo_identity_matches_observed_artifact_manifest():
    manifest = (
        Path(__file__).resolve().parents[2]
        / "manifests"
        / "artifacts.json"
    )
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    yolo = next(
        artifact
        for artifact in payload["artifacts"]
        if artifact["path"] == "artifacts/vision/yolov10m.pt"
    )

    assert yolo["bytes"] == SOURCE_YOLO_WEIGHT_SIZE
    assert yolo["sha256"] == SOURCE_YOLO_WEIGHT_SHA256
