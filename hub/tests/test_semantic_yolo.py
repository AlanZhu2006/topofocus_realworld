from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from focus_hub.semantic_yolo import SemanticYoloConfig, reinforce_rednet_prediction
from focus_hub.yolo_detector import YoloDetection, YoloDetector


def test_depth_foreground_reinforces_chair_without_painting_background():
    rednet = np.zeros((6, 8), dtype=np.int16)
    rednet[0, 0] = 15
    depth = np.full((6, 8), 3.0, dtype=np.float32)
    depth[1:5, 2:5] = 1.0
    detection = YoloDetection("chair", 0.8, (1.0, 1.0, 7.0, 5.0))

    fused, evidence = reinforce_rednet_prediction(
        rednet,
        depth,
        [detection],
        SemanticYoloConfig(
            minimum_valid_pixels=1,
            allowed_map_categories=("chair", "table"),
        ),
    )

    assert np.all(fused[1:5, 2:5] == 4)
    assert np.all(fused[1:5, 5:7] == 0)
    assert fused[0, 0] == 15
    assert len(evidence) == 1
    assert evidence[0].map_category == "chair"
    assert evidence[0].labelled_pixels == 12


def test_low_confidence_and_unsupported_classes_do_not_change_rednet():
    rednet = np.full((4, 4), 3, dtype=np.int16)
    depth = np.ones((4, 4), dtype=np.float32)
    detections = [
        YoloDetection("chair", 0.2, (0.0, 0.0, 4.0, 4.0)),
        YoloDetection("person", 0.99, (0.0, 0.0, 4.0, 4.0)),
        YoloDetection("tv", 0.99, (0.0, 0.0, 4.0, 4.0)),
    ]

    fused, evidence = reinforce_rednet_prediction(
        rednet,
        depth,
        detections,
        SemanticYoloConfig(
            minimum_valid_pixels=1,
            allowed_map_categories=("chair", "table"),
        ),
    )

    np.testing.assert_array_equal(fused, rednet)
    assert evidence == []


def test_central_depth_anchor_rejects_small_nearer_occluder():
    rednet = np.zeros((8, 10), dtype=np.int16)
    depth = np.full((8, 10), 2.0, dtype=np.float32)
    depth[:, :2] = 1.0
    detection = YoloDetection("chair", 0.9, (0.0, 0.0, 10.0, 8.0))

    fused, evidence = reinforce_rednet_prediction(
        rednet,
        depth,
        [detection],
        SemanticYoloConfig(minimum_valid_pixels=1),
    )

    assert np.all(fused[:, :2] == 0)
    assert np.all(fused[:, 2:] == 4)
    assert evidence[0].depth_anchor_m == 2.0
    assert evidence[0].depth_range_m == (1.55, 2.45)
    assert evidence[0].depth_anchor_source == "central_box"


def test_higher_confidence_detection_wins_when_supported_boxes_overlap():
    rednet = np.zeros((4, 4), dtype=np.int16)
    depth = np.ones((4, 4), dtype=np.float32)
    detections = [
        YoloDetection("dining table", 0.9, (0.0, 0.0, 4.0, 4.0)),
        YoloDetection("chair", 0.5, (0.0, 0.0, 4.0, 4.0)),
    ]

    fused, _ = reinforce_rednet_prediction(
        rednet,
        depth,
        detections,
        SemanticYoloConfig(
            minimum_valid_pixels=1,
            allowed_map_categories=("chair", "table"),
        ),
    )

    assert np.all(fused == 6)


def test_yolo_detector_retains_boxes_and_legacy_dictionary_shape():
    result = SimpleNamespace(
        names={0: "person", 56: "chair"},
        boxes=SimpleNamespace(
            xyxy=np.array([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]]),
            cls=np.array([0.0, 56.0]),
            conf=np.array([0.7, 0.8]),
        ),
    )
    detector = YoloDetector.__new__(YoloDetector)
    detector.conf = 0.2
    detector.model = lambda **_kwargs: [result]

    boxes = detector.detect_boxes(np.zeros((2, 2, 3), dtype=np.uint8))
    legacy = detector.detect(np.zeros((2, 2, 3), dtype=np.uint8))

    assert boxes == [
        YoloDetection("person", 0.7, (1.0, 2.0, 3.0, 4.0)),
        YoloDetection("chair", 0.8, (5.0, 6.0, 7.0, 8.0)),
    ]
    assert legacy == {"person": 0.7, "chair": 0.8}
