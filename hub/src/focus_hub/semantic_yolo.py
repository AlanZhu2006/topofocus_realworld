"""Depth-grounded YOLO reinforcement for the real-camera semantic map.

The upstream Focus_realworld project already runs YOLOv10 for the Perception
VLM prompt, but only RedNet contributes to its BEV semantic channels.  Real
WSJ recordings show a repeatable RedNet domain gap for chairs.  This module is
therefore a deployment extension: supported high-confidence YOLO boxes are
converted into sparse per-pixel MP3D labels, then the existing CentralMapper
performs the authoritative aligned-depth, world-pose and height-band
projection.

Boxes are not treated as segmentation masks.  A depth anchor is estimated
from the central 40% of each box by default, then only returns within a
symmetric depth tolerance are labelled.  This rejects both a small nearer
occluder and the farther wall visible through an open chair.  The output
remains model-derived and unverified in the absence of labelled real-world
ground truth.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .yolo_detector import YoloDetection


# COCO names emitted by the upstream yolov10m weights -> the unique MP3D-40
# ids already consumed by CentralMapper.  Sink is intentionally omitted:
# upstream maps MP3D id 16 to both sink and stairs, so injecting that raw id
# would create two contradictory BEV channels.  These mappings are limited to
# direct category equivalents; they do not invent unsupported classes.
YOLO_TO_MP3D_ID: dict[str, int] = {
    "chair": 4,
    "couch": 11,
    "potted plant": 15,
    "bed": 12,
    "toilet": 19,
    "tv": 23,
    "refrigerator": 38,
    "oven": 38,
    "microwave": 38,
    "toaster": 38,
    "dining table": 6,
}

YOLO_TO_HM3D_NAME: dict[str, str] = {
    "chair": "chair",
    "couch": "sofa",
    "potted plant": "plant",
    "bed": "bed",
    "toilet": "toilet",
    "tv": "tv",
    "refrigerator": "appliances",
    "oven": "appliances",
    "microwave": "appliances",
    "toaster": "appliances",
    "dining table": "table",
}


@dataclass(frozen=True)
class SemanticYoloConfig:
    minimum_confidence: float = 0.35
    depth_anchor_quantile: float = 0.50
    central_crop_fraction: float = 0.40
    depth_tolerance_m: float = 0.45
    minimum_valid_pixels: int = 25
    minimum_depth_m: float = 0.3
    maximum_depth_m: float = 5.0
    allowed_map_categories: tuple[str, ...] = ("chair",)

    def __post_init__(self) -> None:
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in (0, 1]")
        if not 0.0 <= self.depth_anchor_quantile <= 1.0:
            raise ValueError("depth_anchor_quantile must be in [0, 1]")
        if not 0.0 < self.central_crop_fraction <= 1.0:
            raise ValueError("central_crop_fraction must be in (0, 1]")
        if self.depth_tolerance_m <= 0.0:
            raise ValueError("depth_tolerance_m must be positive")
        if self.minimum_valid_pixels <= 0:
            raise ValueError("minimum_valid_pixels must be positive")
        if self.minimum_depth_m < 0.0 or self.maximum_depth_m <= self.minimum_depth_m:
            raise ValueError("invalid semantic YOLO depth range")
        if not self.allowed_map_categories or any(
            not category for category in self.allowed_map_categories
        ):
            raise ValueError("allowed_map_categories must contain non-empty names")


@dataclass(frozen=True)
class SemanticYoloEvidence:
    detector_class: str
    map_category: str
    confidence: float
    xyxy: tuple[float, float, float, float]
    depth_anchor_m: float
    depth_range_m: tuple[float, float]
    depth_anchor_source: str
    labelled_pixels: int
    status: str = "model_inference_depth_projected_unverified"

    def to_dict(self) -> dict[str, object]:
        return {
            "detector_class": self.detector_class,
            "map_category": self.map_category,
            "confidence": self.confidence,
            "xyxy": list(self.xyxy),
            "depth_anchor_m": self.depth_anchor_m,
            "depth_range_m": list(self.depth_range_m),
            "depth_anchor_source": self.depth_anchor_source,
            "labelled_pixels": self.labelled_pixels,
            "status": self.status,
        }


def reinforce_rednet_prediction(
    rednet_prediction: np.ndarray,
    depth_m: np.ndarray,
    detections: list[YoloDetection],
    config: SemanticYoloConfig,
) -> tuple[np.ndarray, list[SemanticYoloEvidence]]:
    """Overlay supported depth-filtered detections onto a RedNet label map.

    Detections are applied from low to high confidence so the strongest class
    wins in overlapping boxes.  CentralMapper still performs the semantic
    height-band and world-frame projection after this function returns.
    """

    if rednet_prediction.shape != depth_m.shape:
        raise ValueError("RedNet prediction and aligned depth must have the same shape")
    if rednet_prediction.ndim != 2:
        raise ValueError("semantic prediction must be a 2-D label image")

    output = np.asarray(rednet_prediction, dtype=np.int16).copy()
    height, width = depth_m.shape
    evidence: list[SemanticYoloEvidence] = []

    for detection in sorted(detections, key=lambda item: item.confidence):
        class_id = YOLO_TO_MP3D_ID.get(detection.class_name)
        map_category = YOLO_TO_HM3D_NAME.get(detection.class_name)
        if (
            class_id is None
            or map_category is None
            or map_category not in config.allowed_map_categories
            or not math.isfinite(detection.confidence)
            or detection.confidence < config.minimum_confidence
        ):
            continue

        x1, y1, x2, y2 = detection.xyxy
        if not all(math.isfinite(value) for value in detection.xyxy):
            continue
        left = max(0, min(width, int(math.floor(min(x1, x2)))))
        right = max(0, min(width, int(math.ceil(max(x1, x2)))))
        top = max(0, min(height, int(math.floor(min(y1, y2)))))
        bottom = max(0, min(height, int(math.ceil(max(y1, y2)))))
        if right <= left or bottom <= top:
            continue

        box_depth = depth_m[top:bottom, left:right]
        valid = (
            np.isfinite(box_depth)
            & (box_depth >= config.minimum_depth_m)
            & (box_depth <= config.maximum_depth_m)
        )
        if int(np.count_nonzero(valid)) < config.minimum_valid_pixels:
            continue

        inset_fraction = (1.0 - config.central_crop_fraction) / 2.0
        box_height, box_width = box_depth.shape
        center_left = int(math.floor(box_width * inset_fraction))
        center_right = int(math.ceil(box_width * (1.0 - inset_fraction)))
        center_top = int(math.floor(box_height * inset_fraction))
        center_bottom = int(math.ceil(box_height * (1.0 - inset_fraction)))
        center_depth = box_depth[
            center_top:center_bottom,
            center_left:center_right,
        ]
        center_valid = (
            np.isfinite(center_depth)
            & (center_depth >= config.minimum_depth_m)
            & (center_depth <= config.maximum_depth_m)
        )
        if int(np.count_nonzero(center_valid)) >= config.minimum_valid_pixels:
            anchor_values = center_depth[center_valid]
            anchor_source = "central_box"
        else:
            anchor_values = box_depth[valid]
            anchor_source = "full_box_fallback"
        depth_anchor = float(
            np.quantile(anchor_values, config.depth_anchor_quantile)
        )
        depth_low = max(
            config.minimum_depth_m,
            depth_anchor - config.depth_tolerance_m,
        )
        depth_high = min(
            config.maximum_depth_m,
            depth_anchor + config.depth_tolerance_m,
        )
        selected = valid & (box_depth >= depth_low) & (box_depth <= depth_high)
        labelled_pixels = int(np.count_nonzero(selected))
        if labelled_pixels < config.minimum_valid_pixels:
            continue

        crop = output[top:bottom, left:right]
        crop[selected] = class_id
        evidence.append(
            SemanticYoloEvidence(
                detector_class=detection.class_name,
                map_category=map_category,
                confidence=float(detection.confidence),
                xyxy=tuple(float(value) for value in detection.xyxy),
                depth_anchor_m=depth_anchor,
                depth_range_m=(depth_low, depth_high),
                depth_anchor_source=anchor_source,
                labelled_pixels=labelled_pixels,
            )
        )

    return output, evidence
