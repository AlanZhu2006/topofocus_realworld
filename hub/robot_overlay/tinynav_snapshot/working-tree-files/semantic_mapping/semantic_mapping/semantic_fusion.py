"""Pure per-pixel weighting for static semantic surface observations."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import distance_transform_edt

from semantic_mapping.semantic_schema import SemanticClassSchema


@dataclass(frozen=True)
class SemanticWeightConfig:
    """Confidence, range, and mask-boundary weighting parameters."""

    min_confidence: float = 0.50
    depth_decay_m: float = 4.0
    edge_margin_px: float = 3.0
    min_edge_weight: float = 0.20

    def __post_init__(self) -> None:
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if not math.isfinite(self.depth_decay_m) or self.depth_decay_m <= 0.0:
            raise ValueError("depth_decay_m must be finite and positive")
        if not math.isfinite(self.edge_margin_px) or self.edge_margin_px <= 0.0:
            raise ValueError("edge_margin_px must be finite and positive")
        if not 0.0 <= self.min_edge_weight <= 1.0:
            raise ValueError("min_edge_weight must be in [0, 1]")


@dataclass(frozen=True)
class SemanticObservationBatch:
    """Filtered static semantic endpoints and diagnostic rejection counts."""

    points: NDArray[np.float32]
    labels: NDArray[np.uint8]
    weights: NDArray[np.float32]
    input_points: int
    unknown_points: int
    dynamic_points: int
    low_confidence_points: int
    invalid_points: int


def semantic_edge_weights(
    label_image: NDArray, config: SemanticWeightConfig
) -> NDArray[np.float32]:
    """Return a low weight at class boundaries and one in mask interiors."""
    labels = np.asarray(label_image)
    if labels.ndim != 2:
        raise ValueError("label_image must be 2D")
    edge = np.zeros(labels.shape, dtype=np.bool_)
    horizontal = labels[:, 1:] != labels[:, :-1]
    edge[:, 1:] |= horizontal
    edge[:, :-1] |= horizontal
    vertical = labels[1:, :] != labels[:-1, :]
    edge[1:, :] |= vertical
    edge[:-1, :] |= vertical
    if not np.any(edge):
        return np.ones(labels.shape, dtype=np.float32)
    distance = distance_transform_edt(~edge)
    normalized = np.clip(distance / config.edge_margin_px, 0.0, 1.0)
    weights = config.min_edge_weight + (1.0 - config.min_edge_weight) * normalized
    return weights.astype(np.float32, copy=False)


def build_semantic_observations(
    points_target: NDArray,
    pixels_uv: NDArray,
    label_image: NDArray,
    confidence_image: NDArray,
    camera_position: NDArray,
    schema: SemanticClassSchema,
    config: SemanticWeightConfig,
) -> SemanticObservationBatch:
    """Sample aligned semantic images at RGB-D pixels and compute fusion weights."""
    points = np.asarray(points_target, dtype=np.float32)
    pixels = np.asarray(pixels_uv)
    labels = np.asarray(label_image)
    confidence = np.asarray(confidence_image, dtype=np.float32)
    camera = np.asarray(camera_position, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points_target must have shape (N, 3), got {points.shape}")
    if pixels.shape != (points.shape[0], 2):
        raise ValueError("pixels_uv must have shape (N, 2) matching points_target")
    if not np.issubdtype(pixels.dtype, np.integer):
        raise ValueError("pixels_uv must contain integer coordinates")
    schema.validate_labels(labels)
    if confidence.shape != labels.shape:
        raise ValueError("confidence_image shape must match label_image")
    if np.any(~np.isfinite(confidence)) or np.any(
        (confidence < 0.0) | (confidence > 1.0)
    ):
        raise ValueError("confidence_image must contain finite values in [0, 1]")
    if camera.shape != (3,) or np.any(~np.isfinite(camera)):
        raise ValueError("camera_position must contain three finite values")

    u = pixels[:, 0].astype(np.int64, copy=False)
    v = pixels[:, 1].astype(np.int64, copy=False)
    if np.any(u < 0) or np.any(v < 0) or np.any(u >= labels.shape[1]) or np.any(
        v >= labels.shape[0]
    ):
        raise ValueError("pixels_uv contains coordinates outside semantic images")

    sampled_labels = labels[v, u].astype(np.uint8, copy=False)
    sampled_confidence = confidence[v, u]
    finite_points = np.all(np.isfinite(points), axis=1)
    unknown = sampled_labels == schema.unknown_id
    dynamic = np.isin(sampled_labels, list(schema.dynamic_class_ids))
    low_confidence = sampled_confidence < config.min_confidence
    valid = finite_points & ~unknown & ~dynamic & ~low_confidence

    edge_weights = semantic_edge_weights(labels, config)[v, u]
    depths = np.linalg.norm(points.astype(np.float64) - camera, axis=1)
    depth_weights = np.exp(-depths / config.depth_decay_m)
    weights = sampled_confidence * edge_weights * depth_weights
    valid &= np.isfinite(weights) & (weights > 0.0)

    return SemanticObservationBatch(
        points=points[valid],
        labels=sampled_labels[valid],
        weights=weights[valid].astype(np.float32, copy=False),
        input_points=int(points.shape[0]),
        unknown_points=int(np.count_nonzero(unknown)),
        dynamic_points=int(np.count_nonzero(dynamic)),
        low_confidence_points=int(np.count_nonzero(low_confidence & ~unknown)),
        invalid_points=int(np.count_nonzero(~finite_points)),
    )
