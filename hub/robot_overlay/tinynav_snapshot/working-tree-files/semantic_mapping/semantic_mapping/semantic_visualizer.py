"""Pure NumPy visualization helpers for semantic image diagnostics."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from semantic_mapping.semantic_schema import SemanticClassSchema


def blend_semantic_overlay(
    rgb_image: NDArray,
    label_image: NDArray,
    confidence_image: NDArray,
    schema: SemanticClassSchema,
    alpha: float = 0.55,
) -> NDArray[np.uint8]:
    """Blend class colors over RGB, weighted by per-pixel confidence."""
    rgb = np.asarray(rgb_image)
    labels = np.asarray(label_image)
    confidence = np.asarray(confidence_image)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("rgb_image must be HxWx3 uint8")
    if labels.shape != rgb.shape[:2] or labels.dtype != np.uint8:
        raise ValueError("label_image must be HxW uint8 matching RGB")
    if confidence.shape != labels.shape:
        raise ValueError("confidence_image must match label_image")
    if not np.all(np.isfinite(confidence)) or np.any(
        (confidence < 0.0) | (confidence > 1.0)
    ):
        raise ValueError("confidence_image values must be finite in [0, 1]")
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be in [0, 1]")

    colors = schema.colorize(labels)
    weights = alpha * confidence.astype(np.float32, copy=False)
    weights = np.where(labels == schema.unknown_id, 0.0, weights)[..., None]
    overlay = rgb.astype(np.float32) * (1.0 - weights)
    overlay += colors.astype(np.float32) * weights
    return np.clip(np.rint(overlay), 0.0, 255.0).astype(np.uint8)
