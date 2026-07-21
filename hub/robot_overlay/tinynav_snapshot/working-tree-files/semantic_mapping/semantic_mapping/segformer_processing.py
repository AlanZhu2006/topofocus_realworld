"""SegFormer preprocessing and closed-set navigation postprocessing."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class SegformerProcessorConfig:
    """Image normalization fields required by a SegFormer export."""

    input_height: int
    input_width: int
    image_mean: tuple[float, float, float]
    image_std: tuple[float, float, float]
    rescale_factor: float

    def __post_init__(self) -> None:
        if self.input_height <= 0 or self.input_width <= 0:
            raise ValueError("SegFormer input dimensions must be positive")
        if any(value <= 0.0 for value in self.image_std):
            raise ValueError("SegFormer image standard deviations must be positive")
        if self.rescale_factor <= 0.0:
            raise ValueError("SegFormer rescale_factor must be positive")

    @classmethod
    def from_json(cls, path: str | Path) -> "SegformerProcessorConfig":
        """Load Hugging Face image-processor settings from JSON."""
        source = Path(path).expanduser()
        with source.open(encoding="utf-8") as stream:
            document = json.load(stream)
        size = document.get("size")
        mean = document.get("image_mean")
        std = document.get("image_std")
        if not isinstance(size, dict):
            raise ValueError("Preprocessor config is missing size")
        if not isinstance(mean, list) or len(mean) != 3:
            raise ValueError("Preprocessor config image_mean must contain three values")
        if not isinstance(std, list) or len(std) != 3:
            raise ValueError("Preprocessor config image_std must contain three values")
        return cls(
            input_height=int(size.get("height", 0)),
            input_width=int(size.get("width", 0)),
            image_mean=tuple(float(value) for value in mean),
            image_std=tuple(float(value) for value in std),
            rescale_factor=float(document.get("rescale_factor", 0.0)),
        )


def prepare_segformer_input(
    rgb_image: NDArray, config: SegformerProcessorConfig
) -> NDArray[np.float32]:
    """Resize and normalize uint8 RGB to contiguous NCHW float32."""
    rgb = np.asarray(rgb_image)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("rgb_image must be HxWx3 uint8")
    resized = cv2.resize(
        rgb,
        (config.input_width, config.input_height),
        interpolation=cv2.INTER_LINEAR,
    ).astype(np.float32)
    resized *= config.rescale_factor
    mean = np.asarray(config.image_mean, dtype=np.float32).reshape(1, 1, 3)
    std = np.asarray(config.image_std, dtype=np.float32).reshape(1, 1, 3)
    normalized = (resized - mean) / std
    return np.ascontiguousarray(normalized.transpose(2, 0, 1)[None])


def navigation_semantics_from_logits(
    logits: NDArray,
    source_to_target: NDArray,
    output_shape: tuple[int, int],
    *,
    unknown_id: int = 0,
    min_confidence: float = 0.35,
) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
    """Convert low-resolution source logits to full-size navigation semantics."""
    values = np.asarray(logits)
    if values.ndim == 4:
        if values.shape[0] != 1:
            raise ValueError("Segmentation logits batch size must be one")
        values = values[0]
    if values.ndim != 3:
        raise ValueError("Segmentation logits must have shape [1,C,H,W] or [C,H,W]")
    lookup = np.asarray(source_to_target)
    if lookup.ndim != 1 or lookup.dtype != np.uint8:
        raise ValueError("source_to_target must be one-dimensional uint8")
    if values.shape[0] != lookup.size:
        raise ValueError("Logit class count does not match source label mapping")
    output_height, output_width = (int(value) for value in output_shape)
    if output_height <= 0 or output_width <= 0:
        raise ValueError("output_shape dimensions must be positive")
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError("min_confidence must be in [0, 1]")

    float_logits = values.astype(np.float32, copy=False)
    source_labels = np.argmax(float_logits, axis=0)
    maxima = np.max(float_logits, axis=0)
    denominator = np.exp(float_logits - maxima[None]).sum(axis=0)
    confidence = np.reciprocal(denominator, dtype=np.float32)
    labels = lookup[source_labels]
    rejected = (labels == unknown_id) | (confidence < min_confidence)
    labels = np.where(rejected, unknown_id, labels).astype(np.uint8)
    confidence = np.where(rejected, 0.0, confidence).astype(np.float32)

    if labels.shape != (output_height, output_width):
        labels = cv2.resize(
            labels,
            (output_width, output_height),
            interpolation=cv2.INTER_NEAREST,
        )
        confidence = cv2.resize(
            confidence,
            (output_width, output_height),
            interpolation=cv2.INTER_NEAREST,
        )
    return np.ascontiguousarray(labels), np.ascontiguousarray(confidence)
