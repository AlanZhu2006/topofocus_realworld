"""Backend-neutral 2D semantic perception contract."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Mapping

import numpy as np
from numpy.typing import NDArray


class SemanticBackendError(RuntimeError):
    """Base error for deterministic backend failures."""


class SemanticFrameUnavailable(SemanticBackendError):
    """No valid semantic observation exists for the requested image stamp."""


@dataclass(frozen=True)
class SemanticFrame:
    """Aligned class IDs and confidence for one RGB observation."""

    label_image: NDArray[np.uint8]
    confidence_image: NDArray[np.float32]
    class_names: Mapping[int, str]
    timestamp_ns: int
    source_timestamp_ns: int | None = None

    def __post_init__(self) -> None:
        labels = np.asarray(self.label_image)
        confidence = np.asarray(self.confidence_image)
        if labels.ndim != 2 or labels.dtype != np.uint8:
            raise ValueError("label_image must be HxW uint8")
        if confidence.shape != labels.shape or confidence.dtype != np.float32:
            raise ValueError("confidence_image must be HxW float32 matching labels")
        if not np.all(np.isfinite(confidence)):
            raise ValueError("confidence_image must contain finite values")
        if np.any((confidence < 0.0) | (confidence > 1.0)):
            raise ValueError("confidence_image values must be in [0, 1]")
        names = {int(class_id): str(name) for class_id, name in self.class_names.items()}
        invalid = set(int(value) for value in np.unique(labels)).difference(names)
        if invalid:
            raise ValueError(f"label_image contains IDs absent from class_names: {invalid}")
        if int(self.timestamp_ns) < 0:
            raise ValueError("timestamp_ns must be non-negative")
        if self.source_timestamp_ns is not None and int(self.source_timestamp_ns) < 0:
            raise ValueError("source_timestamp_ns must be non-negative")
        object.__setattr__(self, "class_names", names)


class SemanticBackend(ABC):
    """Replaceable RGB-to-semantic-frame backend."""

    @property
    @abstractmethod
    def class_names(self) -> Mapping[int, str]:
        """Return the stable class ID-to-name contract."""

    @abstractmethod
    def infer(self, rgb_image: NDArray, timestamp_ns: int) -> SemanticFrame:
        """Produce a semantic observation aligned to the supplied RGB image."""

    def validate_timestamp(self, timestamp_ns: int) -> None:
        """Raise when no observation can exist for a timestamp without decoding RGB."""

    def close(self) -> None:
        """Release optional backend resources."""
