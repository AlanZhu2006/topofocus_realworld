"""Timestamp-matched precomputed semantic label/confidence backend."""

from __future__ import annotations

from bisect import bisect_left
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import yaml

from semantic_mapping.semantic_backend import (
    SemanticBackend,
    SemanticFrame,
    SemanticFrameUnavailable,
)
from semantic_mapping.semantic_schema import SemanticClassSchema


@dataclass(frozen=True)
class PrecomputedMaskRecord:
    """Files associated with one source RGB timestamp."""

    timestamp_ns: int
    label_path: Path
    confidence_path: Path | None


class PrecomputedMaskBackend(SemanticBackend):
    """Load NPY masks from a versioned YAML manifest with bounded time matching."""

    def __init__(
        self,
        directory: str | Path,
        schema: SemanticClassSchema,
        *,
        manifest_name: str = "manifest.yaml",
        max_time_error_ns: int = 50_000_000,
        default_confidence: float = 1.0,
        unknown_confidence: float = 0.0,
        cache_size: int = 8,
    ) -> None:
        self.directory = Path(directory).expanduser().resolve()
        if not self.directory.is_dir():
            raise ValueError(f"Precomputed mask directory does not exist: {directory}")
        if max_time_error_ns < 0:
            raise ValueError("max_time_error_ns must be non-negative")
        if not 0.0 <= default_confidence <= 1.0:
            raise ValueError("default_confidence must be in [0, 1]")
        if not 0.0 <= unknown_confidence <= 1.0:
            raise ValueError("unknown_confidence must be in [0, 1]")
        if cache_size <= 0:
            raise ValueError("cache_size must be positive")
        self.schema = schema
        self.max_time_error_ns = int(max_time_error_ns)
        self.default_confidence = float(default_confidence)
        self.unknown_confidence = float(unknown_confidence)
        self.cache_size = int(cache_size)
        manifest_path = (self.directory / manifest_name).resolve()
        if not manifest_path.is_relative_to(self.directory):
            raise ValueError("Precomputed manifest must stay inside the directory")
        self.records = self._load_manifest(manifest_path)
        self.timestamps = tuple(record.timestamp_ns for record in self.records)
        self.cache: OrderedDict[int, tuple[NDArray[np.uint8], NDArray[np.float32]]] = (
            OrderedDict()
        )

    @property
    def class_names(self) -> dict[int, str]:
        return self.schema.class_names

    def validate_timestamp(self, timestamp_ns: int) -> None:
        requested_timestamp = int(timestamp_ns)
        if requested_timestamp < 0:
            raise ValueError("timestamp_ns must be non-negative")
        self._nearest_record(requested_timestamp)

    def infer(self, rgb_image: NDArray, timestamp_ns: int) -> SemanticFrame:
        rgb = np.asarray(rgb_image)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"rgb_image must have shape (H, W, 3), got {rgb.shape}")
        requested_timestamp = int(timestamp_ns)
        if requested_timestamp < 0:
            raise ValueError("timestamp_ns must be non-negative")
        record = self._nearest_record(requested_timestamp)
        label, confidence = self._load_record(record)
        if label.shape != rgb.shape[:2]:
            raise ValueError(
                f"Precomputed label shape {label.shape} does not match RGB {rgb.shape[:2]}"
            )
        return SemanticFrame(
            label_image=label,
            confidence_image=confidence,
            class_names=self.class_names,
            timestamp_ns=requested_timestamp,
            source_timestamp_ns=record.timestamp_ns,
        )

    def _nearest_record(self, timestamp_ns: int) -> PrecomputedMaskRecord:
        insertion = bisect_left(self.timestamps, timestamp_ns)
        candidates: list[PrecomputedMaskRecord] = []
        if insertion < len(self.records):
            candidates.append(self.records[insertion])
        if insertion > 0:
            candidates.append(self.records[insertion - 1])
        if not candidates:
            raise SemanticFrameUnavailable("Precomputed mask manifest has no frames")
        record = min(
            candidates,
            key=lambda item: (abs(item.timestamp_ns - timestamp_ns), item.timestamp_ns),
        )
        error_ns = abs(record.timestamp_ns - timestamp_ns)
        if error_ns > self.max_time_error_ns:
            raise SemanticFrameUnavailable(
                f"Nearest precomputed mask is {error_ns * 1e-6:.3f} ms from "
                f"RGB stamp; limit is {self.max_time_error_ns * 1e-6:.3f} ms"
            )
        return record

    def _load_record(
        self, record: PrecomputedMaskRecord
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
        cached = self.cache.get(record.timestamp_ns)
        if cached is not None:
            self.cache.move_to_end(record.timestamp_ns)
            return cached

        label = np.load(record.label_path, allow_pickle=False)
        if label.dtype != np.uint8 or label.ndim != 2:
            raise ValueError(f"Label file must contain HxW uint8: {record.label_path}")
        label = np.ascontiguousarray(label)
        self.schema.validate_labels(label)
        if record.confidence_path is None:
            confidence = np.where(
                label == self.schema.unknown_id,
                self.unknown_confidence,
                self.default_confidence,
            ).astype(np.float32)
        else:
            raw_confidence = np.load(record.confidence_path, allow_pickle=False)
            confidence = np.ascontiguousarray(raw_confidence, dtype=np.float32)
        if confidence.shape != label.shape:
            raise ValueError(
                f"Confidence shape {confidence.shape} does not match label {label.shape}"
            )
        if not np.all(np.isfinite(confidence)) or np.any(
            (confidence < 0.0) | (confidence > 1.0)
        ):
            raise ValueError("Precomputed confidence values must be finite in [0, 1]")
        result = (label, confidence)
        self.cache[record.timestamp_ns] = result
        self.cache.move_to_end(record.timestamp_ns)
        while len(self.cache) > self.cache_size:
            self.cache.popitem(last=False)
        return result

    def _load_manifest(self, path: Path) -> tuple[PrecomputedMaskRecord, ...]:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or int(document.get("version", 0)) != 1:
            raise ValueError("Precomputed manifest version must be 1")
        schema_version = int(document.get("semantic_classes_version", 0))
        if schema_version != self.schema.version:
            raise ValueError(
                f"Manifest class version {schema_version} does not match "
                f"schema version {self.schema.version}"
            )
        raw_frames = document.get("frames")
        if not isinstance(raw_frames, list) or not raw_frames:
            raise ValueError("Precomputed manifest must contain non-empty frames")
        records: list[PrecomputedMaskRecord] = []
        for position, raw_frame in enumerate(raw_frames):
            if not isinstance(raw_frame, dict):
                raise ValueError(f"Manifest frame {position} must be a mapping")
            timestamp_ns = int(raw_frame.get("timestamp_ns", -1))
            if timestamp_ns < 0:
                raise ValueError(f"Manifest frame {position} has invalid timestamp_ns")
            label_path = self._resolve_file(raw_frame.get("label"), position)
            raw_confidence = raw_frame.get("confidence")
            confidence_path = (
                None
                if raw_confidence is None
                else self._resolve_file(raw_confidence, position)
            )
            records.append(
                PrecomputedMaskRecord(
                    timestamp_ns=timestamp_ns,
                    label_path=label_path,
                    confidence_path=confidence_path,
                )
            )
        records.sort(key=lambda item: item.timestamp_ns)
        timestamps = [item.timestamp_ns for item in records]
        if len(set(timestamps)) != len(timestamps):
            raise ValueError("Precomputed manifest timestamps must be unique")
        return tuple(records)

    def _resolve_file(self, raw_path: Any, frame_position: int) -> Path:
        if not isinstance(raw_path, str) or not raw_path:
            raise ValueError(f"Manifest frame {frame_position} has invalid file path")
        path = (self.directory / raw_path).resolve()
        if not path.is_relative_to(self.directory):
            raise ValueError("Precomputed mask paths must stay inside the directory")
        if path.suffix != ".npy":
            raise ValueError(f"Phase-3 precomputed files must use .npy: {path}")
        if not path.is_file():
            raise ValueError(f"Precomputed mask file does not exist: {path}")
        return path
