"""Validated source-dataset to navigation-class label mapping."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping

import numpy as np
from numpy.typing import NDArray
import yaml

from semantic_mapping.semantic_schema import SemanticClassSchema


def _normalized_name(value: str) -> str:
    return value.strip().casefold()


def load_huggingface_id2label(path: str | Path) -> dict[int, str]:
    """Load and validate a contiguous id2label mapping from model config JSON."""
    source = Path(path).expanduser()
    with source.open(encoding="utf-8") as stream:
        document = json.load(stream)
    raw_labels = document.get("id2label")
    if not isinstance(raw_labels, dict) or not raw_labels:
        raise ValueError("Model config must contain a non-empty id2label mapping")
    labels = {int(class_id): str(name).strip() for class_id, name in raw_labels.items()}
    if set(labels) != set(range(len(labels))):
        raise ValueError("Model id2label IDs must be contiguous from zero")
    normalized = [_normalized_name(labels[index]) for index in range(len(labels))]
    if any(not name for name in normalized) or len(set(normalized)) != len(normalized):
        raise ValueError("Model id2label names must be non-empty and unique")
    return labels


@dataclass(frozen=True)
class SemanticLabelMapping:
    """Dense lookup from model output IDs to stable navigation class IDs."""

    source_dataset: str
    source_names: tuple[str, ...]
    source_to_target: NDArray[np.uint8]

    def __post_init__(self) -> None:
        lookup = np.asarray(self.source_to_target)
        if not self.source_dataset:
            raise ValueError("source_dataset must not be empty")
        if lookup.ndim != 1 or lookup.dtype != np.uint8:
            raise ValueError("source_to_target must be a one-dimensional uint8 array")
        if lookup.size != len(self.source_names) or lookup.size == 0:
            raise ValueError("source names and lookup size must match")

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
        schema: SemanticClassSchema,
        source_labels: Mapping[int, str],
    ) -> "SemanticLabelMapping":
        """Load a name-based mapping and resolve it against model metadata."""
        source = Path(path).expanduser()
        with source.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or int(document.get("version", 0)) != 1:
            raise ValueError("Semantic label mapping version must be 1")
        if int(document.get("semantic_classes_version", 0)) != schema.version:
            raise ValueError("Mapping semantic class version does not match schema")
        raw_mappings = document.get("mappings")
        if not isinstance(raw_mappings, dict) or not raw_mappings:
            raise ValueError("Semantic label mapping must contain mappings")

        ids = {int(class_id) for class_id in source_labels}
        if ids != set(range(len(source_labels))):
            raise ValueError("Source label IDs must be contiguous from zero")
        source_names = tuple(
            str(source_labels[index]).strip() for index in range(len(source_labels))
        )
        source_name_to_id = {
            _normalized_name(name): index for index, name in enumerate(source_names)
        }
        if len(source_name_to_id) != len(source_names):
            raise ValueError("Source label names must be unique")
        target_name_to_id = {
            item.name.casefold(): item.class_id for item in schema.classes
        }
        lookup = np.full(len(source_names), schema.unknown_id, dtype=np.uint8)
        assigned: set[int] = set()
        for raw_target_name, raw_source_names in raw_mappings.items():
            target_name = str(raw_target_name).casefold()
            if target_name not in target_name_to_id:
                raise ValueError(f"Unknown navigation target class: {raw_target_name}")
            if not isinstance(raw_source_names, list):
                raise ValueError(f"Mapping for {raw_target_name} must be a list")
            for raw_source_name in raw_source_names:
                normalized = _normalized_name(str(raw_source_name))
                if normalized not in source_name_to_id:
                    raise ValueError(f"Unknown source class: {raw_source_name}")
                source_id = source_name_to_id[normalized]
                if source_id in assigned:
                    raise ValueError(f"Source class mapped more than once: {raw_source_name}")
                lookup[source_id] = target_name_to_id[target_name]
                assigned.add(source_id)
        return cls(
            source_dataset=str(document.get("source_dataset", "")).strip(),
            source_names=source_names,
            source_to_target=lookup,
        )

    def map_labels(self, source_label_image: NDArray) -> NDArray[np.uint8]:
        """Map an integer source label image to navigation class IDs."""
        labels = np.asarray(source_label_image)
        if labels.ndim != 2 or not np.issubdtype(labels.dtype, np.integer):
            raise ValueError("Source label image must be a two-dimensional integer array")
        if labels.size and (labels.min() < 0 or labels.max() >= len(self.source_names)):
            raise ValueError("Source label image contains an out-of-range class ID")
        return self.source_to_target[labels]
