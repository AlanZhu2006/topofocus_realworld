"""Semantic class schema loading, validation, and colorization."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import yaml


@dataclass(frozen=True)
class SemanticClass:
    """One stable class ID and its planner/visualization metadata."""

    class_id: int
    name: str
    color_rgb: tuple[int, int, int]
    dynamic: bool = False

    def __post_init__(self) -> None:
        if not 0 <= self.class_id <= 255:
            raise ValueError("Semantic class IDs must fit mono8")
        if not self.name:
            raise ValueError("Semantic class name must not be empty")
        if len(self.color_rgb) != 3 or any(
            not isinstance(value, int) or not 0 <= value <= 255
            for value in self.color_rgb
        ):
            raise ValueError("color_rgb must contain three uint8 values")


@dataclass(frozen=True)
class SemanticClassSchema:
    """Versioned closed-set navigation class contract."""

    version: int
    classes: tuple[SemanticClass, ...]

    def __post_init__(self) -> None:
        if self.version <= 0:
            raise ValueError("Semantic class schema version must be positive")
        if not self.classes:
            raise ValueError("Semantic class schema must contain classes")
        ids = [item.class_id for item in self.classes]
        names = [item.name for item in self.classes]
        if len(set(ids)) != len(ids):
            raise ValueError("Semantic class IDs must be unique")
        if len(set(names)) != len(names):
            raise ValueError("Semantic class names must be unique")
        unknown = [item for item in self.classes if item.name == "unknown"]
        if len(unknown) != 1 or unknown[0].class_id != 0:
            raise ValueError("Class 0 must be the unique 'unknown' class")

    @property
    def class_names(self) -> dict[int, str]:
        return {item.class_id: item.name for item in self.classes}

    @property
    def unknown_id(self) -> int:
        return 0

    @property
    def dynamic_class_ids(self) -> frozenset[int]:
        return frozenset(item.class_id for item in self.classes if item.dynamic)

    def validate_labels(self, label_image: NDArray) -> None:
        labels = np.asarray(label_image)
        if labels.ndim != 2 or labels.dtype != np.uint8:
            raise ValueError("Semantic label image must be HxW uint8")
        valid_lookup = np.zeros(256, dtype=np.bool_)
        valid_lookup[list(self.class_names)] = True
        invalid = np.unique(labels[~valid_lookup[labels]])
        if invalid.size:
            raise ValueError(
                f"Semantic label image contains unknown IDs: {invalid.tolist()}"
            )

    def colorize(self, label_image: NDArray) -> NDArray[np.uint8]:
        self.validate_labels(label_image)
        lookup = np.zeros((256, 3), dtype=np.uint8)
        for item in self.classes:
            lookup[item.class_id] = item.color_rgb
        return lookup[np.asarray(label_image, dtype=np.uint8)]

    def to_metadata(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "classes": [
                {
                    "id": item.class_id,
                    "name": item.name,
                    "color_rgb": list(item.color_rgb),
                    "dynamic": item.dynamic,
                }
                for item in self.classes
            ],
        }

    @classmethod
    def from_yaml(cls, path: str | Path) -> "SemanticClassSchema":
        source = Path(path).expanduser()
        with source.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("Semantic class YAML must contain a mapping")
        raw_classes = document.get("classes")
        if not isinstance(raw_classes, dict):
            raise ValueError("Semantic class YAML is missing 'classes'")
        parsed: list[SemanticClass] = []
        for raw_id, raw_metadata in raw_classes.items():
            if not isinstance(raw_metadata, dict):
                raise ValueError(f"Class {raw_id!r} metadata must be a mapping")
            raw_color = raw_metadata.get("color_rgb")
            if not isinstance(raw_color, list):
                raise ValueError(f"Class {raw_id!r} is missing color_rgb")
            parsed.append(
                SemanticClass(
                    class_id=int(raw_id),
                    name=str(raw_metadata.get("name", "")),
                    color_rgb=tuple(int(value) for value in raw_color),
                    dynamic=bool(raw_metadata.get("dynamic", False)),
                )
            )
        parsed.sort(key=lambda item: item.class_id)
        return cls(version=int(document.get("version", 0)), classes=tuple(parsed))
