from pathlib import Path

import numpy as np
import pytest
import yaml

from semantic_mapping.semantic_label_mapping import SemanticLabelMapping
from semantic_mapping.semantic_schema import SemanticClassSchema


CLASSES_PATH = Path(__file__).parents[1] / "config" / "semantic_classes.yaml"


def _write_mapping(path: Path, mappings: dict[str, list[str]]) -> None:
    document = {
        "version": 1,
        "source_dataset": "test",
        "semantic_classes_version": 1,
        "mappings": mappings,
    }
    path.write_text(yaml.safe_dump(document), encoding="utf-8")


def test_name_based_mapping_and_unknown_fallback(tmp_path: Path) -> None:
    schema = SemanticClassSchema.from_yaml(CLASSES_PATH)
    mapping_path = tmp_path / "mapping.yaml"
    _write_mapping(
        mapping_path,
        {"wall": ["wall"], "floor": ["floor"], "dynamic_object": ["person"]},
    )
    mapping = SemanticLabelMapping.from_yaml(
        mapping_path,
        schema,
        {0: "wall", 1: "floor", 2: "person", 3: "sky"},
    )

    result = mapping.map_labels(np.array([[0, 1, 2, 3]], dtype=np.int64))

    np.testing.assert_array_equal(result, [[2, 1, 10, 0]])


def test_mapping_rejects_duplicate_source_assignment(tmp_path: Path) -> None:
    schema = SemanticClassSchema.from_yaml(CLASSES_PATH)
    mapping_path = tmp_path / "mapping.yaml"
    _write_mapping(mapping_path, {"wall": ["wall"], "floor": ["wall"]})

    with pytest.raises(ValueError, match="more than once"):
        SemanticLabelMapping.from_yaml(mapping_path, schema, {0: "wall"})
