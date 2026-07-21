#!/usr/bin/env python3
"""Reproject saved semantic voxels onto the final saved occupancy BEV grid."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any

import yaml


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "semantic_mapping"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from semantic_mapping.semantic_bev_projector import (  # noqa: E402
    SemanticBEVGrid,
    SemanticBEVProjectionConfig,
    project_semantic_to_bev,
)
from semantic_mapping.semantic_map_serializer import (  # noqa: E402
    load_semantic_voxel_map,
    save_semantic_voxel_map,
)
from semantic_mapping.semantic_schema import (  # noqa: E402
    SemanticClass,
    SemanticClassSchema,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "map_directory",
        type=Path,
        help="TinyNav map directory or its semantic_mapping subdirectory",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PACKAGE_DIR / "config" / "semantic_mapping.yaml",
        help="Semantic mapping YAML used for height bands and class names",
    )
    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def resolve_semantic_directory(map_directory: Path) -> Path:
    source = map_directory.expanduser().resolve()
    candidate = source / "semantic_mapping"
    semantic_directory = candidate if candidate.is_dir() else source
    required = ("metadata.yaml", "semantic_metadata.yaml", "semantic_voxels.npz")
    missing = [name for name in required if not (semantic_directory / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"Semantic map is incomplete in {semantic_directory}: missing "
            + ", ".join(missing)
        )
    return semantic_directory


def schema_from_metadata(metadata: dict[str, Any]) -> SemanticClassSchema:
    raw_schema = metadata.get("semantic_schema")
    if not isinstance(raw_schema, dict):
        raise ValueError("semantic_metadata.yaml is missing semantic_schema")
    raw_classes = raw_schema.get("classes")
    if not isinstance(raw_classes, list):
        raise ValueError("semantic_schema.classes must be a list")
    classes: list[SemanticClass] = []
    for raw_class in raw_classes:
        if not isinstance(raw_class, dict):
            raise ValueError("semantic_schema.classes entries must be mappings")
        raw_color = raw_class.get("color_rgb")
        if not isinstance(raw_color, list):
            raise ValueError("semantic_schema class is missing color_rgb")
        classes.append(
            SemanticClass(
                class_id=int(raw_class["id"]),
                name=str(raw_class["name"]),
                color_rgb=tuple(int(value) for value in raw_color),
                dynamic=bool(raw_class.get("dynamic", False)),
            )
        )
    classes.sort(key=lambda item: item.class_id)
    return SemanticClassSchema(version=int(raw_schema["version"]), classes=tuple(classes))


def semantic_parameters(config_path: Path) -> dict[str, Any]:
    document = load_yaml(config_path)
    try:
        parameters = document["semantic_mapper_node"]["ros__parameters"]
    except KeyError as error:
        raise ValueError(
            f"{config_path} is missing semantic_mapper_node.ros__parameters"
        ) from error
    if not isinstance(parameters, dict):
        raise ValueError("semantic mapper parameters must be a mapping")
    return parameters


def occupancy_grid_from_metadata(
    metadata: dict[str, Any],
) -> tuple[SemanticBEVGrid, float, str, int]:
    raw_bev = metadata.get("bev")
    if not isinstance(raw_bev, dict):
        raise ValueError("metadata.yaml is missing final occupancy BEV geometry")
    raw_origin = raw_bev.get("origin_xy")
    if not isinstance(raw_origin, list) or len(raw_origin) != 2:
        raise ValueError("metadata.yaml bev.origin_xy must contain two values")
    frame_id = metadata.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("metadata.yaml is missing frame_id")
    return (
        SemanticBEVGrid(
            origin_xy=(float(raw_origin[0]), float(raw_origin[1])),
            resolution_m=float(raw_bev["resolution_m"]),
            width=int(raw_bev["width"]),
            height=int(raw_bev["height"]),
        ),
        float(raw_bev["ground_z"]),
        frame_id,
        int(metadata["timestamp_ns"]),
    )


def projection_config(
    parameters: dict[str, Any], ground_z: float
) -> SemanticBEVProjectionConfig:
    return SemanticBEVProjectionConfig(
        resolution_m=float(parameters["bev.resolution_m"]),
        ground_z=ground_z,
        ground_min_z_relative=float(parameters["bev.ground_min_z_relative"]),
        ground_max_z_relative=float(parameters["bev.ground_max_z_relative"]),
        semantic_min_z_relative=float(parameters["bev.semantic_min_z_relative"]),
        semantic_max_z_relative=float(parameters["bev.semantic_max_z_relative"]),
        ignore_above_z_relative=float(parameters["bev.ignore_above_z_relative"]),
        padding_cells=int(parameters["bev.padding_cells"]),
        min_cell_confidence=float(parameters["bev.min_cell_confidence"]),
    )


def floor_class_id(schema: SemanticClassSchema, parameters: dict[str, Any]) -> int | None:
    floor_name = str(parameters["bev.floor_class_name"])
    return next(
        (item.class_id for item in schema.classes if item.name == floor_name), None
    )


def export_semantic_bev(map_directory: Path, config_path: Path) -> Path:
    """Overwrite semantic BEV products so their grid equals final occupancy BEV."""
    semantic_directory = resolve_semantic_directory(map_directory)
    config_path = config_path.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Semantic mapping config does not exist: {config_path}")

    occupancy_metadata = load_yaml(semantic_directory / "metadata.yaml")
    semantic_map, semantic_metadata = load_semantic_voxel_map(semantic_directory)
    schema = schema_from_metadata(semantic_metadata)
    parameters = semantic_parameters(config_path)
    grid, ground_z, frame_id, timestamp_ns = occupancy_grid_from_metadata(
        occupancy_metadata
    )
    semantic_frame = semantic_metadata.get("frame_id")
    if semantic_frame != frame_id:
        raise ValueError(
            "Semantic and occupancy frame IDs differ: "
            f"{semantic_frame!r} != {frame_id!r}"
        )
    bev = project_semantic_to_bev(
        semantic_map,
        projection_config(parameters, ground_z),
        grid=grid,
        floor_class_id=floor_class_id(schema, parameters),
    )
    save_semantic_voxel_map(
        semantic_directory,
        semantic_map,
        schema,
        frame_id=frame_id,
        timestamp_ns=timestamp_ns,
        bev=bev,
    )
    return semantic_directory


def main() -> int:
    args = parse_args()
    output = export_semantic_bev(args.map_directory, args.config)
    with (output / "semantic_metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    bev = metadata["bev"]
    print(
        "Exported semantic BEV: "
        f"{bev['width']}x{bev['height']}, resolution={bev['resolution_m']:.3f} m, "
        f"ground_z={bev['ground_z']:.3f} m"
    )
    print(f"  output: {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, KeyError) as error:
        print(f"export_semantic_bev: {error}", file=sys.stderr)
        raise SystemExit(2) from error
