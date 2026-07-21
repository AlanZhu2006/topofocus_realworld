"""Persistence for the independent Phase-4 semantic voxel layer."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from semantic_mapping.semantic_bev_projector import SemanticBEV
from semantic_mapping.semantic_schema import SemanticClassSchema
from semantic_mapping.semantic_voxel_map import (
    SemanticVoxelConfig,
    SparseSemanticVoxelMap,
)


SEMANTIC_FORMAT_VERSION = 1


def save_semantic_voxel_map(
    directory: str | Path,
    semantic_map: SparseSemanticVoxelMap,
    schema: SemanticClassSchema,
    *,
    frame_id: str,
    timestamp_ns: int,
    bev: SemanticBEV | None = None,
) -> Path:
    """Save deterministic semantic scores without replacing geometry metadata."""
    output = Path(directory).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "semantic_voxels.npz", **semantic_map.to_arrays())
    if bev is not None:
        save_semantic_bev_products(output, bev)
    counts = semantic_map.counts()
    metadata: dict[str, Any] = {
        "format_version": SEMANTIC_FORMAT_VERSION,
        "map_type": "sparse_semantic_phase4",
        "frame_id": frame_id,
        "timestamp_ns": int(timestamp_ns),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "voxel": asdict(semantic_map.config),
        "semantic_schema": schema.to_metadata(),
        "counts": asdict(counts),
    }
    if bev is not None:
        metadata["bev"] = {
            "resolution_m": float(bev.resolution_m),
            "origin_xy": [float(value) for value in bev.origin_xy],
            "width": bev.width,
            "height": bev.height,
            "ground_z": float(bev.ground_z),
            "class_count": int(bev.semantic_scores.shape[2]),
        }
    with (output / "semantic_metadata.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    return output


def save_semantic_bev_products(directory: str | Path, bev: SemanticBEV) -> Path:
    """Persist semantic BEV channels without modifying sparse voxel evidence."""
    output = Path(directory).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    np.save(output / "semantic_bev.npy", bev.semantic_label)
    np.save(output / "semantic_confidence_bev.npy", bev.semantic_confidence)
    np.save(output / "semantic_explored_bev.npy", bev.explored)
    np.save(output / "semantic_height_min_bev.npy", bev.height_min)
    np.save(output / "semantic_height_max_bev.npy", bev.height_max)
    np.savez_compressed(
        output / "semantic_bev_tensor.npz",
        semantic_scores=bev.semantic_scores,
        semantic_label=bev.semantic_label,
        semantic_confidence=bev.semantic_confidence,
        explored=bev.explored,
        height_min=bev.height_min,
        height_max=bev.height_max,
        origin_xy=bev.origin_xy,
        resolution=np.float64(bev.resolution_m),
        ground_z=np.float64(bev.ground_z),
    )
    return output


def load_semantic_voxel_map(
    directory: str | Path,
) -> tuple[SparseSemanticVoxelMap, dict[str, Any]]:
    """Load semantic scores and their class/indexing contract."""
    source = Path(directory).expanduser()
    with (source / "semantic_metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError("semantic_metadata.yaml must contain a mapping")
    if metadata.get("format_version") != SEMANTIC_FORMAT_VERSION:
        raise ValueError(
            f"Unsupported semantic map format {metadata.get('format_version')!r}"
        )
    raw_config = metadata.get("voxel")
    if not isinstance(raw_config, dict):
        raise ValueError("semantic_metadata.yaml is missing voxel configuration")
    config = SemanticVoxelConfig(
        resolution_m=float(raw_config["resolution_m"]),
        origin_xyz=tuple(float(value) for value in raw_config["origin_xyz"]),
        class_count=int(raw_config["class_count"]),
        valid_class_ids=tuple(int(value) for value in raw_config["valid_class_ids"]),
        unknown_class_id=int(raw_config["unknown_class_id"]),
        dynamic_class_ids=tuple(
            int(value) for value in raw_config["dynamic_class_ids"]
        ),
        min_observations=int(raw_config["min_observations"]),
        confirmation_threshold=float(raw_config["confirmation_threshold"]),
    )
    with np.load(source / "semantic_voxels.npz", allow_pickle=False) as archive:
        semantic_map = SparseSemanticVoxelMap.from_arrays(
            config, {name: archive[name] for name in archive.files}
        )
    return semantic_map, metadata
