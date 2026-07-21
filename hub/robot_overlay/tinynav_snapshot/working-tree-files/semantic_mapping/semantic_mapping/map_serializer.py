"""Persistence for Phase-2 sparse occupancy maps and BEV products."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from semantic_mapping.bev_projector import OccupancyBEV
from semantic_mapping.occupancy_voxel_map import (
    OccupancyVoxelConfig,
    SparseOccupancyVoxelMap,
)


FORMAT_VERSION = 1


def save_occupancy_map(
    directory: str | Path,
    voxel_map: SparseOccupancyVoxelMap,
    bev: OccupancyBEV,
    *,
    frame_id: str,
    timestamp_ns: int,
    ground_z: float,
) -> Path:
    """Save deterministic geometry products for later loading and planning."""
    output = Path(directory).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    arrays = voxel_map.to_arrays()
    np.savez_compressed(output / "voxels.npz", **arrays)
    np.save(output / "occupancy_bev.npy", bev.occupancy_grid)
    np.save(output / "occupancy_probability_bev.npy", bev.occupancy_probability)
    np.save(output / "free_bev.npy", bev.free_probability)
    np.save(output / "explored_bev.npy", bev.explored)
    np.save(output / "height_min_bev.npy", bev.height_min)
    np.save(output / "height_max_bev.npy", bev.height_max)
    np.savez_compressed(
        output / "planner_tensor.npz",
        occupancy=bev.occupancy_probability,
        free=bev.free_probability,
        explored=bev.explored,
        occupancy_grid=bev.occupancy_grid,
        height_min=bev.height_min,
        height_max=bev.height_max,
        origin_xy=bev.origin_xy,
        resolution=np.float64(bev.resolution_m),
    )

    metadata: dict[str, Any] = {
        "format_version": FORMAT_VERSION,
        "map_type": "sparse_occupancy_phase2",
        "frame_id": frame_id,
        "timestamp_ns": int(timestamp_ns),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "voxel": asdict(voxel_map.config),
        "bev": {
            "resolution_m": float(bev.resolution_m),
            "origin_xy": [float(value) for value in bev.origin_xy],
            "width": bev.width,
            "height": bev.height,
            "ground_z": float(ground_z),
        },
        "counts": asdict(voxel_map.counts()),
    }
    with (output / "metadata.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    return output


def load_occupancy_voxel_map(
    directory: str | Path,
) -> tuple[SparseOccupancyVoxelMap, dict[str, Any]]:
    """Load a sparse map and its metadata, validating the format version."""
    source = Path(directory).expanduser()
    with (source / "metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError("metadata.yaml must contain a mapping")
    if metadata.get("format_version") != FORMAT_VERSION:
        raise ValueError(
            f"Unsupported occupancy map format {metadata.get('format_version')!r}"
        )
    voxel_metadata = metadata.get("voxel")
    if not isinstance(voxel_metadata, dict):
        raise ValueError("metadata.yaml is missing voxel configuration")
    config = OccupancyVoxelConfig(
        resolution_m=float(voxel_metadata["resolution_m"]),
        origin_xyz=tuple(float(value) for value in voxel_metadata["origin_xyz"]),
        free_update=float(voxel_metadata["free_update"]),
        occupied_update=float(voxel_metadata["occupied_update"]),
        min_log_odds=float(voxel_metadata["min_log_odds"]),
        max_log_odds=float(voxel_metadata["max_log_odds"]),
        free_threshold=float(voxel_metadata["free_threshold"]),
        occupied_threshold=float(voxel_metadata["occupied_threshold"]),
        truncation_distance_m=float(
            voxel_metadata["truncation_distance_m"]
        ),
    )
    with np.load(source / "voxels.npz", allow_pickle=False) as archive:
        arrays = {name: archive[name] for name in archive.files}
    return SparseOccupancyVoxelMap.from_arrays(config, arrays), metadata
