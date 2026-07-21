"""Adapt a finalized TinyNav BuildMap occupancy volume to the Hub grid.

TinyNav saves ``occupancy_grid.npy`` as an ``[x, y, z]`` uint8 array with
three states: 0 unknown, 1 free, and 2 occupied.  The Hub/Foxglove grid uses
``[channel, row=y, column=x]`` and separates obstacle and explored evidence.

This module deliberately does not infer semantics or modify the native map.
It is a deterministic, read-only projection of BuildMapNode's saved result.
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .central_mapping import HM3D_CATEGORY_NAMES


TINYNAV_UNKNOWN = 0
TINYNAV_FREE = 1
TINYNAV_OCCUPIED = 2


@dataclass(frozen=True)
class TinyNavOccupancy:
    grid_xyz: np.ndarray
    origin_xyz_m: tuple[float, float, float]
    resolution_m: float
    grid_path: Path
    meta_path: Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_tinynav_occupancy(record_dir: Path | str) -> TinyNavOccupancy:
    """Load and strictly validate finalized BuildMapNode occupancy files."""
    record_dir = Path(record_dir)
    grid_path = record_dir / "occupancy_grid.npy"
    meta_path = record_dir / "occupancy_meta.npy"
    if not grid_path.is_file() or not meta_path.is_file():
        raise FileNotFoundError(
            "a finalized TinyNav map requires occupancy_grid.npy and "
            f"occupancy_meta.npy under {record_dir}"
        )

    grid = np.load(grid_path, allow_pickle=False)
    meta = np.load(meta_path, allow_pickle=False)
    if grid.ndim != 3 or any(size <= 0 for size in grid.shape):
        raise ValueError(f"occupancy_grid.npy must be a non-empty 3-D array, got {grid.shape}")
    if not np.issubdtype(grid.dtype, np.integer):
        raise ValueError(f"occupancy_grid.npy must have an integer dtype, got {grid.dtype}")
    values = np.unique(grid)
    if not np.all(np.isin(values, (TINYNAV_UNKNOWN, TINYNAV_FREE, TINYNAV_OCCUPIED))):
        raise ValueError(f"occupancy_grid.npy contains values outside {{0,1,2}}: {values.tolist()}")

    meta = np.asarray(meta, dtype=np.float64)
    if meta.shape != (4,):
        raise ValueError(f"occupancy_meta.npy must be [origin_x,y,z,resolution], got {meta.shape}")
    if not np.all(np.isfinite(meta)) or meta[3] <= 0.0:
        raise ValueError(f"occupancy_meta.npy must be finite with positive resolution, got {meta}")

    return TinyNavOccupancy(
        grid_xyz=np.asarray(grid, dtype=np.uint8),
        origin_xyz_m=(float(meta[0]), float(meta[1]), float(meta[2])),
        resolution_m=float(meta[3]),
        grid_path=grid_path.resolve(),
        meta_path=meta_path.resolve(),
    )


def project_tinynav_occupancy(native: TinyNavOccupancy) -> np.ndarray:
    """Return a Hub grid without changing TinyNav's occupancy decisions.

    TinyNav's own 2-D export is ``max(grid_xyz, axis=2)``: any occupied voxel
    makes the XY cell occupied; otherwise any free voxel makes it known-free.
    Transpose converts native ``[x,y]`` into Hub ``[row=y,column=x]``.
    """
    plane_yx = np.ascontiguousarray(np.max(native.grid_xyz, axis=2).T)
    grid = np.zeros(
        (2 + len(HM3D_CATEGORY_NAMES), plane_yx.shape[0], plane_yx.shape[1]),
        dtype=np.float32,
    )
    grid[0] = plane_yx == TINYNAV_OCCUPIED
    grid[1] = plane_yx != TINYNAV_UNKNOWN
    return grid


def _write_json_atomic(path: Path, value: dict) -> None:
    tmp_path = path.with_name(path.name + ".tmp")
    tmp_path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp_path, path)


def write_hub_snapshot(
    native: TinyNavOccupancy,
    out_dir: Path | str,
    *,
    robot_id: str,
    frame_id: str,
    transform_version: str,
) -> dict:
    """Write an atomically-readable Hub/Foxglove snapshot and provenance."""
    if not robot_id or not frame_id or not transform_version:
        raise ValueError("robot_id, frame_id, and transform_version must be non-empty")

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    map_path = out_dir / "central_map.npz"
    summary_path = out_dir / "map_summary.json"
    if map_path.exists() or summary_path.exists():
        raise FileExistsError(f"refusing to overwrite an existing map snapshot in {out_dir}")

    grid = project_tinynav_occupancy(native)
    obstacle = grid[0] > 0.5
    explored = grid[1] > 0.5
    native_plane_xy = np.max(native.grid_xyz, axis=2)
    source_files = {
        "occupancy_grid.npy": {
            "path": str(native.grid_path),
            "size_bytes": native.grid_path.stat().st_size,
            "sha256": _sha256(native.grid_path),
        },
        "occupancy_meta.npy": {
            "path": str(native.meta_path),
            "size_bytes": native.meta_path.stat().st_size,
            "sha256": _sha256(native.meta_path),
        },
    }
    summary = {
        "robot_id": robot_id,
        "source_kind": "tinynav_build_map_native_occupancy",
        "source_status": "observed_finalized_artifact",
        "transform_version": transform_version,
        "frame_id": frame_id,
        "native_shape_xyz": list(native.grid_xyz.shape),
        "grid_shape_yx": list(grid.shape[1:]),
        "native_origin_xyz_m": list(native.origin_xyz_m),
        "origin_xy_m": list(native.origin_xyz_m[:2]),
        "resolution_m": native.resolution_m,
        "projection": "max(native_grid_xyz,axis=z).transpose(xy->yx)",
        "unknown_cells": int((native_plane_xy == TINYNAV_UNKNOWN).sum()),
        "free_cells": int((native_plane_xy == TINYNAV_FREE).sum()),
        "occupied_cells": int((native_plane_xy == TINYNAV_OCCUPIED).sum()),
        "obstacle_cells": int(obstacle.sum()),
        "explored_cells": int(explored.sum()),
        "semantic_cells": 0,
        "source_files": source_files,
    }

    tmp_map_path = out_dir / "central_map.tmp.npz"
    try:
        np.savez_compressed(
            tmp_map_path,
            grid=grid,
            origin_xy_m=np.asarray(native.origin_xyz_m[:2], dtype=np.float64),
            resolution_m=np.asarray(native.resolution_m, dtype=np.float64),
            frame_id=np.asarray(frame_id),
            transform_version=np.asarray(transform_version),
            source_kind=np.asarray("tinynav_build_map_native_occupancy"),
            native_origin_xyz_m=np.asarray(native.origin_xyz_m, dtype=np.float64),
        )
        os.replace(tmp_map_path, map_path)
        _write_json_atomic(summary_path, summary)
    finally:
        tmp_map_path.unlink(missing_ok=True)
        (out_dir / "map_summary.json.tmp").unlink(missing_ok=True)
    return summary

