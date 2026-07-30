"""Immutable provenance contract for a finalized TinyNav saved map."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
from typing import Any, Iterable


SCHEMA_VERSION = "focus-tinynav-saved-map-v1"
REQUIRED_MAP_FILES = (
    "poses.npy",
    "intrinsics.npy",
    "baseline.npy",
    "T_rgb_to_infra1.npy",
    "rgb_camera_intrinsics.npy",
    "occupancy_grid.npy",
    "occupancy_meta.npy",
    "sdf_map.npy",
    "occupancy_2d_image.png",
    "depths.db",
    "features.db",
    "embeddings.db",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def artifact_record(path: Path, *, status: str) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    if not resolved.is_file() or resolved.is_symlink():
        raise ValueError(f"artifact must be a regular non-symlink file: {resolved}")
    size = resolved.stat().st_size
    if size <= 0:
        raise ValueError(f"artifact is empty: {resolved}")
    return {
        "source_path": str(resolved),
        "size_bytes": size,
        "sha256": sha256_file(resolved),
        "status": status,
    }


def _snapshot_digest(files: dict[str, dict[str, Any]]) -> str:
    encoded = json.dumps(
        {
            name: {
                "size_bytes": record["size_bytes"],
                "sha256": record["sha256"],
            }
            for name, record in sorted(files.items())
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def build_saved_map_manifest(
    map_directory: Path,
    *,
    source_files: Iterable[Path] = (),
    created_at_ns: int | None = None,
) -> dict[str, Any]:
    directory = map_directory.expanduser().resolve()
    if not directory.is_dir() or directory.is_symlink():
        raise ValueError(
            f"map directory must be a non-symlink directory: {directory}"
        )
    files = {
        name: artifact_record(
            directory / name,
            status="observed_finalized_map_artifact",
        )
        for name in REQUIRED_MAP_FILES
    }
    snapshot_sha256 = _snapshot_digest(files)
    sources: dict[str, dict[str, Any]] = {}
    for source in source_files:
        record = artifact_record(
            source,
            status="observed_runtime_source",
        )
        key = str(Path(record["source_path"]).name)
        if key in sources:
            raise ValueError(f"duplicate source basename in manifest: {key}")
        sources[key] = record
    return {
        "schema_version": SCHEMA_VERSION,
        "map_id": f"tinynav-saved-map-{snapshot_sha256[:16]}",
        "map_snapshot_sha256": snapshot_sha256,
        "map_directory_at_capture": str(directory),
        "map_frame": "map",
        "tracking_frame_at_build": "world",
        "created_at_ns": (
            time.time_ns() if created_at_ns is None else int(created_at_ns)
        ),
        "result_status": "source_derived_from_observed_finalized_map_files",
        "files": files,
        "runtime_sources": sources,
        "robot_commands_issued": False,
    }


def validate_saved_map_manifest(
    manifest_path: Path,
    *,
    map_directory: Path | None = None,
    verify_hashes: bool = True,
) -> dict[str, Any]:
    path = manifest_path.expanduser().resolve()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read saved-map manifest {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("saved-map manifest must be an object")
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported saved-map manifest schema")
    directory = (
        Path(str(payload.get("map_directory_at_capture", "")))
        if map_directory is None
        else map_directory
    ).expanduser().resolve()
    if not directory.is_dir() or directory.is_symlink():
        raise ValueError(f"saved-map directory is unavailable: {directory}")
    raw_files = payload.get("files")
    if not isinstance(raw_files, dict):
        raise ValueError("saved-map manifest files must be an object")
    checked: dict[str, dict[str, Any]] = {}
    for name in REQUIRED_MAP_FILES:
        raw = raw_files.get(name)
        if not isinstance(raw, dict):
            raise ValueError(f"saved-map manifest is missing {name}")
        candidate = directory / name
        if not candidate.is_file() or candidate.is_symlink():
            raise ValueError(f"saved-map file is unavailable: {candidate}")
        expected_size = raw.get("size_bytes")
        expected_sha256 = raw.get("sha256")
        if not isinstance(expected_size, int) or expected_size <= 0:
            raise ValueError(f"saved-map size contract is invalid for {name}")
        if (
            not isinstance(expected_sha256, str)
            or len(expected_sha256) != 64
            or any(char not in "0123456789abcdef" for char in expected_sha256)
        ):
            raise ValueError(f"saved-map SHA-256 contract is invalid for {name}")
        actual_size = candidate.stat().st_size
        if actual_size != expected_size:
            raise ValueError(
                f"saved-map size mismatch for {name}: "
                f"{actual_size} != {expected_size}"
            )
        if verify_hashes:
            actual_sha256 = sha256_file(candidate)
            if actual_sha256 != expected_sha256:
                raise ValueError(f"saved-map SHA-256 mismatch for {name}")
        checked[name] = {
            "size_bytes": expected_size,
            "sha256": expected_sha256,
        }
    snapshot_sha256 = _snapshot_digest(checked)
    if payload.get("map_snapshot_sha256") != snapshot_sha256:
        raise ValueError("saved-map snapshot digest is inconsistent")
    expected_map_id = f"tinynav-saved-map-{snapshot_sha256[:16]}"
    if payload.get("map_id") != expected_map_id:
        raise ValueError("saved-map ID is inconsistent")
    return payload
