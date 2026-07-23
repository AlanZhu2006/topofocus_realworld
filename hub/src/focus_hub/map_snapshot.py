"""Strict, read-only contract for Hub map snapshots.

Legacy snapshots predated explicit frame metadata and were silently treated as
``shared_world`` by the Foxglove relay.  That is acceptable only for an
operator-requested migration view; it is never sufficient evidence for
cross-robot fusion.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class MapSnapshot:
    grid: np.ndarray
    origin_xy_m: tuple[float, float]
    resolution_m: float
    frame_id: str
    transform_version: str
    shared_frame_calibration_id: str | None
    map_format_version: str | None
    snapshot_id: str | None = None
    legacy_contract: bool = False


def _scalar_string(data, key: str) -> str | None:
    if key not in data:
        return None
    value = str(data[key].item())
    return value or None


def load_map_snapshot(
    npz_path: Path | str, *, allow_legacy: bool = False
) -> MapSnapshot | None:
    """Load and validate one snapshot without inferring a fusion contract."""
    path = Path(npz_path)
    if not path.exists():
        return None
    with np.load(path, allow_pickle=False) as data:
        required = {"grid", "origin_xy_m", "resolution_m"}
        missing = required.difference(data.files)
        if missing:
            raise ValueError(f"map snapshot {path} is missing {sorted(missing)}")
        grid = np.asarray(data["grid"], dtype=np.float32)
        origin = np.asarray(data["origin_xy_m"], dtype=np.float64)
        resolution = float(data["resolution_m"])
        frame_id = _scalar_string(data, "frame_id")
        transform_version = _scalar_string(data, "transform_version")
        calibration_id = _scalar_string(data, "shared_frame_calibration_id")
        format_version = _scalar_string(data, "map_format_version")
        snapshot_id = _scalar_string(data, "snapshot_id")

    if grid.ndim != 3 or grid.shape[0] < 2 or any(size <= 0 for size in grid.shape):
        raise ValueError(f"map grid must have shape (channels>=2,H,W), got {grid.shape}")
    if not np.all(np.isfinite(grid)):
        raise ValueError(f"map grid contains non-finite values: {path}")
    if origin.shape != (2,) or not np.all(np.isfinite(origin)):
        raise ValueError(f"origin_xy_m must contain two finite values: {path}")
    if not np.isfinite(resolution) or resolution <= 0.0:
        raise ValueError(f"resolution_m must be finite and positive: {path}")

    legacy = frame_id is None or transform_version is None
    if legacy and not allow_legacy:
        missing_contract = [
            key
            for key, value in (
                ("frame_id", frame_id),
                ("transform_version", transform_version),
            )
            if value is None
        ]
        raise ValueError(
            f"legacy map snapshot {path} lacks {missing_contract}; "
            "regenerate it or explicitly allow an unverified per-robot view"
        )
    if frame_id is None:
        frame_id = "shared_world"
    if transform_version is None:
        transform_version = "legacy-unverified"

    return MapSnapshot(
        grid=grid,
        origin_xy_m=(float(origin[0]), float(origin[1])),
        resolution_m=resolution,
        frame_id=frame_id,
        transform_version=transform_version,
        shared_frame_calibration_id=calibration_id,
        map_format_version=format_version,
        snapshot_id=snapshot_id,
        legacy_contract=legacy,
    )


def validate_fusion_contract(
    snapshots: list[MapSnapshot],
) -> tuple[str, float, str]:
    """Return common frame/resolution/calibration, or fail closed."""
    if len(snapshots) < 2:
        raise ValueError("fusion requires at least two map snapshots")
    if any(snapshot.legacy_contract for snapshot in snapshots):
        raise ValueError("fusion refuses legacy snapshots with inferred metadata")

    reference_resolution = snapshots[0].resolution_m
    if any(
        not np.isclose(snapshot.resolution_m, reference_resolution, rtol=0.0, atol=1e-12)
        for snapshot in snapshots[1:]
    ):
        raise ValueError(
            "resolution mismatch across robots: "
            f"{[snapshot.resolution_m for snapshot in snapshots]}"
        )
    frame_ids = {snapshot.frame_id for snapshot in snapshots}
    if len(frame_ids) != 1:
        raise ValueError(f"frame mismatch across robots: {frame_ids}")
    calibration_ids = {
        snapshot.shared_frame_calibration_id for snapshot in snapshots
    }
    if None in calibration_ids or "" in calibration_ids:
        raise ValueError(
            "fusion blocked: every map must name the same verified "
            "shared_frame_calibration_id"
        )
    if len(calibration_ids) != 1:
        raise ValueError(
            f"shared-frame calibration mismatch across robots: {calibration_ids}"
        )
    calibration_id = calibration_ids.pop()
    if calibration_id is None:  # narrowed above; defensive for type checkers
        raise RuntimeError("fusion calibration contract became inconsistent")
    return frame_ids.pop(), reference_resolution, calibration_id
