"""Fail-closed helpers for live VLM shadow coordination.

Shadow coordination is deliberately not a command path.  It may compute and
display a would-be high-level target, but the only wire decision it is allowed
to publish is HOLD.  Keeping the display contract in this small module lets
the coordinator and Foxglove relay validate the same frame, calibration,
expiry and target fields.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np

from .map_snapshot import MapSnapshot


SHADOW_SCHEMA_VERSION = "focus-vlm-shadow-v1"
SHADOW_STATUS = "shadow_only_no_motion_authority"


@dataclass(frozen=True)
class ShadowTarget:
    robot_id: str
    frontier_id: str
    goal_category: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    expires_at_ns: int


def sha256_file(path: Path | str) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def filter_semantic_categories(
    grid: np.ndarray,
    category_names: tuple[str, ...],
    trusted_categories: tuple[str, ...],
) -> tuple[np.ndarray, dict[str, int]]:
    """Return a decision-only grid and counts hidden from VLM prompting.

    The source map is never modified.  Geometry/exploration remain exact;
    untrusted semantic channels are zeroed only in the copied shadow-decision
    tensor.  This is how an operator-confirmed RedNet false positive can be
    excluded from scheduling without deleting its provenance from the map.
    """

    evidence = np.asarray(grid)
    expected_channels = 2 + len(category_names)
    if evidence.ndim != 3 or evidence.shape[0] < expected_channels:
        raise ValueError(
            f"grid needs at least {expected_channels} channels, got {evidence.shape}"
        )
    trusted = set(trusted_categories)
    unknown = trusted.difference(category_names)
    if unknown:
        raise ValueError(f"unknown trusted semantic categories: {sorted(unknown)}")
    if not trusted:
        raise ValueError("at least one trusted semantic category is required")

    filtered = np.asarray(evidence, dtype=np.float32).copy()
    hidden_counts: dict[str, int] = {}
    for index, category in enumerate(category_names):
        if category in trusted:
            continue
        channel = filtered[2 + index]
        count = int(np.count_nonzero(channel > 0.1))
        if count:
            hidden_counts[category] = count
        channel.fill(0.0)
    return filtered, hidden_counts


def world_to_cell(
    xy_m: tuple[float, float],
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    shape_yx: tuple[int, int],
) -> tuple[int, int]:
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    col = int(math.floor((xy_m[0] - origin_xy_m[0]) / resolution_m))
    row = int(math.floor((xy_m[1] - origin_xy_m[1]) / resolution_m))
    height, width = shape_yx
    if not (0 <= row < height and 0 <= col < width):
        raise ValueError(
            f"robot position {xy_m} is outside fused grid origin={origin_xy_m}, "
            f"shape={shape_yx}, resolution={resolution_m}"
        )
    return row, col


def heading_deg_from_camera_pose(T_shared_camera: np.ndarray) -> float:
    """Match the live mapper's diagnostic camera-forward approximation."""

    transform = np.asarray(T_shared_camera, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_shared_camera must be a finite 4x4 matrix")
    forward_xy = transform[:2, 2]
    if float(np.linalg.norm(forward_xy)) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(float(forward_xy[1]), float(forward_xy[0])))


def collapse_detection_records(records: list[dict[str, object]]) -> dict[str, float]:
    """Convert persisted YOLO boxes to upstream's class->max confidence map."""

    collapsed: dict[str, float] = {}
    for record in records:
        class_name = str(record.get("class_name", ""))
        confidence = float(record.get("confidence", float("nan")))
        if not class_name or not math.isfinite(confidence):
            continue
        collapsed[class_name] = max(collapsed.get(class_name, 0.0), confidence)
    return collapsed


def validate_shadow_input_timing(
    capture_times_ns: list[int],
    *,
    now_ns: int,
    max_input_age_s: float,
    max_sync_skew_s: float,
    allow_stale_forensic_input: bool,
) -> dict[str, object]:
    """Reject stale or cross-robot asynchronous VLM inputs by default."""

    if len(capture_times_ns) < 2:
        raise ValueError("shadow timing validation requires at least two inputs")
    if now_ns <= 0 or max_input_age_s <= 0.0 or max_sync_skew_s < 0.0:
        raise ValueError("shadow timing thresholds must be valid and positive")
    if any(value <= 0 for value in capture_times_ns):
        raise ValueError("capture timestamps must be positive")

    ages_s = [(now_ns - value) / 1e9 for value in capture_times_ns]
    oldest_age_s = max(ages_s)
    sync_skew_s = (max(capture_times_ns) - min(capture_times_ns)) / 1e9
    violations: list[str] = []
    if min(ages_s) < -1.0:
        violations.append("capture timestamp is more than 1s in the future")
    if oldest_age_s > max_input_age_s:
        violations.append(
            f"oldest input age {oldest_age_s:.3f}s exceeds {max_input_age_s:.3f}s"
        )
    if sync_skew_s > max_sync_skew_s:
        violations.append(
            f"cross-robot capture skew {sync_skew_s:.3f}s exceeds "
            f"{max_sync_skew_s:.3f}s"
        )
    if violations and not allow_stale_forensic_input:
        raise ValueError("; ".join(violations))

    return {
        "status": (
            "accepted_stale_forensic_override"
            if violations
            else "accepted_fresh"
        ),
        "capture_times_ns": capture_times_ns,
        "input_ages_s": ages_s,
        "oldest_input_age_s": oldest_age_s,
        "cross_robot_capture_skew_s": sync_skew_s,
        "max_input_age_s": max_input_age_s,
        "max_sync_skew_s": max_sync_skew_s,
        "violations": violations,
    }


def build_shadow_target_payload(
    *,
    robot_id: str,
    frontier_id: str,
    goal_category: str,
    target_xy_m: tuple[float, float],
    yaw_rad: float,
    snapshot: MapSnapshot,
    created_at_ns: int,
    expires_at_ns: int,
    run_manifest: str,
    map_snapshot_sha256: str,
) -> dict[str, object]:
    if expires_at_ns <= created_at_ns:
        raise ValueError("shadow target expiry must follow creation")
    values = (*target_xy_m, yaw_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("shadow target pose must be finite")
    if not snapshot.shared_frame_calibration_id:
        raise ValueError("shadow target requires a verified shared calibration id")
    return {
        "schema_version": SHADOW_SCHEMA_VERSION,
        "status": SHADOW_STATUS,
        "mode": "SHADOW_WOULD_GOAL",
        "robot_id": robot_id,
        "frontier_id": frontier_id,
        "goal_category": goal_category,
        "frame_id": snapshot.frame_id,
        "transform_version": snapshot.transform_version,
        "shared_frame_calibration_id": snapshot.shared_frame_calibration_id,
        "created_at_ns": created_at_ns,
        "expires_at_ns": expires_at_ns,
        "target": {
            "x": float(target_xy_m[0]),
            "y": float(target_xy_m[1]),
            "z": 0.0,
            "yaw_rad": float(yaw_rad),
        },
        "run_manifest": run_manifest,
        "map_snapshot_sha256": map_snapshot_sha256,
        "authority": "display_only_never_robot_command",
    }


def validate_shadow_target_payload(
    payload: dict[str, object],
    *,
    robot_id: str,
    snapshot: MapSnapshot,
    now_ns: int,
) -> ShadowTarget:
    """Fail closed before a shadow target is rendered in Foxglove."""

    if payload.get("schema_version") != SHADOW_SCHEMA_VERSION:
        raise ValueError("unsupported shadow target schema")
    if payload.get("status") != SHADOW_STATUS:
        raise ValueError("shadow target is not active")
    if payload.get("mode") != "SHADOW_WOULD_GOAL":
        raise ValueError("shadow payload must never claim GOAL authority")
    if payload.get("authority") != "display_only_never_robot_command":
        raise ValueError("shadow target has no display-only authority marker")
    if payload.get("robot_id") != robot_id:
        raise ValueError("shadow target robot_id mismatch")
    if payload.get("frame_id") != snapshot.frame_id:
        raise ValueError("shadow target frame mismatch")
    if payload.get("transform_version") != snapshot.transform_version:
        raise ValueError("shadow target transform mismatch")
    if (
        payload.get("shared_frame_calibration_id")
        != snapshot.shared_frame_calibration_id
    ):
        raise ValueError("shadow target calibration mismatch")

    created_at_ns = int(payload.get("created_at_ns", 0))
    expires_at_ns = int(payload.get("expires_at_ns", 0))
    if created_at_ns <= 0 or expires_at_ns <= created_at_ns:
        raise ValueError("invalid shadow target timestamps")
    if expires_at_ns <= now_ns:
        raise ValueError("shadow target expired")

    target = payload.get("target")
    if not isinstance(target, dict):
        raise ValueError("shadow target pose is missing")
    x_m = float(target.get("x", float("nan")))
    y_m = float(target.get("y", float("nan")))
    z_m = float(target.get("z", float("nan")))
    yaw_rad = float(target.get("yaw_rad", float("nan")))
    if not all(math.isfinite(value) for value in (x_m, y_m, z_m, yaw_rad)):
        raise ValueError("shadow target pose contains non-finite values")

    frontier_id = str(payload.get("frontier_id", ""))
    goal_category = str(payload.get("goal_category", ""))
    if not frontier_id or not goal_category:
        raise ValueError("shadow target labels are missing")
    return ShadowTarget(
        robot_id=robot_id,
        frontier_id=frontier_id,
        goal_category=goal_category,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        yaw_rad=yaw_rad,
        expires_at_ns=expires_at_ns,
    )
