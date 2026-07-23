"""Robot-local map alignment helpers for the v2 physical adapters.

The Hub publishes targets in ``shared_world``.  TinyNav and WATER both plan
in their own saved-map frames, so a receiver must derive and record
``shared_T_robot_map`` before reducing a target.  This module contains only
deterministic transform/provenance logic; it has no network, ROS or actuator
imports.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Sequence

from .geometry import compose_rigid, invert_rigid


IDENTITY: tuple[float, ...] = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)


@dataclass(frozen=True)
class SharedTrackingCalibration:
    robot_id: str
    tracking_frame: str
    transform_version: str
    shared_frame_calibration_id: str
    shared_T_tracking: tuple[float, ...]
    source_path: str
    source_size_bytes: int
    source_sha256: str
    provenance_status: str


def _validate_rigid(matrix: Sequence[float], *, label: str) -> tuple[float, ...]:
    if len(matrix) != 16:
        raise ValueError(f"{label} must contain 16 row-major values")
    values = tuple(float(value) for value in matrix)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains a non-finite value")
    if any(
        abs(values[12 + index] - expected) > 1e-5
        for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))
    ):
        raise ValueError(f"{label} last row is not [0,0,0,1]")
    rotation = (
        values[0:3],
        values[4:7],
        values[8:11],
    )
    for row in rotation:
        if abs(sum(component * component for component in row) - 1.0) > 2e-2:
            raise ValueError(f"{label} rotation row is not unit length")
    for left, right in ((0, 1), (0, 2), (1, 2)):
        if abs(sum(rotation[left][i] * rotation[right][i] for i in range(3))) > 2e-2:
            raise ValueError(f"{label} rotation is not orthogonal")
    return values


def planar_pose_matrix(x: float, y: float, yaw_rad: float, *, z: float = 0.0) -> tuple[float, ...]:
    if not all(math.isfinite(value) for value in (x, y, z, yaw_rad)):
        raise ValueError("planar pose contains a non-finite value")
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        cosine, -sine, 0.0, float(x),
        sine, cosine, 0.0, float(y),
        0.0, 0.0, 1.0, float(z),
        0.0, 0.0, 0.0, 1.0,
    )


def yaw_from_matrix(matrix: Sequence[float]) -> float:
    values = _validate_rigid(matrix, label="transform")
    return math.atan2(values[4], values[0])


def planarize_rigid(
    matrix: Sequence[float],
    *,
    label: str,
    max_tilt_deg: float = 5.0,
) -> tuple[float, ...]:
    """Project a near-planar body/map transform onto SE(2), preserving XYZ.

    The physical planners consume planar goals.  Projection is allowed only
    after bounding roll/pitch; it is never a fallback for an arbitrary 3-D
    transform.
    """

    values = _validate_rigid(matrix, label=label)
    z_alignment = max(-1.0, min(1.0, values[10]))
    tilt_deg = math.degrees(math.acos(z_alignment))
    if tilt_deg > max_tilt_deg:
        raise ValueError(
            f"{label} tilt {tilt_deg:.3f} deg exceeds {max_tilt_deg:.3f} deg"
        )
    return planar_pose_matrix(
        values[3], values[7], math.atan2(values[4], values[0]), z=values[11]
    )


def derive_shared_T_robot_map(
    *,
    shared_T_tracking: Sequence[float],
    tracking_T_body: Sequence[float],
    robot_map_T_body: Sequence[float],
    max_tilt_deg: float = 5.0,
) -> tuple[float, ...]:
    """Align two simultaneous body poses from tracking and planner maps.

    ``shared_T_map = shared_T_tracking @ tracking_T_body @ inv(map_T_body)``.
    This is the Yunji/Odin-to-WATER alignment and also works for any pair of
    robot-local planar localization systems.
    """

    shared_tracking = planarize_rigid(
        shared_T_tracking, label="shared_T_tracking", max_tilt_deg=max_tilt_deg
    )
    tracking_body = planarize_rigid(
        tracking_T_body, label="tracking_T_body", max_tilt_deg=max_tilt_deg
    )
    map_body = planarize_rigid(
        robot_map_T_body, label="robot_map_T_body", max_tilt_deg=max_tilt_deg
    )
    return compose_rigid(
        compose_rigid(shared_tracking, tracking_body), invert_rigid(map_body)
    )


def derive_shared_T_map_from_tracking_map(
    *,
    shared_T_tracking: Sequence[float],
    tracking_T_map: Sequence[float],
    max_tilt_deg: float = 5.0,
) -> tuple[float, ...]:
    """Compose a direct tracking-to-saved-map alignment (the TinyNav path)."""

    return compose_rigid(
        planarize_rigid(
            shared_T_tracking,
            label="shared_T_tracking",
            max_tilt_deg=max_tilt_deg,
        ),
        planarize_rigid(
            tracking_T_map,
            label="tracking_T_map",
            max_tilt_deg=max_tilt_deg,
        ),
    )


def load_shared_tracking_calibration(
    path: Path,
    *,
    robot_id: str,
    expected_transform_version: str,
    expected_calibration_id: str,
) -> SharedTrackingCalibration:
    """Load a measured shared-frame artifact.

    The original board artifact defines ``shared_world`` as the reference
    robot's tracking frame, so older artifacts legitimately omit an explicit
    reference transform and load as identity.  If that tracking process is
    restarted while the robot is held still, a versioned artifact may record
    ``shared_world_from_reference_tracking`` to hand the new tracking world
    back to the already-established shared frame.  The other robot continues
    to use the board-derived yaw-preserving transform.
    """

    resolved = path.expanduser().resolve()
    payload = resolved.read_bytes()
    raw = json.loads(payload)
    if raw.get("passed") is not True:
        raise ValueError("shared calibration artifact did not pass")
    calibration_id = str(raw.get("shared_frame_calibration_id", ""))
    if calibration_id != expected_calibration_id:
        raise ValueError("shared calibration ID mismatch")

    if raw.get("reference_robot") == robot_id:
        reference = raw.get("calibration_frame", {}).get("reference", {})
        transform_version = str(reference.get("transform_version", ""))
        handover = raw.get("shared_world_from_reference_tracking")
        if handover is None:
            tracking_frame = f"{robot_id}_tracking"
            matrix = IDENTITY
            provenance_status = (
                "observed_board_images_source_derived_alignment"
            )
        elif isinstance(handover, dict):
            matrix = tuple(handover.get("matrix", ()))
            tracking_frame = str(
                handover.get("child_frame", f"{robot_id}_tracking")
            )
            provenance_status = (
                "observed_stationary_pose_handover_source_derived_alignment"
            )
        else:
            raise ValueError(
                "shared_world_from_reference_tracking must be an object"
            )
    elif raw.get("other_robot") == robot_id:
        transform = raw.get("shared_world_from_other_odom", {})
        matrix = tuple(transform.get("matrix", ()))
        tracking_frame = str(transform.get("child_frame", f"{robot_id}_tracking"))
        transform_version = str(raw.get("transform_version", ""))
        provenance_status = "observed_board_images_source_derived_alignment"
    else:
        raise ValueError(f"calibration artifact does not contain {robot_id}")
    if transform_version != expected_transform_version:
        raise ValueError("shared tracking transform version mismatch")
    matrix = _validate_rigid(matrix, label="shared_T_tracking")
    return SharedTrackingCalibration(
        robot_id=robot_id,
        tracking_frame=tracking_frame,
        transform_version=transform_version,
        shared_frame_calibration_id=calibration_id,
        shared_T_tracking=matrix,
        source_path=str(resolved),
        source_size_bytes=len(payload),
        source_sha256=hashlib.sha256(payload).hexdigest(),
        provenance_status=provenance_status,
    )


def alignment_artifact(
    *,
    calibration: SharedTrackingCalibration,
    local_map_frame: str,
    shared_T_robot_map: Sequence[float],
    captured_at_ns: int,
    sample_skew_ns: int,
    max_sample_skew_ns: int,
    observed_inputs: dict[str, object],
) -> dict[str, object]:
    if sample_skew_ns < 0 or sample_skew_ns > max_sample_skew_ns:
        raise ValueError("map-alignment samples exceed the allowed skew")
    matrix = planarize_rigid(
        shared_T_robot_map, label="shared_T_robot_map", max_tilt_deg=5.0
    )
    return {
        "schema_version": "focus-robot-map-alignment-v1",
        "robot_id": calibration.robot_id,
        "frame_contract": {
            "parent_frame": "shared_world",
            "child_frame": local_map_frame,
            "matrix": list(matrix),
            "convention": "row_major_T_parent_child",
        },
        "transform_version": calibration.transform_version,
        "shared_frame_calibration_id": calibration.shared_frame_calibration_id,
        "captured_at_ns": int(captured_at_ns),
        "sample_skew_ns": int(sample_skew_ns),
        "max_sample_skew_ns": int(max_sample_skew_ns),
        "source_calibration": {
            "path": calibration.source_path,
            "size_bytes": calibration.source_size_bytes,
            "sha256": calibration.source_sha256,
            "status": calibration.provenance_status,
        },
        "observed_inputs": observed_inputs,
        "result_status": "source_derived_from_observed_localization_samples",
        "robot_commands_issued": False,
    }
