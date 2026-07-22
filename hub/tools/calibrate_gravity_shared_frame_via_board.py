#!/usr/bin/env python3
"""Build a gravity-preserving shared frame from a board seen by both robots.

Unlike ``calibrate_shared_frame.py``'s unconstrained camera-pose alignment,
this read-only tool reuses the existing symmetric-circle-board PnP solver and
aligns the *board pose* with a yaw-only transform.  Board origins match
exactly at calibration while shared +Z remains gravity +Z for every future
robot yaw.  Optional moved-board frames provide an independent holdout gate.

No robot interface is opened and no motion command exists in this program.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "hub" / "tools"))

from calibrate_camera_offset_via_board import find_board_pose  # noqa: E402
from focus_hub.calibration import (  # noqa: E402
    compute_gravity_preserving_alignment,
    gravity_tilt_deg,
)
from focus_hub.models import ObservationMetadata  # noqa: E402


def file_artifact(path: Path) -> dict:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": digest.hexdigest(),
    }


def read_metadata(spool: Path, robot_id: str, sequence: int):
    directory = spool / robot_id / f"{sequence:020d}"
    metadata_path = directory / "metadata.json"
    image_path = directory / "rgb.jpg"
    if not metadata_path.is_file() or not image_path.is_file():
        raise SystemExit(f"missing observation files under {directory}")
    metadata = ObservationMetadata.model_validate_json(
        metadata_path.read_text(encoding="utf-8")
    )
    pose = np.asarray(metadata.pose.shared_T_camera.matrix, dtype=np.float64).reshape(
        4, 4
    )
    intrinsics = metadata.intrinsics
    matrix = np.array(
        [
            [intrinsics.fx, 0.0, intrinsics.cx],
            [0.0, intrinsics.fy, intrinsics.cy],
            [0.0, 0.0, 1.0],
        ]
    )
    distortion = np.asarray(intrinsics.distortion or [0.0] * 5, dtype=np.float64)
    return directory, metadata, pose, matrix, distortion


def load_extrinsic(path: Path) -> np.ndarray:
    artifact = json.loads(path.read_text(encoding="utf-8"))
    values = artifact["base_link_from_camera"]["matrix"]
    matrix = np.asarray(values, dtype=np.float64).reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise SystemExit(f"non-finite extrinsic: {path}")
    return matrix


def load_shared_transform(path: Path | None) -> np.ndarray:
    if path is None:
        return np.eye(4)
    artifact = json.loads(path.read_text(encoding="utf-8"))
    values = artifact["shared_world_from_other_odom"]["matrix"]
    return np.asarray(values, dtype=np.float64).reshape(4, 4)


def board_pose(
    directory: Path,
    matrix: np.ndarray,
    distortion: np.ndarray,
    rows: int,
    cols: int,
    spacing_m: float,
) -> np.ndarray:
    return find_board_pose(
        str(directory / "rgb.jpg"),
        rows,
        cols,
        spacing_m,
        matrix,
        distortion,
    )


def angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(
        np.clip(
            abs(np.dot(first, second))
            / (np.linalg.norm(first) * np.linalg.norm(second)),
            -1.0,
            1.0,
        )
    )
    return math.degrees(math.acos(cosine))


def other_local_board(
    recorded_camera_pose: np.ndarray,
    old_shared_transform: np.ndarray,
    old_extrinsic: np.ndarray,
    corrected_extrinsic: np.ndarray,
    camera_from_board: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    odom_from_old_camera = np.linalg.inv(old_shared_transform) @ recorded_camera_pose
    odom_from_base = odom_from_old_camera @ np.linalg.inv(old_extrinsic)
    odom_from_new_camera = odom_from_base @ corrected_extrinsic
    return odom_from_new_camera @ camera_from_board, odom_from_new_camera


def observation_provenance(directory: Path, metadata: ObservationMetadata) -> dict:
    return {
        "sequence": metadata.sequence,
        "capture_time_ns": metadata.capture_time_ns,
        "transform_version": metadata.pose.transform_version,
        "metadata": file_artifact(directory / "metadata.json"),
        "rgb": file_artifact(directory / "rgb.jpg"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--reference-robot", default="robot-0")
    parser.add_argument("--other-robot", default="robot-1")
    parser.add_argument("--reference-sequence", type=int, required=True)
    parser.add_argument("--other-sequence", type=int, required=True)
    parser.add_argument("--old-other-extrinsic", type=Path, required=True)
    parser.add_argument("--corrected-other-extrinsic", type=Path, required=True)
    parser.add_argument(
        "--other-recorded-shared-transform",
        type=Path,
        default=None,
        help="transform already applied to the calibration observation; omit for raw odom",
    )
    parser.add_argument("--holdout-reference-sequence", type=int)
    parser.add_argument("--holdout-other-sequence", type=int)
    parser.add_argument("--holdout-other-recorded-shared-transform", type=Path)
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--cols", type=int, default=10)
    parser.add_argument("--spacing-m", type=float, default=0.04)
    parser.add_argument("--transform-version", required=True)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--max-sync-skew-s", type=float, default=0.25)
    parser.add_argument("--max-holdout-center-residual-m", type=float, default=0.05)
    parser.add_argument("--max-holdout-normal-residual-deg", type=float, default=3.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    holdout_values = (
        args.holdout_reference_sequence,
        args.holdout_other_sequence,
        args.holdout_other_recorded_shared_transform,
    )
    if any(value is not None for value in holdout_values) and not all(
        value is not None for value in holdout_values
    ):
        parser.error(
            "holdout requires both sequences and "
            "--holdout-other-recorded-shared-transform"
        )
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")

    old_extrinsic = load_extrinsic(args.old_other_extrinsic)
    corrected_extrinsic = load_extrinsic(args.corrected_other_extrinsic)
    recorded_transform = load_shared_transform(args.other_recorded_shared_transform)

    ref_dir, ref_meta, ref_camera, ref_k, ref_dist = read_metadata(
        args.spool, args.reference_robot, args.reference_sequence
    )
    other_dir, other_meta, other_camera, other_k, other_dist = read_metadata(
        args.spool, args.other_robot, args.other_sequence
    )
    sync_skew_s = abs(ref_meta.capture_time_ns - other_meta.capture_time_ns) / 1e9
    if sync_skew_s > args.max_sync_skew_s:
        raise SystemExit(
            f"calibration sync skew {sync_skew_s:.3f}s exceeds {args.max_sync_skew_s:.3f}s"
        )
    ref_camera_board = board_pose(
        ref_dir, ref_k, ref_dist, args.rows, args.cols, args.spacing_m
    )
    other_camera_board = board_pose(
        other_dir, other_k, other_dist, args.rows, args.cols, args.spacing_m
    )
    reference_board = ref_camera @ ref_camera_board
    local_board, corrected_camera_at_sync = other_local_board(
        other_camera,
        recorded_transform,
        old_extrinsic,
        corrected_extrinsic,
        other_camera_board,
    )
    transform = np.asarray(
        compute_gravity_preserving_alignment(
            reference_board.reshape(-1), local_board.reshape(-1)
        ),
        dtype=np.float64,
    ).reshape(4, 4)
    mapped_board = transform @ local_board
    calibration_center_residual = float(
        np.linalg.norm(reference_board[:3, 3] - mapped_board[:3, 3])
    )
    calibration_normal_residual = angle_deg(reference_board[:3, 2], mapped_board[:3, 2])

    holdout = None
    passed = True
    if args.holdout_reference_sequence is not None:
        holdout_transform = load_shared_transform(
            args.holdout_other_recorded_shared_transform
        )
        href_dir, href_meta, href_camera, href_k, href_dist = read_metadata(
            args.spool,
            args.reference_robot,
            args.holdout_reference_sequence,
        )
        hother_dir, hother_meta, hother_camera, hother_k, hother_dist = read_metadata(
            args.spool,
            args.other_robot,
            args.holdout_other_sequence,
        )
        holdout_skew_s = (
            abs(href_meta.capture_time_ns - hother_meta.capture_time_ns) / 1e9
        )
        href_camera_board = board_pose(
            href_dir, href_k, href_dist, args.rows, args.cols, args.spacing_m
        )
        hother_camera_board = board_pose(
            hother_dir, hother_k, hother_dist, args.rows, args.cols, args.spacing_m
        )
        href_board = href_camera @ href_camera_board
        hother_board, _ = other_local_board(
            hother_camera,
            holdout_transform,
            old_extrinsic,
            corrected_extrinsic,
            hother_camera_board,
        )
        mapped_holdout = transform @ hother_board
        center_residual = float(
            np.linalg.norm(href_board[:3, 3] - mapped_holdout[:3, 3])
        )
        normal_residual = angle_deg(href_board[:3, 2], mapped_holdout[:3, 2])
        checks = {
            "sync_skew": holdout_skew_s <= args.max_sync_skew_s,
            "board_center_residual": center_residual
            <= args.max_holdout_center_residual_m,
            "board_normal_residual": normal_residual
            <= args.max_holdout_normal_residual_deg,
        }
        passed = all(checks.values())
        holdout = {
            "reference": observation_provenance(href_dir, href_meta),
            "other": observation_provenance(hother_dir, hother_meta),
            "sync_skew_s": holdout_skew_s,
            "board_center_translation_residual_m": center_residual,
            "board_normal_residual_deg": normal_residual,
            "checks": checks,
        }

    tilt = gravity_tilt_deg(transform.reshape(-1))
    passed = passed and tilt <= 1e-7
    output = {
        "schema_version": 2,
        "computed_at_ns": time.time_ns(),
        "reference_robot": args.reference_robot,
        "other_robot": args.other_robot,
        "transform_version": args.transform_version,
        "shared_frame_calibration_id": args.calibration_id,
        "calibration_method": "shared_circle_board_yaw_only_gravity_preserving",
        "shared_world_from_other_odom": {
            "parent_frame": "shared_world",
            "child_frame": f"{args.other_robot}_odom",
            "matrix": transform.reshape(-1).tolist(),
        },
        "corrected_other_camera_pose_at_sync": {
            "matrix": corrected_camera_at_sync.reshape(-1).tolist()
        },
        "board": {
            "type": "symmetric_circle_grid",
            "rows": args.rows,
            "cols": args.cols,
            "spacing_m": args.spacing_m,
        },
        "calibration_frame": {
            "reference": observation_provenance(ref_dir, ref_meta),
            "other": observation_provenance(other_dir, other_meta),
            "sync_skew_s": sync_skew_s,
            "board_center_translation_residual_m": calibration_center_residual,
            "board_normal_residual_deg": calibration_normal_residual,
        },
        "gravity_validation": {
            "shared_transform_tilt_deg": tilt,
            "passed": tilt <= 1e-7,
        },
        "holdout_validation": holdout,
        "passed": passed,
        "thresholds": {
            "max_sync_skew_s": args.max_sync_skew_s,
            "max_holdout_center_residual_m": args.max_holdout_center_residual_m,
            "max_holdout_normal_residual_deg": args.max_holdout_normal_residual_deg,
        },
        "input_provenance": {
            "old_other_extrinsic": file_artifact(args.old_other_extrinsic),
            "corrected_other_extrinsic": file_artifact(args.corrected_other_extrinsic),
            "calibration_recorded_shared_transform": (
                file_artifact(args.other_recorded_shared_transform)
                if args.other_recorded_shared_transform is not None
                else None
            ),
            "holdout_recorded_shared_transform": (
                file_artifact(args.holdout_other_recorded_shared_transform)
                if args.holdout_other_recorded_shared_transform is not None
                else None
            ),
            "status": "observed_spooled_images_and_source_derived_rigid_alignment",
        },
        "safety": {
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
            "archived_observations_only": True,
        },
        "note": (
            "Apply this transform after the corrected base_link-camera extrinsic. "
            "The rotation is yaw-only; it cannot rotate gravity when the base yaws."
        ),
    }
    if not passed:
        raise SystemExit("gravity shared-frame calibration failed validation gates")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"wrote {args.output}; holdout={holdout is not None}, tilt={tilt:.9f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
