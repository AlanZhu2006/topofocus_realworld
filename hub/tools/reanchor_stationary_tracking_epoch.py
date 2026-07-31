#!/usr/bin/env python3
"""Re-anchor one restarted tracking epoch while the robot stayed stationary.

The input observations were encoded with the previous shared transform.  This
tool estimates a planar correction that maps the post-restart camera anchor
back to the pre-restart camera anchor, then composes that correction with the
board-derived tracking transform.  It never contacts a robot or emits a
command.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Iterable

import numpy as np


CONFIRMATION = "OPERATOR_CONFIRMS_ROBOT_STATIONARY_ACROSS_TRACKING_RESTART"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_identity(path: Path, *, workspace: Path) -> dict[str, object]:
    resolved = path.resolve()
    try:
        display_path = str(resolved.relative_to(workspace.resolve()))
    except ValueError:
        display_path = str(resolved)
    return {
        "path": display_path,
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def resolve_artifact_identity(
    identity: object, *, workspace: Path, label: str
) -> Path:
    if not isinstance(identity, dict):
        raise ValueError(f"{label} identity must be an object")
    raw_path = identity.get("path")
    if not isinstance(raw_path, str) or not raw_path:
        raise ValueError(f"{label} identity lacks a path")
    path = Path(raw_path)
    resolved = (
        path.resolve()
        if path.is_absolute()
        else (workspace / path).resolve()
    )
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    if resolved.stat().st_size != int(identity.get("size_bytes", -1)):
        raise ValueError(f"{label} artifact size drift")
    if sha256_file(resolved) != str(identity.get("sha256", "")):
        raise ValueError(f"{label} artifact hash drift")
    return resolved


def require_independent_board_holdout(payload: dict[str, object]) -> None:
    holdout = payload.get("holdout_validation")
    checks = holdout.get("checks") if isinstance(holdout, dict) else None
    if not isinstance(checks, dict) or not all(
        checks.get(name) is True
        for name in (
            "sync_skew",
            "board_center_residual",
            "board_normal_residual",
            "board_moved_independently",
        )
    ):
        raise ValueError(
            "source calibration chain lacks an independent moved-board holdout"
        )


def board_source_for(
    source: dict[str, object],
    *,
    source_path: Path,
    workspace: Path,
) -> tuple[Path, dict[str, object], bool]:
    """Resolve and verify the immutable board root of a calibration chain."""
    try:
        require_independent_board_holdout(source)
    except ValueError:
        board_path = resolve_artifact_identity(
            source.get("derived_from_board_calibration"),
            workspace=workspace,
            label="source board calibration",
        )
        board = json.loads(board_path.read_text(encoding="utf-8"))
        if board.get("passed") is not True:
            raise ValueError("source board calibration did not pass")
        require_independent_board_holdout(board)
        if (
            board.get("reference_robot") != source.get("reference_robot")
            or board.get("other_robot") != source.get("other_robot")
        ):
            raise ValueError("calibration chain robot identities changed")
        return board_path, board, True
    return source_path, source, False


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def average_rotation(rotations: Iterable[np.ndarray]) -> np.ndarray:
    values = tuple(rotations)
    if not values:
        raise ValueError("cannot average an empty rotation collection")
    left, _, right = np.linalg.svd(sum(values))
    result = left @ right
    if np.linalg.det(result) < 0.0:
        left[:, -1] *= -1.0
        result = left @ right
    return result


def validate_rigid(matrix: np.ndarray, *, label: str) -> None:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{label} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6):
        raise ValueError(f"{label} has an invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4):
        raise ValueError(f"{label} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-4):
        raise ValueError(f"{label} rotation determinant is not +1")


def sequence_paths(
    spool: Path, *, first: int, last: int
) -> tuple[Path, ...]:
    if first < 0 or last < first:
        raise ValueError("invalid inclusive sequence range")
    paths = tuple(spool / f"{sequence:020d}" for sequence in range(first, last + 1))
    missing = [str(path) for path in paths if not path.is_dir()]
    if missing:
        raise FileNotFoundError(
            "missing observation directories: " + ", ".join(missing)
        )
    return paths


def load_observations(
    paths: Iterable[Path],
    *,
    workspace: Path,
    expected_robot_id: str,
    expected_transform_version: str,
) -> tuple[list[np.ndarray], list[dict[str, object]]]:
    matrices: list[np.ndarray] = []
    evidence: list[dict[str, object]] = []
    previous_capture_ns = -1
    for directory in paths:
        metadata_path = directory / "metadata.json"
        rgb_path = directory / "rgb.jpg"
        depth_path = directory / "depth.png"
        for required in (metadata_path, rgb_path, depth_path):
            if not required.is_file():
                raise FileNotFoundError(required)
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        sequence = int(metadata.get("sequence", -1))
        if sequence != int(directory.name):
            raise ValueError(f"sequence mismatch in {metadata_path}")
        if metadata.get("robot_id") != expected_robot_id:
            raise ValueError(f"robot ID mismatch in {metadata_path}")
        capture_ns = int(metadata.get("capture_time_ns", -1))
        if capture_ns <= previous_capture_ns:
            raise ValueError("capture timestamps are not strictly increasing")
        previous_capture_ns = capture_ns
        pose = metadata.get("pose")
        if not isinstance(pose, dict):
            raise ValueError(f"missing pose in {metadata_path}")
        transform_version = str(pose.get("transform_version", ""))
        if transform_version != expected_transform_version:
            raise ValueError(
                f"transform version mismatch in {metadata_path}: "
                f"{transform_version!r}"
            )
        wire = pose.get("shared_T_camera")
        if not isinstance(wire, dict):
            raise ValueError(f"missing shared_T_camera in {metadata_path}")
        if wire.get("parent_frame") != "shared_world":
            raise ValueError(f"unexpected pose parent in {metadata_path}")
        matrix = np.asarray(wire.get("matrix"), dtype=np.float64).reshape(4, 4)
        validate_rigid(matrix, label=str(metadata_path))
        if int(metadata.get("rgb_size_bytes", -1)) != rgb_path.stat().st_size:
            raise ValueError(f"RGB size mismatch in {metadata_path}")
        if str(metadata.get("rgb_sha256", "")) != sha256_file(rgb_path):
            raise ValueError(f"RGB checksum mismatch in {metadata_path}")
        if int(metadata.get("depth_size_bytes", -1)) != depth_path.stat().st_size:
            raise ValueError(f"depth size mismatch in {metadata_path}")
        if str(metadata.get("depth_sha256", "")) != sha256_file(depth_path):
            raise ValueError(f"depth checksum mismatch in {metadata_path}")
        matrices.append(matrix)
        evidence.append(
            {
                "sequence": sequence,
                "capture_time_ns": capture_ns,
                "metadata": artifact_identity(metadata_path, workspace=workspace),
                "rgb": artifact_identity(rgb_path, workspace=workspace),
                "depth": artifact_identity(depth_path, workspace=workspace),
            }
        )
    return matrices, evidence


def anchor_pose(matrices: list[np.ndarray]) -> np.ndarray:
    if len(matrices) < 3:
        raise ValueError("at least three observations are required per epoch")
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = average_rotation(matrix[:3, :3] for matrix in matrices)
    result[:3, 3] = np.median(
        np.asarray([matrix[:3, 3] for matrix in matrices]), axis=0
    )
    return result


def stability_metrics(
    matrices: list[np.ndarray], anchor: np.ndarray
) -> dict[str, float]:
    translations = [
        float(np.linalg.norm(matrix[:3, 3] - anchor[:3, 3]))
        for matrix in matrices
    ]
    rotations = [
        rotation_angle_deg(anchor[:3, :3].T @ matrix[:3, :3])
        for matrix in matrices
    ]
    return {
        "max_translation_m": max(translations),
        "median_translation_m": float(np.median(translations)),
        "max_rotation_deg": max(rotations),
        "median_rotation_deg": float(np.median(rotations)),
    }


def planar_correction(
    pre_anchor: np.ndarray, post_anchor: np.ndarray
) -> tuple[np.ndarray, float]:
    raw_rotation = pre_anchor[:3, :3] @ post_anchor[:3, :3].T
    yaw = math.atan2(float(raw_rotation[1, 0]), float(raw_rotation[0, 0]))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = (
        (cosine, -sine, 0.0),
        (sine, cosine, 0.0),
        (0.0, 0.0, 1.0),
    )
    result[:3, 3] = (
        pre_anchor[:3, 3] - result[:3, :3] @ post_anchor[:3, 3]
    )
    tilt_residual_deg = rotation_angle_deg(
        result[:3, :3].T @ raw_rotation
    )
    return result, tilt_residual_deg


def matrix_wire(matrix: np.ndarray, *, child_frame: str) -> dict[str, object]:
    return {
        "parent_frame": "shared_world",
        "child_frame": child_frame,
        "matrix": [float(value) for value in matrix.reshape(-1)],
    }


def build_artifact(args: argparse.Namespace) -> dict[str, object]:
    workspace = args.workspace.resolve()
    source_path = args.source_calibration.resolve()
    source = json.loads(source_path.read_text(encoding="utf-8"))
    if source.get("passed") is not True:
        raise ValueError("source calibration did not pass")
    board_path, _, chained = board_source_for(
        source, source_path=source_path, workspace=workspace
    )
    reference = source.get("calibration_frame", {}).get("reference")
    if not isinstance(reference, dict):
        raise ValueError("source calibration lacks reference epoch identity")
    if source.get("other_robot") == args.robot_id:
        role = "other"
        old_version = str(source.get("transform_version", ""))
        old_wire = source.get("shared_world_from_other_odom")
        if not isinstance(old_wire, dict):
            raise ValueError("source calibration lacks other tracking transform")
        tracking_child_frame = str(
            old_wire.get("child_frame", f"{args.robot_id}_tracking")
        )
    elif source.get("reference_robot") == args.robot_id:
        role = "reference"
        old_version = str(reference.get("transform_version", ""))
        old_wire = source.get("shared_world_from_reference_tracking")
        if old_wire is None:
            old_wire = matrix_wire(
                np.eye(4, dtype=np.float64),
                child_frame=f"{args.robot_id}_tracking",
            )
        if not isinstance(old_wire, dict):
            raise ValueError(
                "source reference tracking handover must be an object"
            )
        tracking_child_frame = str(
            old_wire.get("child_frame", f"{args.robot_id}_tracking")
        )
    else:
        raise ValueError("source calibration robot mismatch")
    old_transform = np.asarray(old_wire.get("matrix"), dtype=np.float64).reshape(
        4, 4
    )
    validate_rigid(old_transform, label="source shared transform")

    pre_matrices, pre_evidence = load_observations(
        sequence_paths(
            args.spool, first=args.pre_first, last=args.pre_last
        ),
        workspace=workspace,
        expected_robot_id=args.robot_id,
        expected_transform_version=old_version,
    )
    post_transform_version = args.post_transform_version or old_version
    post_matrices, post_evidence = load_observations(
        sequence_paths(
            args.spool, first=args.post_first, last=args.post_last
        ),
        workspace=workspace,
        expected_robot_id=args.robot_id,
        expected_transform_version=post_transform_version,
    )
    if pre_evidence[-1]["capture_time_ns"] >= post_evidence[0]["capture_time_ns"]:
        raise ValueError("post-restart observations do not follow pre-restart ones")
    pre_anchor = anchor_pose(pre_matrices)
    post_raw_anchor = anchor_pose(post_matrices)
    if args.post_observations_are_raw_tracking:
        post_matrices = [old_transform @ matrix for matrix in post_matrices]
    post_anchor = anchor_pose(post_matrices)
    pre_stability = stability_metrics(pre_matrices, pre_anchor)
    post_stability = stability_metrics(post_matrices, post_anchor)
    correction, orientation_residual_deg = planar_correction(
        pre_anchor, post_anchor
    )
    corrected_post_anchor = correction @ post_anchor
    translation_residual_m = float(
        np.linalg.norm(
            corrected_post_anchor[:3, 3] - pre_anchor[:3, 3]
        )
    )
    new_transform = correction @ old_transform
    validate_rigid(new_transform, label="re-anchored shared transform")

    checks = {
        "operator_confirmed_stationary": (
            args.operator_confirmation == CONFIRMATION
        ),
        "pre_epoch_stable": (
            pre_stability["max_translation_m"]
            <= args.max_stability_translation_m
            and pre_stability["max_rotation_deg"]
            <= args.max_stability_rotation_deg
        ),
        "post_epoch_stable": (
            post_stability["max_translation_m"]
            <= args.max_stability_translation_m
            and post_stability["max_rotation_deg"]
            <= args.max_stability_rotation_deg
        ),
        "anchor_translation_residual": (
            translation_residual_m <= args.max_anchor_translation_residual_m
        ),
        "gravity_preserving_orientation_residual": (
            orientation_residual_deg
            <= args.max_anchor_orientation_residual_deg
        ),
    }
    passed = all(checks.values())
    if not passed:
        failed = sorted(name for name, value in checks.items() if not value)
        raise ValueError("stationary re-anchor validation failed: " + ", ".join(failed))

    board_identity = artifact_identity(board_path, workspace=workspace)
    board_identity["classification"] = (
        "observed_board_calibration_with_independent_moved_board_holdout"
    )
    immediate_source_identity = artifact_identity(
        source_path, workspace=workspace
    )
    immediate_source_identity["classification"] = (
        "validated_shared_frame_calibration_chain_input"
        if chained
        else "observed_board_calibration_with_independent_moved_board_holdout"
    )
    new_reference = dict(reference)
    if role == "reference":
        new_reference["transform_version"] = args.new_transform_version
    validation_key = f"{role}_reanchor_validation"
    validation = {
        "passed": True,
        "classification": (
            "observed_stationary_pose_handover_source_derived_alignment"
        ),
        "robot_role": role,
        "operator_confirmation": args.operator_confirmation,
        "tracking_restart_boot_id": args.tracking_restart_boot_id,
        "old_transform_version": old_version,
        "post_observation_transform_version": post_transform_version,
        "post_observation_pose_model": (
            "raw_tracking_normalized_with_old_shared_transform"
            if args.post_observations_are_raw_tracking
            else "old_shared_transform_already_applied"
        ),
        "new_transform_version": args.new_transform_version,
        "pre_restart_observations": pre_evidence,
        "post_restart_observations": post_evidence,
        "pre_restart_anchor_reported_with_old_transform": matrix_wire(
            pre_anchor, child_frame=tracking_child_frame
        ),
        "post_restart_anchor_reported_with_old_transform": matrix_wire(
            post_anchor, child_frame=tracking_child_frame
        ),
        "post_restart_anchor_as_recorded": matrix_wire(
            post_raw_anchor, child_frame=tracking_child_frame
        ),
        "planar_shared_frame_correction": matrix_wire(
            correction, child_frame="previous_shared_world"
        ),
        "corrected_post_restart_anchor": matrix_wire(
            corrected_post_anchor,
            child_frame=tracking_child_frame,
        ),
        "metrics": {
            "pre_epoch_stability": pre_stability,
            "post_epoch_stability": post_stability,
            "anchor_translation_residual_m": translation_residual_m,
            "anchor_orientation_residual_deg": orientation_residual_deg,
        },
        "thresholds": {
            "max_stability_translation_m": (
                args.max_stability_translation_m
            ),
            "max_stability_rotation_deg": args.max_stability_rotation_deg,
            "max_anchor_translation_residual_m": (
                args.max_anchor_translation_residual_m
            ),
            "max_anchor_orientation_residual_deg": (
                args.max_anchor_orientation_residual_deg
            ),
        },
        "checks": checks,
    }
    artifact = {
        "schema_version": 3,
        "passed": True,
        "calibration_method": (
            "stationary_tracking_epoch_reanchor_of_validated_board_alignment"
        ),
        "computed_at_ns": time.time_ns(),
        "reference_robot": source.get("reference_robot"),
        "other_robot": source.get("other_robot"),
        "shared_frame_calibration_id": args.new_calibration_id,
        "transform_version": (
            args.new_transform_version
            if role == "other"
            else source.get("transform_version")
        ),
        "calibration_frame": {"reference": new_reference},
        "shared_world_from_other_odom": (
            matrix_wire(new_transform, child_frame=tracking_child_frame)
            if role == "other"
            else source.get("shared_world_from_other_odom")
        ),
        "derived_from_board_calibration": board_identity,
        validation_key: validation,
        "input_provenance": {
            "status": (
                "operator_observed_stationary_robot_plus_spooled_rgbd_pose_"
                "samples_and_source_derived_planar_epoch_alignment"
            ),
            "source_board_calibration": board_identity,
            "immediate_source_calibration": immediate_source_identity,
        },
        "safety": {
            "archived_observations_only": True,
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
        },
        "note": (
            "The robot was operator-confirmed stationary across an Odin "
            "tracking restart. The new yaw-only transform preserves gravity "
            "and hands the new odometry epoch into the board-defined shared "
            "frame."
        ),
    }
    if role == "reference":
        artifact["shared_world_from_reference_tracking"] = matrix_wire(
            new_transform, child_frame=tracking_child_frame
        )
    elif source.get("shared_world_from_reference_tracking") is not None:
        artifact["shared_world_from_reference_tracking"] = source.get(
            "shared_world_from_reference_tracking"
        )
    if chained:
        artifact["schema_version"] = 4
        artifact["derived_from_calibration"] = immediate_source_identity
        artifact["note"] = (
            "The robot was operator-confirmed stationary across another "
            "tracking restart. The new yaw-only transform extends the "
            "verified calibration chain while retaining the immutable "
            "moved-board root and the other robot's current transform."
        )
    return artifact


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--source-calibration", type=Path, required=True)
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--robot-id", default="robot-1")
    parser.add_argument("--pre-first", type=int, required=True)
    parser.add_argument("--pre-last", type=int, required=True)
    parser.add_argument("--post-first", type=int, required=True)
    parser.add_argument("--post-last", type=int, required=True)
    parser.add_argument(
        "--post-transform-version",
        default=None,
        help=(
            "transform version carried by post-restart observations; defaults "
            "to the source calibration's old version"
        ),
    )
    parser.add_argument(
        "--post-observations-are-raw-tracking",
        action="store_true",
        help=(
            "post-restart poses are raw tracking poses; normalize them with "
            "the source shared transform before computing the handover"
        ),
    )
    parser.add_argument("--new-calibration-id", required=True)
    parser.add_argument("--new-transform-version", required=True)
    parser.add_argument("--tracking-restart-boot-id", required=True)
    parser.add_argument("--operator-confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-stability-translation-m", type=float, default=0.01)
    parser.add_argument("--max-stability-rotation-deg", type=float, default=1.0)
    parser.add_argument(
        "--max-anchor-translation-residual-m", type=float, default=0.01
    )
    parser.add_argument(
        "--max-anchor-orientation-residual-deg", type=float, default=3.0
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.operator_confirmation != CONFIRMATION:
        raise SystemExit(
            "stationary re-anchor requires --operator-confirmation "
            f"{CONFIRMATION}"
        )
    artifact = build_artifact(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(
        f".{args.output.name}.{os.getpid()}.tmp"
    )
    temporary.write_text(
        json.dumps(artifact, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, args.output)
    validation = artifact.get(
        "other_reanchor_validation",
        artifact.get("reference_reanchor_validation"),
    )
    if not isinstance(validation, dict):
        raise RuntimeError("re-anchor artifact lacks validation metrics")
    metrics = validation["metrics"]
    reference = artifact.get("calibration_frame", {}).get("reference", {})
    emitted_transform_version = (
        reference.get("transform_version")
        if "reference_reanchor_validation" in artifact
        else artifact.get("transform_version")
    )
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "calibration_id": artifact["shared_frame_calibration_id"],
                "transform_version": emitted_transform_version,
                "metrics": metrics,
                "robot_commands_issued": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
