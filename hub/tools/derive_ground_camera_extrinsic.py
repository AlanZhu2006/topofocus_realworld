#!/usr/bin/env python3
"""Derive a gravity-corrected local-camera mount from archived floor views.

This tool is read-only with respect to robots.  It fits a floor plane in each
selected spooled RGB-D observation, expresses every upward floor normal in
the camera optical frame, and applies the smallest rotation to the supplied
nominal ``T_base_link_camera`` that maps their equal-weight mean to base +Z.
Translation is deliberately preserved: floor observations constrain mount
orientation and camera-to-floor distance, but do not independently identify
the base_link origin.
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

from focus_hub.ground_plane import (  # noqa: E402
    GroundPlaneConfig,
    depth_points_world,
    fit_ground_candidate,
    plane_normal,
)
from focus_hub.pipeline import iter_spooled_observations  # noqa: E402


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


def load_matrix(path: Path) -> np.ndarray:
    artifact = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(artifact, dict) and "base_link_from_camera" in artifact:
        values = artifact["base_link_from_camera"]["matrix"]
    elif isinstance(artifact, dict) and "matrix" in artifact:
        values = artifact["matrix"]
    else:
        values = artifact
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16:
        raise SystemExit(f"{path}: expected a 16-element matrix")
    matrix = matrix.reshape(4, 4)
    rotation = matrix[:3, :3]
    if (
        not np.all(np.isfinite(matrix))
        or not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8)
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)
    ):
        raise SystemExit(f"{path}: matrix is not a finite rigid transform")
    return matrix


def exact_observation(spool: Path, robot_id: str, sequence: int):
    observations = iter_spooled_observations(
        spool, robot_id, after_sequence=sequence - 1
    )
    observation = next(observations, None)
    if observation is None or observation.sequence != sequence:
        raise SystemExit(
            f"missing exact observation {robot_id}/{sequence:020d} under {spool}"
        )
    return observation


def rotation_aligning_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return the minimum-angle proper rotation mapping source to target."""
    first = np.asarray(source, dtype=np.float64)
    second = np.asarray(target, dtype=np.float64)
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    cross = np.cross(first, second)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
    if sine < 1e-12:
        if cosine > 0.0:
            return np.eye(3)
        raise ValueError("antiparallel normals do not define a unique correction")
    cross_matrix = np.array(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ]
    )
    return (
        np.eye(3)
        + cross_matrix
        + cross_matrix @ cross_matrix * ((1.0 - cosine) / sine**2)
    )


def vector_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(
        np.clip(
            np.dot(first, second) / (np.linalg.norm(first) * np.linalg.norm(second)),
            -1.0,
            1.0,
        )
    )
    return math.degrees(math.acos(cosine))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--robot-id", default="robot-1")
    parser.add_argument("--sequence", type=int, action="append", required=True)
    parser.add_argument("--nominal-extrinsic", type=Path, required=True)
    parser.add_argument("--camera-model", required=True)
    parser.add_argument("--camera-frame", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-residual-tilt-deg", type=float, default=2.0)
    args = parser.parse_args()

    if len(args.sequence) < 3:
        parser.error("provide at least three --sequence floor observations")
    if len(set(args.sequence)) != len(args.sequence):
        parser.error("--sequence values must be unique")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")

    nominal = load_matrix(args.nominal_extrinsic)
    config = GroundPlaneConfig()
    camera_normals: list[np.ndarray] = []
    records: list[dict] = []
    for sequence in args.sequence:
        observation = exact_observation(args.spool, args.robot_id, sequence)
        intrinsics = observation.metadata.intrinsics
        matrix = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.cx],
                [0.0, intrinsics.fy, intrinsics.cy],
                [0.0, 0.0, 1.0],
            ]
        )
        candidate = fit_ground_candidate(
            depth_points_world(observation, matrix, config),
            observation.T_shared_camera[:3, 3],
            config,
        )
        if not candidate.accepted or candidate.plane_coefficients is None:
            raise SystemExit(
                f"sequence {sequence}: rejected ground candidate ({candidate.reason})"
            )
        normal_camera = observation.T_shared_camera[:3, :3].T @ plane_normal(
            candidate.plane_coefficients
        )
        # Resolve the mathematical plane-normal sign using the nominal mount:
        # an upward normal must have positive base-frame Z.
        if float((nominal[:3, :3] @ normal_camera)[2]) < 0.0:
            normal_camera *= -1.0
        normal_camera /= np.linalg.norm(normal_camera)
        camera_normals.append(normal_camera)

        entry = args.spool / args.robot_id / f"{sequence:020d}"
        records.append(
            {
                "sequence": sequence,
                "transform_version": observation.metadata.pose.transform_version,
                "capture_time_ns": observation.metadata.capture_time_ns,
                "candidate": {
                    "ground_z_m": candidate.ground_z_m,
                    "plane_coefficients": list(candidate.plane_coefficients),
                    "tilt_deg_in_recorded_frame": candidate.tilt_deg,
                    "candidate_points": candidate.candidate_points,
                    "inlier_points": candidate.inlier_points,
                    "inlier_ratio": candidate.inlier_ratio,
                    "up_normal_camera": normal_camera.tolist(),
                },
                "input_provenance": {
                    "metadata": file_artifact(entry / "metadata.json"),
                    "depth": file_artifact(entry / "depth.png"),
                },
            }
        )

    mean_normal = np.mean(np.stack(camera_normals), axis=0)
    mean_normal /= np.linalg.norm(mean_normal)
    nominal_base_normal = nominal[:3, :3] @ mean_normal
    correction = rotation_aligning_vectors(
        nominal_base_normal, np.array([0.0, 0.0, 1.0])
    )
    corrected = nominal.copy()
    corrected[:3, :3] = correction @ nominal[:3, :3]
    residual_tilts = [
        vector_angle_deg(
            corrected[:3, :3] @ normal,
            np.array([0.0, 0.0, 1.0]),
        )
        for normal in camera_normals
    ]
    correction_angle = vector_angle_deg(nominal_base_normal, np.array([0.0, 0.0, 1.0]))
    passed = max(residual_tilts) <= args.max_residual_tilt_deg

    output = {
        "schema_version": 1,
        "computed_at_ns": time.time_ns(),
        "camera_model": args.camera_model,
        "camera_frame": args.camera_frame,
        "classification": "source-derived_from_observed_spooled_depth",
        "base_link_from_camera": {
            "parent_frame": "base_link",
            "child_frame": args.camera_frame,
            "matrix": corrected.reshape(-1).tolist(),
        },
        "derivation": {
            "method": "equal_weight_camera_floor_normals_minimum_rotation_to_base_positive_z",
            "mean_up_normal_camera": mean_normal.tolist(),
            "nominal_up_normal_base": nominal_base_normal.tolist(),
            "mount_rotation_correction_deg": correction_angle,
            "translation_policy": "preserved_from_nominal_unverified_by_floor_normals",
        },
        "validation": {
            "passed": passed,
            "max_residual_tilt_deg_threshold": args.max_residual_tilt_deg,
            "residual_tilt_deg_by_sequence": {
                str(sequence): tilt
                for sequence, tilt in zip(args.sequence, residual_tilts, strict=True)
            },
            "max_residual_tilt_deg": max(residual_tilts),
        },
        "inputs": {
            "nominal_extrinsic": file_artifact(args.nominal_extrinsic),
            "observations": records,
        },
        "safety": {
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
            "archived_observations_only": True,
        },
    }
    if not passed:
        raise SystemExit(
            "derived extrinsic failed residual tilt gate: "
            f"{max(residual_tilts):.3f} > {args.max_residual_tilt_deg:.3f} deg"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"wrote {args.output}; correction={correction_angle:.3f} deg, "
        f"max residual={max(residual_tilts):.3f} deg"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
