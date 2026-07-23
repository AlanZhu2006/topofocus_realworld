#!/usr/bin/env python3
"""Replay live RGB-D spool geometry through a bounded parameter sweep.

This tool intentionally bypasses RedNet: obstacle/explored geometry depends on
depth, pose and ground only, while the current semantic model has a separately
documented real-camera domain gap.  It reuses the live daemon's startup pose,
RANSAC ground and keyframe gates, then compares a small set of occupancy
policies on exactly the same accepted frames.

The resulting metrics describe coverage, density and fragmentation.  They are
not an accuracy benchmark because no ground-truth floor plan is available.
Nothing here talks to a robot or issues commands.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.central_mapping import CentralMapper, MapperConfig  # noqa: E402
from focus_hub.ground_plane import estimate_startup_ground  # noqa: E402
from focus_hub.map_quality import compute_map_quality  # noqa: E402
from focus_hub.map_visualization import colorize_geometry_grid  # noqa: E402
from focus_hub.pipeline import iter_spooled_observations  # noqa: E402
from focus_hub.pose_gate import (  # noqa: E402
    KeyframeConfig,
    KeyframeSelector,
    StartupPoseConfig,
    StartupPoseGate,
    pose_delta,
)


@dataclass(frozen=True)
class _GeometryFrame:
    depth_m: np.ndarray
    T_world_infra1: np.ndarray


@dataclass(frozen=True)
class SweepProfile:
    name: str
    obstacle_band_low_m: float
    obstacle_band_high_m: float
    obstacle_fusion_mode: str
    obstacle_min_hits: int
    obstacle_probability_threshold: float = 0.70


DEFAULT_PROFILES = (
    SweepProfile("live_default", 0.15, 0.75, "log_odds", 2, 0.70),
    SweepProfile("floor_clearance_0p20", 0.20, 0.75, "log_odds", 2, 0.70),
    SweepProfile("upstream_floor_clearance_0p25", 0.25, 0.75, "log_odds", 2, 0.70),
    SweepProfile("floor_clearance_0p30", 0.30, 0.75, "log_odds", 2, 0.70),
    SweepProfile("persistence_3", 0.15, 0.75, "log_odds", 3, 0.70),
    SweepProfile("lower_band_0p60", 0.15, 0.60, "log_odds", 2, 0.70),
    SweepProfile("legacy_irreversible_max", 0.15, 0.75, "max", 1, 0.70),
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_record(spool: Path, robot_id: str, observation) -> dict:
    source_dir = spool / robot_id / f"{observation.sequence:020d}"
    metadata_path = source_dir / "metadata.json"
    rgb_name = "rgb.jpg" if observation.metadata.rgb_encoding == "jpeg" else "rgb.png"
    rgb_path = source_dir / rgb_name
    depth_path = source_dir / "depth.png"
    return {
        "sequence": observation.sequence,
        "source_dir": str(source_dir.resolve()),
        "metadata": {
            "path": str(metadata_path.resolve()),
            "size_bytes": metadata_path.stat().st_size,
            "sha256": sha256_file(metadata_path),
        },
        "rgb": {
            "path": str(rgb_path.resolve()),
            "size_bytes": rgb_path.stat().st_size,
            "sha256": observation.metadata.rgb_sha256,
            "checksum_source": "verified_wire_metadata",
        },
        "depth": {
            "path": str(depth_path.resolve()),
            "size_bytes": depth_path.stat().st_size,
            "sha256": observation.metadata.depth_sha256,
            "checksum_source": "verified_wire_metadata",
        },
        "capture_time_ns": observation.metadata.capture_time_ns,
        "transform_version": observation.metadata.pose.transform_version,
    }


def trajectory_metrics(poses: list[np.ndarray]) -> dict[str, float | int]:
    if not poses:
        return {
            "accepted_poses": 0,
            "xy_path_length_m": 0.0,
            "xy_net_displacement_m": 0.0,
            "max_step_translation_m": 0.0,
            "max_step_rotation_deg": 0.0,
        }
    xy = np.stack([pose[:2, 3] for pose in poses])
    xy_steps = np.linalg.norm(np.diff(xy, axis=0), axis=1) if len(xy) > 1 else np.array([])
    deltas = [pose_delta(first, second) for first, second in zip(poses, poses[1:])]
    return {
        "accepted_poses": len(poses),
        "xy_path_length_m": round(float(xy_steps.sum()), 6),
        "xy_net_displacement_m": round(float(np.linalg.norm(xy[-1] - xy[0])), 6),
        "max_step_translation_m": round(max((item[0] for item in deltas), default=0.0), 6),
        "max_step_rotation_deg": round(max((item[1] for item in deltas), default=0.0), 6),
    }


def write_geometry_png(path: Path, grid: np.ndarray) -> None:
    rgba = colorize_geometry_grid(grid)
    bgr = cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        raise OSError(f"failed to write {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--start-after-sequence", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-observations", type=int, default=500)
    parser.add_argument("--max-keyframes", type=int, default=30)
    parser.add_argument("--checkpoint-every", type=int, default=5)
    parser.add_argument("--depth-stride", type=int, default=1)
    parser.add_argument("--ray-trace-steps", type=int, default=40)
    args = parser.parse_args()

    positive = (
        args.max_observations,
        args.max_keyframes,
        args.checkpoint_every,
        args.depth_stride,
        args.ray_trace_steps,
    )
    if any(value <= 0 for value in positive):
        parser.error("observation/keyframe/checkpoint/depth/ray limits must be positive")
    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2

    startup_gate = StartupPoseGate(StartupPoseConfig())
    startup_pending = []
    selector: KeyframeSelector | None = None
    mappers: dict[str, CentralMapper] = {}
    profile_configs: dict[str, MapperConfig] = {}
    source_observations: list[dict] = []
    accepted_poses: list[np.ndarray] = []
    accepted_sequences: list[int] = []
    checkpoints: dict[str, list[dict]] = {
        profile.name: [] for profile in DEFAULT_PROFILES
    }
    startup_result: dict | None = None
    pose_jump: dict | None = None
    contract_error: dict | None = None
    startup_transform_version: str | None = None
    startup_frame_id: str | None = None
    observations_seen = 0

    for observation in iter_spooled_observations(
        args.spool, args.robot_id, after_sequence=args.start_after_sequence
    ):
        observations_seen += 1
        source_observations.append(source_record(args.spool, args.robot_id, observation))
        if observations_seen > args.max_observations:
            source_observations.pop()
            observations_seen -= 1
            break

        observation_transform = observation.metadata.pose.transform_version
        observation_frame = observation.metadata.pose.shared_T_camera.parent_frame
        if selector is None:
            if (
                startup_transform_version is not None
                and (
                    observation_transform != startup_transform_version
                    or observation_frame != startup_frame_id
                )
            ):
                startup_gate.reset()
                startup_pending.clear()
            startup_transform_version = observation_transform
            startup_frame_id = observation_frame
            startup_decision = startup_gate.evaluate(
                observation.T_shared_camera,
                observation.metadata.capture_time_ns,
            )
            if startup_decision.reset:
                startup_pending = [observation]
            else:
                startup_pending.append(observation)
            if not startup_decision.ready:
                continue

            stable = startup_pending[-3:]
            intrinsics = stable[-1].metadata.intrinsics
            K = np.array(
                [
                    [intrinsics.fx, 0.0, intrinsics.cx],
                    [0.0, intrinsics.fy, intrinsics.cy],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            ground = estimate_startup_ground(stable, K)
            if not ground.accepted or ground.ground_z_m is None:
                startup_pending = startup_pending[-3:]
                continue
            positions = np.stack(
                [item.T_shared_camera[:3, 3] for item in stable], axis=0
            )
            center = np.median(positions, axis=0)
            map_size_m = 2.0 * (MapperConfig().max_range_m + 8.0)
            origin = (
                float(center[0] - map_size_m / 2.0),
                float(center[1] - map_size_m / 2.0),
            )
            for profile in DEFAULT_PROFILES:
                config = MapperConfig(
                    map_size_m=map_size_m,
                    depth_stride=args.depth_stride,
                    ray_trace_steps=args.ray_trace_steps,
                    obstacle_band_low_m=profile.obstacle_band_low_m,
                    obstacle_band_high_m=profile.obstacle_band_high_m,
                    obstacle_fusion_mode=profile.obstacle_fusion_mode,
                    obstacle_min_hits=profile.obstacle_min_hits,
                    obstacle_probability_threshold=profile.obstacle_probability_threshold,
                )
                profile_configs[profile.name] = config
                mappers[profile.name] = CentralMapper(
                    config=config,
                    K_infra1=K,
                    K_rgb=K,
                    T_rgb_to_infra1=np.eye(4),
                    origin_xy_m=origin,
                    floor_z_m=ground.ground_z_m,
                )
            selector = KeyframeSelector(KeyframeConfig())
            startup_result = {
                "stable_sequences": [item.sequence for item in stable],
                "center_xyz_m": center.tolist(),
                "origin_xy_m": list(origin),
                "map_size_m": map_size_m,
                "ground_z_m": ground.ground_z_m,
                "ground_source": "three_frame_ransac_consensus",
                "ground_candidates": [asdict(item) for item in ground.candidates],
                "frame_id": stable[-1].metadata.pose.shared_T_camera.parent_frame,
                "transform_version": stable[-1].metadata.pose.transform_version,
            }
            observations_to_integrate = stable
            startup_pending.clear()
        else:
            if (
                observation_transform != startup_result["transform_version"]
                or observation_frame != startup_result["frame_id"]
            ):
                contract_error = {
                    "sequence": observation.sequence,
                    "expected_transform_version": startup_result["transform_version"],
                    "observed_transform_version": observation_transform,
                    "expected_frame_id": startup_result["frame_id"],
                    "observed_frame_id": observation_frame,
                }
                break
            observations_to_integrate = [observation]

        for item in observations_to_integrate:
            decision = selector.evaluate(
                item.T_shared_camera, item.metadata.capture_time_ns
            )
            if decision.pose_jump:
                pose_jump = {
                    "sequence": item.sequence,
                    "translation_m": decision.translation_m,
                    "rotation_deg": decision.rotation_deg,
                }
                break
            if not decision.accept:
                continue
            geometry_frame = _GeometryFrame(
                depth_m=item.depth_m,
                T_world_infra1=item.T_shared_camera,
            )
            zero_semantics = np.zeros(item.depth_m.shape, dtype=np.int16)
            for name, mapper in mappers.items():
                mapper.integrate(geometry_frame, zero_semantics)
                if (
                    (len(accepted_sequences) + 1) % args.checkpoint_every == 0
                    or len(accepted_sequences) == 0
                ):
                    checkpoints[name].append({
                        "keyframes": len(accepted_sequences) + 1,
                        "sequence": item.sequence,
                        "metrics": compute_map_quality(mapper.map.grid).to_dict(),
                    })
            accepted_sequences.append(item.sequence)
            accepted_poses.append(item.T_shared_camera.copy())
            if len(accepted_sequences) >= args.max_keyframes:
                break
        if (
            contract_error is not None
            or pose_jump is not None
            or len(accepted_sequences) >= args.max_keyframes
        ):
            break

    if startup_result is None or selector is None or not accepted_sequences:
        print("no stable startup window/keyframe found in the requested range", file=sys.stderr)
        return 1

    args.output.mkdir(parents=True)
    profile_results = {}
    for profile in DEFAULT_PROFILES:
        mapper = mappers[profile.name]
        final_metrics = compute_map_quality(mapper.map.grid).to_dict()
        if (
            not checkpoints[profile.name]
            or checkpoints[profile.name][-1]["keyframes"] != len(accepted_sequences)
        ):
            checkpoints[profile.name].append({
                "keyframes": len(accepted_sequences),
                "sequence": accepted_sequences[-1],
                "metrics": final_metrics,
            })
        image_path = args.output / f"{profile.name}_geometry.png"
        write_geometry_png(image_path, mapper.map.grid)
        profile_results[profile.name] = {
            "config": asdict(profile_configs[profile.name]),
            "final_metrics": final_metrics,
            "checkpoints": checkpoints[profile.name],
            "geometry_image": {
                "path": str(image_path.resolve()),
                "size_bytes": image_path.stat().st_size,
                "sha256": sha256_file(image_path),
                "status": "observed_offline_replay",
            },
        }

    manifest_bytes = json.dumps(
        source_observations, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    result = {
        "schema_version": 1,
        "result_status": "observed_offline_replay_without_ground_truth",
        "limitations": [
            "No surveyed floor plan or per-cell ground truth is available.",
            "Semantic predictions are intentionally zeroed; this sweep evaluates geometry only.",
            "A bounded observation/keyframe window may not represent a complete moved run.",
        ],
        "input": {
            "spool": str(args.spool.resolve()),
            "robot_id": args.robot_id,
            "start_after_sequence": args.start_after_sequence,
            "observations_seen": observations_seen,
            "accepted_sequences": accepted_sequences,
            "source_manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
            "source_observations": source_observations,
        },
        "startup": startup_result,
        "contract_error": contract_error,
        "pose_jump": pose_jump,
        "trajectory": trajectory_metrics(accepted_poses),
        "profiles": profile_results,
    }
    summary_path = args.output / "map_parameter_sweep.json"
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "output": str(summary_path),
        "accepted_keyframes": len(accepted_sequences),
        "first_sequence": accepted_sequences[0],
        "last_sequence": accepted_sequences[-1],
        "trajectory": result["trajectory"],
        "profiles": {
            name: values["final_metrics"] for name, values in profile_results.items()
        },
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
