#!/usr/bin/env python3
"""Read-only acceptance gate for an operator-present moved mapping run.

Compare copied before/after map directories and the matching append-only Hub
spool range.  The tool verifies map/session continuity, pose-step bounds,
actual accepted motion, reversible map changes and basic geometry quality.  It
does not connect to a robot, publish a target or authorize autonomous motion.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.map_quality import compute_map_quality, compare_map_grids  # noqa: E402
from focus_hub.map_snapshot import load_map_snapshot  # noqa: E402
from focus_hub.pose_gate import KeyframeConfig, KeyframeSelector, pose_delta  # noqa: E402


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return value


def artifact(path: Path) -> dict:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def load_trajectory(
    spool: Path,
    robot_id: str,
    start_sequence: int,
    end_sequence: int,
    expected_transform: str,
    expected_frame: str,
) -> tuple[list[np.ndarray], list[int], list[int], list[dict], list[dict]]:
    robot_root = spool / robot_id
    poses: list[np.ndarray] = []
    sequences: list[int] = []
    timestamps_ns: list[int] = []
    provenance: list[dict] = []
    contract_errors: list[dict] = []
    for sequence in range(start_sequence, end_sequence + 1):
        metadata_path = robot_root / f"{sequence:020d}" / "metadata.json"
        if not metadata_path.is_file():
            continue
        metadata = load_json(metadata_path)
        pose_meta = metadata.get("pose", {})
        transform = pose_meta.get("transform_version")
        transform_meta = pose_meta.get("shared_T_camera", {})
        frame = transform_meta.get("parent_frame")
        matrix = np.asarray(transform_meta.get("matrix", []), dtype=np.float64)
        capture_time_ns = metadata.get("capture_time_ns")
        if transform != expected_transform or frame != expected_frame:
            contract_errors.append({
                "sequence": sequence,
                "transform_version": transform,
                "frame_id": frame,
            })
            continue
        if matrix.size != 16 or not np.all(np.isfinite(matrix)):
            contract_errors.append({
                "sequence": sequence,
                "error": "pose matrix is not 16 finite values",
            })
            continue
        if not isinstance(capture_time_ns, int) or capture_time_ns < 0:
            contract_errors.append({
                "sequence": sequence,
                "error": "capture_time_ns is missing or invalid",
            })
            continue
        poses.append(matrix.reshape(4, 4))
        sequences.append(sequence)
        timestamps_ns.append(capture_time_ns)
        provenance.append({
            "sequence": sequence,
            "path": str(metadata_path.resolve()),
            "size_bytes": metadata_path.stat().st_size,
            "sha256": sha256_file(metadata_path),
        })
    return poses, sequences, timestamps_ns, provenance, contract_errors


def trajectory_metrics(
    poses: list[np.ndarray], sequences: list[int], timestamps_ns: list[int]
) -> tuple[dict, list[int], list[dict]]:
    selector = KeyframeSelector(KeyframeConfig())
    accepted_poses: list[np.ndarray] = []
    accepted_sequences: list[int] = []
    pose_jumps: list[dict] = []
    for sequence, pose, timestamp_ns in zip(sequences, poses, timestamps_ns):
        decision = selector.evaluate(pose, timestamp_ns)
        if decision.pose_jump:
            pose_jumps.append({
                "sequence": sequence,
                "translation_m": decision.translation_m,
                "rotation_deg": decision.rotation_deg,
            })
        elif decision.accept:
            accepted_poses.append(pose)
            accepted_sequences.append(sequence)

    xy = (
        np.stack([pose[:2, 3] for pose in accepted_poses])
        if accepted_poses
        else np.empty((0, 2))
    )
    xy_steps = np.linalg.norm(np.diff(xy, axis=0), axis=1) if len(xy) > 1 else np.array([])
    adjacent_deltas = [
        pose_delta(first, second) for first, second in zip(poses, poses[1:])
    ]
    metrics = {
        "observations": len(poses),
        "first_sequence": sequences[0] if sequences else None,
        "last_sequence": sequences[-1] if sequences else None,
        "accepted_keyframes": len(accepted_poses),
        "accepted_sequences": accepted_sequences,
        "xy_path_length_m": round(float(xy_steps.sum()), 6),
        "xy_net_displacement_m": (
            round(float(np.linalg.norm(xy[-1] - xy[0])), 6) if len(xy) else 0.0
        ),
        "max_adjacent_translation_m": round(
            max((item[0] for item in adjacent_deltas), default=0.0), 6
        ),
        "max_adjacent_rotation_deg": round(
            max((item[1] for item in adjacent_deltas), default=0.0), 6
        ),
    }
    return metrics, accepted_sequences, pose_jumps


def check(name: str, passed: bool, observed, requirement: str) -> dict:
    return {
        "name": name,
        "pass": bool(passed),
        "observed": observed,
        "requirement": requirement,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True, help="copied pre-motion map directory")
    parser.add_argument("--after", type=Path, required=True, help="copied post-motion map directory")
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-path-length-m", type=float, default=0.50)
    parser.add_argument("--min-integrated-keyframes", type=int, default=3)
    parser.add_argument("--min-newly-explored-cells", type=int, default=25)
    parser.add_argument("--max-obstacle-explored-ratio", type=float, default=0.50)
    parser.add_argument("--max-pose-step-m", type=float, default=2.0)
    parser.add_argument("--max-pose-step-deg", type=float, default=90.0)
    args = parser.parse_args()

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2
    if args.min_path_length_m <= 0 or args.min_integrated_keyframes <= 0:
        parser.error("motion/keyframe thresholds must be positive")

    before_map_path = args.before / "central_map.npz"
    after_map_path = args.after / "central_map.npz"
    before_summary_path = args.before / "map_summary.json"
    after_summary_path = args.after / "map_summary.json"
    before_map = load_map_snapshot(before_map_path)
    after_map = load_map_snapshot(after_map_path)
    if before_map is None or after_map is None:
        print("both before/after directories must contain central_map.npz", file=sys.stderr)
        return 2
    before_summary = load_json(before_summary_path)
    after_summary = load_json(after_summary_path)

    start_sequence = int(before_summary["last_observation_sequence"])
    end_sequence = int(after_summary["last_observation_sequence"])
    if end_sequence <= start_sequence:
        print("after snapshot does not contain later observations", file=sys.stderr)
        return 2
    poses, sequences, timestamps_ns, pose_provenance, contract_errors = load_trajectory(
        args.spool,
        args.robot_id,
        start_sequence,
        end_sequence,
        before_map.transform_version,
        before_map.frame_id,
    )
    motion, _, trajectory_pose_jumps = trajectory_metrics(
        poses, sequences, timestamps_ns
    )
    before_quality = compute_map_quality(before_map.grid)
    after_quality = compute_map_quality(after_map.grid)

    same_contract = (
        before_map.grid.shape == after_map.grid.shape
        and before_map.origin_xy_m == after_map.origin_xy_m
        and np.isclose(before_map.resolution_m, after_map.resolution_m)
        and before_map.frame_id == after_map.frame_id
        and before_map.transform_version == after_map.transform_version
        and before_map.shared_frame_calibration_id
        == after_map.shared_frame_calibration_id
    )
    comparison = (
        compare_map_grids(before_map.grid, after_map.grid)
        if before_map.grid.shape == after_map.grid.shape
        else None
    )
    integrated_delta = int(after_summary["frames_processed"]) - int(
        before_summary["frames_processed"]
    )
    pose_jump_delta = int(after_summary.get("pose_jump_events", 0)) - int(
        before_summary.get("pose_jump_events", 0)
    )

    checks = [
        check(
            "same_map_session_contract",
            same_contract,
            {
                "before_frame": before_map.frame_id,
                "after_frame": after_map.frame_id,
                "before_transform": before_map.transform_version,
                "after_transform": after_map.transform_version,
            },
            "frame, transform, calibration, origin, resolution and shape must stay identical",
        ),
        check(
            "no_contract_errors_in_spool_range",
            not contract_errors,
            contract_errors,
            "every pose in the compared sequence range must use the map contract",
        ),
        check(
            "mapping_not_halted",
            after_summary.get("mapping_blocked_reason") is None,
            after_summary.get("mapping_blocked_reason"),
            "post-run mapping_blocked_reason must be null",
        ),
        check(
            "no_new_pose_jump",
            pose_jump_delta == 0 and not trajectory_pose_jumps,
            {"summary_delta": pose_jump_delta, "trajectory": trajectory_pose_jumps},
            "no pose jump may be observed by either daemon summary or replay",
        ),
        check(
            "integrated_keyframes",
            integrated_delta >= args.min_integrated_keyframes,
            integrated_delta,
            f"at least {args.min_integrated_keyframes} additional keyframes",
        ),
        check(
            "controlled_path_length",
            motion["xy_path_length_m"] >= args.min_path_length_m,
            motion["xy_path_length_m"],
            f"at least {args.min_path_length_m:.3f} m accepted XY path",
        ),
        check(
            "bounded_pose_steps",
            motion["max_adjacent_translation_m"] <= args.max_pose_step_m
            and motion["max_adjacent_rotation_deg"] <= args.max_pose_step_deg,
            {
                "translation_m": motion["max_adjacent_translation_m"],
                "rotation_deg": motion["max_adjacent_rotation_deg"],
            },
            f"adjacent steps <= {args.max_pose_step_m} m / {args.max_pose_step_deg} deg",
        ),
        check(
            "map_cells_changed",
            comparison is not None and comparison["changed_xy_cells"] > 0,
            None if comparison is None else comparison["changed_xy_cells"],
            "at least one XY map cell must change",
        ),
        check(
            "new_coverage",
            comparison is not None
            and comparison["newly_explored_cells"] >= args.min_newly_explored_cells,
            None if comparison is None else comparison["newly_explored_cells"],
            f"at least {args.min_newly_explored_cells} newly explored cells",
        ),
        check(
            "bounded_obstacle_density",
            after_quality.obstacle_explored_ratio
            <= args.max_obstacle_explored_ratio,
            after_quality.obstacle_explored_ratio,
            f"obstacle/explored <= {args.max_obstacle_explored_ratio}",
        ),
    ]
    overall_pass = all(item["pass"] for item in checks)
    provenance_manifest = json.dumps(
        pose_provenance, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    result = {
        "schema_version": 1,
        "result_status": "observed_operator_run_evaluation",
        "overall_pass": overall_pass,
        "safety_scope": (
            "read-only evaluation; this result does not enable GOAL output or authorize motion"
        ),
        "checks": checks,
        "motion": motion,
        "before_quality": before_quality.to_dict(),
        "after_quality": after_quality.to_dict(),
        "map_change": comparison,
        "sequence_range": [start_sequence, end_sequence],
        "provenance": {
            "before_map": artifact(before_map_path),
            "before_summary": artifact(before_summary_path),
            "after_map": artifact(after_map_path),
            "after_summary": artifact(after_summary_path),
            "pose_metadata_manifest_sha256": hashlib.sha256(
                provenance_manifest
            ).hexdigest(),
            "pose_metadata": pose_provenance,
        },
        "limitations": [
            "No surveyed ground-truth trajectory or floor plan is available.",
            "Passing establishes map update continuity/quality bounds, not metric SLAM accuracy.",
            "The operator and robot-local safety stack retain final movement authority.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "overall_pass": overall_pass,
        "output": str(args.output),
        "failed_checks": [item["name"] for item in checks if not item["pass"]],
        "motion": motion,
        "map_change": comparison,
    }, indent=2))
    return 0 if overall_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
