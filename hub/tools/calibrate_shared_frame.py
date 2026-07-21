#!/usr/bin/env python3
"""Session-start shared-frame calibration CLI.

Reads each robot's most recently spooled observation (its raw camera pose,
still expressed in that robot's own local odometry frame — see
``focus_hub.calibration`` for why the wire format's ``parent_frame":
"shared_world"`` label is aspirational until this tool runs), and computes
the one fixed transform that registers the "other" robot's pose stream into
the reference robot's frame, given that the two robots were physically
co-located (or a known, measured offset apart) at the moment those two
observations were captured.

This does NOT talk to any robot and sends no commands — it only reads
already-spooled metadata.json files and writes a calibration config. Running
it correctly requires an operator to have physically driven/placed both
robots together (or measured their offset) immediately before capturing the
two observations it reads.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.calibration import compute_shared_frame_transform  # noqa: E402


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


def latest_observation(spool_root: Path, robot_id: str) -> dict:
    robot_root = spool_root / robot_id
    if not robot_root.is_dir():
        raise SystemExit(f"no spool directory for {robot_id!r}: {robot_root}")
    candidates = sorted(p for p in robot_root.iterdir() if (p / "metadata.json").is_file())
    if not candidates:
        raise SystemExit(f"no spooled observations for {robot_id!r} under {robot_root}")
    metadata_path = candidates[-1] / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["_source_path"] = str(metadata_path)
    metadata["_source_artifact"] = file_artifact(metadata_path)
    return metadata


def load_offset_matrix(path: Path | None) -> tuple[float, ...] | None:
    if path is None:
        return None
    values = json.loads(path.read_text(encoding="utf-8"))
    matrix = values["matrix"] if isinstance(values, dict) else values
    if len(matrix) != 16:
        raise SystemExit(f"offset file {path} must contain a flat 16-element row-major 4x4 matrix")
    return tuple(float(v) for v in matrix)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spool", type=Path, required=True, help="hub spool root (contains <robot_id>/ dirs)")
    parser.add_argument("--reference-robot", default="robot-0")
    parser.add_argument("--other-robot", default="robot-1")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transform-version", required=True,
                         help="new transform_version label to stamp on the calibrated frame")
    parser.add_argument(
        "--calibration-id",
        default=None,
        help=(
            "explicit shared_frame_calibration_id to bind both map daemons; "
            "defaults to --transform-version for backward-compatible calls"
        ),
    )
    parser.add_argument("--offset-file", type=Path, default=None,
                         help="JSON file with a measured 16-element row-major 4x4 matrix "
                              "from the reference robot's pose to the other robot's pose "
                              "at the sync instant; omit if the robots were placed coincident")
    parser.add_argument("--max-sync-skew-s", type=float, default=5.0,
                         help="error out if the two observations' capture times differ by more than this")
    args = parser.parse_args()

    calibration_id = args.calibration_id or args.transform_version
    if not calibration_id.strip():
        parser.error("--calibration-id/--transform-version must be non-empty")

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2

    reference_obs = latest_observation(args.spool, args.reference_robot)
    other_obs = latest_observation(args.spool, args.other_robot)

    skew_s = abs(reference_obs["capture_time_ns"] - other_obs["capture_time_ns"]) / 1e9
    print(f"sync skew between latest observations: {skew_s:.3f}s "
          f"({args.reference_robot}={reference_obs['_source_path']}, "
          f"{args.other_robot}={other_obs['_source_path']})")
    if skew_s > args.max_sync_skew_s:
        print(f"refusing: skew {skew_s:.3f}s exceeds --max-sync-skew-s {args.max_sync_skew_s}s — "
              f"these two observations are not evidence the robots were co-located when captured; "
              f"capture a fresh pair with both robots physically together right before running this",
              file=sys.stderr)
        return 1

    reference_pose = tuple(reference_obs["pose"]["shared_T_camera"]["matrix"])
    other_pose = tuple(other_obs["pose"]["shared_T_camera"]["matrix"])
    offset = load_offset_matrix(args.offset_file)

    transform = compute_shared_frame_transform(reference_pose, other_pose, offset)

    now_ns = time.time_ns()
    output = {
        "schema_version": 1,
        "computed_at_ns": now_ns,
        "reference_robot": args.reference_robot,
        "other_robot": args.other_robot,
        "transform_version": args.transform_version,
        "shared_frame_calibration_id": calibration_id,
        "sync_skew_s": round(skew_s, 6),
        "sync_capture_time_ns": {
            args.reference_robot: reference_obs["capture_time_ns"],
            args.other_robot: other_obs["capture_time_ns"],
        },
        "reference_pose_at_sync": {
            "parent_frame": reference_obs["pose"]["shared_T_camera"]["parent_frame"],
            "child_frame": reference_obs["pose"]["shared_T_camera"]["child_frame"],
            "matrix": list(reference_pose),
        },
        "other_pose_at_sync": {
            "parent_frame": other_obs["pose"]["shared_T_camera"]["parent_frame"],
            "child_frame": other_obs["pose"]["shared_T_camera"]["child_frame"],
            "matrix": list(other_pose),
        },
        "reference_to_other_offset_at_sync": list(offset) if offset is not None else None,
        "shared_world_from_other_odom": {
            "parent_frame": "shared_world",
            "child_frame": f"{args.other_robot}_odom",
            "matrix": list(transform),
        },
        "input_provenance": {
            "reference_observation_metadata": reference_obs["_source_artifact"],
            "other_observation_metadata": other_obs["_source_artifact"],
            "reference_to_other_offset": (
                file_artifact(args.offset_file) if args.offset_file is not None else None
            ),
            "status": "observed_spooled_metadata_and_operator_supplied_offset",
        },
        "note": (
            "reference robot's own poses need no further transform (shared_world is "
            "defined as the reference robot's own local pose frame by convention); "
            "apply shared_world_from_other_odom to every subsequent pose the other "
            "robot's sender publishes, via focus_hub.calibration.apply_shared_frame_transform. "
            "This transform does not correct for odometry drift after the sync instant."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {args.output} (shared_frame_calibration_id={calibration_id})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
