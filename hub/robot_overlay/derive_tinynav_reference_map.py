#!/usr/bin/env python3
"""Derive an immutable prefix reference map from a finalized TinyNav map.

A long online mapping session can observe the same physical start twice after
raw VIO translation drift.  Both appearances then become valid PnP landmarks
with different map coordinates, so a later saved-map relocalizer can
consistently choose the duplicated return branch.

This tool keeps the first, reviewed outbound traversal by timestamp.  It
rewrites only ``poses.npy``; immutable image/depth/feature databases and map
geometry are hard-linked read-only inputs from the parent snapshot.  The
result receives its own complete manifest plus explicit parent/cutoff
provenance.  It never imports ROS or a robot SDK.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.tinynav_map_contract import (  # noqa: E402
    REQUIRED_MAP_FILES,
    artifact_record,
    build_saved_map_manifest,
    validate_saved_map_manifest,
)


def atomic_json(path: Path, payload: dict) -> None:
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode()
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        with temporary.open("xb") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, 0o444)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent-map", type=Path, required=True)
    parser.add_argument("--parent-manifest", type=Path)
    parser.add_argument("--output-map", type=Path, required=True)
    parser.add_argument("--cutoff-stamp-ns", type=int, required=True)
    parser.add_argument(
        "--cutoff-reason",
        required=True,
        help="reviewed provenance for selecting the last retained keyframe",
    )
    parser.add_argument("--minimum-keyframes", type=int, default=50)
    args = parser.parse_args()
    parent = args.parent_map.expanduser().resolve()
    parent_manifest = (
        parent / "focus_saved_map_manifest.json"
        if args.parent_manifest is None
        else args.parent_manifest.expanduser().resolve()
    )
    output = args.output_map.expanduser().resolve()
    if output.exists():
        parser.error(f"refusing existing output map: {output}")
    if output.parent != parent.parent:
        parser.error("derived map must be a sibling of the parent map")
    if args.cutoff_stamp_ns <= 0 or args.minimum_keyframes < 2:
        parser.error("cutoff and minimum keyframe count must be positive")
    if not args.cutoff_reason.strip():
        parser.error("--cutoff-reason must be non-empty")
    try:
        parent_contract = validate_saved_map_manifest(
            parent_manifest,
            map_directory=parent,
            verify_hashes=False,
        )
    except ValueError as exc:
        parser.error(str(exc))
    try:
        raw_poses = np.load(parent / "poses.npy", allow_pickle=True).item()
    except Exception as exc:  # noqa: BLE001 - malformed map must fail closed
        parser.error(f"cannot load parent poses.npy: {exc}")
    if not isinstance(raw_poses, dict) or not raw_poses:
        parser.error("parent poses.npy is not a non-empty dictionary")
    ordered = list(raw_poses.items())
    if any(
        not isinstance(stamp, (int, np.integer))
        or int(stamp) <= 0
        or not isinstance(matrix, np.ndarray)
        or matrix.shape != (4, 4)
        or not np.all(np.isfinite(matrix))
        for stamp, matrix in ordered
    ):
        parser.error("parent poses contain an invalid key or transform")
    if any(
        int(first[0]) >= int(second[0])
        for first, second in zip(ordered, ordered[1:])
    ):
        parser.error("parent pose timestamps are not strictly increasing")
    retained = {
        int(stamp): matrix
        for stamp, matrix in ordered
        if int(stamp) <= args.cutoff_stamp_ns
    }
    if len(retained) < args.minimum_keyframes:
        parser.error(
            f"cutoff retains only {len(retained)} keyframes; "
            f"minimum is {args.minimum_keyframes}"
        )
    if len(retained) >= len(ordered):
        parser.error("cutoff must exclude at least one parent keyframe")
    retained_last = next(reversed(retained))

    output.mkdir(mode=0o755)
    completed = False
    try:
        poses_path = output / "poses.npy"
        with poses_path.open("xb") as handle:
            np.save(handle, retained, allow_pickle=True)
            handle.flush()
            os.fsync(handle.fileno())
        for name in REQUIRED_MAP_FILES:
            if name == "poses.npy":
                continue
            source = parent / name
            destination = output / name
            if not source.is_file() or source.is_symlink():
                raise ValueError(f"parent artifact is invalid: {source}")
            os.link(source, destination)

        derivation = {
            "schema_version": "focus-tinynav-reference-map-derivation-v1",
            "kind": "reviewed_prefix_keyframe_reference_map",
            "parent_map_id": parent_contract["map_id"],
            "parent_map_snapshot_sha256": parent_contract[
                "map_snapshot_sha256"
            ],
            "parent_manifest": artifact_record(
                parent_manifest,
                status="observed_validated_parent_map_manifest",
            ),
            "cutoff_stamp_ns_requested": args.cutoff_stamp_ns,
            "cutoff_stamp_ns_retained": retained_last,
            "cutoff_reason": args.cutoff_reason.strip(),
            "parent_keyframes": len(ordered),
            "retained_keyframes": len(retained),
            "excluded_keyframes": len(ordered) - len(retained),
            "storage_contract": (
                "poses_rewritten_other_required_files_hardlinked_to_parent"
            ),
            "result_status": (
                "source_derived_from_observed_reviewed_trajectory_boundary"
            ),
            "robot_commands_issued": False,
        }
        derivation_path = output / "focus_reference_map_derivation.json"
        atomic_json(derivation_path, derivation)
        manifest = build_saved_map_manifest(
            output,
            source_files=[
                parent_manifest,
                derivation_path,
                Path(__file__).resolve(),
            ],
        )
        manifest["derivation"] = derivation
        manifest_path = output / "focus_saved_map_manifest.json"
        atomic_json(manifest_path, manifest)
        validate_saved_map_manifest(
            manifest_path,
            map_directory=output,
            verify_hashes=False,
        )
        completed = True
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    finally:
        if not completed and output.exists():
            shutil.rmtree(output)

    print(
        json.dumps(
            {
                "output_map": str(output),
                "manifest": str(
                    output / "focus_saved_map_manifest.json"
                ),
                "map_id": manifest["map_id"],
                "map_snapshot_sha256": manifest["map_snapshot_sha256"],
                "parent_map_id": parent_contract["map_id"],
                "retained_keyframes": len(retained),
                "excluded_keyframes": len(ordered) - len(retained),
                "cutoff_stamp_ns": retained_last,
                "result_status": derivation["result_status"],
                "robot_commands_issued": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
