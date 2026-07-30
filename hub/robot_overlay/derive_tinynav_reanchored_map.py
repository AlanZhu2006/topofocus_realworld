#!/usr/bin/env python3
"""Derive a saved map with reviewed same-place keyframes reanchored.

Long raw-VIO sessions can store a second coordinate for the same physical
start after a manually driven return.  The later images are valuable for
visual relocalization, but their drifted poses must not become a second start.

This tool keeps the reviewed outbound prefix, discards the uncertain return
transition, and appends a reviewed stationary tail after left-multiplying its
poses by the rigid transform between a same-place source/target anchor pair.
Only ``poses.npy`` is rewritten.  The immutable TinyNav databases and map
geometry are hard-linked from the finalized parent snapshot.  The result has
its own manifest and complete derivation provenance.  It imports neither ROS
nor a robot SDK and cannot issue robot commands.
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


def validate_rigid_transform(matrix: np.ndarray, *, label: str) -> None:
    if (
        not isinstance(matrix, np.ndarray)
        or matrix.shape != (4, 4)
        or not np.all(np.isfinite(matrix))
    ):
        raise ValueError(f"{label} is not a finite 4x4 matrix")
    if not np.allclose(
        matrix[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-6
    ):
        raise ValueError(f"{label} has an invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3):
        raise ValueError(f"{label} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-3):
        raise ValueError(f"{label} rotation determinant is not +1")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent-map", type=Path, required=True)
    parser.add_argument("--parent-manifest", type=Path)
    parser.add_argument("--output-map", type=Path, required=True)
    parser.add_argument("--retain-through-stamp-ns", type=int, required=True)
    parser.add_argument("--append-from-stamp-ns", type=int, required=True)
    parser.add_argument("--target-anchor-stamp-ns", type=int, required=True)
    parser.add_argument("--source-anchor-stamp-ns", type=int, required=True)
    parser.add_argument(
        "--anchor-reason",
        required=True,
        help="reviewed evidence that source and target are one physical pose",
    )
    parser.add_argument("--minimum-prefix-keyframes", type=int, default=50)
    parser.add_argument("--minimum-appended-keyframes", type=int, default=10)
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
    if (
        args.retain_through_stamp_ns <= 0
        or args.append_from_stamp_ns <= 0
        or args.target_anchor_stamp_ns <= 0
        or args.source_anchor_stamp_ns <= 0
        or args.minimum_prefix_keyframes < 2
        or args.minimum_appended_keyframes < 2
    ):
        parser.error("timestamps and minimum keyframe counts must be positive")
    if args.retain_through_stamp_ns >= args.append_from_stamp_ns:
        parser.error("retained prefix and appended tail must not overlap")
    if not args.anchor_reason.strip():
        parser.error("--anchor-reason must be non-empty")

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

    ordered: list[tuple[int, np.ndarray]] = []
    try:
        for stamp, matrix in raw_poses.items():
            if not isinstance(stamp, (int, np.integer)) or int(stamp) <= 0:
                raise ValueError("parent pose timestamp is invalid")
            validate_rigid_transform(
                matrix, label=f"parent pose at {int(stamp)}"
            )
            ordered.append((int(stamp), matrix))
    except ValueError as exc:
        parser.error(str(exc))
    if any(
        first[0] >= second[0]
        for first, second in zip(ordered, ordered[1:])
    ):
        parser.error("parent pose timestamps are not strictly increasing")

    poses_by_stamp = dict(ordered)
    if args.target_anchor_stamp_ns not in poses_by_stamp:
        parser.error("target anchor timestamp is absent from parent poses")
    if args.source_anchor_stamp_ns not in poses_by_stamp:
        parser.error("source anchor timestamp is absent from parent poses")
    if args.target_anchor_stamp_ns > args.retain_through_stamp_ns:
        parser.error("target anchor is outside the retained prefix")
    if args.source_anchor_stamp_ns < args.append_from_stamp_ns:
        parser.error("source anchor is outside the appended tail")

    prefix = [
        (stamp, matrix)
        for stamp, matrix in ordered
        if stamp <= args.retain_through_stamp_ns
    ]
    tail = [
        (stamp, matrix)
        for stamp, matrix in ordered
        if stamp >= args.append_from_stamp_ns
    ]
    if len(prefix) < args.minimum_prefix_keyframes:
        parser.error(
            f"retained prefix has only {len(prefix)} keyframes; "
            f"minimum is {args.minimum_prefix_keyframes}"
        )
    if len(tail) < args.minimum_appended_keyframes:
        parser.error(
            f"appended tail has only {len(tail)} keyframes; "
            f"minimum is {args.minimum_appended_keyframes}"
        )
    if len(prefix) + len(tail) >= len(ordered):
        parser.error("selection must discard at least one transition keyframe")

    target_anchor = poses_by_stamp[args.target_anchor_stamp_ns]
    source_anchor = poses_by_stamp[args.source_anchor_stamp_ns]
    source_to_target = target_anchor @ np.linalg.inv(source_anchor)
    try:
        validate_rigid_transform(
            source_to_target, label="source-to-target anchor transform"
        )
    except ValueError as exc:
        parser.error(str(exc))
    reanchored_tail = [
        (stamp, source_to_target @ matrix) for stamp, matrix in tail
    ]
    reanchored_source = dict(reanchored_tail)[args.source_anchor_stamp_ns]
    if not np.allclose(reanchored_source, target_anchor, atol=1e-6):
        parser.error("source anchor did not map exactly to target anchor")
    selected = dict(prefix + reanchored_tail)

    output.mkdir(mode=0o755)
    completed = False
    try:
        poses_path = output / "poses.npy"
        with poses_path.open("xb") as handle:
            np.save(handle, selected, allow_pickle=True)
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

        discarded = len(ordered) - len(selected)
        derivation = {
            "schema_version": "focus-tinynav-reanchored-map-derivation-v1",
            "kind": "reviewed_same_place_pose_reanchored_reference_map",
            "parent_map_id": parent_contract["map_id"],
            "parent_map_snapshot_sha256": parent_contract[
                "map_snapshot_sha256"
            ],
            "parent_manifest": artifact_record(
                parent_manifest,
                status="observed_validated_parent_map_manifest",
            ),
            "selection": {
                "parent_keyframes": len(ordered),
                "retained_prefix_keyframes": len(prefix),
                "retained_prefix_last_stamp_ns": prefix[-1][0],
                "appended_tail_keyframes": len(tail),
                "appended_tail_first_stamp_ns": tail[0][0],
                "discarded_transition_keyframes": discarded,
            },
            "anchor": {
                "target_stamp_ns": args.target_anchor_stamp_ns,
                "source_stamp_ns": args.source_anchor_stamp_ns,
                "target_pose_row_major": target_anchor.tolist(),
                "source_pose_row_major": source_anchor.tolist(),
                "source_to_target_left_transform_row_major": (
                    source_to_target.tolist()
                ),
                "reason": args.anchor_reason.strip(),
                "status": (
                    "source_derived_from_operator_reported_same_physical_pose"
                ),
            },
            "pose_contract": (
                "prefix_pose_unchanged;"
                "tail_pose=target_anchor@inverse(source_anchor)@parent_tail_pose"
            ),
            "storage_contract": (
                "poses_rewritten_other_required_files_hardlinked_to_parent"
            ),
            "result_status": (
                "source_derived_from_observed_map_and_reviewed_same_place_anchor"
            ),
            "robot_commands_issued": False,
        }
        derivation_path = output / "focus_reanchored_map_derivation.json"
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
                "map_snapshot_sha256": manifest[
                    "map_snapshot_sha256"
                ],
                "parent_map_id": parent_contract["map_id"],
                "retained_prefix_keyframes": len(prefix),
                "appended_tail_keyframes": len(tail),
                "discarded_transition_keyframes": discarded,
                "target_anchor_stamp_ns": args.target_anchor_stamp_ns,
                "source_anchor_stamp_ns": args.source_anchor_stamp_ns,
                "result_status": derivation["result_status"],
                "robot_commands_issued": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
