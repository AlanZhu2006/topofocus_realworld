#!/usr/bin/env python3
"""Run pinned TinyNav MapNode as an isolated saved-map relocalizer.

The upstream node also contains a legacy POI planner.  This wrapper keeps its
relocalization source unchanged, verifies the exact source/map provenance, and
remaps every legacy navigation topic into a private namespace.  Only
``/map/relocalization`` and the ``world -> map`` TF remain public.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.tinynav_map_contract import (  # noqa: E402
    sha256_file,
    validate_saved_map_manifest,
)


PINNED_MAP_NODE_SIZE = 40_254
PINNED_MAP_NODE_SHA256 = (
    "d71ed2cc7a77a87804fdf1225830028f20c4aaf559505e13d3553359e646550e"
)
PRIVATE_REMAPS = {
    "/mapping/cmd_pois": "/focus/maploc/source/cmd_pois",
    "/mapping/pose_graph_trajectory": (
        "/focus/maploc/source/pose_graph_trajectory"
    ),
    "/mapping/current_pose_in_map": (
        "/focus/maploc/source/current_pose_in_map"
    ),
    "/benchmark/stop": "/focus/maploc/source/benchmark_stop",
    "/benchmark/data_saved": "/focus/maploc/source/data_saved",
    "/mapping/poi": "/focus/maploc/source/poi",
    "/mapping/poi_change": "/focus/maploc/source/poi_change",
    "/mapping/nav_done": "/focus/maploc/source/nav_done",
    "/mapping/nav_progress": "/focus/maploc/source/nav_progress",
    "/mapping/current_pose": "/focus/maploc/source/current_pose",
    "/mapping/global_plan": "/focus/maploc/source/global_plan",
    "/control/target_pose": "/focus/maploc/source/target_pose",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-directory", type=Path, required=True)
    parser.add_argument("--map-manifest", type=Path)
    parser.add_argument("--scratch-directory", type=Path, required=True)
    parser.add_argument(
        "--map-node-source",
        type=Path,
        default=Path("/home/nvidia/twork/tinynav/tinynav/core/map_node.py"),
    )
    parser.add_argument("--verify-map-hashes", action="store_true")
    parser.add_argument("--verbose-timer", action="store_true")
    args = parser.parse_args()

    map_directory = args.map_directory.expanduser().resolve()
    manifest = (
        map_directory / "focus_saved_map_manifest.json"
        if args.map_manifest is None
        else args.map_manifest.expanduser().resolve()
    )
    scratch = args.scratch_directory.expanduser().resolve()
    source = args.map_node_source.expanduser().resolve()
    try:
        validate_saved_map_manifest(
            manifest,
            map_directory=map_directory,
            verify_hashes=args.verify_map_hashes,
        )
    except ValueError as exc:
        parser.error(str(exc))
    if not source.is_file() or source.is_symlink():
        parser.error(f"MapNode source is unavailable: {source}")
    actual_size = source.stat().st_size
    actual_sha256 = sha256_file(source)
    if (
        actual_size != PINNED_MAP_NODE_SIZE
        or actual_sha256 != PINNED_MAP_NODE_SHA256
    ):
        parser.error(
            "MapNode source contract mismatch: "
            f"path={source} size={actual_size}/{PINNED_MAP_NODE_SIZE} "
            f"sha256={actual_sha256}/{PINNED_MAP_NODE_SHA256}"
        )
    if scratch == map_directory or map_directory in scratch.parents:
        parser.error("scratch directory must be outside the immutable map")
    if scratch.exists():
        parser.error(f"refusing existing localization scratch path: {scratch}")
    scratch.parent.mkdir(parents=True, exist_ok=True)

    command = [
        sys.executable,
        "-u",
        str(source),
        "--tinynav_db_path",
        str(scratch),
        "--tinynav_map_path",
        str(map_directory),
        (
            "--verbose_timer"
            if args.verbose_timer
            else "--no_verbose_timer"
        ),
        "--ros-args",
    ]
    for original, private in PRIVATE_REMAPS.items():
        command.extend(("-r", f"{original}:={private}"))
    # Preserve the wrapper's process and signal semantics while making the
    # loaded source path visible in /proc and tmux provenance.
    os.execv(sys.executable, command)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
