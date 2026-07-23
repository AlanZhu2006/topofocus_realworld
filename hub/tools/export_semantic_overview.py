#!/usr/bin/env python3
"""Export the Foxglove semantic overview as reproducible PNG evidence.

This is a read-only operator tool. It loads already-written map snapshots and
live-status pose trails, renders the same example-style image used by the
Foxglove relay, and records source/output checksums. It has no Hub publication,
planner, receiver, WATER, ROS command, or robot-control dependency.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
import shutil
import sys
import time

import cv2
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES  # noqa: E402
from focus_hub.frontiers import extract_frontiers  # noqa: E402
from focus_hub.fusion import align_and_fuse_grids  # noqa: E402
from focus_hub.map_snapshot import (  # noqa: E402
    MapSnapshot,
    load_map_snapshot,
    validate_fusion_contract,
)
from focus_hub.map_visualization import (  # noqa: E402
    RobotMapOverlay,
    render_semantic_overview,
)


@dataclass(frozen=True)
class RobotInput:
    name: str
    directory: Path


def parse_robot(value: str) -> RobotInput:
    name, separator, raw_directory = value.partition(":")
    if not separator or not name or not raw_directory:
        raise argparse.ArgumentTypeError(
            f"expected NAME:SNAPSHOT_DIR, got {value!r}"
        )
    return RobotInput(name=name, directory=Path(raw_directory).expanduser())


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact(path: Path, status: str) -> dict[str, object]:
    resolved = path.resolve()
    return {
        "path": str(resolved),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
        "status": status,
    }


def _finite_xy(value: object) -> tuple[float, float] | None:
    if (
        isinstance(value, list)
        and len(value) == 2
        and all(
            isinstance(item, (int, float)) and np.isfinite(item)
            for item in value
        )
    ):
        return float(value[0]), float(value[1])
    return None


def _finite_heading(value: object) -> float | None:
    if isinstance(value, (int, float)) and np.isfinite(value):
        return float(value)
    return None


def _trajectory(value: object) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list):
        return ()
    points: list[tuple[float, float]] = []
    for item in value:
        point = _finite_xy(item)
        if point is None:
            return ()
        points.append(point)
    return tuple(points[-2000:])


def overlay_from_status(
    name: str,
    status: dict[str, object],
) -> tuple[RobotMapOverlay, str]:
    pose = _finite_xy(status.get("last_robot_xy_m"))
    heading = _finite_heading(status.get("last_robot_heading_deg"))
    trajectory = _trajectory(status.get("robot_trajectory_xy_m"))
    pose_source = "calibrated_base_link"
    if pose is None:
        pose = _finite_xy(status.get("last_camera_xy_m"))
        heading = _finite_heading(status.get("last_camera_heading_deg"))
        trajectory = _trajectory(status.get("trajectory_xy_m"))
        pose_source = "historical_camera_pose_fallback"

    if name.casefold() == "yunji":
        trail_bgr = (70, 190, 30)
        pose_bgr = (0, 130, 255)
    else:
        trail_bgr = (255, 110, 30)
        pose_bgr = (0, 0, 255)
    return (
        RobotMapOverlay(
            label=name,
            trajectory_xy_m=trajectory,
            pose_xy_m=pose,
            heading_deg=heading,
            trajectory_bgr=trail_bgr,
            pose_bgr=pose_bgr,
        ),
        pose_source,
    )


def semantic_stats(grid: np.ndarray) -> dict[str, object]:
    categories: dict[str, dict[str, object]] = {}
    for index, name in enumerate(HM3D_CATEGORY_NAMES):
        mask = np.asarray(grid[2 + index] > 0.1, dtype=np.uint8)
        if not np.any(mask):
            continue
        count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
            mask,
            connectivity=8,
        )
        areas = sorted(
            (
                int(stats[component, cv2.CC_STAT_AREA])
                for component in range(1, count)
            ),
            reverse=True,
        )
        categories[name] = {
            "cells": int(np.count_nonzero(mask)),
            "components": len(areas),
            "component_areas_desc": areas,
        }
    return {
        "obstacle_cells": int(np.count_nonzero(grid[0] > 0.5)),
        "explored_cells": int(np.count_nonzero(grid[1] > 0.5)),
        "categories": categories,
    }


def write_overview(
    path: Path,
    snapshot: MapSnapshot,
    overlays: tuple[RobotMapOverlay, ...],
    *,
    minimum_component_cells: int,
) -> None:
    image = render_semantic_overview(
        snapshot.grid,
        HM3D_CATEGORY_NAMES,
        snapshot.origin_xy_m,
        snapshot.resolution_m,
        robot_overlays=overlays,
        frontiers=tuple(
            extract_frontiers(
                snapshot.grid,
                snapshot.origin_xy_m,
                snapshot.resolution_m,
            )
        ),
        minimum_component_cells=minimum_component_cells,
    )
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write overview image: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        action="append",
        type=parse_robot,
        required=True,
        help="repeatable NAME:SNAPSHOT_DIR input",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--fuse",
        action="store_true",
        help="also export a strict shared-frame fused overview",
    )
    parser.add_argument(
        "--minimum-component-cells",
        type=int,
        default=3,
        help="display-only connected-component speckle threshold",
    )
    parser.add_argument(
        "--reference-image",
        type=Path,
        default=WORKSPACE / "media" / "image" / "example.png",
    )
    args = parser.parse_args()
    if args.minimum_component_cells < 1:
        parser.error("--minimum-component-cells must be positive")
    if args.fuse and len(args.robot) < 2:
        parser.error("--fuse requires at least two --robot inputs")

    output = args.output_dir.expanduser().resolve()
    if output.exists():
        print(f"refusing to overwrite existing output: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)

    records: list[dict[str, object]] = []
    snapshots: list[MapSnapshot] = []
    overlays: list[RobotMapOverlay] = []
    output_artifacts: list[dict[str, object]] = []
    seen_names: set[str] = set()
    for spec in args.robot:
        if spec.name in seen_names:
            raise ValueError(f"duplicate robot name: {spec.name!r}")
        seen_names.add(spec.name)
        directory = spec.directory.expanduser().resolve()
        map_path = directory / "central_map.npz"
        status_path = directory / "live_status.json"
        frozen_input_dir = output / "inputs" / spec.name
        frozen_input_dir.mkdir(parents=True)
        frozen_map_path = frozen_input_dir / "central_map.npz"
        frozen_status_path = frozen_input_dir / "live_status.json"
        # The producer atomically replaces each source path. An already-opened
        # file descriptor remains bound to one complete generation while
        # copyfile reads it, so the exporter never assembles a torn NPZ/JSON.
        shutil.copyfile(map_path, frozen_map_path)
        shutil.copyfile(status_path, frozen_status_path)
        snapshot = load_map_snapshot(frozen_map_path)
        if snapshot is None:
            raise FileNotFoundError(frozen_map_path)
        status = json.loads(frozen_status_path.read_text(encoding="utf-8"))
        overlay, pose_source = overlay_from_status(spec.name, status)
        image_path = output / f"{spec.name}_semantic_overview.png"
        write_overview(
            image_path,
            snapshot,
            (overlay,),
            minimum_component_cells=args.minimum_component_cells,
        )
        records.append(
            {
                "name": spec.name,
                "pose_source": pose_source,
                "frame_id": snapshot.frame_id,
                "transform_version": snapshot.transform_version,
                "shared_frame_calibration_id": (
                    snapshot.shared_frame_calibration_id
                ),
                "source_paths_at_export": {
                    "map": str(map_path),
                    "live_status": str(status_path),
                },
                "map": artifact(
                    frozen_map_path,
                    "frozen model/source-derived semantic map input",
                ),
                "live_status": artifact(
                    frozen_status_path,
                    "frozen observed pose/status input",
                ),
                "stats": semantic_stats(snapshot.grid),
            }
        )
        output_artifacts.append(
            artifact(
                image_path,
                "source/model-derived operator visualization; no control authority",
            )
        )
        snapshots.append(snapshot)
        overlays.append(overlay)

    if args.fuse:
        frame_id, resolution_m, calibration_id = validate_fusion_contract(
            snapshots
        )
        fused_grid, fused_origin = align_and_fuse_grids(
            [snapshot.grid for snapshot in snapshots],
            [snapshot.origin_xy_m for snapshot in snapshots],
            resolution_m,
        )
        fused_snapshot = MapSnapshot(
            grid=fused_grid,
            origin_xy_m=fused_origin,
            resolution_m=resolution_m,
            frame_id=frame_id,
            transform_version="fused-read-only-export",
            shared_frame_calibration_id=calibration_id,
            map_format_version="focus-hub-fused-read-only-v1",
        )
        fused_path = output / "fused_semantic_overview.png"
        write_overview(
            fused_path,
            fused_snapshot,
            tuple(overlays),
            minimum_component_cells=args.minimum_component_cells,
        )
        output_artifacts.append(
            artifact(
                fused_path,
                "source/model-derived fused operator visualization; no control authority",
            )
        )

    reference = args.reference_image.expanduser().resolve()
    renderer_path = (
        WORKSPACE / "hub" / "src" / "focus_hub" / "map_visualization.py"
    )
    manifest = {
        "schema_version": "focus-semantic-overview-export-v1",
        "created_at_ns": time.time_ns(),
        "reference": artifact(reference, "observed user-supplied visual reference"),
        "renderer": artifact(renderer_path, "implemented read-only renderer"),
        "inputs": records,
        "outputs": output_artifacts,
        "minimum_component_cells": args.minimum_component_cells,
        "fused": bool(args.fuse),
        "safety": {
            "robot_commands_issued": False,
            "hub_decisions_published": False,
            "planner_or_receiver_contacted": False,
        },
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
