#!/usr/bin/env python3
"""Read-only Foxglove dashboard relay: republishes each robot's camera feed
and incrementally-built semantic map, plus explicit staleness, over a
Foxglove WebSocket server for live multi-robot visualization.

Why this exists: Foxglove's live-connection model is one connection per
layout (confirmed against Foxglove's own docs, 2026-07-20 research pass) --
you cannot point a single layout at wsj's ROS 2 topics, Yunji's ROS 1
topics, AND the hub's own semantic-map state all at once. This process is
the single aggregation point a Foxglove client connects to instead.

Two independent data paths feed it, deliberately different because the two
channels have very different freshness needs:

- **Camera**: pushed. A lightweight per-robot preview publisher (see
  `robot_overlay/wsj_camera_preview.py` / `yunji_camera_preview.py`) POSTs
  raw JPEG bytes straight to this process's `/camera/{name}` HTTP endpoint
  the instant a frame is captured -- no polling interval in the loop at
  all. Deliberately NOT going through `hub_pipeline_daemon.py`'s spool/map
  pipeline: that pipeline requires a full synchronized RGB+depth+pose
  observation (real pose is not optional in the wire protocol), so on wsj
  specifically it was blocked entirely behind SLAM relocalization
  succeeding -- a raw camera preview has no reason to depend on that.
- **Map**: still polled from `hub_pipeline_daemon.py`'s on-disk snapshot
  (`central_map.npz`, written via `pipeline.save()` on --snapshot-interval-s).
  This one's fine to stay slow and disk-based: the map changes far more
  slowly than the camera, doesn't need push latency, and reading the
  daemon's own map keeps the dashboard showing exactly what (would) drive
  decisions rather than a second, independently-computed copy (see
  `write_map_snapshot`'s docstring in hub_pipeline_daemon.py).

Cross-robot fusion is opt-in and fail-closed: every input snapshot must carry
the same explicit ``frame_id`` and ``shared_frame_calibration_id``. Merely
naming two independent odometry frames ``shared_world`` is not accepted as a
calibration. Per-robot maps remain viewable independently.

Staleness is first-class, not an afterthought: a background thread
republishes each robot's camera/map age as a Log message, since the whole
point of showing two robots with very different update cadences side by
side is that an operator must never be misled into thinking two panels are
synchronized when they aren't.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, Header, HTTPException, Request, Response

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

import foxglove  # noqa: E402
from foxglove.channels import (  # noqa: E402
    CompressedImageChannel, FrameTransformChannel, GridChannel, LogChannel,
    SceneUpdateChannel,
)
from foxglove.messages import (  # noqa: E402
    Color, CompressedImage, CubePrimitive, CylinderPrimitive, Duration,
    FrameTransform, Grid, LinePrimitive, LinePrimitiveLineType, Log, LogLevel,
    PackedElementField,
    PackedElementFieldNumericType, Point3, Pose, Quaternion, SceneEntity, SceneUpdate,
    TextPrimitive, Timestamp, Vector2, Vector3,
)

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES  # noqa: E402
from focus_hub.frontiers import extract_frontiers  # noqa: E402
from focus_hub.fusion import align_and_fuse_grids  # noqa: E402
from focus_hub.map_visualization import (  # noqa: E402
    RobotMapOverlay,
    colorize_geometry_grid,
    colorize_semantic_grid,
    downsample_evidence_grid,
    render_semantic_overview,
    semantic_evidence_cells,
)
from focus_hub.map_snapshot import (  # noqa: E402
    MapSnapshot,
    load_map_snapshot,
    validate_fusion_contract,
)
from focus_hub.shadow_coordination import (  # noqa: E402
    SHADOW_STATUS,
    validate_shadow_target_payload,
)


@dataclass
class RobotSource:
    robot_id: str
    name: str
    snapshot_dir: Path
    camera_channel: CompressedImageChannel
    map_channel: GridChannel
    geometry_channel: GridChannel
    pose_channel: SceneUpdateChannel
    status_channel: LogChannel
    overview_channel: CompressedImageChannel | None = None
    last_map_published_at_s: float = 0.0
    last_camera_pushed_at_ns: int | None = None
    camera_frames_pushed: int = 0
    last_camera_message: CompressedImage | None = None
    last_overview_message: CompressedImage | None = None
    trajectory_xy_m: list[tuple[float, float]] = field(default_factory=list)
    trajectory_frame_id: str | None = None
    last_pose_xy_m: tuple[float, float] | None = None
    last_heading_deg: float | None = None
    last_map_stats: dict[str, int] | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)


def now_ts() -> Timestamp:
    ns = time.time_ns()
    return Timestamp(ns // 1_000_000_000, ns % 1_000_000_000)


def parse_robot_arg(spec: str) -> tuple[str, str, Path]:
    """--robot robot-0:wsj:/path/to/out_dir -> (robot_id, name, dir)."""
    parts = spec.split(":", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"expected ROBOT_ID:NAME:SNAPSHOT_DIR, got {spec!r}")
    robot_id, name, snapshot_dir = parts
    return robot_id, name, Path(snapshot_dir)


def colorize_grid(grid: np.ndarray, category_names: tuple[str, ...]) -> np.ndarray:
    """Same coloring rule as frontiers.render_semantic_decision_map's
    background (obstacle/explored/per-category argmax), returned as an
    (h, w, 4) uint8 RGBA array -- kept in sync with that function's palette
    so the dashboard's colors match any saved decision-map PNGs.
    """
    return colorize_semantic_grid(grid, category_names)


def grid_to_message(
    grid: np.ndarray, origin_xy_m: tuple[float, float], resolution_m: float,
    category_names: tuple[str, ...], downsample_factor: int = 1,
    frame_id: str = "shared_world",
    *, view: str = "semantic",
) -> Grid:
    reduced = downsample_evidence_grid(grid, downsample_factor)
    if view == "semantic":
        rgba = colorize_semantic_grid(reduced, category_names)
    elif view == "geometry":
        rgba = colorize_geometry_grid(reduced)
    else:
        raise ValueError(f"unknown map view {view!r}")
    resolution_m *= downsample_factor
    h, w = rgba.shape[:2]
    fields = [
        PackedElementField(name="red", offset=0, type=PackedElementFieldNumericType.Uint8),
        PackedElementField(name="green", offset=1, type=PackedElementFieldNumericType.Uint8),
        PackedElementField(name="blue", offset=2, type=PackedElementFieldNumericType.Uint8),
        PackedElementField(name="alpha", offset=3, type=PackedElementFieldNumericType.Uint8),
    ]
    return Grid(
        timestamp=now_ts(),
        frame_id=frame_id,
        pose=Pose(
            position=Vector3(x=float(origin_xy_m[0]), y=float(origin_xy_m[1]), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        column_count=w,
        cell_size=Vector2(x=resolution_m, y=resolution_m),
        row_stride=w * 4,
        cell_stride=4,
        fields=fields,
        data=rgba.tobytes(),
    )


def load_grid_npz(
    npz_path: Path, *, allow_legacy: bool = False,
) -> MapSnapshot | None:
    """Compatibility wrapper around the strict shared snapshot contract."""
    return load_map_snapshot(npz_path, allow_legacy=allow_legacy)


def build_grid_message(
    npz_path: Path, category_names: tuple[str, ...], downsample_factor: int = 1,
    *, allow_legacy: bool = False, view: str = "semantic",
) -> Grid | None:
    loaded = load_grid_npz(npz_path, allow_legacy=allow_legacy)
    if loaded is None:
        return None
    return grid_to_message(
        loaded.grid,
        loaded.origin_xy_m,
        loaded.resolution_m,
        category_names,
        downsample_factor,
        loaded.frame_id,
        view=view,
    )


def robot_map_overlay(source: RobotSource) -> RobotMapOverlay:
    """Take one thread-safe snapshot of a robot's persisted display pose."""

    with source.lock:
        trajectory = tuple(source.trajectory_xy_m)
        pose = source.last_pose_xy_m
        heading = source.last_heading_deg
    if source.name.lower() == "yunji":
        return RobotMapOverlay(
            label=source.name,
            trajectory_xy_m=trajectory,
            pose_xy_m=pose,
            heading_deg=heading,
            trajectory_bgr=(70, 190, 30),
            pose_bgr=(0, 130, 255),
        )
    return RobotMapOverlay(
        label=source.name,
        trajectory_xy_m=trajectory,
        pose_xy_m=pose,
        heading_deg=heading,
        trajectory_bgr=(255, 110, 30),
        pose_bgr=(0, 0, 255),
    )


def semantic_overview_message(
    snapshot: MapSnapshot,
    category_names: tuple[str, ...],
    *,
    overlays: tuple[RobotMapOverlay, ...],
) -> CompressedImage:
    """Encode the readable 2-D map raster used by Foxglove Image panels."""

    frontiers = tuple(
        extract_frontiers(
            snapshot.grid,
            snapshot.origin_xy_m,
            snapshot.resolution_m,
        )
    )
    image = render_semantic_overview(
        snapshot.grid,
        category_names,
        snapshot.origin_xy_m,
        snapshot.resolution_m,
        robot_overlays=overlays,
        frontiers=frontiers,
    )
    ok, encoded = cv2.imencode(
        ".png", image, [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
    )
    if not ok:
        raise RuntimeError("failed to encode semantic overview PNG")
    return CompressedImage(
        timestamp=now_ts(),
        frame_id=snapshot.frame_id,
        data=encoded.tobytes(),
        format="png",
    )


def map_poll_loop(
    sources: list[RobotSource], category_names, *, interval_s: float,
    downsample: int, allow_legacy: bool,
) -> None:
    """Background thread: the map is the one channel still snapshot-polled
    on purpose -- see the module docstring for why. Runs forever until the
    process exits."""
    while True:
        now_s = time.monotonic()
        for source in sources:
            if now_s - source.last_map_published_at_s < interval_s:
                continue
            try:
                snapshot = load_grid_npz(
                    source.snapshot_dir / "central_map.npz",
                    allow_legacy=allow_legacy,
                )
                if snapshot is None:
                    semantic_msg = None
                    geometry_msg = None
                    overview_msg = None
                else:
                    semantic_msg = grid_to_message(
                        snapshot.grid,
                        snapshot.origin_xy_m,
                        snapshot.resolution_m,
                        category_names,
                        downsample,
                        snapshot.frame_id,
                        view="semantic",
                    )
                    geometry_msg = grid_to_message(
                        snapshot.grid,
                        snapshot.origin_xy_m,
                        snapshot.resolution_m,
                        category_names,
                        downsample,
                        snapshot.frame_id,
                        view="geometry",
                    )
                    stats = {
                        "obstacle_cells": int(np.count_nonzero(snapshot.grid[0] > 0.5)),
                        "explored_cells": int(np.count_nonzero(snapshot.grid[1] > 0.5)),
                        "semantic_cells": semantic_evidence_cells(snapshot.grid),
                    }
                    with source.lock:
                        source.last_map_stats = stats
                    semantic_scene = semantic_scene_from_snapshot(
                        source.name, snapshot, category_names
                    )
                    try:
                        overview_msg = semantic_overview_message(
                            snapshot,
                            tuple(category_names),
                            overlays=(robot_map_overlay(source),),
                        )
                    except (RuntimeError, ValueError) as exc:
                        overview_msg = None
                        source.status_channel.log(Log(
                            timestamp=now_ts(),
                            level=LogLevel.Warning,
                            name=source.name,
                            message=f"semantic overview skipped: {exc}",
                        ))
                    try:
                        shadow_scene = load_shadow_target_scene(
                            source.name,
                            source.robot_id,
                            source.snapshot_dir / "shadow_target.json",
                            snapshot,
                        )
                    except (OSError, ValueError) as exc:
                        shadow_scene = None
                        source.status_channel.log(Log(
                            timestamp=now_ts(),
                            level=LogLevel.Warning,
                            name=source.name,
                            message=f"ignored invalid VLM shadow target: {exc}",
                        ))
            except (OSError, EOFError, ValueError, zipfile.BadZipFile) as exc:
                # pipeline.py's save() writes atomically (temp file +
                # os.replace) so this shouldn't happen -- kept as defense in
                # depth so one bad read skips this cycle instead of killing
                # the thread a client may be actively depending on.
                semantic_msg = None
                geometry_msg = None
                overview_msg = None
                source.status_channel.log(Log(
                    timestamp=now_ts(),
                    level=LogLevel.Error,
                    name=source.name,
                    message=f"map publish blocked: {exc}",
                ))
                source.last_map_published_at_s = now_s
            if semantic_msg is not None and geometry_msg is not None:
                source.map_channel.log(semantic_msg)
                source.geometry_channel.log(geometry_msg)
                source.pose_channel.log(semantic_scene)
                if shadow_scene is not None:
                    source.pose_channel.log(shadow_scene)
                if overview_msg is not None and source.overview_channel is not None:
                    source.overview_channel.log(overview_msg)
                    with source.lock:
                        source.last_overview_message = overview_msg
                source.last_map_published_at_s = now_s
        time.sleep(min(interval_s, 1.0))


def fusion_poll_loop(
    sources: list[RobotSource],
    fused_semantic_channel: GridChannel,
    fused_geometry_channel: GridChannel,
    fused_status_channel: LogChannel,
    category_names, *, interval_s: float, downsample: int,
    fused_overview_channel: CompressedImageChannel | None = None,
) -> None:
    """Background thread: real cross-robot map fusion (upstream's
    element-wise max rule, `focus_hub.fusion.align_and_fuse_grids`), not
    just two side-by-side panels.

    Only meaningful once a real G4 calibration has been applied to the
    non-reference robot's sender (`--shared-frame-transform-file`) -- that's
    what makes both daemons' otherwise-independent `origin_xy_m` bounding
    boxes actually live in the same physical shared_world frame in the
    first place. Each daemon still picks its own extent independently
    (never coordinated with the other), so `align_and_fuse_grids` handles
    the resulting different-origin alignment; it does NOT compensate for a
    missing or wrong calibration -- if that's off, this fuses two maps that
    are subtly (or badly) misaligned, silently. See
    `audit/SHARED_FRAME_V2_20260722.md` for the current board calibration and
    its independent moved-board holdout.
    """
    last_published_at_s = 0.0
    while True:
        now_s = time.monotonic()
        if now_s - last_published_at_s >= interval_s:
            try:
                loaded = [
                    load_grid_npz(s.snapshot_dir / "central_map.npz")
                    for s in sources
                ]
                if all(item is not None for item in loaded):
                    snapshots = [item for item in loaded if item is not None]
                    frame_id, resolution_m, calibration_id = validate_fusion_contract(
                        snapshots
                    )
                    grids = [item.grid for item in snapshots]
                    origins = [item.origin_xy_m for item in snapshots]
                    fused_grid, fused_origin = align_and_fuse_grids(
                        grids, origins, resolution_m
                    )
                    fused_semantic_channel.log(
                        grid_to_message(
                            fused_grid, fused_origin, resolution_m,
                            category_names, downsample, frame_id, view="semantic",
                        )
                    )
                    fused_geometry_channel.log(
                        grid_to_message(
                            fused_grid, fused_origin, resolution_m,
                            category_names, downsample, frame_id, view="geometry",
                        )
                    )
                    if fused_overview_channel is not None:
                        fused_snapshot = MapSnapshot(
                            grid=fused_grid,
                            origin_xy_m=fused_origin,
                            resolution_m=resolution_m,
                            frame_id=frame_id,
                            transform_version="fused-read-only-view",
                            shared_frame_calibration_id=calibration_id,
                            map_format_version="focus-hub-fused-read-only-v1",
                        )
                        try:
                            fused_overview_channel.log(
                                semantic_overview_message(
                                    fused_snapshot,
                                    tuple(category_names),
                                    overlays=tuple(
                                        robot_map_overlay(source)
                                        for source in sources
                                    ),
                                )
                            )
                        except (RuntimeError, ValueError) as exc:
                            fused_status_channel.log(Log(
                                timestamp=now_ts(),
                                level=LogLevel.Warning,
                                name="fused",
                                message=f"fused semantic overview skipped: {exc}",
                            ))
                    explored = int(np.count_nonzero(fused_grid[1] > 0.5))
                    obstacles = int(np.count_nonzero(fused_grid[0] > 0.5))
                    semantic = semantic_evidence_cells(fused_grid)
                    fused_status_channel.log(Log(
                        timestamp=now_ts(), level=LogLevel.Info,
                        message=f"fused {len(sources)} robots: shape={fused_grid.shape}, "
                                f"origin={fused_origin}, calibration={calibration_id}, "
                                f"explored={explored}, obstacles={obstacles}, "
                                f"semantic_evidence={semantic}",
                        name="fused"))
                else:
                    fused_status_channel.log(Log(
                        timestamp=now_ts(), level=LogLevel.Warning, name="fused",
                        message="waiting for all robots to have a map snapshot before fusing"))
                last_published_at_s = now_s
            except (OSError, EOFError, ValueError, zipfile.BadZipFile) as exc:
                fused_status_channel.log(Log(
                    timestamp=now_ts(), level=LogLevel.Error, name="fused",
                    message=f"fusion failed this cycle: {exc}"))
                last_published_at_s = now_s
        time.sleep(min(interval_s, 1.0))


def frame_tree_loop(channel: FrameTransformChannel, *, interval_s: float = 5.0) -> None:
    """Background thread: periodically publishes an identity transform
    registering `shared_world` as a known frame in Foxglove's frame tree.

    Defense in depth, not a confirmed fix -- every Grid/map message this
    relay publishes uses `frame_id="shared_world"`, but this process never
    published anything establishing that frame in a transform tree at all
    (no `/tf`-equivalent). Whether Foxglove's 3D panel requires a frame to
    appear in its transform tree before it will render data anchored to it
    is genuinely unverified here (this environment is headless, no way to
    confirm against the real app -- see audit/FOXGLOVE_DASHBOARD_20260720.md).
    This costs nothing to publish and is standard practice regardless (a
    real deployment would normally have at least one static transform), so
    it's added rather than left as an open question with no mitigation.
    """
    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    while True:
        channel.log(FrameTransform(
            timestamp=now_ts(), parent_frame_id="world", child_frame_id="shared_world",
            translation=Vector3(x=0.0, y=0.0, z=0.0), rotation=identity,
        ))
        time.sleep(interval_s)


def update_pose_scene(source: RobotSource, status: dict) -> SceneUpdate | None:
    """Update a relay-lifetime camera trail and return its scene message.

    The wire contract currently exposes camera XY but not a separately
    calibrated body pose.  The marker is therefore labelled as a camera
    position and deliberately carries no fabricated heading.
    """
    xy = status.get("last_camera_xy_m")
    frame_id = status.get("frame_id")
    if (
        not isinstance(xy, list)
        or len(xy) != 2
        or not all(isinstance(value, (int, float)) and np.isfinite(value) for value in xy)
        or not isinstance(frame_id, str)
        or not frame_id
    ):
        return None
    point = (float(xy[0]), float(xy[1]))
    heading = status.get("last_camera_heading_deg")
    if not isinstance(heading, (int, float)) or not np.isfinite(heading):
        heading = None
    transported_trajectory = status.get("trajectory_xy_m")
    valid_trajectory = None
    if isinstance(transported_trajectory, list):
        parsed_trajectory = []
        for item in transported_trajectory:
            if (
                not isinstance(item, list)
                or len(item) != 2
                or not all(
                    isinstance(value, (int, float)) and np.isfinite(value)
                    for value in item
                )
            ):
                parsed_trajectory = []
                break
            parsed_trajectory.append((float(item[0]), float(item[1])))
        if parsed_trajectory:
            valid_trajectory = parsed_trajectory
    with source.lock:
        if source.trajectory_frame_id != frame_id:
            source.trajectory_xy_m.clear()
            source.trajectory_frame_id = frame_id
        if valid_trajectory is not None:
            source.trajectory_xy_m = valid_trajectory[-2000:]
        if (
            not source.trajectory_xy_m
            or np.linalg.norm(
                np.asarray(point) - np.asarray(source.trajectory_xy_m[-1])
            ) >= 0.05
        ):
            source.trajectory_xy_m.append(point)
        trail = tuple(source.trajectory_xy_m[-2000:])
        source.last_pose_xy_m = point
        source.last_heading_deg = (
            None if heading is None else float(heading)
        )

    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    origin_pose = Pose(
        position=Vector3(x=0.0, y=0.0, z=0.0), orientation=identity
    )
    marker_pose = Pose(
        position=Vector3(x=point[0], y=point[1], z=0.08),
        orientation=identity,
    )
    lines = []
    if len(trail) >= 2:
        lines.append(LinePrimitive(
            type=LinePrimitiveLineType.LineStrip,
            pose=origin_pose,
            thickness=0.05,
            scale_invariant=False,
            points=[Point3(x=x, y=y, z=0.05) for x, y in trail],
            color=Color(r=0.10, g=0.45, b=1.0, a=1.0),
        ))
    entity = SceneEntity(
        timestamp=now_ts(),
        frame_id=frame_id,
        id=f"{source.name}-camera-trail",
        frame_locked=True,
        lines=lines,
        cubes=[CubePrimitive(
            pose=marker_pose,
            size=Vector3(x=0.28, y=0.28, z=0.16),
            color=Color(r=0.90, g=0.10, b=0.10, a=1.0),
        )],
        texts=[TextPrimitive(
            pose=Pose(
                position=Vector3(x=point[0], y=point[1], z=0.32),
                orientation=identity,
            ),
            billboard=True,
            font_size=14.0,
            scale_invariant=True,
            color=Color(r=0.90, g=0.10, b=0.10, a=1.0),
            text=f"{source.name} camera",
        )],
    )
    return SceneUpdate(entities=[entity])


def semantic_scene_from_snapshot(
    source_name: str,
    snapshot: MapSnapshot,
    category_names: tuple[str, ...],
) -> SceneUpdate:
    """Render every semantic cell as an exact colored world-frame pixel.

    A 5 cm semantic cell is almost invisible in a 26 m Foxglove grid, and the
    production relay downsamples that grid to 15 cm cells. The Grid remains
    the exact evidence representation; these very shallow cubes copy every
    original 5 cm cell onto the already-visible ``/<robot>/map_pose`` topic so
    the blob is visible above the geometry plane. They preserve the semantic
    palette and component shape instead of replacing it with a bounding box.
    One small label is placed over the largest connected component of each
    present category so the operator can still identify the color.
    """

    categories = snapshot.grid[2 : 2 + len(category_names)]
    if categories.shape[0] != len(category_names):
        raise ValueError(
            f"grid has {snapshot.grid.shape[0] - 2} semantic channels but "
            f"{len(category_names)} names were supplied"
        )

    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    cubes = []
    texts = []
    palette_grid = np.zeros(
        (2 + len(category_names), 1, len(category_names)), dtype=np.float32
    )
    for category_index in range(len(category_names)):
        palette_grid[2 + category_index, 0, category_index] = 1.0
    palette_rgb = colorize_semantic_grid(
        palette_grid, category_names
    )[0, :, :3].astype(np.float64) / 255.0
    for category_index in range(len(category_names)):
        rows, cols = np.nonzero(categories[category_index] > 0.1)
        if rows.size == 0:
            continue
        red, green, blue = palette_rgb[category_index]
        for row, col in zip(rows.tolist(), cols.tolist()):
            center_x = snapshot.origin_xy_m[0] + (
                float(col) + 0.5
            ) * snapshot.resolution_m
            center_y = snapshot.origin_xy_m[1] + (
                float(row) + 0.5
            ) * snapshot.resolution_m
            marker_pose = Pose(
                position=Vector3(x=center_x, y=center_y, z=0.035),
                orientation=identity,
            )
            cubes.append(CubePrimitive(
                pose=marker_pose,
                size=Vector3(
                    x=snapshot.resolution_m,
                    y=snapshot.resolution_m,
                    z=0.07,
                ),
                color=Color(
                    r=float(red),
                    g=float(green),
                    b=float(blue),
                    a=1.0,
                ),
            ))

        # Keep the label attached to evidence instead of drawing an enclosing
        # box. The largest component avoids placing it between a real object
        # and a handful of isolated model-noise cells elsewhere in the map.
        remaining = set(zip(rows.tolist(), cols.tolist()))
        largest_component: list[tuple[int, int]] = []
        while remaining:
            seed = remaining.pop()
            stack = [seed]
            component = [seed]
            while stack:
                row, col = stack.pop()
                for delta_row in (-1, 0, 1):
                    for delta_col in (-1, 0, 1):
                        if delta_row == 0 and delta_col == 0:
                            continue
                        neighbour = (row + delta_row, col + delta_col)
                        if neighbour in remaining:
                            remaining.remove(neighbour)
                            stack.append(neighbour)
                            component.append(neighbour)
            if len(component) > len(largest_component):
                largest_component = component

        component_array = np.asarray(largest_component, dtype=np.float64)
        label_row, label_col = component_array.mean(axis=0)
        label_x = snapshot.origin_xy_m[0] + (
            float(label_col) + 0.5
        ) * snapshot.resolution_m
        label_y = snapshot.origin_xy_m[1] + (
            float(label_row) + 0.5
        ) * snapshot.resolution_m
        texts.append(TextPrimitive(
            pose=Pose(
                position=Vector3(x=label_x, y=label_y, z=0.16),
                orientation=identity,
            ),
            billboard=True,
            font_size=13.0,
            scale_invariant=True,
            color=Color(
                r=float(red),
                g=float(green),
                b=float(blue),
                a=1.0,
            ),
            text=category_names[category_index],
        ))

    return SceneUpdate(entities=[SceneEntity(
        timestamp=now_ts(),
        frame_id=snapshot.frame_id,
        id=f"{source_name}-semantic-objects",
        frame_locked=True,
        cubes=cubes,
        texts=texts,
    )])


def shadow_target_scene_from_payload(
    source_name: str,
    robot_id: str,
    snapshot: MapSnapshot,
    payload: dict[str, object],
    *,
    now_ns: int | None = None,
) -> SceneUpdate:
    """Render one expiring, explicitly non-authoritative VLM target."""

    now_ns = time.time_ns() if now_ns is None else now_ns
    target = validate_shadow_target_payload(
        payload,
        robot_id=robot_id,
        snapshot=snapshot,
        now_ns=now_ns,
    )
    remaining_ns = max(1, target.expires_at_ns - now_ns)
    lifetime = Duration(
        remaining_ns // 1_000_000_000,
        remaining_ns % 1_000_000_000,
    )
    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    marker_color = Color(r=0.95, g=0.15, b=0.85, a=0.90)
    marker_pose = Pose(
        position=Vector3(x=target.x_m, y=target.y_m, z=target.z_m + 0.05),
        orientation=identity,
    )
    text_pose = Pose(
        position=Vector3(x=target.x_m, y=target.y_m, z=target.z_m + 0.35),
        orientation=identity,
    )
    entity = SceneEntity(
        timestamp=now_ts(),
        frame_id=snapshot.frame_id,
        id=f"{source_name}-vlm-shadow-target",
        lifetime=lifetime,
        frame_locked=True,
        cylinders=[CylinderPrimitive(
            pose=marker_pose,
            size=Vector3(x=0.36, y=0.36, z=0.10),
            color=marker_color,
        )],
        texts=[TextPrimitive(
            pose=text_pose,
            billboard=True,
            font_size=14.0,
            scale_invariant=True,
            color=marker_color,
            text=(
                f"SHADOW {target.frontier_id} · {target.goal_category} "
                "· NO MOTION"
            ),
        )],
    )
    return SceneUpdate(entities=[entity])


def load_shadow_target_scene(
    source_name: str,
    robot_id: str,
    path: Path,
    snapshot: MapSnapshot,
) -> SceneUpdate | None:
    """Load an atomic shadow artifact without letting it block map display."""

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    if not isinstance(payload, dict) or payload.get("status") != SHADOW_STATUS:
        return None
    expires_at_ns = int(payload.get("expires_at_ns", 0))
    now_ns = time.time_ns()
    if expires_at_ns <= now_ns:
        return None
    return shadow_target_scene_from_payload(
        source_name,
        robot_id,
        snapshot,
        payload,
        now_ns=now_ns,
    )


def legend_loop(channel: LogChannel, *, interval_s: float = 30.0) -> None:
    message = (
        "MAP LEGEND: gray=unknown; white=observed free; black=current geometric "
        "obstacle; blue=camera trail since relay start; red=current camera XY. "
        "Colored 5cm blocks=model-derived semantic cells (chair=red), copied "
        "exactly above geometry; one label identifies each present category; "
        "/<name>/semantic_overview and /fused/semantic_overview are cropped "
        "2D image views with heading arrows, persistent trajectories and "
        "lettered frontier candidates; "
        "magenta SHADOW markers are display-only VLM would-targets and NEVER "
        "robot commands; semantic Grid is also available separately."
    )
    while True:
        channel.log(Log(
            timestamp=now_ts(), level=LogLevel.Info, name="map-legend", message=message
        ))
        time.sleep(interval_s)


def status_loop(sources: list[RobotSource], *, interval_s: float) -> None:
    """Background thread: republishes staleness for both channels. Camera
    staleness comes from real push timestamps (in-memory, set by the HTTP
    handler below) when available; falls back to the map snapshot's own
    `live_status.json` (written by hub_pipeline_daemon.py) for robots that
    haven't been switched to a push-based camera preview yet, or purely for
    frames_total/last_camera_xy_m context the push path doesn't carry."""
    while True:
        now_ns = time.time_ns()
        for source in sources:
            with source.lock:
                camera_pushed_at = source.last_camera_pushed_at_ns
                frames_pushed = source.camera_frames_pushed
                map_stats = dict(source.last_map_stats) if source.last_map_stats else None
            camera_age_s = (now_ns - camera_pushed_at) / 1e9 if camera_pushed_at else None

            snapshot_status = None
            status_path = source.snapshot_dir / "live_status.json"
            if status_path.exists():
                try:
                    snapshot_status = json.loads(status_path.read_text())
                except (json.JSONDecodeError, OSError):
                    snapshot_status = None
            map_age_s = None
            map_path = source.snapshot_dir / "central_map.npz"
            try:
                map_age_s = (now_ns - map_path.stat().st_mtime_ns) / 1e9
            except OSError:
                pass

            parts = [f"{source.name} ({source.robot_id}):"]
            level = LogLevel.Info
            if camera_age_s is not None:
                parts.append(f"camera age {camera_age_s:.1f}s ({frames_pushed} pushed)")
                if camera_age_s > 10.0:
                    level = LogLevel.Warning
            else:
                parts.append("camera: no pushed frames yet")
                level = LogLevel.Warning
            if map_age_s is not None:
                parts.append(f"map snapshot age {map_age_s:.1f}s")
                if map_age_s > 30.0:
                    level = LogLevel.Warning
            else:
                parts.append("map: no snapshot yet")

            if snapshot_status is not None:
                integrated = snapshot_status.get("frames_total")
                observed = snapshot_status.get("observations_seen")
                skipped = snapshot_status.get("skipped_non_keyframes")
                if integrated is not None and observed is not None:
                    parts.append(
                        f"map keyframes {integrated}/{observed} observations"
                        + (f" ({skipped} skipped)" if skipped is not None else "")
                    )
                blocked = snapshot_status.get("mapping_blocked_reason")
                if blocked:
                    parts.append(f"MAPPING HALTED: {blocked}")
                    level = LogLevel.Error
                scene = update_pose_scene(source, snapshot_status)
                if scene is not None:
                    source.pose_channel.log(scene)

            if map_stats is not None:
                obstacle = map_stats["obstacle_cells"]
                explored = map_stats["explored_cells"]
                ratio = 100.0 * obstacle / explored if explored else 0.0
                parts.append(
                    f"geometry {obstacle}/{explored} obstacle/explored ({ratio:.1f}%)"
                )
                semantic = map_stats["semantic_cells"]
                parts.append(f"semantic evidence {semantic} cells")

            source.status_channel.log(Log(
                timestamp=now_ts(), level=level, message=" ".join(parts), name=source.name))
        time.sleep(interval_s)


def camera_latch_loop(sources: list[RobotSource], *, interval_s: float) -> None:
    """Republish the latest image with its original timestamp.

    Foxglove image channels are live streams rather than latched ROS topics.
    Re-emitting the last message lets a newly subscribed/reconnected panel
    render immediately. Keeping the original timestamp is important: this is
    a retained preview, not a fabricated fresh camera frame, and ``status_loop``
    continues to report age from real pushes only.
    """
    while True:
        for source in sources:
            with source.lock:
                message = source.last_camera_message
                overview_message = source.last_overview_message
            if message is not None:
                source.camera_channel.log(message)
            if (
                overview_message is not None
                and source.overview_channel is not None
            ):
                source.overview_channel.log(overview_message)
        time.sleep(interval_s)


def build_app(sources_by_name: dict[str, RobotSource], tokens_by_robot_id: dict[str, str]) -> FastAPI:
    app = FastAPI()

    @app.post("/camera/{name}")
    async def push_camera(name: str, request: Request, x_robot_token: str = Header(default="")):
        source = sources_by_name.get(name)
        if source is None:
            raise HTTPException(404, f"unknown robot name {name!r}")
        expected_token = tokens_by_robot_id.get(source.robot_id)
        if expected_token is None or x_robot_token != expected_token:
            raise HTTPException(401, "invalid or missing X-Robot-Token")
        data = await request.body()
        if not data:
            raise HTTPException(400, "empty body")
        if not data.startswith(b"\xff\xd8"):
            raise HTTPException(400, "camera payload is not a JPEG")
        message = CompressedImage(
            timestamp=now_ts(), frame_id=name, data=data, format="jpeg"
        )
        source.camera_channel.log(message)
        with source.lock:
            source.last_camera_pushed_at_ns = time.time_ns()
            source.camera_frames_pushed += 1
            source.last_camera_message = message
        return Response(status_code=204)

    @app.get("/healthz")
    async def healthz():
        return {"status": "ok", "robots": list(sources_by_name.keys())}

    return app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot", dest="robots", action="append", required=True, type=parse_robot_arg,
        metavar="ROBOT_ID:NAME:SNAPSHOT_DIR",
        help="repeatable; one per robot, e.g. --robot robot-0:wsj:hub/runtime/map_out_wsj "
             "--robot robot-1:yunji:hub/runtime/map_out_yunji")
    parser.add_argument("--port", type=int, default=8765, help="Foxglove WebSocket port")
    parser.add_argument("--preview-port", type=int, default=8766,
                         help="HTTP port camera-preview publishers push JPEG frames to "
                              "(POST /camera/{name}, X-Robot-Token header)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--tokens-file", type=Path, default=WORKSPACE / "hub" / "runtime" / "tokens.json",
                         help="robot_id -> token JSON, same file focus_hub_up.sh generates; camera "
                              "preview pushes must present the matching robot's token")
    parser.add_argument("--map-interval-s", type=float, default=5.0,
                         help="map republish cadence -- slower than the camera on purpose, see "
                              "the module docstring")
    parser.add_argument("--map-downsample", type=int, default=3,
                         help="block-average factor applied to the map before publishing, to "
                              "keep each message well under common WebSocket size limits (a "
                              "full-resolution 520x520 grid is ~1 MiB; factor 3 brings that "
                              "under 150 KiB)")
    parser.add_argument("--status-interval-s", type=float, default=2.0)
    parser.add_argument(
        "--camera-latch-interval-s",
        type=float,
        default=2.0,
        help="republish the retained last camera message for late subscribers",
    )
    parser.add_argument(
        "--allow-legacy-maps",
        action="store_true",
        help=(
            "show old per-robot snapshots that lack frame metadata; this "
            "never makes them eligible for fusion"
        ),
    )
    parser.add_argument("--fuse", action="store_true",
                         help="also publish real cross-robot fused maps on /fused/geometry_map "
                              "and /fused/semantic_map "
                              "(focus_hub.fusion.align_and_fuse_grids -- element-wise max, "
                              "upstream's rule) and /fused/status. Only meaningful once a real "
                              "G4 calibration has been applied to the non-reference robot's "
                              "sender; requires at least 2 --robot sources.")
    parser.add_argument("--fuse-interval-s", type=float, default=8.0,
                         help="fusion republish cadence -- slower than a single map on purpose, "
                              "it does strictly more work (reads every robot's map, aligns, "
                              "then fuses)")
    args = parser.parse_args()
    if args.camera_latch_interval_s <= 0.0:
        parser.error("--camera-latch-interval-s must be positive")

    tokens_by_robot_id = json.loads(args.tokens_file.read_text())

    server = foxglove.start_server(
        host=args.host, port=args.port, name="focus-hub-dashboard", capabilities=None,
    )
    print(f"Foxglove relay listening on ws://{args.host}:{args.port} "
          f"(connect Foxglove to this address)")

    sources = []
    for robot_id, name, snapshot_dir in args.robots:
        if robot_id not in tokens_by_robot_id:
            print(f"warning: no token for {robot_id!r} in {args.tokens_file} -- "
                  f"camera pushes for {name!r} will be rejected", file=sys.stderr)
        sources.append(RobotSource(
            robot_id=robot_id, name=name, snapshot_dir=snapshot_dir,
            camera_channel=CompressedImageChannel(f"/{name}/camera"),
            map_channel=GridChannel(f"/{name}/semantic_map"),
            geometry_channel=GridChannel(f"/{name}/geometry_map"),
            pose_channel=SceneUpdateChannel(f"/{name}/map_pose"),
            status_channel=LogChannel(f"/{name}/status"),
            overview_channel=CompressedImageChannel(
                f"/{name}/semantic_overview"
            ),
        ))
        print(f"  robot_id={robot_id} name={name} snapshot_dir={snapshot_dir}")
    sources_by_name = {s.name: s for s in sources}

    threading.Thread(
        target=frame_tree_loop, args=(FrameTransformChannel("/tf"),), daemon=True,
    ).start()
    threading.Thread(
        target=map_poll_loop, args=(sources, HM3D_CATEGORY_NAMES),
        kwargs={
            "interval_s": args.map_interval_s,
            "downsample": args.map_downsample,
            "allow_legacy": args.allow_legacy_maps,
        },
        daemon=True,
    ).start()
    threading.Thread(
        target=status_loop, args=(sources,), kwargs={"interval_s": args.status_interval_s},
        daemon=True,
    ).start()
    threading.Thread(
        target=camera_latch_loop,
        args=(sources,),
        kwargs={"interval_s": args.camera_latch_interval_s},
        daemon=True,
    ).start()
    threading.Thread(
        target=legend_loop,
        args=(LogChannel("/map/legend"),),
        daemon=True,
    ).start()

    if args.fuse:
        if len(sources) < 2:
            print("warning: --fuse given with fewer than 2 --robot sources; nothing to fuse",
                  file=sys.stderr)
        else:
            fused_semantic_channel = GridChannel("/fused/semantic_map")
            fused_geometry_channel = GridChannel("/fused/geometry_map")
            fused_overview_channel = CompressedImageChannel(
                "/fused/semantic_overview"
            )
            fused_status_channel = LogChannel("/fused/status")
            threading.Thread(
                target=fusion_poll_loop,
                args=(sources, fused_semantic_channel, fused_geometry_channel,
                      fused_status_channel, HM3D_CATEGORY_NAMES),
                kwargs={
                    "interval_s": args.fuse_interval_s,
                    "downsample": args.map_downsample,
                    "fused_overview_channel": fused_overview_channel,
                },
                daemon=True,
            ).start()
            print(f"  fused maps: /fused/geometry_map + /fused/semantic_map "
                  f"+ /fused/semantic_overview "
                  f"(every {args.fuse_interval_s}s, "
                  f"{len(sources)} robots)")

    print(f"Camera preview push endpoint on http://{args.host}:{args.preview_port} "
          f"(POST /camera/{{name}}, X-Robot-Token header)")
    app = build_app(sources_by_name, tokens_by_robot_id)
    try:
        uvicorn.run(app, host=args.host, port=args.preview_port, log_level="warning")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
