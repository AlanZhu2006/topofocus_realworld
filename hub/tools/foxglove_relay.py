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

import numpy as np
import uvicorn
from fastapi import FastAPI, Header, HTTPException, Request, Response

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

import foxglove  # noqa: E402
from foxglove.channels import (  # noqa: E402
    CompressedImageChannel, FrameTransformChannel, GridChannel, LogChannel,
)
from foxglove.messages import (  # noqa: E402
    CompressedImage, FrameTransform, Grid, Log, LogLevel, PackedElementField,
    PackedElementFieldNumericType, Pose, Quaternion, Timestamp, Vector2, Vector3,
)

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES  # noqa: E402
from focus_hub.frontiers import _category_palette  # noqa: E402
from focus_hub.fusion import align_and_fuse_grids  # noqa: E402
from focus_hub.map_snapshot import (  # noqa: E402
    MapSnapshot,
    load_map_snapshot,
    validate_fusion_contract,
)


@dataclass
class RobotSource:
    robot_id: str
    name: str
    snapshot_dir: Path
    camera_channel: CompressedImageChannel
    map_channel: GridChannel
    status_channel: LogChannel
    last_map_published_at_s: float = 0.0
    last_camera_pushed_at_ns: int | None = None
    camera_frames_pushed: int = 0
    last_camera_message: CompressedImage | None = None
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
    obstacle = grid[0] > 0.5
    explored = grid[1] > 0.5
    cat = grid[2:2 + len(category_names)]
    h, w = obstacle.shape
    rgb = np.full((h, w, 3), 96, dtype=np.uint8)          # unknown: dark grey
    rgb[explored] = (235, 235, 235)                        # explored free: light
    rgb[obstacle] = (40, 40, 40)                            # obstacle: near-black
    if len(category_names) > 0:
        palette = _category_palette(len(category_names))
        has_category = cat.max(axis=0) > 0.1
        best_category = cat.argmax(axis=0)
        rgb[has_category] = palette[best_category[has_category]][:, ::-1]  # BGR -> RGB
    alpha = np.full((h, w, 1), 255, dtype=np.uint8)
    alpha[~explored & ~obstacle & (cat.max(axis=0) <= 0.1 if len(category_names) else True)] = 60
    return np.concatenate([rgb, alpha], axis=-1)


def downsample_rgba(rgba: np.ndarray, factor: int) -> np.ndarray:
    """Block-averages an (h, w, 4) uint8 RGBA image by an integer factor.

    A 520x520 map at 4 bytes/cell is ~1 MiB per message -- too much to push
    once a second to a remote viewer over a real (non-loopback) network link
    (this is exactly the "consider the delay" requirement: the map is by far
    the biggest payload of the three channels, and doesn't need sub-second
    freshness the way the camera feed does). Any leftover rows/cols that
    don't divide evenly by `factor` are cropped, not padded.
    """
    if factor <= 1:
        return rgba
    h, w = rgba.shape[:2]
    h2, w2 = h - h % factor, w - w % factor
    cropped = rgba[:h2, :w2].astype(np.float32)
    pooled = cropped.reshape(h2 // factor, factor, w2 // factor, factor, 4).mean(axis=(1, 3))
    return pooled.astype(np.uint8)


def grid_to_message(
    grid: np.ndarray, origin_xy_m: tuple[float, float], resolution_m: float,
    category_names: tuple[str, ...], downsample_factor: int = 1,
    frame_id: str = "shared_world",
) -> Grid:
    rgba = colorize_grid(grid, category_names)
    rgba = downsample_rgba(rgba, downsample_factor)
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
    *, allow_legacy: bool = False,
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
                grid_msg = build_grid_message(
                    source.snapshot_dir / "central_map.npz",
                    category_names,
                    downsample,
                    allow_legacy=allow_legacy,
                )
            except (OSError, EOFError, ValueError, zipfile.BadZipFile) as exc:
                # pipeline.py's save() writes atomically (temp file +
                # os.replace) so this shouldn't happen -- kept as defense in
                # depth so one bad read skips this cycle instead of killing
                # the thread a client may be actively depending on.
                grid_msg = None
                source.status_channel.log(Log(
                    timestamp=now_ts(),
                    level=LogLevel.Error,
                    name=source.name,
                    message=f"map publish blocked: {exc}",
                ))
                source.last_map_published_at_s = now_s
            if grid_msg is not None:
                source.map_channel.log(grid_msg)
                source.last_map_published_at_s = now_s
        time.sleep(min(interval_s, 1.0))


def fusion_poll_loop(
    sources: list[RobotSource], fused_channel: GridChannel, fused_status_channel: LogChannel,
    category_names, *, interval_s: float, downsample: int,
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
    `audit/G4_REAL_CALIBRATION_20260720.md` for this calibration's own
    caveats (coincident-assumption precision, one unresolved height anomaly).
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
                    fused_grid, fused_origin = align_and_fuse_grids(grids, origins, resolution_m)
                    fused_channel.log(grid_to_message(
                        fused_grid, fused_origin, resolution_m, category_names, downsample,
                        frame_id))
                    fused_status_channel.log(Log(
                        timestamp=now_ts(), level=LogLevel.Info,
                        message=f"fused {len(sources)} robots: shape={fused_grid.shape}, "
                                f"origin={fused_origin}, calibration={calibration_id}",
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
            if message is not None:
                source.camera_channel.log(message)
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
                         help="also publish a real cross-robot fused map on /fused/semantic_map "
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
            status_channel=LogChannel(f"/{name}/status"),
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

    if args.fuse:
        if len(sources) < 2:
            print("warning: --fuse given with fewer than 2 --robot sources; nothing to fuse",
                  file=sys.stderr)
        else:
            fused_channel = GridChannel("/fused/semantic_map")
            fused_status_channel = LogChannel("/fused/status")
            threading.Thread(
                target=fusion_poll_loop,
                args=(sources, fused_channel, fused_status_channel, HM3D_CATEGORY_NAMES),
                kwargs={"interval_s": args.fuse_interval_s, "downsample": args.map_downsample},
                daemon=True,
            ).start()
            print(f"  fused map: /fused/semantic_map (every {args.fuse_interval_s}s, "
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
