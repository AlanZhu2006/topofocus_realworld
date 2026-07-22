#!/usr/bin/env python3
"""Incremental hub pipeline daemon for long-duration soak runs.

Tails the observation spool, integrates new frames into the central RedNet
map as they arrive, and every --decision-interval seconds runs the real
3-stage VLM decision cascade (Perception -> Judgment -> gate -> Decision,
see `focus_hub.vlm_decision`/`vlm_prompts` — this replaced a
Decision-VLM-only call on 2026-07-19 once the fuller upstream pipeline was
found) and publishes a HOLD decision carrying the would-be choice (GOAL
stays policy-blocked: this daemon never relaxes safety). Pass --no-cascade
to fall back to the original single-call behavior (e.g. if YOLO/GLM startup
cost isn't wanted for a quick smoke test).

Writes JSONL telemetry (frame timings, decision latencies, RSS/GPU memory) to
--log for the soak audit, and a final summary JSON on SIGINT/SIGTERM.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "dependencies"))
sys.path.insert(0, str(WORKSPACE / "source" / "Focus_realworld"))

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES, MapperConfig, RedNetSegmenter  # noqa: E402
from focus_hub.directional_memory import DirectionalMemory  # noqa: E402
from focus_hub.frontiers import (  # noqa: E402
    extract_frontiers, render_annotated_bev, render_semantic_decision_map)
from focus_hub.ground_plane import GroundPlaneConfig, estimate_startup_ground  # noqa: E402
from focus_hub.models import Decision  # noqa: E402
from focus_hub.pipeline import SpoolMappingPipeline, iter_spooled_observations  # noqa: E402
from focus_hub.pose_gate import (  # noqa: E402
    KeyframeConfig,
    StartupPoseConfig,
    StartupPoseGate,
)
from focus_hub.vlm_decision import choose_frontier_fallback, choose_frontier_glm, run_decision_cascade  # noqa: E402
from focus_hub.vlm_prompts import extract_scene_objects, format_scene_objects_for_prompt  # noqa: E402
from focus_hub.yolo_detector import YoloDetector  # noqa: E402


def heading_deg_from_pose(T_shared_camera: np.ndarray) -> float:
    """Approximate world-frame heading from the camera's own forward axis
    projected onto the XY plane. Upstream's `start_o` comes directly from
    Habitat's own agent state; real cameras don't carry a separate "robot
    forward" concept in this wire protocol (especially for wsj, which has
    no base_link pose at all — see `focus_ros_sender.py`). This is used
    only for the visualization arrow and the directional-memory angle
    bucket, not anything safety-critical — a documented approximation, not
    a fabricated precise value.
    """
    forward_world = T_shared_camera[:2, 2]  # camera's +Z column, XY part
    if np.linalg.norm(forward_world) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(forward_world[1], forward_world[0]))


def rss_mib() -> float:
    with open(f"/proc/{os.getpid()}/statm") as f:
        return int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 2**20


def gpu_mib() -> int:
    import subprocess

    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
        capture_output=True, text=True,
    )
    return int(out.stdout.split()[0]) if out.returncode == 0 else -1


def write_camera_snapshot(
    pipeline: SpoolMappingPipeline, out_dir: Path, robot_id: str,
    last_metadata, frames_total: int,
) -> None:
    """Writes the cheap, fast-changing half of what a live dashboard
    (foxglove_relay.py) polls: the latest RGB frame and enough timestamps to
    compute honest staleness. Deliberately called every processed frame, not
    gated by --snapshot-interval-s -- a JPEG encode+write is cheap (unlike
    the map save below) and the camera feed is the one channel a viewer
    actually notices lag on frame-to-frame. This is not a live push -- a
    poller reads whatever was last written here and must show its own age,
    not pretend to be real-time.
    """
    status = {
        "robot_id": robot_id,
        "frames_total": frames_total,
        "observations_seen": pipeline.observations_seen,
        "skipped_non_keyframes": pipeline.skipped_non_keyframes,
        "pose_jump_events": pipeline.pose_jump_events,
        "mapping_blocked_reason": pipeline.mapping_blocked_reason,
        "frame_id": pipeline.frame_id,
        "transform_version": pipeline.transform_version,
        "shared_frame_calibration_id": pipeline.shared_frame_calibration_id,
        "floor_z_m": pipeline.mapper.map.floor_z_m,
        "floor_plane_coefficients": list(
            pipeline.mapper.map.floor_plane_coefficients
        ),
        "floor_source": pipeline.floor_source,
        "ground_rejected_frames": pipeline.ground_rejected_frames,
        "ground_drift_events": pipeline.ground_drift_events,
        "last_ground_sequence": pipeline.last_ground_sequence,
        "last_ground_reason": pipeline.last_ground_reason,
        "last_ground_tilt_delta_deg": pipeline.last_ground_tilt_delta_deg,
        "last_ground_height_delta_m": pipeline.last_ground_height_delta_m,
        "written_at_ns": time.time_ns(),
        "last_capture_time_ns": last_metadata.capture_time_ns if last_metadata else None,
        "last_sent_time_ns": last_metadata.sent_time_ns if last_metadata else None,
        "last_camera_xy_m": list(pipeline.last_camera_xy) if pipeline.last_camera_xy else None,
    }
    (out_dir / "live_status.json").write_text(json.dumps(status) + "\n")
    if pipeline.last_rgb_bgr is not None:
        ok, jpeg = cv2.imencode(".jpg", pipeline.last_rgb_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok:
            (out_dir / "latest_rgb.jpg").write_bytes(jpeg.tobytes())


def write_map_snapshot(pipeline: SpoolMappingPipeline, out_dir: Path) -> None:
    """Writes the expensive, slow-changing half: the current map, via the
    existing pipeline.save contract. Gated by --snapshot-interval-s on
    purpose -- the map changes far more slowly than the camera feed and a
    compressed-npz save is real work, unlike the camera write above.
    """
    pipeline.save(out_dir)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--robot-id", default="robot-0")
    parser.add_argument("--hub-url", required=True)
    parser.add_argument("--admin-token-file", type=Path, required=True)
    parser.add_argument("--glm-url", default=None)
    parser.add_argument("--decision-interval", type=float, default=60.0)
    parser.add_argument("--decision-expiry-s", type=float, default=30.0)
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument(
        "--camera-height",
        type=float,
        default=0.4,
        help="fallback height used only with --ground-mode camera-height",
    )
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--no-cascade", action="store_true",
                        help="fall back to the original Decision-VLM-only call (no Perception/"
                             "Judgment stages, no YOLO, no directional memory)")
    parser.add_argument("--yolo-weights", type=Path,
                        default=WORKSPACE / "artifacts" / "vision" / "yolov10m.pt")
    parser.add_argument("--early-episode-steps", type=int, default=125,
                        help="upstream's l_step<=125 override: Decision VLM always runs during "
                             "this many early decision cycles regardless of the Judgment gate")
    parser.add_argument("--snapshot-interval-s", type=float, default=0.0,
                        help="if > 0, periodically write central_map.npz/map_summary.json (via "
                             "pipeline.save), a latest_rgb.jpg, and a live_status.json (capture/"
                             "sent timestamps, frame count) to --out-dir on this cadence, for a "
                             "live viewer (e.g. foxglove_relay.py) to poll. 0 disables (default: "
                             "only the final on-shutdown save happens, as before).")
    parser.add_argument("--start-after-sequence", type=int, default=-1,
                        help="skip spooled observations at or before this sequence number "
                             "(default -1: process the whole spool from the start, as before). "
                             "Real use case: a sender's pose source changed mid-run (e.g. a G4 "
                             "shared-frame calibration applied via --shared-frame-transform-file "
                             "part-way through a long-running sender) -- the daemon's map extent "
                             "is fixed from whichever observation it processes first "
                             "(see mapper_init below), so replaying pre-change observations in "
                             "the same run would lock in bounds from the OLD coordinate frame "
                             "and silently drop every post-change observation that falls outside "
                             "them once the robot moves. Set this to the highest sequence number "
                             "already spooled right before restarting the daemon, to start fresh "
                             "from only the new, consistently-framed data.")
    parser.add_argument(
        "--expected-transform-version",
        default=None,
        help=(
            "bind this map run to one transform/session version and fail before "
            "integration if any observation differs; when omitted, bind to the "
            "first processed observation"
        ),
    )
    parser.add_argument(
        "--shared-frame-calibration-id",
        default=None,
        help=(
            "explicit common calibration/session id for maps eligible for "
            "cross-robot fusion; omit for an independent per-robot map"
        ),
    )
    parser.add_argument("--startup-stable-frames", type=int, default=3)
    parser.add_argument("--startup-max-pose-delta-m", type=float, default=2.0)
    parser.add_argument("--startup-max-rotation-delta-deg", type=float, default=90.0)
    parser.add_argument("--startup-max-interval-s", type=float, default=10.0)
    parser.add_argument("--keyframe-translation-m", type=float, default=0.20)
    parser.add_argument("--keyframe-rotation-deg", type=float, default=10.0)
    parser.add_argument("--keyframe-max-interval-s", type=float, default=5.0)
    parser.add_argument(
        "--ground-mode",
        choices=("ransac", "camera-height"),
        default="ransac",
        help=(
            "estimate a three-frame ground-plane consensus, or explicitly "
            "use camera z minus --camera-height"
        ),
    )
    parser.add_argument("--obstacle-band-low-m", type=float, default=0.15)
    parser.add_argument("--obstacle-band-high-m", type=float, default=0.75)
    parser.add_argument(
        "--max-ground-tilt-delta-deg",
        type=float,
        default=3.0,
        help=(
            "with --ground-mode ransac, halt this map before integration if "
            "a frame's floor normal differs this much from startup"
        ),
    )
    parser.add_argument(
        "--max-ground-height-delta-m",
        type=float,
        default=0.08,
        help=(
            "with --ground-mode ransac, halt this map before integration if "
            "the local floor height differs this much from startup"
        ),
    )
    parser.add_argument(
        "--obstacle-fusion-mode",
        choices=("max", "log_odds"),
        default="log_odds",
        help=(
            "max preserves upstream replay fusion; log_odds uses reversible "
            "free-ray/occupied-endpoint evidence for live depth"
        ),
    )
    parser.add_argument(
        "--obstacle-min-hits",
        type=int,
        default=2,
        help="minimum accepted keyframes supporting a cell before it is an obstacle",
    )
    args = parser.parse_args()
    if args.ground_mode == "ransac" and args.startup_stable_frames < 3:
        parser.error("--ground-mode ransac requires --startup-stable-frames >= 3")
    if args.max_ground_tilt_delta_deg <= 0.0:
        parser.error("--max-ground-tilt-delta-deg must be positive")
    if args.max_ground_height_delta_m <= 0.0:
        parser.error("--max-ground-height-delta-m must be positive")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    log = args.log.open("a", buffering=1)

    def emit(kind: str, **fields) -> None:
        log.write(json.dumps({"t": time.time(), "kind": kind, **fields}) + "\n")

    running = True

    def stop(_sig, _frm):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    segmenter = RedNetSegmenter(WORKSPACE / "artifacts" / "checkpoints" / "rednet_semmap_mp3d_40.pth")
    yolo = None
    if not args.no_cascade and args.glm_url:
        yolo = YoloDetector(args.yolo_weights)
    memory = DirectionalMemory()
    decision_step = 0
    pre_goal_point: tuple[int, int] | None = None
    emit("startup", rss_mib=round(rss_mib(), 1), gpu_mib=gpu_mib(), cascade_enabled=not args.no_cascade)

    pipeline: SpoolMappingPipeline | None = None
    startup_gate = StartupPoseGate(StartupPoseConfig(
        required_consecutive=args.startup_stable_frames,
        max_translation_delta_m=args.startup_max_pose_delta_m,
        max_rotation_delta_deg=args.startup_max_rotation_delta_deg,
        max_interval_s=args.startup_max_interval_s,
    ))
    startup_pending = []
    ground_config = GroundPlaneConfig()
    startup_transform_version: str | None = None
    highest_sequence = args.start_after_sequence
    if highest_sequence >= 0:
        emit("skip_to_sequence", start_after_sequence=highest_sequence)
    frames_total = 0
    observations_total = 0
    decisions_total = 0
    frame_ms: list[float] = []
    last_decision_at = time.monotonic()
    last_snapshot_at = time.monotonic()
    last_metadata = None
    admin_token = args.admin_token_file.read_text().strip()

    import httpx

    while running:
        new = list(iter_spooled_observations(
            args.spool, args.robot_id, after_sequence=highest_sequence))
        for observation in new:
            highest_sequence = max(highest_sequence, observation.sequence)
            observations_total += 1
            last_metadata = observation.metadata
            if pipeline is None:
                observation_version = observation.metadata.pose.transform_version
                if (
                    args.expected_transform_version is not None
                    and observation_version != args.expected_transform_version
                ):
                    raise ValueError(
                        "startup observation transform version mismatch: "
                        f"expected={args.expected_transform_version!r}, "
                        f"observed={observation_version!r}, "
                        f"sequence={observation.sequence}"
                    )
                if (
                    startup_transform_version is not None
                    and observation_version != startup_transform_version
                ):
                    emit(
                        "startup_transform_reset",
                        previous=startup_transform_version,
                        current=observation_version,
                        sequence=observation.sequence,
                    )
                    startup_gate.reset()
                    startup_pending.clear()
                startup_transform_version = observation_version
                startup_decision = startup_gate.evaluate(
                    observation.T_shared_camera,
                    observation.metadata.capture_time_ns,
                )
                if startup_decision.reset:
                    rejected = [item.sequence for item in startup_pending]
                    startup_pending = [observation]
                    emit(
                        "startup_pose_reset",
                        reason=startup_decision.reason,
                        rejected_sequences=rejected,
                        new_candidate_sequence=observation.sequence,
                        translation_m=round(startup_decision.translation_m, 4),
                        rotation_deg=round(startup_decision.rotation_deg, 3),
                        elapsed_sec=round(startup_decision.elapsed_sec, 3),
                    )
                else:
                    startup_pending.append(observation)
                if not startup_decision.ready:
                    continue

                stable = startup_pending[-args.startup_stable_frames:]
                stable_positions = np.stack(
                    [item.T_shared_camera[:3, 3] for item in stable], axis=0
                )
                x0, y0, z0 = np.median(stable_positions, axis=0)
                margin = MapperConfig().max_range_m + 8.0
                K = stable[-1].metadata.intrinsics
                K_matrix = np.array(
                    [[K.fx, 0, K.cx], [0, K.fy, K.cy], [0, 0, 1.0]]
                )
                if args.ground_mode == "ransac":
                    ground = estimate_startup_ground(stable, K_matrix, ground_config)
                    if (
                        not ground.accepted
                        or ground.ground_z_m is None
                        or ground.plane_coefficients is None
                    ):
                        emit(
                            "startup_ground_rejected",
                            reason=ground.reason,
                            sequences=[item.sequence for item in stable],
                            candidates=[
                                {
                                    "accepted": item.accepted,
                                    "reason": item.reason,
                                    "ground_z_m": item.ground_z_m,
                                    "candidate_points": item.candidate_points,
                                    "inlier_points": item.inlier_points,
                                    "inlier_ratio": round(item.inlier_ratio, 4),
                                    "tilt_deg": item.tilt_deg,
                                    "plane_coefficients": item.plane_coefficients,
                                }
                                for item in ground.candidates
                            ],
                        )
                        startup_pending = startup_pending[-args.startup_stable_frames:]
                        continue
                    floor_z = ground.ground_z_m
                    floor_plane = ground.plane_coefficients
                    floor_source = "three_frame_ransac_plane_consensus"
                else:
                    floor_z = float(z0) - args.camera_height
                    floor_plane = (0.0, 0.0, floor_z)
                    floor_source = "explicit_camera_height"
                config = MapperConfig(
                    map_size_m=2 * margin,
                    obstacle_fusion_mode=args.obstacle_fusion_mode,
                    obstacle_min_hits=args.obstacle_min_hits,
                    obstacle_band_low_m=args.obstacle_band_low_m,
                    obstacle_band_high_m=args.obstacle_band_high_m,
                )
                bound_transform_version = (
                    args.expected_transform_version
                    or stable[-1].metadata.pose.transform_version
                )
                pipeline = SpoolMappingPipeline(
                    segmenter,
                    K_matrix,
                    config,
                    (float(x0) - margin, float(y0) - margin),
                    floor_z,
                    expected_transform_version=bound_transform_version,
                    floor_plane_coefficients=floor_plane,
                    ground_plane_config=(
                        ground_config if args.ground_mode == "ransac" else None
                    ),
                    max_ground_tilt_delta_deg=args.max_ground_tilt_delta_deg,
                    max_ground_height_delta_m=args.max_ground_height_delta_m,
                    frame_id=stable[-1].metadata.pose.shared_T_camera.parent_frame,
                    robot_id=args.robot_id,
                    shared_frame_calibration_id=args.shared_frame_calibration_id,
                    floor_source=floor_source,
                    keyframe_config=KeyframeConfig(
                        translation_threshold_m=args.keyframe_translation_m,
                        rotation_threshold_deg=args.keyframe_rotation_deg,
                        max_interval_sec=args.keyframe_max_interval_s,
                        pose_jump_translation_m=args.startup_max_pose_delta_m,
                        pose_jump_rotation_deg=args.startup_max_rotation_delta_deg,
                    ),
                    halt_on_pose_jump=True,
                )
                emit(
                    "mapper_init",
                    origin=[float(x0) - margin, float(y0) - margin],
                    floor_z=floor_z,
                    floor_plane_coefficients=floor_plane,
                    floor_source=floor_source,
                    transform_version=pipeline.transform_version,
                    frame_id=pipeline.frame_id,
                    shared_frame_calibration_id=pipeline.shared_frame_calibration_id,
                    stable_sequences=[item.sequence for item in stable],
                    obstacle_fusion_mode=args.obstacle_fusion_mode,
                    obstacle_min_hits=args.obstacle_min_hits,
                    obstacle_band_m=[
                        args.obstacle_band_low_m,
                        args.obstacle_band_high_m,
                    ],
                )
                observations_to_process = stable
                startup_pending.clear()
            else:
                observations_to_process = [observation]

            for mapping_observation in observations_to_process:
                last_metadata = mapping_observation.metadata
                t0 = time.perf_counter()
                keyframe_decision = pipeline.process(mapping_observation)
                elapsed_ms = (time.perf_counter() - t0) * 1e3
                if keyframe_decision.accept:
                    frame_ms.append(elapsed_ms)
                    frames_total += 1
                    if frames_total % 100 == 0:
                        emit(
                            "progress",
                            frames=frames_total,
                            observations=observations_total,
                            skipped=pipeline.skipped_non_keyframes,
                            mean_frame_ms=round(float(np.mean(frame_ms[-100:])), 1),
                            rss_mib=round(rss_mib(), 1),
                            gpu_mib=gpu_mib(),
                        )
                elif keyframe_decision.pose_jump:
                    emit(
                        "mapping_halted_pose_jump",
                        sequence=mapping_observation.sequence,
                        translation_m=round(keyframe_decision.translation_m, 4),
                        rotation_deg=round(keyframe_decision.rotation_deg, 3),
                        reason=pipeline.mapping_blocked_reason,
                    )
                elif keyframe_decision.reason == "ground_drift":
                    emit(
                        "mapping_halted_ground_drift",
                        sequence=mapping_observation.sequence,
                        tilt_delta_deg=pipeline.last_ground_tilt_delta_deg,
                        height_delta_m=pipeline.last_ground_height_delta_m,
                        reason=pipeline.mapping_blocked_reason,
                    )
                elif keyframe_decision.reason.startswith("ground_"):
                    emit(
                        "mapping_skipped_ground_rejected",
                        sequence=mapping_observation.sequence,
                        reason=keyframe_decision.reason,
                    )
                if args.snapshot_interval_s > 0:
                    write_camera_snapshot(
                        pipeline,
                        args.out_dir,
                        args.robot_id,
                        last_metadata,
                        frames_total,
                    )

        now = time.monotonic()
        if (args.snapshot_interval_s > 0 and pipeline is not None
                and now - last_snapshot_at >= args.snapshot_interval_s):
            last_snapshot_at = now
            write_map_snapshot(pipeline, args.out_dir)

        if pipeline is not None and now - last_decision_at >= args.decision_interval:
            last_decision_at = now
            decision_step += 1
            t0 = time.perf_counter()
            grid = pipeline.mapper.map.grid
            frontiers = [] if pipeline.mapping_blocked_reason else extract_frontiers(
                grid, pipeline.mapper.map.origin_xy_m, pipeline.mapper.config.resolution_m)
            choice = None
            source = "none"
            cascade_info: dict | None = None
            if frontiers:
                robot_rc = None
                if pipeline.last_camera_xy is not None:
                    row, col = pipeline.mapper.map.world_to_cell(
                        np.array([pipeline.last_camera_xy[0]]),
                        np.array([pipeline.last_camera_xy[1]]))
                    robot_rc = (int(row[0]), int(col[0]))

                if args.glm_url and not args.no_cascade and robot_rc is not None and pipeline.last_rgb_bgr is not None:
                    try:
                        scene_objects_dict = extract_scene_objects(grid[2:2 + len(HM3D_CATEGORY_NAMES)], HM3D_CATEGORY_NAMES)
                        scene_objects_str = format_scene_objects_for_prompt(scene_objects_dict)
                        detections = yolo.detect(pipeline.last_rgb_bgr) if yolo else {}
                        heading = heading_deg_from_pose(pipeline.last_camera_T)
                        judgment_map = render_semantic_decision_map(
                            grid, HM3D_CATEGORY_NAMES, frontiers, robot_rc, heading,
                            history_nodes=memory.history_nodes)
                        decision_map = render_semantic_decision_map(
                            grid, HM3D_CATEGORY_NAMES, frontiers, robot_rc, heading)
                        cascade = run_decision_cascade(
                            rgb_bgr=pipeline.last_rgb_bgr, judgment_map_bgr=judgment_map,
                            decision_map_bgr=decision_map, frontiers=frontiers,
                            target=args.goal_category, detections=detections,
                            scene_objects=scene_objects_str, cur_location_rc=robot_rc,
                            heading_deg=heading, pre_goal_point=pre_goal_point, step=decision_step,
                            early_episode_step_threshold=args.early_episode_steps,
                            memory=memory, base_url=args.glm_url)
                        choice = cascade.frontier_choice
                        source = choice.source if choice else "gated-no-decision"
                        if choice is not None:
                            pre_goal_point = (choice.frontier.row, choice.frontier.col)
                        cascade_info = {
                            "perception_pr": cascade.perception_pr, "judgment_pr": cascade.judgment_pr,
                            "gate_passed": cascade.gate_passed, "gate_reason": cascade.gate_reason,
                            "errors": cascade.errors,
                        }
                        if cascade.errors:
                            emit("cascade_stage_errors", errors=cascade.errors)
                    except Exception as exc:  # noqa: BLE001 - soak must keep going
                        emit("decision_error", error=str(exc)[:300])
                else:
                    bev = render_annotated_bev(grid, frontiers, robot_rc)
                    try:
                        if args.glm_url:
                            choice = choose_frontier_glm(
                                bev, frontiers, base_url=args.glm_url,
                                goal_category=args.goal_category)
                        else:
                            choice = choose_frontier_fallback(frontiers)
                        source = choice.source
                    except Exception as exc:  # noqa: BLE001 - soak must keep going
                        emit("decision_error", error=str(exc)[:300])
            vlm_ms = (time.perf_counter() - t0) * 1e3

            now_ns = time.time_ns()
            if pipeline.mapping_blocked_reason is not None:
                reason = f"mapping halted: {pipeline.mapping_blocked_reason}"
            elif choice is not None:
                reason = f"GOAL blocked by policy; would explore frontier {choice.frontier.frontier_id} ({source})"
            elif cascade_info is not None and not cascade_info["gate_passed"]:
                reason = f"judgment gate declined a new frontier this cycle: {cascade_info['gate_reason']}"
            else:
                reason = "no frontier available"
            decision = Decision(
                robot_id=args.robot_id,
                decision_id=f"soak-{uuid.uuid4().hex[:12]}",
                mode="HOLD",
                map_version=0,
                transform_version=pipeline.transform_version or "UNSET",
                issued_at_ns=now_ns,
                expires_at_ns=now_ns + int(args.decision_expiry_s * 1e9),
                reason=reason[:512],
            )
            try:
                response = httpx.post(
                    f"{args.hub_url}/v1/admin/decisions",
                    json=json.loads(decision.model_dump_json()),
                    headers={"X-Admin-Token": admin_token},
                    timeout=10.0,
                )
                decisions_total += 1
                emit("decision", status=response.status_code,
                     decision_id=decision.decision_id, vlm_ms=round(vlm_ms, 1),
                     frontier=None if choice is None else choice.frontier.frontier_id,
                     probabilities=None if choice is None else choice.probabilities,
                     cascade=cascade_info,
                     frames_so_far=frames_total,
                     rss_mib=round(rss_mib(), 1), gpu_mib=gpu_mib())
            except Exception as exc:  # noqa: BLE001
                emit("decision_publish_error", error=str(exc)[:300])

        if not new:
            time.sleep(0.5)

    if pipeline is not None:
        pipeline.save(args.out_dir)
    summary = {
        "frames_total": frames_total,
        "observations_total": observations_total,
        "startup_waiting": pipeline is None,
        "mapping_blocked_reason": None if pipeline is None else pipeline.mapping_blocked_reason,
        "decisions_total": decisions_total,
        "mean_frame_ms": round(float(np.mean(frame_ms)), 1) if frame_ms else None,
        "p95_frame_ms": round(float(np.percentile(frame_ms, 95)), 1) if frame_ms else None,
        "final_rss_mib": round(rss_mib(), 1),
        "final_gpu_mib": gpu_mib(),
    }
    (args.out_dir / "soak_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    emit("shutdown", **summary)
    log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
