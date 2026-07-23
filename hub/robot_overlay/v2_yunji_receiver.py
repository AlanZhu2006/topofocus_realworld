#!/usr/bin/env python3
"""Yunji robot-local v2 receiver using WATER's own planner/controller.

By default this is a read-only dry run: it derives the Odin-odom to WATER-map
alignment, polls the Hub and validates goals, but reports GOAL as rejected
because live output is disabled.  Live mode emits only WATER high-level point
goals; it never emits wheel velocity.  HOLD, expiry and disconnect cancel the
local WATER navigation task.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Any


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.robot_map_alignment import (  # noqa: E402
    alignment_artifact,
    derive_shared_T_robot_map,
    load_shared_tracking_calibration,
    planar_pose_matrix,
)
from focus_hub.base_camera_calibration import load_base_camera_calibration  # noqa: E402
from focus_hub.geometry import compose_rigid, invert_rigid  # noqa: E402
from focus_hub.transport_v2 import NavigationStatusV2  # noqa: E402
from focus_hub.v2_goal_adapter import (  # noqa: E402
    LocalHighLevelGoal,
    V2AdapterAction,
    V2GoalAdapter,
    V2GoalAdapterConfig,
)
from focus_hub.v2_robot_runtime import (  # noqa: E402
    HubV2RobotClient,
    PathAccumulator,
    WaterTcpClient,
    navigation_event,
    parse_water_current_pose,
    require_water_ok,
    water_move_state,
    water_robot_health,
)


LIVE_CONFIRMATION = "OPERATOR_PRESENT_AND_YUNJI_CLEAR"
ACCESSIBLE_POINT_MIN_VERSION = (0, 10, 7)


def water_version_tuple(value: object) -> tuple[int, ...]:
    """Parse the numeric prefix of WATER's non-semver firmware label."""

    match = re.match(r"^\s*(\d+(?:\.\d+)*)", str(value))
    if match is None:
        return ()
    return tuple(int(part) for part in match.group(1).split("."))


def bounded_legacy_subgoal(
    current_pose: tuple[float, float, float],
    final_goal: LocalHighLevelGoal,
    *,
    step_m: float,
) -> LocalHighLevelGoal:
    """Return one high-level WATER goal no farther than ``step_m``.

    Old WATER firmware has no reachable-point query.  This helper never
    bypasses WATER's planner/controller: it only bounds each successive
    ``/api/move`` target while retaining the original final goal locally.
    """

    if not math.isfinite(step_m) or step_m <= 0.0:
        raise ValueError("legacy subgoal step must be finite and positive")
    dx = final_goal.x - current_pose[0]
    dy = final_goal.y - current_pose[1]
    distance_m = math.hypot(dx, dy)
    if distance_m <= step_m:
        return final_goal
    scale = step_m / distance_m
    return replace(
        final_goal,
        x=current_pose[0] + dx * scale,
        y=current_pose[1] + dy * scale,
        yaw_rad=math.atan2(dy, dx),
        arrival_radius_m=None,
    )


def local_goal_arrival_radius(
    goal: LocalHighLevelGoal,
    *,
    default_m: float = 0.35,
) -> float:
    radius = default_m if goal.arrival_radius_m is None else goal.arrival_radius_m
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("local goal arrival radius must be finite and positive")
    return radius


def quaternion_pose_matrix(message: Any) -> tuple[float, ...]:
    pose = message.pose.pose
    position = pose.position
    quaternion = pose.orientation
    qx, qy, qz, qw = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Odin odometry quaternion has zero/non-finite norm")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    x, y, z = float(position.x), float(position.y), float(position.z)
    values = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Odin odometry pose contains a non-finite value")
    return (
        1 - 2 * (qy * qy + qz * qz),
        2 * (qx * qy - qz * qw),
        2 * (qx * qz + qy * qw),
        x,
        2 * (qx * qy + qz * qw),
        1 - 2 * (qx * qx + qz * qz),
        2 * (qy * qz - qx * qw),
        y,
        2 * (qx * qz - qy * qw),
        2 * (qy * qz + qx * qw),
        1 - 2 * (qx * qx + qy * qy),
        z,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def canonical_sha256(payload: object) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def status_results(client: WaterTcpClient) -> dict[str, object]:
    response = require_water_ok(
        client.request("/api/robot_status"), command="/api/robot_status"
    )
    results = response.get("results")
    if not isinstance(results, dict):
        raise RuntimeError("WATER robot_status returned malformed results")
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18089")
    parser.add_argument("--robot-id", default="robot-1")
    parser.add_argument("--token-file", type=Path, default=OVERLAY / ".token")
    parser.add_argument("--robot-host", default="192.168.10.10")
    parser.add_argument("--tcp-port", type=int, default=31001)
    parser.add_argument("--water-timeout-s", type=float, default=1.5)
    parser.add_argument("--odom-topic", default="/odin1/odometry")
    parser.add_argument("--calibration-file", type=Path, required=True)
    parser.add_argument("--base-camera-calibration-file", type=Path, required=True)
    parser.add_argument("--odin-factory-calibration-file", type=Path, required=True)
    parser.add_argument("--transform-version", required=True)
    parser.add_argument("--shared-frame-calibration-id", required=True)
    parser.add_argument("--local-map-frame", default="yunji/water_map")
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--odom-timeout-s", type=float, default=0.5)
    parser.add_argument("--alignment-max-skew-s", type=float, default=0.25)
    parser.add_argument("--max-goal-distance-m", type=float, default=8.0)
    parser.add_argument(
        "--legacy-firmware-max-goal-distance-m",
        type=float,
        default=0.50,
        help=(
            "maximum short goal accepted on WATER firmware older than 0.10.7; "
            "the native move_base planner remains final motion authority"
        ),
    )
    parser.add_argument(
        "--legacy-firmware-subgoal-step-m",
        type=float,
        default=0.45,
        help=(
            "maximum receding-horizon /api/move step on WATER firmware older "
            "than 0.10.7"
        ),
    )
    parser.add_argument(
        "--legacy-firmware-min-segment-progress-m",
        type=float,
        default=0.03,
        help="minimum progress required before issuing another legacy subgoal",
    )
    parser.add_argument(
        "--legacy-firmware-max-segments",
        type=int,
        default=32,
        help="fail-closed cap on subgoals for one high-level leg",
    )
    parser.add_argument("--alignment-output", type=Path)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--enable-live-water-motion", action="store_true")
    parser.add_argument("--operator-confirmation", default="")
    args = parser.parse_args()
    if args.robot_id != "robot-1":
        parser.error("the Yunji receiver is fixed to canonical robot-1")
    if min(
        args.poll_s,
        args.water_timeout_s,
        args.odom_timeout_s,
        args.alignment_max_skew_s,
        args.max_goal_distance_m,
        args.legacy_firmware_max_goal_distance_m,
        args.legacy_firmware_subgoal_step_m,
        args.legacy_firmware_min_segment_progress_m,
    ) <= 0:
        parser.error("timeouts, poll interval and distance limit must be positive")
    if (
        args.legacy_firmware_subgoal_step_m
        > args.legacy_firmware_max_goal_distance_m
    ):
        parser.error(
            "--legacy-firmware-subgoal-step-m must not exceed "
            "--legacy-firmware-max-goal-distance-m"
        )
    if args.legacy_firmware_max_segments <= 0:
        parser.error("--legacy-firmware-max-segments must be positive")
    live = bool(args.enable_live_water_motion)
    if live and args.operator_confirmation != LIVE_CONFIRMATION:
        parser.error(
            "live WATER output requires --operator-confirmation " + LIVE_CONFIRMATION
        )

    token = os.environ.get("FOCUS_ROBOT_TOKEN", "")
    if not token and args.token_file.is_file():
        token = args.token_file.read_text(encoding="utf-8").strip()
    if not token:
        parser.error("FOCUS_ROBOT_TOKEN or a non-empty --token-file is required")

    calibration = load_shared_tracking_calibration(
        args.calibration_file,
        robot_id=args.robot_id,
        expected_transform_version=args.transform_version,
        expected_calibration_id=args.shared_frame_calibration_id,
    )
    factory_payload = args.odin_factory_calibration_file.expanduser().read_bytes()
    factory_artifact = json.loads(factory_payload)
    if factory_artifact.get("sensor_model") != "odin1":
        parser.error("Odin factory calibration has the wrong sensor model")
    odin_camera_frame = str(factory_artifact.get("camera_frame", ""))
    try:
        odin_base_T_camera = tuple(
            float(value)
            for value in factory_artifact["imu_from_camera"]["matrix"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        parser.error(f"invalid Odin factory imu_from_camera transform: {exc}")
    if len(odin_base_T_camera) != 16:
        parser.error("Odin factory imu_from_camera transform must contain 16 values")
    base_camera_calibration = load_base_camera_calibration(
        args.base_camera_calibration_file,
        expected_robot_id=args.robot_id,
        expected_camera_frame=odin_camera_frame,
    )
    state_dir = Path(
        os.environ.get(
            "FOCUS_ROBOT_STATE_DIR", str(Path.home() / ".local/state/topofocus")
        )
    ).expanduser()
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    alignment_output = (
        args.alignment_output.expanduser()
        if args.alignment_output
        else state_dir / f"yunji-v2-map-alignment-{stamp}.json"
    )
    log_path = (
        args.log.expanduser()
        if args.log
        else state_dir / f"yunji-v2-receiver-{stamp}.jsonl"
    )
    for output in (alignment_output, log_path):
        if output.exists():
            parser.error(f"refusing to overwrite existing output: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("a", encoding="utf-8", buffering=1)

    def emit(event: str, **fields: object) -> None:
        log.write(
            json.dumps(
                {"t_ns": time.time_ns(), "event": event, **fields},
                separators=(",", ":"),
            )
            + "\n"
        )
        log.flush()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class OdinPoseNode(Node):
        def __init__(self) -> None:
            super().__init__("focus_v2_yunji_receiver")
            self.latest_matrix: tuple[float, ...] | None = None
            self.latest_received_ns = 0
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)

        def on_odom(self, message: Odometry) -> None:
            try:
                self.latest_matrix = quaternion_pose_matrix(message)
                self.latest_received_ns = time.time_ns()
            except ValueError as exc:
                emit("odometry_rejected", error=str(exc))

    rclpy.init()
    node = OdinPoseNode()
    water = WaterTcpClient(
        args.robot_host, args.tcp_port, timeout_s=args.water_timeout_s
    )
    hub = HubV2RobotClient(args.base_url, args.robot_id, token)
    max_skew_ns = int(args.alignment_max_skew_s * 1e9)
    water_firmware_version = ""
    water_firmware_parsed: tuple[int, ...] = ()
    try:
        version_response = require_water_ok(
            water.request("/api/software/get_version"),
            command="/api/software/get_version",
        )
        water_firmware_version = str(version_response.get("results", ""))
        water_firmware_parsed = water_version_tuple(water_firmware_version)
    except Exception as exc:  # noqa: BLE001 - unknown capability must fail closed
        emit("water_firmware_query_failed", error=str(exc)[:300])
    accessible_point_supported = (
        water_firmware_parsed >= ACCESSIBLE_POINT_MIN_VERSION
    )
    emit(
        "water_firmware_capability",
        firmware_version=water_firmware_version or "unknown",
        accessible_point_supported=accessible_point_supported,
        accessible_point_min_version="0.10.7",
        legacy_short_goal_limit_m=args.legacy_firmware_max_goal_distance_m,
    )

    # Alignment is read-only and completes before any decision can be honored.
    alignment_deadline = time.monotonic() + 20.0
    aligned_status: dict[str, object] | None = None
    while time.monotonic() < alignment_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_matrix is None:
            continue
        try:
            query_started_ns = time.time_ns()
            candidate_status = status_results(water)
            query_finished_ns = time.time_ns()
            water_pose = parse_water_current_pose(candidate_status)
        except Exception as exc:  # noqa: BLE001 - retry bounded read-only alignment
            emit("alignment_sample_rejected", error=str(exc)[:300])
            continue
        status_sample_ns = (query_started_ns + query_finished_ns) // 2
        skew_ns = abs(status_sample_ns - node.latest_received_ns)
        if skew_ns <= max_skew_ns:
            aligned_status = candidate_status
            break
        emit("alignment_sample_rejected", reason="sample_skew", skew_ns=skew_ns)
    if aligned_status is None or node.latest_matrix is None:
        emit("startup_failed", reason="no_synchronized_odin_water_pose")
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        return 3

    water_pose = parse_water_current_pose(aligned_status)
    status_sample_ns = (query_started_ns + query_finished_ns) // 2
    sample_skew_ns = abs(status_sample_ns - node.latest_received_ns)
    water_map_T_base = planar_pose_matrix(*water_pose)
    # Odin odometry's child is its IMU/base, while WATER current_pose refers
    # to chassis base_link. Use the measured WATER-base-to-Odin-camera mount
    # plus Odin's factory IMU-to-camera extrinsic so both pose samples refer
    # to the same physical chassis origin before aligning their map frames.
    tracking_T_camera = compose_rigid(node.latest_matrix, odin_base_T_camera)
    tracking_T_water_base = compose_rigid(
        tracking_T_camera, invert_rigid(base_camera_calibration.matrix)
    )
    shared_T_water_map = derive_shared_T_robot_map(
        shared_T_tracking=calibration.shared_T_tracking,
        tracking_T_body=tracking_T_water_base,
        robot_map_T_body=water_map_T_base,
    )
    artifact = alignment_artifact(
        calibration=calibration,
        local_map_frame=args.local_map_frame,
        shared_T_robot_map=shared_T_water_map,
        captured_at_ns=max(status_sample_ns, node.latest_received_ns),
        sample_skew_ns=sample_skew_ns,
        max_sample_skew_ns=max_skew_ns,
        observed_inputs={
            "odin_odometry": {
                "topic": args.odom_topic,
                "received_at_ns": node.latest_received_ns,
                "tracking_T_odin_base": list(node.latest_matrix),
                "tracking_T_water_base": list(tracking_T_water_base),
                "status": "observed",
            },
            "odin_factory_calibration": {
                "path": str(args.odin_factory_calibration_file.expanduser().resolve()),
                "size_bytes": len(factory_payload),
                "sha256": hashlib.sha256(factory_payload).hexdigest(),
                "status": "observed_device_calibration",
            },
            "water_base_camera_calibration": {
                "path": base_camera_calibration.source_path,
                "size_bytes": base_camera_calibration.source_size_bytes,
                "sha256": base_camera_calibration.source_sha256,
                "status": base_camera_calibration.measurement_status,
            },
            "water_robot_status": {
                "endpoint": "/api/robot_status",
                "sample_time_ns": status_sample_ns,
                "robot_map_T_body": list(water_map_T_base),
                "selected_fields_sha256": canonical_sha256(aligned_status),
                "status": "observed",
            },
        },
    )
    atomic_write_json(alignment_output, artifact)
    emit(
        "alignment_ready",
        output=str(alignment_output),
        shared_T_water_map=list(shared_T_water_map),
        sample_skew_ns=sample_skew_ns,
        live=live,
    )

    adapter = V2GoalAdapter(
        V2GoalAdapterConfig(
            robot_id=args.robot_id,
            transform_version=args.transform_version,
            shared_frame_calibration_id=args.shared_frame_calibration_id,
            shared_T_robot_map=shared_T_water_map,
            output_kind="water_move",
            local_frame_id=args.local_map_frame,
            max_goal_distance_m=args.max_goal_distance_m,
        )
    )
    path = PathAccumulator()
    path_episode_id: str | None = None
    last_decision_id: str | None = None
    active_decision = None
    active_goal = None
    active_task_id: str | None = None
    active_segment_goal: LocalHighLevelGoal | None = None
    segment_start_remaining_m: float | None = None
    legacy_segment_count = 0
    move_started_monotonic = 0.0
    last_feedback_monotonic = 0.0
    accessible_queries = 0
    accessible_selected: tuple[float, float, float, float] | None = None
    reachability_reference_pose: tuple[float, float, float] | None = None

    def post(decision, status, reason_code, pose, *, zero=False, goal=None,
             detail="", terminal=False) -> bool:
        event = navigation_event(
            decision,
            status=status,
            reason_code=reason_code,
            local_pose=pose,
            path_length_m=path.length_m,
            velocity_zero_confirmed=zero,
            local_goal=goal,
            detail=detail,
            terminal=terminal,
            adapter_name=(
                "water-accessible-point-v1"
                if accessible_point_supported
                else "water-legacy-receding-horizon-v1"
            ),
        )
        try:
            ack = hub.post_event(event)
        except Exception as exc:  # noqa: BLE001
            emit(
                "navigation_event_failed",
                event_id=event.event_id,
                status=status.value,
                error=str(exc)[:500],
            )
            return False
        emit(
            "navigation_event",
            event_id=event.event_id,
            decision_id=decision.decision_id,
            status=status.value,
            reason_code=reason_code,
            hub_status=ack.status,
        )
        return True

    def reset_segment_state() -> None:
        nonlocal active_task_id
        nonlocal active_segment_goal
        nonlocal segment_start_remaining_m
        nonlocal legacy_segment_count
        active_task_id = None
        active_segment_goal = None
        segment_start_remaining_m = None
        legacy_segment_count = 0

    def cancel(reason: str) -> bool:
        if not live:
            emit("water_cancel_skipped_dry_run", reason=reason)
            reset_segment_state()
            return True
        try:
            response = require_water_ok(
                water.request("/api/move/cancel"), command="/api/move/cancel"
            )
            emit("water_move_canceled", reason=reason, response=response)
            reset_segment_state()
            return True
        except Exception as exc:  # noqa: BLE001
            emit("water_cancel_failed", reason=reason, error=str(exc)[:500])
            return False

    def accessible(x: float, y: float) -> bool:
        nonlocal accessible_queries, accessible_selected
        if not accessible_point_supported:
            if not water_firmware_parsed or reachability_reference_pose is None:
                emit(
                    "legacy_native_planner_gate",
                    firmware_version=water_firmware_version or "unknown",
                    requested=[x, y],
                    accepted=False,
                    reason="firmware_capability_or_current_pose_unknown",
                )
                return False
            distance_m = math.hypot(
                x - reachability_reference_pose[0],
                y - reachability_reference_pose[1],
            )
            accepted = distance_m <= args.max_goal_distance_m
            emit(
                "legacy_receding_horizon_gate",
                firmware_version=water_firmware_version,
                requested=[x, y],
                current_pose=list(reachability_reference_pose),
                distance_m=distance_m,
                final_goal_limit_m=args.max_goal_distance_m,
                subgoal_step_m=args.legacy_firmware_subgoal_step_m,
                accepted=accepted,
                final_authority="WATER move_base/local planner/controller",
            )
            if accepted:
                accessible_selected = (x, y, x, y)
            return accepted
        # The official API returns the nearest reachable point.  Accept this
        # exact candidate only when the returned adjustment is small, so the
        # transmitted semantic arrival region remains authoritative.
        accessible_queries += 1
        if accessible_queries > 3:
            return False
        try:
            response = require_water_ok(
                water.request("/api/map/accessible_point_query", x=f"{x:.4f}", y=f"{y:.4f}"),
                command="/api/map/accessible_point_query",
            )
            results = response.get("results")
            position = results.get("position") if isinstance(results, dict) else None
            if not isinstance(position, dict):
                emit(
                    "accessible_point_query_result",
                    requested=[x, y],
                    accepted=False,
                    reason="malformed_position",
                )
                return False
            rx, ry = float(position["x"]), float(position["y"])
            adjustment_m = math.hypot(rx - x, ry - y)
            accepted = adjustment_m <= 0.20
            emit(
                "accessible_point_query_result",
                requested=[x, y],
                returned=[rx, ry],
                adjustment_m=adjustment_m,
                accepted=accepted,
            )
            if not accepted:
                return False
            accessible_selected = (x, y, rx, ry)
            return True
        except Exception as exc:  # noqa: BLE001
            emit("accessible_point_query_failed", error=str(exc)[:300])
            return False

    def start_water_move(
        decision,
        final_goal: LocalHighLevelGoal,
        current_pose: tuple[float, float, float],
        *,
        reason: str,
    ) -> None:
        nonlocal active_task_id
        nonlocal active_segment_goal
        nonlocal segment_start_remaining_m
        nonlocal legacy_segment_count
        nonlocal move_started_monotonic
        if accessible_point_supported:
            segment_goal = final_goal
        else:
            if legacy_segment_count >= args.legacy_firmware_max_segments:
                raise RuntimeError("legacy receding-horizon segment cap reached")
            segment_goal = bounded_legacy_subgoal(
                current_pose,
                final_goal,
                step_m=args.legacy_firmware_subgoal_step_m,
            )
        segment_distance_m = math.hypot(
            segment_goal.x - current_pose[0],
            segment_goal.y - current_pose[1],
        )
        if (
            not accessible_point_supported
            and segment_distance_m
            > args.legacy_firmware_max_goal_distance_m + 1e-6
        ):
            raise RuntimeError("legacy subgoal exceeded the verified distance bound")
        response = require_water_ok(
            water.request(
                "/api/move",
                location=(
                    f"{segment_goal.x:.4f},"
                    f"{segment_goal.y:.4f},"
                    f"{segment_goal.yaw_rad:.4f}"
                ),
            ),
            command="/api/move",
        )
        active_task_id = str(response.get("task_id", "")) or None
        active_segment_goal = segment_goal
        segment_start_remaining_m = math.hypot(
            final_goal.x - current_pose[0],
            final_goal.y - current_pose[1],
        )
        if not accessible_point_supported:
            legacy_segment_count += 1
        move_started_monotonic = time.monotonic()
        emit(
            "water_move_started",
            decision_id=decision.decision_id,
            leg_id=decision.leg_id,
            task_id=active_task_id,
            reason=reason,
            segment_index=(
                None if accessible_point_supported else legacy_segment_count
            ),
            segment_goal=[
                segment_goal.x,
                segment_goal.y,
                segment_goal.yaw_rad,
            ],
            segment_distance_m=segment_distance_m,
            final_goal=[
                final_goal.x,
                final_goal.y,
                final_goal.yaw_rad,
            ],
            final_remaining_m=segment_start_remaining_m,
            final_authority="WATER move_base/local planner/controller",
        )

    exit_code = 0
    try:
        while rclpy.ok():
            cycle_started = time.monotonic()
            move_started_this_cycle = False
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                current_status = status_results(water)
                current_pose = parse_water_current_pose(current_status)
            except Exception as exc:  # noqa: BLE001
                if active_decision is not None:
                    cancel("water_status_failure")
                    active_decision = None
                    active_goal = None
                emit("water_status_failed", error=str(exc)[:500])
                time.sleep(args.poll_s)
                continue
            path.update(current_pose[0], current_pose[1])
            odometry_fresh = (
                node.latest_matrix is not None
                and time.time_ns() - node.latest_received_ns
                <= int(args.odom_timeout_s * 1e9)
            )
            health = water_robot_health(
                current_status, odometry_fresh=odometry_fresh
            )
            if live:
                try:
                    hub.post_heartbeat(health)
                except Exception as exc:  # noqa: BLE001 - lost health link revokes motion
                    if active_decision is not None:
                        cancel("heartbeat_delivery_failed")
                        active_decision = None
                        active_goal = None
                    emit("heartbeat_failed_local_hold", error=str(exc)[:500])
                    time.sleep(args.poll_s)
                    continue
            if health.estop_engaged and active_decision is not None:
                cancel("local_estop")
                post(
                    active_decision,
                    NavigationStatusV2.LOCAL_ESTOP,
                    "LOCAL_STOP_LATCHED",
                    current_pose,
                    detail=health.detail,
                    terminal=True,
                )
                active_decision = None
                active_goal = None

            try:
                decision = hub.latest_decision()
            except Exception as exc:  # noqa: BLE001 - disconnect revokes authority
                if active_decision is not None:
                    cancel("hub_disconnect")
                    active_decision = None
                    active_goal = None
                emit("hub_poll_failed_local_hold", error=str(exc)[:500])
                time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
                continue

            if decision is None:
                if active_decision is not None:
                    zero = cancel("missing_or_expired_lease")
                    post(
                        active_decision,
                        NavigationStatusV2.HOLDING if zero else NavigationStatusV2.REJECTED,
                        "EXPIRED",
                        current_pose,
                        zero=zero,
                        detail="Hub returned no effective decision",
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
                continue

            if decision.decision_id != last_decision_id:
                last_decision_id = decision.decision_id
                if path_episode_id != decision.episode_id:
                    path = PathAccumulator()
                    path.update(current_pose[0], current_pose[1])
                    path_episode_id = decision.episode_id
                if not post(
                    decision,
                    NavigationStatusV2.RECEIVED,
                    "DECISION_RECEIVED",
                    current_pose,
                    detail="authenticated v2 decision parsed locally",
                ):
                    cancel("received_event_delivery_failed")
                    time.sleep(args.poll_s)
                    continue
                accessible_queries = 0
                accessible_selected = None
                reachability_reference_pose = current_pose
                same_leg = (
                    active_decision is not None
                    and active_decision.leg_id == decision.leg_id
                    and active_goal is not None
                )
                result = adapter.evaluate(
                    decision,
                    now_ns=time.time_ns(),
                    health=health,
                    current_position_robot_map=(current_pose[0], current_pose[1], 0.0),
                    is_local_goal_reachable=(
                        (lambda _x, _y: True) if same_leg else accessible
                    ),
                )
                if (
                    same_leg
                    and result.action == V2AdapterAction.GOAL
                    and active_goal is not None
                ):
                    result = replace(
                        result,
                        local_goal=active_goal,
                        command_preview=None,
                        detail=(
                            result.detail
                            + "; lease renewal preserves the original local final goal"
                        ),
                    )
                # Semantic-region selection calls ``accessible`` while choosing
                # a candidate. Frontier points do not, so make the same
                # robot-local WATER reachability gate apply to both target
                # kinds before any move request can be emitted.
                if (
                    result.action == V2AdapterAction.GOAL
                    and result.local_goal is not None
                    and not same_leg
                    and accessible_selected is None
                    and not accessible(result.local_goal.x, result.local_goal.y)
                ):
                    result = replace(
                        result,
                        action=V2AdapterAction.HOLD,
                        reason_code="UNREACHABLE",
                        detail="WATER found no nearby reachable local goal",
                        local_goal=None,
                        command_preview=None,
                    )
                if (
                    result.action == V2AdapterAction.GOAL
                    and result.local_goal is not None
                    and accessible_point_supported
                    and accessible_selected is not None
                    and math.hypot(
                        result.local_goal.x - accessible_selected[0],
                        result.local_goal.y - accessible_selected[1],
                    ) <= 1e-4
                ):
                    adjusted_goal = replace(
                        result.local_goal,
                        x=accessible_selected[2],
                        y=accessible_selected[3],
                    )
                    request_uuid = hashlib.sha256(
                        decision.decision_id.encode("utf-8")
                    ).hexdigest()[:12]
                    result = replace(
                        result,
                        local_goal=adjusted_goal,
                        command_preview=(
                            f"/api/move?location={adjusted_goal.x:.4f},"
                            f"{adjusted_goal.y:.4f},{adjusted_goal.yaw_rad:.4f}"
                            f"&uuid={request_uuid}"
                        ),
                        detail=(
                            result.detail
                            + "; WATER accessible-point adjustment applied locally"
                        ),
                    )
                if (
                    result.action == V2AdapterAction.GOAL
                    and time.time_ns() >= decision.expires_at_ns
                ):
                    result = replace(
                        result,
                        action=V2AdapterAction.HOLD,
                        reason_code="EXPIRED",
                        detail="decision expired during local reachability checks",
                        local_goal=None,
                        command_preview=None,
                    )
                emit(
                    "decision_evaluated",
                    decision_id=decision.decision_id,
                    action=result.action.value,
                    reason_code=result.reason_code,
                    command_preview=result.command_preview,
                    live=live,
                )
                if result.action == V2AdapterAction.GOAL:
                    if not live:
                        post(
                            decision,
                            NavigationStatusV2.REJECTED,
                            "UNSAFE",
                            current_pose,
                            detail="live WATER output is disabled; validation preview only",
                        )
                    else:
                        if active_decision is not None and not same_leg:
                            if not cancel("new_goal_leg"):
                                post(
                                    decision,
                                    NavigationStatusV2.REJECTED,
                                    "LOCAL_PLANNER_REJECTED",
                                    current_pose,
                                    detail="could not cancel previous WATER task",
                                )
                                time.sleep(args.poll_s)
                                continue
                        final_goal = (
                            active_goal
                            if same_leg and active_goal is not None
                            else result.local_goal
                        )
                        accepted = post(
                            decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_GOAL_ACCEPTED",
                            current_pose,
                            goal=final_goal,
                            detail=result.detail,
                        )
                        if not accepted:
                            cancel("accepted_event_delivery_failed")
                            time.sleep(args.poll_s)
                            continue
                        if time.time_ns() >= decision.expires_at_ns:
                            zero = cancel("lease_expired_before_move_submission")
                            post(
                                decision,
                                NavigationStatusV2.REJECTED,
                                "EXPIRED",
                                current_pose,
                                zero=zero,
                                detail="lease expired before WATER move submission",
                                terminal=True,
                            )
                            active_decision = None
                            active_goal = None
                            time.sleep(args.poll_s)
                            continue
                        if not same_leg:
                            reset_segment_state()
                            active_goal = final_goal
                            remaining_m = math.hypot(
                                final_goal.x - current_pose[0],
                                final_goal.y - current_pose[1],
                            )
                            try:
                                if remaining_m <= local_goal_arrival_radius(
                                    final_goal
                                ):
                                    post(
                                        decision,
                                        NavigationStatusV2.ARRIVED,
                                        "LOCAL_ARRIVAL_RADIUS_SATISFIED",
                                        current_pose,
                                        zero=True,
                                        goal=final_goal,
                                        detail=(
                                            f"already within final arrival radius; "
                                            f"remaining_m={remaining_m:.3f}"
                                        ),
                                        terminal=True,
                                    )
                                    active_decision = None
                                    active_goal = None
                                    reset_segment_state()
                                else:
                                    start_water_move(
                                        decision,
                                        final_goal,
                                        current_pose,
                                        reason="new_high_level_leg",
                                    )
                                    move_started_this_cycle = True
                                    active_decision = decision
                            except Exception as exc:  # noqa: BLE001
                                post(
                                    decision,
                                    NavigationStatusV2.REJECTED,
                                    "LOCAL_PLANNER_REJECTED",
                                    current_pose,
                                    detail=str(exc),
                                )
                                active_decision = None
                                active_goal = None
                                reset_segment_state()
                                time.sleep(args.poll_s)
                                continue
                        else:
                            active_decision = decision
                            active_goal = final_goal
                        if active_decision is not None:
                            post(
                                decision,
                                NavigationStatusV2.NAVIGATING,
                                "LOCAL_PLANNER_ACTIVE",
                                current_pose,
                                detail=(
                                    f"WATER task_id={active_task_id or 'not_returned'}; "
                                    f"segment={legacy_segment_count or 1}"
                                ),
                            )
                            last_feedback_monotonic = time.monotonic()
                elif result.action == V2AdapterAction.STOP:
                    zero = cancel("hub_stop")
                    post(
                        decision,
                        NavigationStatusV2.STOPPED if zero else NavigationStatusV2.REJECTED,
                        "LOCAL_STOP_LATCHED",
                        current_pose,
                        zero=zero,
                        detail=result.detail,
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                elif decision.mode.value == "HOLD":
                    zero = cancel("hub_hold")
                    post(
                        decision,
                        NavigationStatusV2.HOLDING if zero else NavigationStatusV2.REJECTED,
                        "HUB_HOLD",
                        current_pose,
                        zero=zero,
                        detail=result.detail,
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                else:
                    cancel("goal_rejected")
                    post(
                        decision,
                        NavigationStatusV2.REJECTED,
                        result.reason_code,
                        current_pose,
                        detail=result.detail,
                    )
                    active_decision = None
                    active_goal = None

            if active_decision is not None and not move_started_this_cycle:
                if time.time_ns() >= active_decision.expires_at_ns:
                    zero = cancel("local_lease_expiry")
                    post(
                        active_decision,
                        NavigationStatusV2.HOLDING if zero else NavigationStatusV2.REJECTED,
                        "EXPIRED",
                        current_pose,
                        zero=zero,
                        detail="local lease timer expired",
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                else:
                    move_state = water_move_state(current_status)
                    in_start_grace = (
                        move_started_monotonic > 0.0
                        and time.monotonic() - move_started_monotonic < 2.0
                    )
                    if move_state == "ARRIVED" and not in_start_grace:
                        if active_goal is None:
                            post(
                                active_decision,
                                NavigationStatusV2.REJECTED,
                                "LOCAL_PLANNER_REJECTED",
                                current_pose,
                                detail="WATER arrived without a retained final goal",
                                terminal=True,
                            )
                            active_decision = None
                            reset_segment_state()
                        else:
                            remaining_m = math.hypot(
                                active_goal.x - current_pose[0],
                                active_goal.y - current_pose[1],
                            )
                            arrival_radius_m = local_goal_arrival_radius(active_goal)
                            if remaining_m <= arrival_radius_m:
                                post(
                                    active_decision,
                                    NavigationStatusV2.ARRIVED,
                                    "LOCAL_PLANNER_ARRIVED",
                                    current_pose,
                                    zero=True,
                                    goal=active_goal,
                                    detail=(
                                        f"WATER move_status="
                                        f"{current_status.get('move_status')}; "
                                        f"final_remaining_m={remaining_m:.3f}; "
                                        f"arrival_radius_m={arrival_radius_m:.3f}"
                                    ),
                                    terminal=True,
                                )
                                active_decision = None
                                active_goal = None
                                reset_segment_state()
                            elif accessible_point_supported:
                                post(
                                    active_decision,
                                    NavigationStatusV2.REJECTED,
                                    "LOCAL_PLANNER_REJECTED",
                                    current_pose,
                                    detail=(
                                        "WATER reported arrival outside the "
                                        f"final radius; remaining_m={remaining_m:.3f}"
                                    ),
                                    terminal=True,
                                )
                                active_decision = None
                                active_goal = None
                                reset_segment_state()
                            else:
                                progress_m = (
                                    -math.inf
                                    if segment_start_remaining_m is None
                                    else segment_start_remaining_m - remaining_m
                                )
                                if (
                                    not math.isfinite(progress_m)
                                    or progress_m
                                    < args.legacy_firmware_min_segment_progress_m
                                ):
                                    post(
                                        active_decision,
                                        NavigationStatusV2.REJECTED,
                                        "LOCAL_PLANNER_REJECTED",
                                        current_pose,
                                        detail=(
                                            "legacy subgoal made insufficient "
                                            f"progress; progress_m={progress_m:.3f}"
                                        ),
                                        terminal=True,
                                    )
                                    active_decision = None
                                    active_goal = None
                                    reset_segment_state()
                                else:
                                    try:
                                        start_water_move(
                                            active_decision,
                                            active_goal,
                                            current_pose,
                                            reason="legacy_segment_continuation",
                                        )
                                        move_started_this_cycle = True
                                        post(
                                            active_decision,
                                            NavigationStatusV2.NAVIGATING,
                                            "LOCAL_PLANNER_ACTIVE",
                                            current_pose,
                                            detail=(
                                                "legacy receding-horizon "
                                                f"segment={legacy_segment_count}; "
                                                f"remaining_m={remaining_m:.3f}"
                                            ),
                                        )
                                        last_feedback_monotonic = time.monotonic()
                                    except Exception as exc:  # noqa: BLE001
                                        post(
                                            active_decision,
                                            NavigationStatusV2.REJECTED,
                                            "LOCAL_PLANNER_REJECTED",
                                            current_pose,
                                            detail=str(exc),
                                            terminal=True,
                                        )
                                        active_decision = None
                                        active_goal = None
                                        reset_segment_state()
                    elif move_state in {"FAILED", "ZERO"} and not in_start_grace:
                        post(
                            active_decision,
                            NavigationStatusV2.REJECTED,
                            "LOCAL_PLANNER_REJECTED",
                            current_pose,
                            detail=f"unexpected WATER move_status={current_status.get('move_status')}",
                            terminal=True,
                        )
                        active_decision = None
                        active_goal = None
                        reset_segment_state()
                    elif (
                        move_state == "ACTIVE"
                        and time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        post(
                            active_decision,
                            NavigationStatusV2.NAVIGATING,
                            "LOCAL_PLANNER_ACTIVE",
                            current_pose,
                            detail=f"WATER task_id={active_task_id or 'not_returned'}",
                        )
                        last_feedback_monotonic = time.monotonic()
            time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
    except KeyboardInterrupt:
        if active_decision is not None:
            cancel("operator_interrupt")
        emit("receiver_stopped", reason="operator_interrupt")
    except Exception as exc:  # noqa: BLE001 - any receiver fault revokes motion
        exit_code = 4
        if active_decision is not None:
            cancel("receiver_fault")
        emit("receiver_fault", error=str(exc)[:1000])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
