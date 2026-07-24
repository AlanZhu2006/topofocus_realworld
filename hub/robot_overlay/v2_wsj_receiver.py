#!/usr/bin/env python3
"""Robot-local v2 receiver for TinyNav POI navigation.

The VLM/Hub supplies only an expiring high-level target. TinyNav keeps global
planning, local planning and velocity control. In explicitly armed live mode
this node gates TinyNav's raw ``/cmd_vel`` onto a separate guarded topic; the
Go2 bridge must subscribe only to that guarded topic. Lease expiry, HOLD,
disconnect or receiver failure closes the gate and publishes zero locally.

Default mode is read-only: it aligns ``shared_world`` to TinyNav's map,
validates decisions and reachability, but never publishes POI, pause or Twist.
The same transport and lease gate is used by WSJ/Go2 and Yunji/WATER; only the
final guarded velocity bridge differs.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.base_camera_calibration import (  # noqa: E402
    load_base_camera_calibration,
)
from focus_hub.geometry import compose_rigid, invert_rigid  # noqa: E402
from focus_hub.models import (  # noqa: E402
    LocalizationState,
    RobotHealth,
    SafetyState,
)
from focus_hub.robot_map_alignment import (  # noqa: E402
    alignment_artifact,
    derive_shared_T_map_from_tracking_map,
    load_shared_tracking_calibration,
    yaw_from_matrix,
)
from focus_hub.transport_v2 import NavigationStatusV2  # noqa: E402
from focus_hub.v2_goal_adapter import (  # noqa: E402
    V2AdapterAction,
    V2GoalAdapter,
    V2GoalAdapterConfig,
)
from focus_hub.v2_robot_runtime import (  # noqa: E402
    HubV2RobotClient,
    OccupancyGrid2D,
    PathAccumulator,
    navigation_event,
)


LIVE_CONFIRMATION = "OPERATOR_PRESENT_AND_WSJ_CLEAR"
LIVE_CONFIRMATIONS = {
    "robot-0": LIVE_CONFIRMATION,
    "robot-1": "OPERATOR_PRESENT_AND_YUNJI_CLEAR",
}
# Mirror the independently enforced sender thresholds exactly.  The receiver
# still recomputes every interval check instead of trusting the producer's
# ``imu_intervals_valid`` boolean, but must not reject telemetry that the
# deployment sender has already classified with a different numeric policy.
SLAM_IMU_MIN_COVERAGE_RATIO = 0.80
SLAM_IMU_MAX_SAMPLE_GAP_S = 0.05
SLAM_IMU_END_TOLERANCE_S = 0.01
TRANSIENT_SLAM_FAILURES = frozenset(
    {
        "imu_intervals_invalid",
        "imu_intervals_missing",
        "imu_interval_invalid",
        "imu_interval_threshold",
    }
)
EXTERNAL_ODOMETRY_MAX_POS_VAR_M2 = 0.01
EXTERNAL_ODOMETRY_MAX_YAW_VAR_RAD2 = 0.01


def quaternion_pose_matrix(pose: Any) -> tuple[float, ...]:
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
        raise ValueError("pose quaternion has zero/non-finite norm")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    x, y, z = float(position.x), float(position.y), float(position.z)
    if not all(math.isfinite(value) for value in (x, y, z, qx, qy, qz, qw)):
        raise ValueError("pose contains a non-finite value")
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


def transform_message_matrix(message: Any) -> tuple[float, ...]:
    transform = message.transform
    pose = type("Pose", (), {})()
    pose.position = transform.translation
    pose.orientation = transform.rotation
    return quaternion_pose_matrix(pose)


def slam_metrics_gate(raw_json: str) -> tuple[bool, str]:
    """Mirror the sender's independent optimizer/IMU health gate."""

    try:
        payload = json.loads(raw_json)
        stats = payload["stats"]
        metrics = payload["metrics"]
        if stats.get("optimizer_status") != "ok":
            return False, f"optimizer_status={stats.get('optimizer_status')}"
        initial = float(metrics["initial_error"])
        final = float(metrics["final_error"])
        if not all(math.isfinite(value) for value in (initial, final)):
            return False, "optimizer_nonfinite"
        if final > initial + max(1e-9, abs(initial) * 1e-6):
            return False, "optimizer_worsened"
        if int(metrics["num_factors"]) <= 0 or int(metrics["num_variables"]) <= 0:
            return False, "optimizer_graph_empty"
        if metrics.get("imu_intervals_valid") is not True:
            return False, "imu_intervals_invalid"
        if int(stats.get("imu_messages_overwritten", 0)) > 0:
            return False, "imu_buffer_overwritten"
        intervals = metrics.get("imu_intervals")
        if not isinstance(intervals, list) or not intervals:
            return False, "imu_intervals_missing"
        for interval in intervals:
            if not isinstance(interval, dict) or interval.get("valid") is not True:
                return False, "imu_interval_invalid"
            if (
                float(interval["duration_s"]) <= 0
                or int(interval["sample_count"]) < 2
                or int(interval["expected_count"]) <= 0
                or float(interval["coverage_ratio"]) < SLAM_IMU_MIN_COVERAGE_RATIO
                or float(interval["max_sample_gap_s"]) > SLAM_IMU_MAX_SAMPLE_GAP_S
                or float(interval["end_error_s"]) > SLAM_IMU_END_TOLERANCE_S
            ):
                return False, "imu_interval_threshold"
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return False, "slam_metrics_malformed"
    return True, "slam_optimizer_imu_valid"


def external_odometry_covariance_gate(
    covariance: Any,
) -> tuple[bool, str]:
    """Fail closed on the same Odin covariance contract used by its sender."""

    try:
        values = [float(value) for value in covariance]
    except (TypeError, ValueError):
        return False, "external_odometry_covariance_malformed"
    if len(values) != 36 or not all(math.isfinite(value) for value in values):
        return False, "external_odometry_covariance_malformed"
    var_x, var_y, var_yaw = values[0], values[7], values[35]
    if min(var_x, var_y, var_yaw) < 0.0:
        return False, "external_odometry_covariance_invalid"
    if (
        max(var_x, var_y) > EXTERNAL_ODOMETRY_MAX_POS_VAR_M2
        or var_yaw > EXTERNAL_ODOMETRY_MAX_YAW_VAR_RAD2
    ):
        return False, "external_odometry_covariance_not_tracking"
    return True, "external_odometry_covariance_tracking"


class SlamHealthDebouncer:
    """Tolerate one diagnostic interval blip, never a persistent/hard fault."""

    def __init__(
        self,
        *,
        max_transient_failures: int = 1,
        max_last_good_age_s: float = 2.0,
    ) -> None:
        if max_transient_failures < 0:
            raise ValueError("max_transient_failures must be non-negative")
        if (
            not math.isfinite(max_last_good_age_s)
            or max_last_good_age_s <= 0
        ):
            raise ValueError("max_last_good_age_s must be finite and positive")
        self.max_transient_failures = max_transient_failures
        self.max_last_good_age_s = max_last_good_age_s
        self.last_good_ns = 0
        self.transient_failures = 0

    def update(self, raw_json: str, *, received_ns: int) -> tuple[bool, str]:
        passed, detail = slam_metrics_gate(raw_json)
        if passed:
            self.last_good_ns = received_ns
            self.transient_failures = 0
            return True, detail
        if detail not in TRANSIENT_SLAM_FAILURES:
            self.transient_failures = 0
            return False, detail
        self.transient_failures += 1
        good_age_s = (
            math.inf
            if self.last_good_ns <= 0
            else (received_ns - self.last_good_ns) / 1e9
        )
        if (
            self.transient_failures <= self.max_transient_failures
            and 0.0 <= good_age_s <= self.max_last_good_age_s
        ):
            return (
                True,
                f"{detail}_transient_tolerated_"
                f"{self.transient_failures}/{self.max_transient_failures}",
            )
        return False, detail


def occupancy_from_message(
    message: Any, *, expected_frame: str | None = None
) -> OccupancyGrid2D:
    if expected_frame is not None and message.header.frame_id != expected_frame:
        raise ValueError(
            f"occupancy frame {message.header.frame_id!r} is not "
            f"{expected_frame!r}"
        )
    orientation = message.info.origin.orientation
    if (
        abs(float(orientation.x)) > 1e-3
        or abs(float(orientation.y)) > 1e-3
        or abs(float(orientation.z)) > 1e-3
        or abs(float(orientation.w) - 1.0) > 1e-3
    ):
        raise ValueError("rotated OccupancyGrid origin is unsupported")
    return OccupancyGrid2D(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution_m=float(message.info.resolution),
        origin_x_m=float(message.info.origin.position.x),
        origin_y_m=float(message.info.origin.position.y),
        data=tuple(int(value) for value in message.data),
    )


def planar_transform_delta(
    first: tuple[float, ...], second: tuple[float, ...]
) -> tuple[float, float]:
    distance = math.hypot(first[3] - second[3], first[7] - second[7])
    yaw = abs(
        (yaw_from_matrix(first) - yaw_from_matrix(second) + math.pi)
        % (2 * math.pi)
        - math.pi
    )
    return distance, yaw


def robot_map_base_pose(
    *,
    tracking_T_map: tuple[float, ...],
    tracking_T_camera: tuple[float, ...],
    base_T_camera: tuple[float, ...],
) -> tuple[float, float, float]:
    """Project TinyNav's optical-camera odometry onto the measured robot base."""

    map_T_camera = compose_rigid(
        invert_rigid(tracking_T_map), tracking_T_camera
    )
    map_T_base = compose_rigid(map_T_camera, invert_rigid(base_T_camera))
    forward_x, forward_y = map_T_base[0], map_T_base[4]
    if math.hypot(forward_x, forward_y) < 1e-6:
        raise RuntimeError("base forward axis has no usable XY projection")
    return (
        map_T_base[3],
        map_T_base[7],
        math.atan2(forward_y, forward_x),
    )


def atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18089")
    parser.add_argument("--robot-id", default="robot-0")
    parser.add_argument("--token-file", type=Path, default=OVERLAY / ".token")
    parser.add_argument("--calibration-file", type=Path, required=True)
    parser.add_argument(
        "--base-camera-calibration-file",
        type=Path,
        required=True,
        help="measured base_link_T_camera artifact for this robot",
    )
    parser.add_argument(
        "--base-camera-frame",
        default="camera",
        help="camera frame declared by the measured mount artifact",
    )
    parser.add_argument("--transform-version", required=True)
    parser.add_argument("--shared-frame-calibration-id", required=True)
    parser.add_argument("--tracking-frame", default="world")
    parser.add_argument("--tinynav-map-frame", default="map")
    parser.add_argument("--local-map-frame", default="wsj/map")
    parser.add_argument("--odom-topic", default="/slam/odometry")
    parser.add_argument("--slam-data-topic", default="/slam/data")
    parser.add_argument(
        "--external-odometry-health",
        action="store_true",
        help=(
            "derive localization freshness from the externally validated "
            "odometry stream instead of TinyNav optimizer diagnostics"
        ),
    )
    parser.add_argument(
        "--platform-health-topic",
        default="",
        help="optional local chassis-bridge JSON status topic",
    )
    parser.add_argument("--occupancy-topic", default="/mapping/static_occupancy_grid")
    parser.add_argument("--cmd-pois-topic", default="/mapping/cmd_pois")
    parser.add_argument("--nav-done-topic", default="/mapping/nav_done")
    parser.add_argument(
        "--router-status-topic", default="/mapping/buildmap_online_status"
    )
    parser.add_argument("--pause-topic", default="/nav/paused")
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/focus_guarded_cmd_vel")
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--local-data-timeout-s", type=float, default=2.0)
    parser.add_argument(
        "--slam-max-transient-failures",
        type=int,
        default=1,
        help="number of consecutive IMU-interval diagnostic blips tolerated",
    )
    parser.add_argument(
        "--slam-transient-grace-s",
        type=float,
        default=2.0,
        help="maximum age of the last passing SLAM report during that blip",
    )
    parser.add_argument("--max-goal-distance-m", type=float, default=8.0)
    parser.add_argument("--reachability-clearance-m", type=float, default=0.05)
    parser.add_argument("--start-snap-radius-m", type=float, default=0.35)
    parser.add_argument(
        "--start-footprint-override-m",
        type=float,
        default=0.18,
        help=(
            "online BuildMap only: bounded measured-base footprint used to "
            "escape a self-occupied start into genuinely free map cells"
        ),
    )
    parser.add_argument("--max-alignment-shift-m", type=float, default=0.15)
    parser.add_argument("--max-alignment-yaw-deg", type=float, default=5.0)
    parser.add_argument("--alignment-output", type=Path)
    parser.add_argument("--log", type=Path)
    parser.add_argument(
        "--online-buildmap-world",
        action="store_true",
        help=(
            "Use the current TinyNav tracking world as the fresh BuildMap map "
            "frame; this explicitly disables saved-map relocalization."
        ),
    )
    parser.add_argument("--enable-live-go2-motion", action="store_true")
    parser.add_argument(
        "--enable-live-tinynav-motion",
        action="store_true",
        help="enable the platform-neutral guarded TinyNav command gate",
    )
    parser.add_argument("--operator-confirmation", default="")
    args = parser.parse_args()
    if args.robot_id not in LIVE_CONFIRMATIONS:
        parser.error("robot ID must be canonical robot-0 or robot-1")
    if args.enable_live_go2_motion and args.robot_id != "robot-0":
        parser.error("--enable-live-go2-motion is valid only for robot-0")
    if args.raw_cmd_topic == args.guarded_cmd_topic:
        parser.error("raw and guarded cmd_vel topics must differ")
    if min(
        args.poll_s,
        args.local_data_timeout_s,
        args.slam_transient_grace_s,
        args.max_goal_distance_m,
        args.max_alignment_shift_m,
        args.max_alignment_yaw_deg,
    ) <= 0:
        parser.error("timeouts, limits and poll interval must be positive")
    if (
        args.slam_max_transient_failures < 0
        or args.reachability_clearance_m < 0
        or args.start_snap_radius_m < 0
        or args.start_footprint_override_m < 0
    ):
        parser.error("reachability distances must be non-negative")
    live = bool(
        args.enable_live_go2_motion or args.enable_live_tinynav_motion
    )
    expected_confirmation = LIVE_CONFIRMATIONS[args.robot_id]
    if live and args.operator_confirmation != expected_confirmation:
        parser.error(
            "live TinyNav output requires --operator-confirmation "
            + expected_confirmation
        )
    if args.online_buildmap_world:
        if args.tracking_frame != args.tinynav_map_frame:
            parser.error(
                "--online-buildmap-world requires identical --tracking-frame "
                "and --tinynav-map-frame"
            )
    elif args.tracking_frame == args.tinynav_map_frame:
        parser.error(
            "identical tracking/map frames require explicit --online-buildmap-world"
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
    base_camera_calibration = load_base_camera_calibration(
        args.base_camera_calibration_file,
        expected_robot_id=args.robot_id,
        expected_camera_frame=args.base_camera_frame,
    )

    state_dir = Path(
        os.environ.get(
            "FOCUS_ROBOT_STATE_DIR", str(Path.home() / ".local/state/topofocus")
        )
    ).expanduser()
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    robot_label = "wsj" if args.robot_id == "robot-0" else "yunji"
    alignment_output = (
        args.alignment_output.expanduser()
        if args.alignment_output
        else state_dir / f"{robot_label}-v2-map-alignment-{stamp}.json"
    )
    log_path = (
        args.log.expanduser()
        if args.log
        else state_dir / f"{robot_label}-v2-receiver-{stamp}.jsonl"
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
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.time import Time
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    class WsjReceiverNode(Node):
        def __init__(self) -> None:
            super().__init__(
                f"focus_v2_{args.robot_id.replace('-', '_')}_tinynav_receiver"
            )
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.world_T_camera: tuple[float, ...] | None = None
            self.odom_received_ns = 0
            self.occupancy: OccupancyGrid2D | None = None
            self.occupancy_received_ns = 0
            self.slam_pass = False
            self.slam_detail = "slam_metrics_missing"
            self.slam_received_ns = 0
            self.slam_gate = SlamHealthDebouncer(
                max_transient_failures=args.slam_max_transient_failures,
                max_last_good_age_s=args.slam_transient_grace_s,
            )
            self.platform_pass = not bool(args.platform_health_topic)
            self.platform_detail = (
                "platform_health_not_configured"
                if not args.platform_health_topic
                else "platform_health_missing"
            )
            self.platform_received_ns = 0
            self.platform_estop = False
            self.nav_done = False
            self.raw_cmd_received_ns = 0
            self.trajectory_received_ns = 0
            self.router_status_received_ns = 0
            self.router_state = ""
            self.router_reason = ""
            self.router_decision_id: str | None = None
            self.router_affected_decision_id: str | None = None
            self.authority_deadline_ns = 0
            self.authorized = False
            self.poi_publisher = self.create_publisher(String, args.cmd_pois_topic, 10)
            pause_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.pause_publisher = self.create_publisher(
                Bool, args.pause_topic, pause_qos
            )
            self.guarded_publisher = self.create_publisher(
                Twist, args.guarded_cmd_topic, 10
            )
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
            if not args.external_odometry_health:
                self.create_subscription(
                    String, args.slam_data_topic, self.on_slam, 20
                )
            if args.platform_health_topic:
                self.create_subscription(
                    String,
                    args.platform_health_topic,
                    self.on_platform_health,
                    pause_qos,
                )
            self.create_subscription(
                OccupancyGrid,
                args.occupancy_topic,
                self.on_occupancy,
                pause_qos,
            )
            self.create_subscription(Bool, args.nav_done_topic, self.on_nav_done, 10)
            self.create_subscription(
                String,
                args.router_status_topic,
                self.on_router_status,
                pause_qos,
            )
            self.create_subscription(Twist, args.raw_cmd_topic, self.on_raw_cmd, 20)
            self.create_subscription(
                RosPath, "/planning/trajectory_path", self.on_trajectory, 10
            )
            self.create_timer(0.05, self.enforce_gate)
            if live:
                paused = Bool()
                paused.data = True
                self.pause_publisher.publish(paused)

        def on_odom(self, message: Odometry) -> None:
            try:
                self.world_T_camera = quaternion_pose_matrix(message.pose.pose)
                self.odom_received_ns = time.time_ns()
                if args.external_odometry_health:
                    self.slam_pass, self.slam_detail = (
                        external_odometry_covariance_gate(
                            message.pose.covariance
                        )
                    )
                    self.slam_received_ns = self.odom_received_ns
            except ValueError as exc:
                emit("odometry_rejected", error=str(exc))

        def on_slam(self, message: String) -> None:
            received_ns = time.time_ns()
            self.slam_pass, self.slam_detail = self.slam_gate.update(
                message.data,
                received_ns=received_ns,
            )
            self.slam_received_ns = received_ns

        def on_platform_health(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("platform status is not an object")
                schema = str(payload.get("schema_version", ""))
                if schema != "focus-water-cmd-bridge-v1":
                    raise ValueError(f"unsupported platform schema {schema!r}")
                if live and payload.get("live") is not True:
                    raise ValueError("platform bridge is not live")
                self.platform_pass = payload.get("ready") is True
                water = payload.get("water")
                if not isinstance(water, dict):
                    water = {}
                self.platform_estop = bool(water.get("estop_engaged"))
                self.platform_detail = (
                    f"water_bridge_ready={self.platform_pass}; "
                    f"last_reason={payload.get('last_reason', '')}; "
                    f"error_code={water.get('error_code', '')}"
                )
                self.platform_received_ns = time.time_ns()
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self.platform_pass = False
                self.platform_detail = f"platform_status_rejected:{exc}"
                self.platform_received_ns = time.time_ns()

        def on_occupancy(self, message: OccupancyGrid) -> None:
            try:
                self.occupancy = occupancy_from_message(
                    message, expected_frame=args.tinynav_map_frame
                )
                self.occupancy_received_ns = time.time_ns()
            except ValueError as exc:
                emit("occupancy_rejected", error=str(exc))

        def on_nav_done(self, message: Bool) -> None:
            if message.data:
                self.nav_done = True

        def on_trajectory(self, _message: RosPath) -> None:
            self.trajectory_received_ns = time.time_ns()

        def on_router_status(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                state = str(payload["state"])
                reason = str(payload["reason"])
                if state not in {"HOLD", "ACCEPTED", "NAVIGATING", "ARRIVED"}:
                    raise ValueError("unknown router state")
                decision_id = payload.get("decision_id")
                affected_decision_id = payload.get("affected_decision_id")
                self.router_state = state
                self.router_reason = reason
                self.router_decision_id = (
                    None if decision_id is None else str(decision_id)
                )
                self.router_affected_decision_id = (
                    None
                    if affected_decision_id is None
                    else str(affected_decision_id)
                )
                self.router_status_received_ns = time.time_ns()
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                emit("router_status_rejected", error=str(exc)[:300])

        def on_raw_cmd(self, message: Twist) -> None:
            self.raw_cmd_received_ns = time.time_ns()
            if live and self.authorized and time.time_ns() < self.authority_deadline_ns:
                self.guarded_publisher.publish(message)
            elif live:
                self.guarded_publisher.publish(Twist())

        def enforce_gate(self) -> None:
            if not live:
                return
            if self.authorized and time.time_ns() >= self.authority_deadline_ns:
                self.authorized = False
            if not self.authorized:
                self.guarded_publisher.publish(Twist())

        def authorize(self, expires_at_ns: int) -> None:
            if live:
                self.authority_deadline_ns = expires_at_ns
                self.authorized = True

        def revoke(self, *, pause: bool = True) -> bool:
            self.authorized = False
            self.authority_deadline_ns = 0
            if live:
                self.guarded_publisher.publish(Twist())
                if pause:
                    paused = Bool()
                    paused.data = True
                    self.pause_publisher.publish(paused)
            return True

        def publish_goal(self, payload: str, expires_at_ns: int) -> None:
            if not live:
                raise RuntimeError("live TinyNav output is disabled")
            self.nav_done = False
            paused = Bool()
            paused.data = False
            self.pause_publisher.publish(paused)
            message = String()
            message.data = payload
            self.poi_publisher.publish(message)
            self.authorize(expires_at_ns)

        def tracking_T_map(self) -> tuple[float, ...]:
            if args.online_buildmap_world:
                return (
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                )
            transform = self.tf_buffer.lookup_transform(
                args.tracking_frame, args.tinynav_map_frame, Time()
            )
            return transform_message_matrix(transform)

        def planner_graph_ready(self) -> tuple[bool, str]:
            poi_subscribers = self.get_subscriptions_info_by_topic(args.cmd_pois_topic)
            poi_publishers = self.get_publishers_info_by_topic(args.cmd_pois_topic)
            raw_publishers = self.get_publishers_info_by_topic(args.raw_cmd_topic)
            raw_subscribers = self.get_subscriptions_info_by_topic(args.raw_cmd_topic)
            guarded_subscribers = self.get_subscriptions_info_by_topic(
                args.guarded_cmd_topic
            )
            router_status_publishers = self.get_publishers_info_by_topic(
                args.router_status_topic
            )
            occupancy_publishers = self.get_publishers_info_by_topic(
                args.occupancy_topic
            )
            unexpected_raw = [
                endpoint
                for endpoint in raw_subscribers
                if endpoint.node_name != self.get_name()
            ]
            unexpected_poi = [
                endpoint
                for endpoint in poi_publishers
                if endpoint.node_name != self.get_name()
            ]
            checks = {
                "tiny_nav_poi_subscriber": bool(poi_subscribers),
                "poi_has_no_bypass_publisher": not unexpected_poi,
                "tiny_nav_cmd_publisher": bool(raw_publishers),
                "raw_cmd_has_no_direct_bridge": not unexpected_raw,
                "guarded_bridge_subscriber": bool(guarded_subscribers) if live else True,
                "occupancy_publisher": bool(occupancy_publishers),
                "online_router_status_publisher": (
                    bool(router_status_publishers)
                    if args.online_buildmap_world
                    else True
                ),
            }
            return all(checks.values()), json.dumps(checks, sort_keys=True)

    rclpy.init()
    node = WsjReceiverNode()
    # The receiver's command/HTTP loop runs at 2 Hz, while TinyNav odometry and
    # the local zero-velocity gate run at 10-20 Hz.  Calling spin_once only once
    # per command cycle starves ROS callbacks and can falsely age healthy local
    # data into LOST.  Keep all ROS callbacks on one dedicated executor thread;
    # the main thread still owns every high-level decision and authority change.
    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(node)
    ros_spin_thread = threading.Thread(
        target=ros_executor.spin,
        name="focus-v2-wsj-ros",
        daemon=True,
    )
    ros_spin_thread.start()
    hub = HubV2RobotClient(args.base_url, args.robot_id, token)

    tracking_T_map: tuple[float, ...] | None = None
    alignment_deadline = time.monotonic() + 30.0
    while time.monotonic() < alignment_deadline:
        time.sleep(0.05)
        if node.world_T_camera is None:
            continue
        if not args.online_buildmap_world and (
            node.occupancy is None or not node.slam_pass
        ):
            continue
        try:
            tracking_T_map = node.tracking_T_map()
            break
        except Exception:  # noqa: BLE001 - TF can be unavailable until relocalization
            continue
    if tracking_T_map is None or node.world_T_camera is None:
        emit("startup_failed", reason="tinynav_map_alignment_not_available")
        ros_executor.shutdown(timeout_sec=2.0)
        ros_spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        return 3

    shared_T_robot_map = derive_shared_T_map_from_tracking_map(
        shared_T_tracking=calibration.shared_T_tracking,
        tracking_T_map=tracking_T_map,
    )
    if node.occupancy is None:
        occupancy_provenance = {
            "topic": args.occupancy_topic,
            "status": "unverified_not_yet_observed",
            "detail": (
                "online world-frame alignment is source-derived identity; "
                "runtime health remains HOLD until a fresh occupancy grid arrives"
            ),
        }
    else:
        occupancy_provenance = {
            "topic": args.occupancy_topic,
            "width": node.occupancy.width,
            "height": node.occupancy.height,
            "resolution_m": node.occupancy.resolution_m,
            "payload_sha256": hashlib.sha256(
                bytes((value + 1) & 0xFF for value in node.occupancy.data)
            ).hexdigest(),
            "status": "observed",
        }
    artifact = alignment_artifact(
        calibration=calibration,
        local_map_frame=args.local_map_frame,
        shared_T_robot_map=shared_T_robot_map,
        captured_at_ns=time.time_ns(),
        sample_skew_ns=0,
        max_sample_skew_ns=0,
        observed_inputs={
            "tinynav_tf": {
                "lookup": f"{args.tracking_frame}_T_{args.tinynav_map_frame}",
                "matrix": list(tracking_T_map),
                "status": (
                    "source_derived_session_local_identity"
                    if args.online_buildmap_world
                    else "observed_latest_relocalization_transform"
                ),
            },
            "tinynav_odometry": {
                "topic": args.odom_topic,
                "received_at_ns": node.odom_received_ns,
                "tracking_T_camera": list(node.world_T_camera),
                "status": "observed",
            },
            "base_camera_calibration": {
                "source_path": base_camera_calibration.source_path,
                "source_size_bytes": (
                    base_camera_calibration.source_size_bytes
                ),
                "source_sha256": base_camera_calibration.source_sha256,
                "measurement_status": (
                    base_camera_calibration.measurement_status
                ),
                "base_T_camera": list(base_camera_calibration.matrix),
                "status": "observed_measured_artifact",
            },
            "tinynav_occupancy": occupancy_provenance,
        },
    )
    atomic_write_json(alignment_output, artifact)
    emit(
        "alignment_ready",
        output=str(alignment_output),
        shared_T_robot_map=list(shared_T_robot_map),
        live=live,
    )

    adapter = V2GoalAdapter(
        V2GoalAdapterConfig(
            robot_id=args.robot_id,
            transform_version=args.transform_version,
            shared_frame_calibration_id=args.shared_frame_calibration_id,
            shared_T_robot_map=shared_T_robot_map,
            output_kind="tinynav_poi",
            local_frame_id=args.local_map_frame,
            max_goal_distance_m=args.max_goal_distance_m,
            allow_unreachable_semantic_projection=args.online_buildmap_world,
        )
    )
    path = PathAccumulator()
    path_episode_id: str | None = None
    last_decision_id: str | None = None
    active_decision = None
    active_goal = None
    last_feedback_monotonic = 0.0
    goal_issued_ns = 0

    def current_pose() -> tuple[float, float, float]:
        if node.world_T_camera is None:
            raise RuntimeError("TinyNav odometry is unavailable")
        return robot_map_base_pose(
            tracking_T_map=node.tracking_T_map(),
            tracking_T_camera=node.world_T_camera,
            base_T_camera=base_camera_calibration.matrix,
        )

    def post(decision, status, reason_code, pose, *, zero=False, goal=None,
             detail="", terminal=False) -> bool:
        event = navigation_event(
            decision,
            status=status,
            reason_code=reason_code,
            local_pose=pose,
            episode_start_pose=(
                None
                if path.first_xy is None
                else (path.first_xy[0], path.first_xy[1], pose[2])
            ),
            path_length_m=path.length_m,
            velocity_zero_confirmed=zero,
            local_goal=goal,
            detail=detail,
            terminal=terminal,
            adapter_name="tinynav-occupancy-region-v1",
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

    exit_code = 0
    try:
        while rclpy.ok():
            cycle_started = time.monotonic()
            try:
                pose = current_pose()
                current_tracking_T_map = node.tracking_T_map()
            except Exception as exc:  # noqa: BLE001
                if active_decision is not None:
                    node.revoke()
                    active_decision = None
                    active_goal = None
                emit("localization_failed_local_hold", error=str(exc)[:500])
                time.sleep(args.poll_s)
                continue
            path.update(pose[0], pose[1])
            now_ns = time.time_ns()
            alignment_shift, alignment_yaw = planar_transform_delta(
                tracking_T_map, current_tracking_T_map
            )
            alignment_stable = (
                alignment_shift <= args.max_alignment_shift_m
                and math.degrees(alignment_yaw) <= args.max_alignment_yaw_deg
            )
            local_fresh = (
                now_ns - node.odom_received_ns
                <= int(args.local_data_timeout_s * 1e9)
                and now_ns - node.slam_received_ns
                <= int(args.local_data_timeout_s * 1e9)
            )
            platform_fresh = (
                not args.platform_health_topic
                or (
                    node.platform_received_ns > 0
                    and now_ns - node.platform_received_ns
                    <= int(args.local_data_timeout_s * 1e9)
                )
            )
            graph_ready, graph_detail = node.planner_graph_ready()
            ready = (
                local_fresh
                and node.slam_pass
                and alignment_stable
                and graph_ready
                and node.occupancy is not None
                and platform_fresh
                and node.platform_pass
            )
            health = RobotHealth(
                safety_state=SafetyState.READY if ready else SafetyState.HOLD,
                localization_state=(
                    LocalizationState.TRACKING
                    if local_fresh and node.slam_pass and alignment_stable
                    else LocalizationState.LOST
                ),
                estop_engaged=node.platform_estop,
                collision_avoidance_ready=bool(node.occupancy is not None and graph_ready),
                motor_controller_ready=bool(
                    graph_ready and platform_fresh and node.platform_pass
                ),
                detail=(
                    f"{node.slam_detail}; alignment_shift={alignment_shift:.3f}m; "
                    f"alignment_yaw={math.degrees(alignment_yaw):.2f}deg; {graph_detail}; "
                    f"{node.platform_detail}; "
                    + (
                        "Go2 handheld remote retains final local priority"
                        if args.robot_id == "robot-0"
                        else "WATER local status/watchdog retains final authority"
                    )
                ),
            )
            if live:
                try:
                    hub.post_heartbeat(health)
                except Exception as exc:  # noqa: BLE001 - lost health link revokes motion
                    if active_decision is not None:
                        node.revoke()
                        active_decision = None
                        active_goal = None
                    emit("heartbeat_failed_local_hold", error=str(exc)[:500])
                    time.sleep(args.poll_s)
                    continue
            if not ready and active_decision is not None:
                node.revoke()
                post(
                    active_decision,
                    NavigationStatusV2.REJECTED,
                    "HEALTH_NOT_READY",
                    pose,
                    zero=True,
                    detail=health.detail,
                    terminal=True,
                )
                active_decision = None
                active_goal = None
            if not alignment_stable and active_decision is not None:
                node.revoke()
                post(
                    active_decision,
                    NavigationStatusV2.REJECTED,
                    "TRANSFORM_MISMATCH",
                    pose,
                    detail=health.detail,
                    terminal=True,
                )
                active_decision = None
                active_goal = None
            if (
                args.online_buildmap_world
                and active_decision is not None
                and node.router_status_received_ns >= goal_issued_ns
                and node.router_state == "HOLD"
                and (
                    node.router_decision_id == active_decision.decision_id
                    or node.router_affected_decision_id
                    == active_decision.decision_id
                    or (
                        node.router_decision_id is None
                        and node.router_affected_decision_id is None
                        and now_ns - goal_issued_ns > 1_000_000_000
                    )
                )
            ):
                held_decision = active_decision
                node.revoke()
                post(
                    held_decision,
                    NavigationStatusV2.REJECTED,
                    "LOCAL_ROUTER_HOLD",
                    pose,
                    zero=True,
                    detail=(
                        f"online router state={node.router_state} "
                        f"reason={node.router_reason}"
                    ),
                    terminal=True,
                )
                emit(
                    "online_router_local_hold",
                    state=node.router_state,
                    reason=node.router_reason,
                    decision_id=held_decision.decision_id,
                )
                active_decision = None
                active_goal = None

            try:
                decision = hub.latest_decision()
            except Exception as exc:  # noqa: BLE001 - disconnect revokes authority
                if active_decision is not None:
                    node.revoke()
                    active_decision = None
                    active_goal = None
                emit("hub_poll_failed_local_hold", error=str(exc)[:500])
                time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
                continue
            if decision is None:
                if active_decision is not None:
                    node.revoke()
                    post(
                        active_decision,
                        NavigationStatusV2.HOLDING,
                        "EXPIRED",
                        pose,
                        zero=True,
                        detail="Hub returned no effective decision",
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
                continue

            goal_published_this_cycle = False
            if decision.decision_id != last_decision_id:
                last_decision_id = decision.decision_id
                if path_episode_id != decision.episode_id:
                    path = PathAccumulator()
                    path.update(pose[0], pose[1])
                    path_episode_id = decision.episode_id
                if not post(
                    decision,
                    NavigationStatusV2.RECEIVED,
                    "DECISION_RECEIVED",
                    pose,
                    detail="authenticated v2 decision parsed locally",
                ):
                    node.revoke()
                    time.sleep(args.poll_s)
                    continue
                occupancy = node.occupancy
                clearance_cells = (
                    0
                    if occupancy is None
                    else math.ceil(
                        args.reachability_clearance_m
                        / occupancy.resolution_m
                    )
                )
                component = (
                    frozenset()
                    if occupancy is None
                    else occupancy.reachable_component(
                        pose[0],
                        pose[1],
                        clearance_cells=clearance_cells,
                        start_snap_radius_m=args.start_snap_radius_m,
                        start_footprint_override_m=(
                            args.start_footprint_override_m
                            if args.online_buildmap_world
                            else 0.0
                        ),
                    )
                )
                result = adapter.evaluate(
                    decision,
                    now_ns=time.time_ns(),
                    health=health,
                    current_position_robot_map=(pose[0], pose[1], 0.0),
                    is_local_goal_reachable=(
                        None
                        if occupancy is None
                        else lambda x, y: occupancy.point_in_component(x, y, component)
                    ),
                )
                if (
                    result.action == V2AdapterAction.GOAL
                    and result.local_goal is not None
                    and occupancy is not None
                    and not (
                        args.online_buildmap_world
                        and result.local_goal.target_kind
                        in {"FRONTIER_POINT", "SEMANTIC_REGION"}
                    )
                    and not (
                        occupancy.point_in_component(
                            result.local_goal.x,
                            result.local_goal.y,
                        )
                        or occupancy.component_within_radius(
                            result.local_goal.x,
                            result.local_goal.y,
                            result.local_goal.arrival_radius_m or 0.0,
                            component,
                        )
                    )
                ):
                    result = type(result)(
                        action=V2AdapterAction.HOLD,
                        reason_code="UNREACHABLE",
                        detail=(
                            "no reachable TinyNav free component from the "
                            "measured robot base"
                            if not component
                            else "goal is outside TinyNav's reachable free "
                            "component"
                        ),
                    )
                if (
                    result.action == V2AdapterAction.GOAL
                    and time.time_ns() >= decision.expires_at_ns
                ):
                    result = type(result)(
                        action=V2AdapterAction.HOLD,
                        reason_code="EXPIRED",
                        detail="decision expired during local occupancy checks",
                    )
                emit(
                    "decision_evaluated",
                    decision_id=decision.decision_id,
                    action=result.action.value,
                    reason_code=result.reason_code,
                    command_preview=result.command_preview,
                    live=live,
                    reachable_component_cells=len(component),
                    reachability_clearance_cells=clearance_cells,
                    start_snap_radius_m=args.start_snap_radius_m,
                    start_footprint_override_m=(
                        args.start_footprint_override_m
                        if args.online_buildmap_world
                        else 0.0
                    ),
                    online_frontier_projection_required=bool(
                        result.action == V2AdapterAction.GOAL
                        and result.local_goal is not None
                        and result.local_goal.target_kind == "FRONTIER_POINT"
                        and not occupancy.point_in_component(
                            result.local_goal.x,
                            result.local_goal.y,
                            component,
                        )
                    ),
                    online_semantic_projection_required=bool(
                        result.action == V2AdapterAction.GOAL
                        and result.local_goal is not None
                        and result.local_goal.target_kind == "SEMANTIC_REGION"
                        and not occupancy.component_within_radius(
                            result.local_goal.x,
                            result.local_goal.y,
                            result.local_goal.arrival_radius_m or 0.0,
                            component,
                        )
                    ),
                )
                if result.action == V2AdapterAction.GOAL:
                    if not live:
                        post(
                            decision,
                            NavigationStatusV2.REJECTED,
                            "UNSAFE",
                            pose,
                            detail=(
                                "live TinyNav output is disabled; "
                                "validation preview only"
                            ),
                        )
                    else:
                        same_leg = (
                            active_decision is not None
                            and active_decision.leg_id == decision.leg_id
                        )
                        if active_decision is not None and not same_leg:
                            node.revoke()
                        accepted = post(
                            decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_GOAL_ACCEPTED",
                            pose,
                            goal=result.local_goal,
                            detail=result.detail,
                        )
                        if not accepted:
                            node.revoke()
                            time.sleep(args.poll_s)
                            continue
                        if time.time_ns() >= decision.expires_at_ns:
                            node.revoke()
                            post(
                                decision,
                                NavigationStatusV2.REJECTED,
                                "EXPIRED",
                                pose,
                                detail="lease expired before TinyNav POI publication",
                            )
                            time.sleep(args.poll_s)
                            continue
                        node.publish_goal(
                            result.command_preview, decision.expires_at_ns
                        )
                        goal_published_this_cycle = True
                        goal_issued_ns = time.time_ns()
                        if not same_leg:
                            emit(
                                "tinynav_poi_published",
                                decision_id=decision.decision_id,
                                topic=args.cmd_pois_topic,
                            )
                        else:
                            emit(
                                "tinynav_poi_lease_renewed",
                                decision_id=decision.decision_id,
                                topic=args.cmd_pois_topic,
                            )
                        active_decision = decision
                        active_goal = result.local_goal
                        last_feedback_monotonic = 0.0
                elif result.action == V2AdapterAction.STOP:
                    node.revoke()
                    post(
                        decision,
                        NavigationStatusV2.STOPPED,
                        "LOCAL_STOP_LATCHED",
                        pose,
                        zero=True,
                        detail=result.detail,
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                elif decision.mode.value == "HOLD":
                    node.revoke()
                    post(
                        decision,
                        NavigationStatusV2.HOLDING,
                        "HUB_HOLD",
                        pose,
                        zero=True,
                        detail=result.detail,
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                else:
                    node.revoke()
                    post(
                        decision,
                        NavigationStatusV2.REJECTED,
                        result.reason_code,
                        pose,
                        detail=result.detail,
                    )
                    active_decision = None
                    active_goal = None

            if active_decision is not None:
                if time.time_ns() >= active_decision.expires_at_ns:
                    node.revoke()
                    post(
                        active_decision,
                        NavigationStatusV2.HOLDING,
                        "EXPIRED",
                        pose,
                        zero=True,
                        detail="local lease timer expired",
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                elif node.nav_done:
                    node.nav_done = False
                    node.revoke()
                    post(
                        active_decision,
                        NavigationStatusV2.ARRIVED,
                        "LOCAL_PLANNER_ARRIVED",
                        pose,
                        zero=True,
                        detail="TinyNav /mapping/nav_done reported true",
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                elif not goal_published_this_cycle and (
                    time.monotonic() - last_feedback_monotonic >= 0.5
                ):
                    planner_active = (
                        node.trajectory_received_ns >= goal_issued_ns
                        or node.raw_cmd_received_ns >= goal_issued_ns
                    )
                    post(
                        active_decision,
                        (
                            NavigationStatusV2.NAVIGATING
                            if planner_active
                            else NavigationStatusV2.ACCEPTED
                        ),
                        (
                            "LOCAL_PLANNER_ACTIVE"
                            if planner_active
                            else "LOCAL_GOAL_ACCEPTED"
                        ),
                        pose,
                        goal=active_goal,
                        detail=(
                            "TinyNav trajectory/cmd_vel observed"
                            if planner_active
                            else "waiting for first TinyNav trajectory"
                        ),
                    )
                    last_feedback_monotonic = time.monotonic()
            time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
    except KeyboardInterrupt:
        node.revoke()
        emit("receiver_stopped", reason="operator_interrupt")
    except Exception as exc:  # noqa: BLE001 - any receiver fault revokes motion
        exit_code = 4
        node.revoke()
        emit("receiver_fault", error=str(exc)[:1000])
    finally:
        ros_executor.shutdown(timeout_sec=2.0)
        ros_spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
