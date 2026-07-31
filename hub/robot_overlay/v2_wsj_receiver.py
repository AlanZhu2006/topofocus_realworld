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
    HubHeartbeatPump,
    HubV2RobotClient,
    OccupancyGrid2D,
    PathAccumulator,
    bind_path_to_episode,
    cached_map_valid_for_pose,
    navigation_event,
)


LIVE_CONFIRMATION = "OPERATOR_PRESENT_AND_WSJ_CLEAR"
LIVE_CONFIRMATIONS = {
    "robot-0": LIVE_CONFIRMATION,
    "robot-1": "OPERATOR_PRESENT_AND_YUNJI_CLEAR",
}
TARGET_REFRESH_REQUEST_SCHEMA_VERSION = (
    "focus-tinynav-target-refresh-request-v1"
)
PLANNER_CANDIDATE_STATUS_SCHEMA_VERSION = (
    "focus-tinynav-candidate-status-v1"
)
# Mirror the independently enforced sender thresholds exactly.  The receiver
# still recomputes every interval check instead of trusting the producer's
# ``imu_intervals_valid`` boolean, but must not reject telemetry that the
# deployment sender has already classified with a different numeric policy.
SLAM_IMU_MIN_COVERAGE_RATIO = 0.80
SLAM_IMU_MAX_SAMPLE_GAP_S = 0.05
SLAM_IMU_END_TOLERANCE_S = 0.01
SLAM_OVERWRITE_RECOVERY_MIN_REPORTS = 3
SLAM_OVERWRITE_RECOVERY_MIN_S = 2.0
TRANSIENT_SLAM_FAILURES = frozenset(
    {
        # The bounded perception adapter freezes its last accepted pose,
        # zeroes its internal velocity, and publishes no pose for this frame.
        # Treat only one such report after a recent good report like the
        # equivalent interval-level diagnostic blips below.
        "optimizer_status=skipped_imu_invalid",
        "imu_intervals_invalid",
        "imu_intervals_missing",
        "imu_interval_invalid",
        "imu_interval_threshold",
    }
)
EXTERNAL_ODOMETRY_MAX_POS_VAR_M2 = 0.01
EXTERNAL_ODOMETRY_MAX_YAW_VAR_RAD2 = 0.01
ROBOT_HEALTH_DETAIL_MAX_LENGTH = 512
RECOVERABLE_ROUTER_HOLD_REASONS = frozenset(
    {
        "ODOMETRY_STALE",
        "OCCUPANCY_STALE_AFTER_MOTION",
    }
)
REPLANNABLE_TARGET_KINDS = frozenset(
    {
        "FRONTIER_POINT",
        "SEMANTIC_REGION",
    }
)
RECOVERY_RENEWAL_REJECTED_EVENTS = {
    "occupancy": "occupancy_recovery_renewal_rejected",
    "slam": "slam_recovery_renewal_rejected",
    "odometry": "odometry_recovery_renewal_rejected",
    "combined": "combined_recovery_renewal_rejected",
    "heartbeat": "heartbeat_delivery_recovery_renewal_rejected",
}
RECOVERY_RENEWAL_FEEDBACK_FAILED_EVENTS = {
    "occupancy": "occupancy_recovery_renewal_feedback_failed",
    "slam": "slam_recovery_renewal_feedback_failed",
    "odometry": "odometry_recovery_renewal_feedback_failed",
    "combined": "combined_recovery_renewal_feedback_failed",
    "heartbeat": "heartbeat_delivery_recovery_renewal_feedback_failed",
}
RECOVERY_LEASE_RENEWED_EVENTS = {
    "occupancy": "occupancy_recovery_lease_renewed",
    "slam": "slam_recovery_lease_renewed",
    "odometry": "odometry_recovery_lease_renewed",
    "combined": "combined_recovery_lease_renewed",
    "heartbeat": "heartbeat_delivery_recovery_lease_renewed",
}
DEFAULT_CONTROLLER_PAUSE_SERVICE = "/focus/set_navigation_paused"
DEFAULT_CONTROLLER_PAUSE_ACK_TIMEOUT_S = 1.0
DEFAULT_CONTROLLER_PAUSE_RETRY_S = 0.10
DEFAULT_CONTROLLER_PAUSE_STARTUP_TIMEOUT_S = 15.0


def recoverable_router_hold(
    reason: str, *, receiver_runtime_ready: bool
) -> bool:
    """Retry only transient router input lag while the receiver stays READY."""

    return (
        receiver_runtime_ready
        and reason in RECOVERABLE_ROUTER_HOLD_REASONS
    )


def no_known_free_path_requires_replan(
    target_kind: str | None,
    router_reason: str,
) -> bool:
    """Translate a measured A* no-path result into a fresh source round.

    Both protocol target kinds are high-level goals.  TinyNav has final
    authority to reject either one when its current known-free map contains no
    route.  Treating the same router result as recoverable for a frontier but
    terminal for a semantic region made the behavior depend on target
    encoding rather than physical feasibility.
    """

    return bool(
        target_kind in REPLANNABLE_TARGET_KINDS
        and router_reason == "NO_KNOWN_FREE_PATH"
    )


def router_hold_recovery_eligible(
    target_kind: str | None,
    router_reason: str,
    *,
    receiver_runtime_ready: bool,
) -> bool:
    """Keep a zero-velocity leg while fresh local-map evidence can recover it.

    The online occupancy grid updates more slowly than the router replans, so
    the first ``NO_KNOWN_FREE_PATH`` may describe an incomplete known-free
    component rather than a physically impossible route. Preserve the same
    bounded recovery window used for transient router input lag, but only
    while every independent receiver health gate remains READY.
    """

    return bool(
        recoverable_router_hold(
            router_reason,
            receiver_runtime_ready=receiver_runtime_ready,
        )
        or (
            receiver_runtime_ready
            and no_known_free_path_requires_replan(
                target_kind,
                router_reason,
            )
        )
    )


def occupancy_recovery_eligible(
    *,
    recovery_elapsed_s: float,
    recovery_grace_s: float,
    all_other_health_ready: bool,
    occupancy_observed: bool,
) -> bool:
    """Allow a bounded zero-velocity wait after the occupancy gate closes.

    The independent 20 Hz physical gate has already closed because neither a
    fresh grid nor the bounded cached-map displacement contract is valid.
    This helper controls only whether that local HOLD is immediately promoted
    to a terminal episode rejection.
    """

    if not math.isfinite(recovery_grace_s) or recovery_grace_s <= 0.0:
        raise ValueError("occupancy recovery bound must be positive")
    return bool(
        all_other_health_ready
        and occupancy_observed
        and math.isfinite(recovery_elapsed_s)
        and 0.0 <= recovery_elapsed_s <= recovery_grace_s
    )


def heartbeat_delivery_recovery_eligible(
    *,
    recovery_elapsed_s: float,
    recovery_grace_s: float,
    sensor_ready: bool,
    heartbeat_delivery_ready: bool,
) -> bool:
    """Bound a stopped wait for one transient Hub-heartbeat delivery gap.

    The independent physical velocity gate is already closed whenever heartbeat
    delivery is stale.  This helper controls only whether an already accepted
    immutable leg may remain non-terminal while every robot-local sensor,
    collision, platform and graph check stays READY.
    """

    if not math.isfinite(recovery_grace_s) or recovery_grace_s <= 0.0:
        raise ValueError("heartbeat delivery recovery bound must be positive")
    return bool(
        sensor_ready
        and not heartbeat_delivery_ready
        and math.isfinite(recovery_elapsed_s)
        and 0.0 <= recovery_elapsed_s <= recovery_grace_s
    )


def heartbeat_delivery_recovery_renewal_health(
    health: RobotHealth,
) -> RobotHealth:
    """Validate a same-leg lease without reopening the heartbeat gate."""

    return health.model_copy(update={"safety_state": SafetyState.READY})


def occupancy_recovery_renewal_health(health: RobotHealth) -> RobotHealth:
    """Validate one same-leg renewal without reopening the velocity gate.

    ``occupancy_recovery_eligible`` has already proved that localization,
    platform authority and the command graph remain healthy; only occupancy
    freshness is holding motion at zero.  The adapter still needs a
    goal-capable health value to validate lease ordering, target identity and
    provenance.  Override only the two occupancy-derived fields here, leaving
    estop, localization and motor readiness authoritative.
    """

    return health.model_copy(
        update={
            "safety_state": SafetyState.READY,
            "collision_avoidance_ready": True,
        }
    )


def slam_recovery_eligible(
    *,
    recovery_elapsed_s: float,
    recovery_grace_s: float,
    slam_detail: str,
    all_non_slam_health_ready: bool,
) -> bool:
    """Keep one transient SLAM incident as a bounded zero-velocity wait."""

    if not math.isfinite(recovery_grace_s) or recovery_grace_s <= 0.0:
        raise ValueError("SLAM recovery bound must be positive")
    return bool(
        all_non_slam_health_ready
        and slam_detail in TRANSIENT_SLAM_FAILURES
        and math.isfinite(recovery_elapsed_s)
        and 0.0 <= recovery_elapsed_s <= recovery_grace_s
    )


def slam_recovery_renewal_health(health: RobotHealth) -> RobotHealth:
    """Validate a same-leg lease without reopening the physical gate."""

    return health.model_copy(
        update={
            "safety_state": SafetyState.READY,
            "localization_state": LocalizationState.TRACKING,
        }
    )


def odometry_recovery_eligible(
    *,
    recovery_elapsed_s: float,
    recovery_grace_s: float,
    all_non_odometry_health_ready: bool,
    odometry_observed: bool,
) -> bool:
    """Keep a transient odometry publication gap as a bounded stopped wait."""

    if not math.isfinite(recovery_grace_s) or recovery_grace_s <= 0.0:
        raise ValueError("odometry recovery bound must be positive")
    return bool(
        all_non_odometry_health_ready
        and odometry_observed
        and math.isfinite(recovery_elapsed_s)
        and 0.0 <= recovery_elapsed_s <= recovery_grace_s
    )


def odometry_recovery_renewal_health(health: RobotHealth) -> RobotHealth:
    """Validate the immutable leg without reopening the physical gate."""

    return health.model_copy(
        update={
            "safety_state": SafetyState.READY,
            "localization_state": LocalizationState.TRACKING,
        }
    )


def odometry_slam_recovery_eligible(
    *,
    odometry_recovery_elapsed_s: float,
    odometry_recovery_grace_s: float,
    slam_recovery_elapsed_s: float,
    slam_recovery_grace_s: float,
    slam_detail: str,
    all_non_odometry_slam_health_ready: bool,
    odometry_observed: bool,
) -> bool:
    """Bound a shared-upstream odometry and transient-SLAM stopped wait.

    The control-odometry publication gap keeps the physical velocity gate
    closed.  A simultaneous exact transient SLAM diagnostic gets its own
    shorter timer, and both original timers must remain valid.  This prevents
    a handoff between the two recovery states from extending either budget.
    """

    if (
        not math.isfinite(odometry_recovery_grace_s)
        or odometry_recovery_grace_s <= 0.0
    ):
        raise ValueError("odometry recovery bound must be positive")
    if (
        not math.isfinite(slam_recovery_grace_s)
        or slam_recovery_grace_s <= 0.0
    ):
        raise ValueError("SLAM recovery bound must be positive")
    return bool(
        all_non_odometry_slam_health_ready
        and odometry_observed
        and slam_detail in TRANSIENT_SLAM_FAILURES
        and math.isfinite(odometry_recovery_elapsed_s)
        and 0.0
        <= odometry_recovery_elapsed_s
        <= odometry_recovery_grace_s
        and math.isfinite(slam_recovery_elapsed_s)
        and 0.0 <= slam_recovery_elapsed_s <= slam_recovery_grace_s
    )


def inherited_recovery_start_ns(
    *,
    now_ns: int,
    active_leg_id: str,
    current_started_ns: int,
    current_leg_id: str | None,
    handoff_started_ns: int,
    handoff_leg_id: str | None,
) -> int:
    """Preserve one sensor incident's timer across a same-leg handoff."""

    if current_started_ns > 0 and current_leg_id == active_leg_id:
        return current_started_ns
    if handoff_started_ns > 0 and handoff_leg_id == active_leg_id:
        return handoff_started_ns
    return now_ns


def combined_sensor_recovery_eligible(
    *,
    occupancy_recovery_elapsed_s: float,
    occupancy_recovery_grace_s: float,
    slam_recovery_elapsed_s: float,
    slam_recovery_grace_s: float,
    slam_detail: str,
    all_non_sensor_health_ready: bool,
    occupancy_observed: bool,
) -> bool:
    """Bound a simultaneous occupancy and transient-SLAM zero-velocity wait.

    Each sensor retains its own original timer, so entering this combined
    state cannot extend either single-sensor recovery budget.  Motion remains
    closed by the independent 20 Hz gate throughout.
    """

    if (
        not math.isfinite(occupancy_recovery_grace_s)
        or occupancy_recovery_grace_s <= 0.0
    ):
        raise ValueError("occupancy recovery bound must be positive")
    if (
        not math.isfinite(slam_recovery_grace_s)
        or slam_recovery_grace_s <= 0.0
    ):
        raise ValueError("SLAM recovery bound must be positive")
    return bool(
        all_non_sensor_health_ready
        and occupancy_observed
        and slam_detail in TRANSIENT_SLAM_FAILURES
        and math.isfinite(occupancy_recovery_elapsed_s)
        and 0.0
        <= occupancy_recovery_elapsed_s
        <= occupancy_recovery_grace_s
        and math.isfinite(slam_recovery_elapsed_s)
        and 0.0 <= slam_recovery_elapsed_s <= slam_recovery_grace_s
    )


def combined_sensor_recovery_renewal_health(
    health: RobotHealth,
) -> RobotHealth:
    """Validate the immutable leg while both sensor gates remain closed."""

    return health.model_copy(
        update={
            "safety_state": SafetyState.READY,
            "localization_state": LocalizationState.TRACKING,
            "collision_avoidance_ready": True,
        }
    )


def closed_gate_recovery_kind(
    *,
    occupancy_recovery_active: bool,
    slam_recovery_active: bool,
    odometry_recovery_active: bool,
    combined_sensor_recovery_active: bool,
) -> str | None:
    """Select exactly one fail-stopped recovery owner for the active leg."""

    active_count = sum(
        (
            occupancy_recovery_active,
            slam_recovery_active,
            odometry_recovery_active,
            combined_sensor_recovery_active,
        )
    )
    if active_count > 1:
        raise RuntimeError(
            "closed-gate recovery predicates must be mutually exclusive"
        )
    if combined_sensor_recovery_active:
        return "combined"
    if occupancy_recovery_active:
        return "occupancy"
    if slam_recovery_active:
        return "slam"
    if odometry_recovery_active:
        return "odometry"
    return None


class GoalProgressWatchdog:
    """Bound how long one fixed local goal may make no metric progress."""

    def __init__(
        self,
        *,
        timeout_s: float,
        minimum_improvement_m: float,
    ) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        if (
            not math.isfinite(minimum_improvement_m)
            or minimum_improvement_m <= 0.0
        ):
            raise ValueError(
                "minimum_improvement_m must be finite and positive"
            )
        self.timeout_s = timeout_s
        self.minimum_improvement_m = minimum_improvement_m
        self.reset()

    def reset(self) -> None:
        self.leg_id: str | None = None
        self.best_remaining_m = math.inf
        self.last_progress_monotonic = 0.0
        self.motion_anchor_xy: tuple[float, float] | None = None

    def observe(
        self,
        *,
        leg_id: str,
        remaining_m: float,
        now_monotonic: float,
        position_xy: tuple[float, float] | None = None,
    ) -> tuple[bool, float]:
        if not leg_id:
            raise ValueError("leg_id is required")
        if not math.isfinite(remaining_m) or remaining_m < 0.0:
            raise ValueError("remaining_m must be finite and non-negative")
        if not math.isfinite(now_monotonic):
            raise ValueError("now_monotonic must be finite")
        if position_xy is not None and (
            len(position_xy) != 2
            or not all(math.isfinite(value) for value in position_xy)
        ):
            raise ValueError("position_xy must contain two finite values")
        if self.leg_id != leg_id:
            self.leg_id = leg_id
            self.best_remaining_m = remaining_m
            self.last_progress_monotonic = now_monotonic
            self.motion_anchor_xy = position_xy
            return False, 0.0
        metric_progress = (
            remaining_m
            <= self.best_remaining_m - self.minimum_improvement_m
        )
        motion_progress = False
        if position_xy is not None:
            if self.motion_anchor_xy is None:
                self.motion_anchor_xy = position_xy
            else:
                motion_progress = (
                    math.hypot(
                        position_xy[0] - self.motion_anchor_xy[0],
                        position_xy[1] - self.motion_anchor_xy[1],
                    )
                    >= self.minimum_improvement_m
                )
        if metric_progress or motion_progress:
            if metric_progress:
                self.best_remaining_m = remaining_m
            if position_xy is not None:
                self.motion_anchor_xy = position_xy
            self.last_progress_monotonic = now_monotonic
            return False, 0.0
        stalled_s = max(
            0.0, now_monotonic - self.last_progress_monotonic
        )
        return stalled_s >= self.timeout_s, stalled_s


def bounded_protocol_detail(
    value: str, *, max_length: int = ROBOT_HEALTH_DETAIL_MAX_LENGTH
) -> str:
    """Bound diagnostic text without dropping either end of its provenance.

    ``RobotHealth.detail`` is descriptive only; all safety decisions are also
    carried by structured fields.  Runtime graph and platform diagnostics can
    nevertheless exceed the transport model's 512-character limit.  Preserve
    the leading localization evidence and trailing platform-authority evidence,
    with a digest of the complete source string between them.
    """

    if max_length <= 0:
        raise ValueError("max_length must be positive")
    if len(value) <= max_length:
        return value
    digest = hashlib.sha256(value.encode("utf-8")).hexdigest()[:16]
    marker = f"; truncated_sha256={digest}; "
    if len(marker) >= max_length:
        return marker[:max_length]
    remaining = max_length - len(marker)
    head_length = (remaining + 1) // 2
    tail_length = remaining - head_length
    return value[:head_length] + marker + value[-tail_length:]


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


def trajectory_message_summary(
    message: Any, *, expected_frame: str
) -> tuple[int, tuple[float, float], tuple[float, float]]:
    """Validate a complete local trajectory before refreshing its lease."""

    frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
    if frame_id != expected_frame:
        raise ValueError(
            f"trajectory frame {frame_id!r} != {expected_frame!r}"
        )
    poses = getattr(message, "poses", None)
    if poses is None or len(poses) < 2:
        raise ValueError("trajectory must contain at least two poses")
    for index, pose_stamped in enumerate(poses):
        try:
            quaternion_pose_matrix(pose_stamped.pose)
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(
                f"trajectory pose {index} is invalid: {exc}"
            ) from exc
    first_pose = poses[0].pose
    first = first_pose.position
    first_orientation = first_pose.orientation
    first_xy = (float(first.x), float(first.y))
    first_xyz = (*first_xy, float(first.z))
    first_quaternion = (
        float(first_orientation.x),
        float(first_orientation.y),
        float(first_orientation.z),
        float(first_orientation.w),
    )
    first_quaternion_norm = math.sqrt(
        sum(value * value for value in first_quaternion)
    )
    normalized_first_quaternion = tuple(
        value / first_quaternion_norm for value in first_quaternion
    )
    lookahead_xy = None
    for pose_stamped in poses[1:]:
        candidate_pose = pose_stamped.pose
        position = candidate_pose.position
        orientation = candidate_pose.orientation
        candidate = (float(position.x), float(position.y))
        candidate_xyz = (*candidate, float(position.z))
        quaternion = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        quaternion_norm = math.sqrt(
            sum(value * value for value in quaternion)
        )
        normalized_quaternion = tuple(
            value / quaternion_norm for value in quaternion
        )
        quaternion_dot = min(
            1.0,
            abs(
                sum(
                    current * first_value
                    for current, first_value in zip(
                        normalized_quaternion,
                        normalized_first_quaternion,
                    )
                )
            ),
        )
        rotation_distance = 2.0 * math.acos(quaternion_dot)
        translation_distance = math.sqrt(
            sum(
                (current - first_value) ** 2
                for current, first_value in zip(
                    candidate_xyz, first_xyz
                )
            )
        )
        if (
            translation_distance >= 0.01
            or rotation_distance >= math.radians(1.0)
        ):
            lookahead_xy = candidate
            break
    if lookahead_xy is None:
        raise ValueError(
            "trajectory must contain two geometrically distinct poses"
        )
    return (
        len(poses),
        first_xy,
        lookahead_xy,
    )


def twist_components_finite(message: Any) -> bool:
    """Validate all Twist axes at the final robot-local command boundary."""

    try:
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
    except (AttributeError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def transform_message_matrix(message: Any) -> tuple[float, ...]:
    transform = message.transform
    pose = type("Pose", (), {})()
    pose.position = transform.translation
    pose.orientation = transform.rotation
    return quaternion_pose_matrix(pose)


def slam_metrics_gate(
    raw_json: str, *, ignore_cumulative_overwrite: bool = False
) -> tuple[bool, str]:
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
        if (
            int(stats.get("imu_messages_overwritten", 0)) > 0
            and not ignore_cumulative_overwrite
        ):
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


def local_tracking_freshness(
    *,
    now_ns: int,
    odom_received_ns: int,
    slam_received_ns: int,
    odom_timeout_s: float,
    slam_timeout_s: float,
) -> tuple[bool, float, float]:
    """Check control odometry and slower SLAM diagnostics independently."""

    if odom_timeout_s <= 0.0 or slam_timeout_s <= 0.0:
        raise ValueError("local tracking timeouts must be positive")

    def age_s(received_ns: int) -> float:
        if received_ns <= 0:
            return math.inf
        return max(0.0, (now_ns - received_ns) / 1e9)

    odom_age_s = age_s(odom_received_ns)
    slam_age_s = age_s(slam_received_ns)
    return (
        odom_age_s <= odom_timeout_s and slam_age_s <= slam_timeout_s,
        odom_age_s,
        slam_age_s,
    )


def trajectory_gate_state(
    *,
    now_ns: int,
    authority_started_ns: int,
    trajectory_received_ns: int,
    stale_timeout_s: float,
    start_grace_s: float,
    recovery_timeout_s: float,
) -> tuple[bool, bool, float, bool]:
    """Separate immediate velocity gating from terminal planner failure.

    A trajectory older than ``stale_timeout_s`` must close the physical
    velocity gate.  A previously observed trajectory may nevertheless recover
    until ``recovery_timeout_s``; only then is the semantic leg terminally
    rejected.  A leg that never produced a path uses the independent,
    shorter ``start_grace_s`` deadline.
    """

    if min(stale_timeout_s, start_grace_s, recovery_timeout_s) <= 0.0:
        raise ValueError("trajectory timeouts must be positive")
    if recovery_timeout_s <= stale_timeout_s:
        raise ValueError(
            "trajectory recovery timeout must exceed stale timeout"
        )
    if authority_started_ns <= 0 or now_ns < authority_started_ns:
        return False, False, math.inf, False
    observed_for_authority = (
        trajectory_received_ns >= authority_started_ns
    )
    reference_ns = (
        trajectory_received_ns
        if observed_for_authority
        else authority_started_ns
    )
    age_s = max(0.0, (now_ns - reference_ns) / 1e9)
    gate_fresh = (
        observed_for_authority and age_s <= stale_timeout_s
    )
    terminal_failure = age_s > (
        recovery_timeout_s
        if observed_for_authority
        else start_grace_s
    )
    return gate_fresh, terminal_failure, age_s, observed_for_authority


def planner_target_refresh_eligible(
    *,
    authorized: bool,
    router_recovery_gate_closed: bool,
    trajectory_fresh: bool,
    trajectory_failed: bool,
    trajectory_age_s: float,
    trajectory_stale_timeout_s: float,
    router_state: str,
    router_reason: str,
    router_decision_id: str | None,
    active_decision_id: str,
    router_waypoint: tuple[float, float] | None,
    all_candidates_in_collision: bool = False,
) -> bool:
    """Allow only a same-leg, zero-velocity planner handoff repair."""

    refresh_required = all_candidates_in_collision or (
        not trajectory_fresh
        and trajectory_age_s >= trajectory_stale_timeout_s
    )
    if (
        not authorized
        or router_recovery_gate_closed
        or trajectory_failed
        or not refresh_required
        or router_state != "NAVIGATING"
        or router_decision_id != active_decision_id
        or router_waypoint is None
        or len(router_waypoint) != 2
        or not all(math.isfinite(float(value)) for value in router_waypoint)
    ):
        return False
    return router_reason.startswith(
        ("ONLINE_PATH_READY", "ONLINE_PARTIAL_PATH_READY")
    )


def parse_planner_candidate_status(raw_json: str) -> dict[str, object]:
    """Validate the local planner's collision-scored lattice status."""

    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise ValueError("planner candidate status is not JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("planner candidate status is not an object")
    if payload.get("schema_version") != PLANNER_CANDIDATE_STATUS_SCHEMA_VERSION:
        raise ValueError("planner candidate status schema is unsupported")
    all_collision = payload.get("all_candidates_in_collision")
    candidate_count = payload.get("candidate_count")
    finite_count = payload.get("finite_candidate_count")
    in_place_count = payload.get("finite_in_place_candidate_count")
    evaluated_at_ns = payload.get("evaluated_at_ns")
    if not isinstance(all_collision, bool):
        raise ValueError("planner candidate collision flag is malformed")
    for value, name in (
        (candidate_count, "candidate count"),
        (finite_count, "finite candidate count"),
        (in_place_count, "finite in-place candidate count"),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"planner {name} is malformed")
    if candidate_count <= 0 or finite_count > candidate_count:
        raise ValueError("planner candidate counts are inconsistent")
    if in_place_count > finite_count:
        raise ValueError("planner in-place candidate count is inconsistent")
    if (
        isinstance(evaluated_at_ns, bool)
        or not isinstance(evaluated_at_ns, int)
        or evaluated_at_ns <= 0
    ):
        raise ValueError("planner candidate evaluation time is malformed")
    if all_collision != (finite_count == 0):
        raise ValueError("planner collision flag differs from finite candidates")
    return {
        "all_candidates_in_collision": all_collision,
        "candidate_count": candidate_count,
        "finite_candidate_count": finite_count,
        "finite_in_place_candidate_count": in_place_count,
        "evaluated_at_ns": evaluated_at_ns,
        "robot_profile": payload.get("robot_profile"),
    }


def planner_collision_gate_state(
    *,
    now_ns: int,
    authority_started_ns: int,
    status_received_ns: int,
    collision_since_ns: int,
    all_candidates_in_collision: bool,
    status_timeout_s: float,
    rejection_timeout_s: float,
) -> tuple[bool, bool, float, bool]:
    """Close velocity immediately and reject only persistent fresh collision."""

    if status_timeout_s <= 0.0 or rejection_timeout_s <= 0.0:
        raise ValueError("planner collision timeouts must be positive")
    observed_for_authority = bool(
        authority_started_ns > 0
        and status_received_ns >= authority_started_ns
        and collision_since_ns >= authority_started_ns
    )
    if not observed_for_authority or not all_candidates_in_collision:
        return False, False, 0.0, observed_for_authority
    status_age_s = max(0.0, (now_ns - status_received_ns) / 1e9)
    collision_age_s = max(0.0, (now_ns - collision_since_ns) / 1e9)
    gate_closed = status_age_s <= status_timeout_s
    terminal = gate_closed and collision_age_s >= rejection_timeout_s
    return gate_closed, terminal, collision_age_s, True


def physical_velocity_gate_reason(
    *,
    now_ns: int,
    authorized: bool,
    authority_deadline_ns: int,
    trajectory_fresh: bool,
    reverse_required: bool,
    health_pass: bool,
    health_evaluated_ns: int,
    health_timeout_s: float,
    odom_received_ns: int,
    slam_received_ns: int,
    odom_timeout_s: float,
    slam_timeout_s: float,
    slam_pass: bool,
    occupancy_received_ns: int,
    occupancy_timeout_s: float,
    cached_occupancy_motion_valid: bool = False,
    platform_required: bool,
    platform_received_ns: int,
    platform_timeout_s: float,
    platform_pass: bool,
    router_recovery_gate_closed: bool = False,
    all_candidates_in_collision: bool = False,
    turn_stalled: bool = False,
) -> str | None:
    """Evaluate every final velocity-authority input at control rate."""

    if now_ns <= 0:
        raise ValueError("now_ns must be positive")
    if min(
        health_timeout_s,
        odom_timeout_s,
        slam_timeout_s,
        occupancy_timeout_s,
        platform_timeout_s,
    ) <= 0.0:
        raise ValueError("physical velocity gate timeouts must be positive")
    if not authorized:
        return "authority_closed"
    if authority_deadline_ns <= 0 or now_ns >= authority_deadline_ns:
        return "authority_expired"
    if router_recovery_gate_closed:
        return "router_recovery_gate_closed"
    if not health_pass:
        return "health_not_ready"
    if (
        health_evaluated_ns <= 0
        or now_ns - health_evaluated_ns > int(health_timeout_s * 1e9)
    ):
        return "health_evaluation_stale"
    local_fresh, _, _ = local_tracking_freshness(
        now_ns=now_ns,
        odom_received_ns=odom_received_ns,
        slam_received_ns=slam_received_ns,
        odom_timeout_s=odom_timeout_s,
        slam_timeout_s=slam_timeout_s,
    )
    if not local_fresh:
        return "local_tracking_stale"
    if not slam_pass:
        return "localization_not_tracking"
    if (
        occupancy_received_ns <= 0
        or (
            now_ns - occupancy_received_ns
            > int(occupancy_timeout_s * 1e9)
            and not cached_occupancy_motion_valid
        )
    ):
        return "occupancy_missing_or_stale"
    if platform_required:
        if (
            platform_received_ns <= 0
            or now_ns - platform_received_ns
            > int(platform_timeout_s * 1e9)
        ):
            return "platform_health_stale"
        if not platform_pass:
            return "platform_health_not_ready"
    if reverse_required:
        return "reverse_trajectory_rejected"
    if all_candidates_in_collision:
        return "all_trajectories_in_collision"
    if turn_stalled:
        return "turn_recovery_stalled"
    if not trajectory_fresh:
        return "trajectory_missing_or_stale"
    return None


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
        self.last_overwritten_count: int | None = None
        self.overwrite_stable_reports = 0
        self.overwrite_stable_since_ns = 0

    def update(self, raw_json: str, *, received_ns: int) -> tuple[bool, str]:
        passed, detail = slam_metrics_gate(raw_json)
        if detail == "imu_buffer_overwritten":
            current_valid, _ = slam_metrics_gate(
                raw_json, ignore_cumulative_overwrite=True
            )
            try:
                overwritten = int(
                    json.loads(raw_json)["stats"][
                        "imu_messages_overwritten"
                    ]
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                current_valid = False
                overwritten = -1
            if current_valid and overwritten >= 0:
                passed, detail = self._recover_cumulative_overwrite(
                    overwritten, received_ns=received_ns
                )
            else:
                self.overwrite_stable_reports = 0
                self.overwrite_stable_since_ns = 0
        elif passed:
            self.last_overwritten_count = 0
            self.overwrite_stable_reports = 0
            self.overwrite_stable_since_ns = 0
        else:
            self.overwrite_stable_reports = 0
            self.overwrite_stable_since_ns = 0
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

    def _recover_cumulative_overwrite(
        self, count: int, *, received_ns: int
    ) -> tuple[bool, str]:
        if self.last_overwritten_count != count:
            self.last_overwritten_count = count
            self.overwrite_stable_reports = 1
            self.overwrite_stable_since_ns = received_ns
            return False, f"imu_buffer_overwritten:{count}"
        self.overwrite_stable_reports += 1
        if self.overwrite_stable_since_ns <= 0:
            self.overwrite_stable_since_ns = received_ns
        stable_s = max(
            0.0, (received_ns - self.overwrite_stable_since_ns) / 1e9
        )
        if (
            self.overwrite_stable_reports
            >= SLAM_OVERWRITE_RECOVERY_MIN_REPORTS
            and stable_s >= SLAM_OVERWRITE_RECOVERY_MIN_S
        ):
            return (
                True,
                "slam_optimizer_imu_valid_after_overwrite_recovery:"
                f"{count}",
            )
        return (
            False,
            "imu_buffer_recovery:"
            f"{self.overwrite_stable_reports}/"
            f"{SLAM_OVERWRITE_RECOVERY_MIN_REPORTS}",
        )


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
    parser.add_argument(
        "--target-refresh-request-topic",
        default="/mapping/target_refresh_request",
        help=(
            "robot-local request channel used only while velocity is already "
            "zero to republish the verified router-to-planner target"
        ),
    )
    parser.add_argument(
        "--target-refresh-request-interval-s",
        type=float,
        default=1.0,
        help=(
            "minimum interval between bounded same-leg planner handoff "
            "repair requests"
        ),
    )
    parser.add_argument(
        "--planner-candidate-status-topic",
        default="/planning/candidate_status",
        help=(
            "collision-scored TinyNav lattice status used to distinguish a "
            "blocked local goal from a missing DDS trajectory stream"
        ),
    )
    parser.add_argument(
        "--planner-collision-status-timeout-s",
        type=float,
        default=1.0,
        help=(
            "freshness bound for immediately closing velocity on an "
            "all-candidates-in-collision planner report"
        ),
    )
    parser.add_argument(
        "--planner-collision-rejection-s",
        type=float,
        default=7.0,
        help=(
            "continuous fresh all-collision duration before rejecting the "
            "leg as LOCAL_GOAL_UNREACHABLE; velocity remains closed "
            "immediately while the local map/planner is allowed a bounded "
            "recovery window"
        ),
    )
    parser.add_argument("--pause-topic", default="/nav/paused")
    parser.add_argument(
        "--controller-pause-service",
        default=DEFAULT_CONTROLLER_PAUSE_SERVICE,
        help=(
            "robot-local acknowledged controller pause service; GOAL motion "
            "stays closed until an unpause response is received"
        ),
    )
    parser.add_argument(
        "--controller-pause-ack-timeout-s",
        type=float,
        default=DEFAULT_CONTROLLER_PAUSE_ACK_TIMEOUT_S,
    )
    parser.add_argument(
        "--controller-pause-startup-timeout-s",
        type=float,
        default=DEFAULT_CONTROLLER_PAUSE_STARTUP_TIMEOUT_S,
        help=(
            "one-time live startup bound for discovering the controller "
            "service and acknowledging paused=true before readiness"
        ),
    )
    parser.add_argument(
        "--controller-pause-retry-s",
        type=float,
        default=DEFAULT_CONTROLLER_PAUSE_RETRY_S,
    )
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/focus_guarded_cmd_vel")
    parser.add_argument(
        "--reverse-required-topic",
        default="/planning/reverse_required",
        help=(
            "local controller status topic declaring that its lookahead "
            "segment requires reverse motion"
        ),
    )
    parser.add_argument(
        "--reject-reverse-trajectory",
        action="store_true",
        help=(
            "latch a fresh reverse-required status for the current authority, "
            "zero output, and reject that leg for a fresh Hub replan"
        ),
    )
    parser.add_argument(
        "--turn-stalled-topic",
        default="/planning/turn_stalled",
        help=(
            "local controller status topic declaring that one continuous "
            "zero-linear heading recovery exceeded its bounded deadline"
        ),
    )
    parser.add_argument(
        "--reject-stalled-turn",
        action="store_true",
        help=(
            "latch a fresh turn-stalled status for the current authority, "
            "zero output, and reject that leg for a fresh Hub replan"
        ),
    )
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument(
        "--heartbeat-period-s",
        type=float,
        default=0.5,
        help=(
            "period for the independent health worker; heartbeat transport "
            "never blocks high-priority decision polling"
        ),
    )
    parser.add_argument(
        "--heartbeat-request-timeout-s",
        type=float,
        default=1.0,
        help="HTTP timeout used only by the independent heartbeat worker",
    )
    parser.add_argument("--local-data-timeout-s", type=float, default=2.0)
    parser.add_argument(
        "--occupancy-data-timeout-s",
        type=float,
        default=3.0,
        help=(
            "close the final velocity gate if the local collision/reachability "
            "map stops publishing"
        ),
    )
    parser.add_argument(
        "--occupancy-recovery-grace-s",
        type=float,
        default=7.0,
        help=(
            "maximum zero-velocity recovery window after occupancy freshness "
            "expires before the active episode leg is terminally rejected"
        ),
    )
    parser.add_argument(
        "--max-cached-occupancy-motion-m",
        type=float,
        default=0.0,
        help=(
            "after the wall-clock occupancy deadline, permit the exact cached "
            "grid only while base displacement from its receipt stays within "
            "this bound; zero disables cached-map motion"
        ),
    )
    parser.add_argument(
        "--health-gate-timeout-s",
        type=float,
        default=1.5,
        help=(
            "close the 20 Hz physical velocity gate if the independent "
            "receiver health loop stops evaluating"
        ),
    )
    parser.add_argument(
        "--heartbeat-delivery-recovery-grace-s",
        type=float,
        default=3.0,
        help=(
            "maximum zero-velocity wait after heartbeat delivery closes the "
            "physical gate; applies only to an existing leg while every "
            "robot-local health input stays ready"
        ),
    )
    parser.add_argument(
        "--router-recovery-grace-s",
        type=float,
        default=12.0,
        help=(
            "zero-velocity recovery window for transient goal-router "
            "odometry/occupancy lag or a still-maturing known-free map while "
            "this receiver independently remains READY"
        ),
    )
    parser.add_argument(
        "--no-progress-timeout-s",
        type=float,
        default=20.0,
        help=(
            "reject and zero an active fixed goal that does not reduce its "
            "metric remaining distance for this many seconds"
        ),
    )
    parser.add_argument(
        "--minimum-goal-progress-m",
        type=float,
        default=0.05,
        help="minimum remaining-distance reduction that resets the watchdog",
    )
    parser.add_argument(
        "--trajectory-start-grace-s",
        type=float,
        default=12.0,
        help=(
            "terminal deadline from local authorization to the first "
            "non-empty path; physical output remains zero until that path "
            "is fresh"
        ),
    )
    parser.add_argument(
        "--trajectory-stale-timeout-s",
        type=float,
        default=1.0,
        help=(
            "close the physical velocity gate when TinyNav stops publishing "
            "non-empty collision-free paths"
        ),
    )
    parser.add_argument(
        "--trajectory-recovery-timeout-s",
        type=float,
        default=12.0,
        help=(
            "terminal semantic-leg deadline after a previously observed "
            "trajectory becomes stale; the physical gate remains governed "
            "by --trajectory-stale-timeout-s"
        ),
    )
    parser.add_argument(
        "--slam-data-timeout-s",
        type=float,
        default=2.0,
        help=(
            "freshness deadline for the lower-rate SLAM diagnostic stream; "
            "control odometry keeps --local-data-timeout-s"
        ),
    )
    parser.add_argument(
        "--slam-max-transient-failures",
        type=int,
        default=0,
        help=(
            "number of bad SLAM reports allowed to keep the velocity gate "
            "open; zero makes every bad report stop motion immediately"
        ),
    )
    parser.add_argument(
        "--slam-transient-grace-s",
        type=float,
        default=2.0,
        help="maximum age of the last passing SLAM report during that blip",
    )
    parser.add_argument(
        "--slam-recovery-grace-s",
        type=float,
        default=2.0,
        help=(
            "maximum zero-velocity wait for a transient optimizer/IMU "
            "incident before the active episode leg is rejected"
        ),
    )
    parser.add_argument(
        "--odometry-recovery-grace-s",
        type=float,
        default=7.0,
        help=(
            "maximum zero-velocity wait after control odometry crosses its "
            "freshness gate while all independent health inputs remain ready"
        ),
    )
    parser.add_argument("--max-goal-distance-m", type=float, default=8.0)
    parser.add_argument(
        "--semantic-arrival-radius-m",
        type=float,
        default=0.15,
        help=(
            "robot-local terminal radius for a point selected from the "
            "transported semantic approach region"
        ),
    )
    parser.add_argument("--reachability-clearance-m", type=float, default=0.05)
    parser.add_argument("--start-snap-radius-m", type=float, default=0.75)
    parser.add_argument(
        "--start-footprint-override-m",
        type=float,
        default=0.35,
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
    if args.local_data_timeout_s <= 0.0:
        parser.error("--local-data-timeout-s must be positive")
    if args.occupancy_data_timeout_s <= 0.0:
        parser.error("--occupancy-data-timeout-s must be positive")
    if args.slam_data_timeout_s <= 0.0:
        parser.error("--slam-data-timeout-s must be positive")
    if args.robot_id not in LIVE_CONFIRMATIONS:
        parser.error("robot ID must be canonical robot-0 or robot-1")
    if args.enable_live_go2_motion and args.robot_id != "robot-0":
        parser.error("--enable-live-go2-motion is valid only for robot-0")
    if args.raw_cmd_topic == args.guarded_cmd_topic:
        parser.error("raw and guarded cmd_vel topics must differ")
    if not 0.10 <= args.semantic_arrival_radius_m <= 2.0:
        parser.error("--semantic-arrival-radius-m must be within [0.10, 2.0]")
    if min(
        args.poll_s,
        args.heartbeat_period_s,
        args.heartbeat_request_timeout_s,
        args.local_data_timeout_s,
        args.occupancy_data_timeout_s,
        args.occupancy_recovery_grace_s,
        args.health_gate_timeout_s,
        args.heartbeat_delivery_recovery_grace_s,
        args.router_recovery_grace_s,
        args.no_progress_timeout_s,
        args.minimum_goal_progress_m,
        args.trajectory_start_grace_s,
        args.trajectory_stale_timeout_s,
        args.trajectory_recovery_timeout_s,
        args.target_refresh_request_interval_s,
        args.planner_collision_status_timeout_s,
        args.planner_collision_rejection_s,
        args.slam_transient_grace_s,
        args.slam_recovery_grace_s,
        args.odometry_recovery_grace_s,
        args.controller_pause_ack_timeout_s,
        args.controller_pause_startup_timeout_s,
        args.controller_pause_retry_s,
        args.max_goal_distance_m,
        args.semantic_arrival_radius_m,
        args.max_alignment_shift_m,
        args.max_alignment_yaw_deg,
    ) <= 0:
        parser.error("timeouts, limits and poll interval must be positive")
    if (
        args.controller_pause_retry_s
        > args.controller_pause_ack_timeout_s
    ):
        parser.error(
            "--controller-pause-retry-s must not exceed "
            "--controller-pause-ack-timeout-s"
        )
    if (
        args.trajectory_recovery_timeout_s
        <= args.trajectory_stale_timeout_s
    ):
        parser.error(
            "--trajectory-recovery-timeout-s must exceed "
            "--trajectory-stale-timeout-s"
        )
    if (
        args.slam_max_transient_failures < 0
        or args.reachability_clearance_m < 0
        or args.start_snap_radius_m < 0
        or args.start_footprint_override_m < 0
        or not 0.0 <= args.max_cached_occupancy_motion_m <= 2.0
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
    from std_srvs.srv import SetBool
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
            self.occupancy_anchor_received_ns = 0
            self.occupancy_anchor_tracking_T_camera: (
                tuple[float, ...] | None
            ) = None
            self.occupancy_anchor_base_xy: tuple[float, float] | None = None
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
            self.trajectory_pose_count = 0
            self.trajectory_first_xy: tuple[float, float] | None = None
            self.trajectory_lookahead_xy: tuple[float, float] | None = None
            self.planner_candidate_status_received_ns = 0
            self.planner_collision_since_ns = 0
            self.planner_all_candidates_in_collision = False
            self.planner_candidate_count = 0
            self.planner_finite_candidate_count = 0
            self.planner_finite_in_place_candidate_count = 0
            self.planner_collision_refresh_pending = False
            self.planner_collision_refresh_requested_ns = 0
            self.planner_collision_refresh_count = 0
            self.reverse_required = False
            self.reverse_required_received_ns = 0
            self.reverse_required_for_authority = False
            self.turn_stalled = False
            self.turn_stalled_received_ns = 0
            self.turn_stalled_for_authority = False
            self.router_status_lock = threading.Lock()
            self.router_status_received_ns = 0
            self.router_state = ""
            self.router_reason = ""
            self.router_decision_id: str | None = None
            self.router_affected_decision_id: str | None = None
            self.router_waypoint: tuple[float, float] | None = None
            self.router_route_length_m: float | None = None
            self.authority_deadline_ns = 0
            self.authority_started_ns = 0
            self.authorized = False
            self.router_recovery_gate_closed = False
            self.motion_health_pass = False
            self.motion_health_evaluated_ns = 0
            self.cached_occupancy_motion_valid = False
            self.latest_raw_cmd = (0.0, 0.0)
            self.latest_guard_reason = "startup"
            self.last_target_refresh_request_ns = 0
            self.target_refresh_request_count = 0
            self.poi_publisher = self.create_publisher(String, args.cmd_pois_topic, 10)
            self.target_refresh_request_publisher = self.create_publisher(
                String,
                args.target_refresh_request_topic,
                10,
            )
            pause_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.pause_publisher = self.create_publisher(
                Bool, args.pause_topic, pause_qos
            )
            self.pause_client = self.create_client(
                SetBool, args.controller_pause_service
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
            self.create_subscription(
                String,
                args.planner_candidate_status_topic,
                self.on_planner_candidate_status,
                10,
            )
            if args.reject_reverse_trajectory:
                self.create_subscription(
                    Bool,
                    args.reverse_required_topic,
                    self.on_reverse_required,
                    10,
                )
            if args.reject_stalled_turn:
                self.create_subscription(
                    Bool,
                    args.turn_stalled_topic,
                    self.on_turn_stalled,
                    10,
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
                self.occupancy_anchor_tracking_T_camera = (
                    self.world_T_camera
                )
                self.occupancy_received_ns = time.time_ns()
            except ValueError as exc:
                self.occupancy = None
                self.occupancy_received_ns = 0
                self.occupancy_anchor_tracking_T_camera = None
                self.occupancy_anchor_base_xy = None
                if live:
                    self.guarded_publisher.publish(Twist())
                    self.latest_guard_reason = "occupancy_invalid"
                emit("occupancy_rejected", error=str(exc))

        def on_nav_done(self, message: Bool) -> None:
            if message.data:
                self.nav_done = True

        def on_trajectory(self, message: RosPath) -> None:
            try:
                (
                    pose_count,
                    first_xy,
                    lookahead_xy,
                ) = trajectory_message_summary(
                    message,
                    expected_frame=args.tinynav_map_frame,
                )
            except ValueError as exc:
                self.trajectory_received_ns = 0
                self.trajectory_pose_count = 0
                self.trajectory_first_xy = None
                self.trajectory_lookahead_xy = None
                if live:
                    self.guarded_publisher.publish(Twist())
                    self.latest_guard_reason = "trajectory_invalid"
                emit("trajectory_rejected", error=str(exc))
                return
            self.trajectory_received_ns = time.time_ns()
            self.trajectory_pose_count = pose_count
            self.trajectory_first_xy = first_xy
            self.trajectory_lookahead_xy = lookahead_xy

        def on_planner_candidate_status(self, message: String) -> None:
            received_ns = time.time_ns()
            try:
                status = parse_planner_candidate_status(message.data)
            except ValueError as exc:
                emit("planner_candidate_status_rejected", error=str(exc))
                return
            previous_received_ns = self.planner_candidate_status_received_ns
            if (
                self.planner_collision_refresh_pending
                and received_ns
                >= self.planner_collision_refresh_requested_ns
            ):
                # This is the first planner verdict produced after the
                # bounded target republish.  Only this new evidence may start
                # (or clear) the terminal all-collision interval.
                self.planner_collision_refresh_pending = False
            all_collision = bool(status["all_candidates_in_collision"])
            self.planner_candidate_status_received_ns = received_ns
            self.planner_all_candidates_in_collision = all_collision
            self.planner_candidate_count = int(status["candidate_count"])
            self.planner_finite_candidate_count = int(
                status["finite_candidate_count"]
            )
            self.planner_finite_in_place_candidate_count = int(
                status["finite_in_place_candidate_count"]
            )
            observed_for_authority = bool(
                self.authorized
                and self.authority_started_ns > 0
                and received_ns >= self.authority_started_ns
            )
            status_gap_s = (
                math.inf
                if previous_received_ns <= 0
                else (received_ns - previous_received_ns) / 1e9
            )
            if all_collision and observed_for_authority:
                if (
                    self.planner_collision_since_ns
                    < self.authority_started_ns
                    or status_gap_s
                    > args.planner_collision_status_timeout_s
                ):
                    self.planner_collision_since_ns = received_ns
                if live:
                    self.guarded_publisher.publish(Twist())
                    self.latest_guard_reason = (
                        "all_trajectories_in_collision"
                    )
            else:
                self.planner_collision_since_ns = 0

        def planner_collision_state(
            self, now_ns: int
        ) -> tuple[bool, bool, float, bool]:
            if self.planner_collision_refresh_pending:
                age_s = max(
                    0.0,
                    (
                        now_ns
                        - self.planner_collision_refresh_requested_ns
                    )
                    / 1e9,
                )
                # Keep velocity closed while waiting for a post-refresh
                # lattice verdict, but never call the pre-refresh collision
                # interval terminal.
                return True, False, age_s, False
            return planner_collision_gate_state(
                now_ns=now_ns,
                authority_started_ns=self.authority_started_ns,
                status_received_ns=(
                    self.planner_candidate_status_received_ns
                ),
                collision_since_ns=self.planner_collision_since_ns,
                all_candidates_in_collision=(
                    self.planner_all_candidates_in_collision
                ),
                status_timeout_s=(
                    args.planner_collision_status_timeout_s
                ),
                rejection_timeout_s=args.planner_collision_rejection_s,
            )

        def on_reverse_required(self, message: Bool) -> None:
            received_ns = time.time_ns()
            self.reverse_required = bool(message.data)
            self.reverse_required_received_ns = received_ns
            if (
                self.reverse_required
                and self.authorized
                and self.authority_started_ns > 0
                and received_ns >= self.authority_started_ns
            ):
                # Latch for this authority even if a later planner update
                # returns to a forward path before the 2 Hz protocol loop
                # observes it.  A new authority explicitly clears the latch.
                self.reverse_required_for_authority = True
                if live:
                    self.guarded_publisher.publish(Twist())
                    self.latest_guard_reason = "reverse_trajectory_rejected"

        def on_turn_stalled(self, message: Bool) -> None:
            received_ns = time.time_ns()
            self.turn_stalled = bool(message.data)
            self.turn_stalled_received_ns = received_ns
            if (
                self.turn_stalled
                and self.authorized
                and self.authority_started_ns > 0
                and received_ns >= self.authority_started_ns
            ):
                # Latch for this authority exactly like a reverse rejection.
                # Later controller callbacks may clear the status topic, but
                # only a fresh high-level authority may clear the failure.
                self.turn_stalled_for_authority = True
                if live:
                    self.guarded_publisher.publish(Twist())
                    self.latest_guard_reason = "turn_recovery_stalled"

        def on_router_status(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                state = str(payload["state"])
                reason = str(payload["reason"])
                if state not in {"HOLD", "ACCEPTED", "NAVIGATING", "ARRIVED"}:
                    raise ValueError("unknown router state")
                decision_id = payload.get("decision_id")
                affected_decision_id = payload.get("affected_decision_id")
                parsed_decision_id = (
                    None if decision_id is None else str(decision_id)
                )
                parsed_affected_decision_id = (
                    None
                    if affected_decision_id is None
                    else str(affected_decision_id)
                )
                waypoint = payload.get("waypoint")
                parsed_waypoint = (
                    (float(waypoint[0]), float(waypoint[1]))
                    if (
                        isinstance(waypoint, list)
                        and len(waypoint) == 2
                    )
                    else None
                )
                route_length = payload.get("route_length_m")
                parsed_route_length_m = (
                    None
                    if route_length is None
                    else float(route_length)
                )
                received_ns = time.time_ns()
                # The HTTP decision loop and ROS executor run on different
                # threads.  Publish one coherent router status generation so
                # a HOLD cannot be tested and then logged/classified using a
                # newer ACCEPTED callback (or vice versa).
                with self.router_status_lock:
                    self.router_state = state
                    self.router_reason = reason
                    self.router_decision_id = parsed_decision_id
                    self.router_affected_decision_id = (
                        parsed_affected_decision_id
                    )
                    self.router_waypoint = parsed_waypoint
                    self.router_route_length_m = parsed_route_length_m
                    self.router_status_received_ns = received_ns
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                emit("router_status_rejected", error=str(exc)[:300])

        def router_status_snapshot(
            self,
        ) -> tuple[
            int,
            str,
            str,
            str | None,
            str | None,
            tuple[float, float] | None,
            float | None,
        ]:
            with self.router_status_lock:
                return (
                    self.router_status_received_ns,
                    self.router_state,
                    self.router_reason,
                    self.router_decision_id,
                    self.router_affected_decision_id,
                    self.router_waypoint,
                    self.router_route_length_m,
                )

        def on_raw_cmd(self, message: Twist) -> None:
            now_ns = time.time_ns()
            self.raw_cmd_received_ns = now_ns
            if not twist_components_finite(message):
                self.latest_raw_cmd = (0.0, 0.0)
                if live:
                    self.guarded_publisher.publish(Twist())
                self.latest_guard_reason = "raw_command_nonfinite"
                emit("raw_command_rejected", reason="nonfinite_twist")
                return
            self.latest_raw_cmd = (
                float(message.linear.x),
                float(message.angular.z),
            )
            reason = self.velocity_gate_reason(now_ns)
            if live and reason is None:
                self.guarded_publisher.publish(message)
                self.latest_guard_reason = "authorized_all_gates_ready"
            elif live:
                self.guarded_publisher.publish(Twist())
                self.latest_guard_reason = reason or "authority_closed"

        def velocity_gate_reason(self, now_ns: int) -> str | None:
            path_fresh, _, _, _ = trajectory_gate_state(
                now_ns=now_ns,
                authority_started_ns=self.authority_started_ns,
                trajectory_received_ns=self.trajectory_received_ns,
                stale_timeout_s=args.trajectory_stale_timeout_s,
                start_grace_s=args.trajectory_start_grace_s,
                recovery_timeout_s=args.trajectory_recovery_timeout_s,
            )
            collision_gate_closed, _, _, _ = (
                self.planner_collision_state(now_ns)
            )
            return physical_velocity_gate_reason(
                now_ns=now_ns,
                authorized=self.authorized,
                authority_deadline_ns=self.authority_deadline_ns,
                trajectory_fresh=path_fresh,
                reverse_required=self.reverse_required_for_authority,
                turn_stalled=self.turn_stalled_for_authority,
                health_pass=self.motion_health_pass,
                health_evaluated_ns=self.motion_health_evaluated_ns,
                health_timeout_s=args.health_gate_timeout_s,
                odom_received_ns=self.odom_received_ns,
                slam_received_ns=self.slam_received_ns,
                odom_timeout_s=args.local_data_timeout_s,
                slam_timeout_s=args.slam_data_timeout_s,
                slam_pass=self.slam_pass,
                occupancy_received_ns=self.occupancy_received_ns,
                occupancy_timeout_s=args.occupancy_data_timeout_s,
                cached_occupancy_motion_valid=(
                    self.cached_occupancy_motion_valid
                ),
                platform_required=bool(args.platform_health_topic),
                platform_received_ns=self.platform_received_ns,
                platform_timeout_s=args.local_data_timeout_s,
                platform_pass=self.platform_pass,
                router_recovery_gate_closed=(
                    self.router_recovery_gate_closed
                ),
                all_candidates_in_collision=collision_gate_closed,
            )

        def update_motion_health(
            self,
            *,
            ready: bool,
            evaluated_ns: int,
            cached_occupancy_motion_valid: bool = False,
        ) -> None:
            self.motion_health_pass = bool(ready)
            self.motion_health_evaluated_ns = evaluated_ns
            self.cached_occupancy_motion_valid = bool(
                cached_occupancy_motion_valid
            )

        def enforce_gate(self) -> None:
            if not live:
                return
            now_ns = time.time_ns()
            if self.authorized and now_ns >= self.authority_deadline_ns:
                self.authorized = False
            reason = self.velocity_gate_reason(now_ns)
            if reason is not None:
                self.guarded_publisher.publish(Twist())
                self.latest_guard_reason = reason

        def authorize(self, expires_at_ns: int) -> None:
            if live:
                if self.router_recovery_gate_closed:
                    raise RuntimeError(
                        "cannot authorize while router recovery gate is closed"
                    )
                if not self.authorized:
                    self.authority_started_ns = time.time_ns()
                    self.reverse_required_for_authority = False
                    self.turn_stalled_for_authority = False
                    self.last_target_refresh_request_ns = 0
                    self.target_refresh_request_count = 0
                    self.planner_collision_refresh_pending = False
                    self.planner_collision_refresh_requested_ns = 0
                    self.planner_collision_refresh_count = 0
                    self.planner_collision_since_ns = 0
                self.authority_deadline_ns = expires_at_ns
                self.authorized = True

        def revoke(self) -> bool:
            self.authorized = False
            self.authority_deadline_ns = 0
            self.authority_started_ns = 0
            self.router_recovery_gate_closed = False
            self.last_target_refresh_request_ns = 0
            self.target_refresh_request_count = 0
            self.planner_collision_refresh_pending = False
            self.planner_collision_refresh_requested_ns = 0
            self.planner_collision_refresh_count = 0
            self.planner_collision_since_ns = 0
            self.latest_guard_reason = "revoked"
            if live:
                self.guarded_publisher.publish(Twist())
                paused = Bool()
                paused.data = True
                self.pause_publisher.publish(paused)
            return True

        def close_router_recovery_gate(self) -> None:
            """Stop physical output without destroying the active leg epoch."""

            if not live:
                raise RuntimeError("live TinyNav output is disabled")
            # This flag is an independent final velocity gate.  Preserve the
            # authority epoch, deadline and reverse-path latch so an overlapping
            # bounded sensor recovery can renew the exact same authenticated
            # leg.  TinyNav remains unpaused to consume queued map/odometry.
            self.router_recovery_gate_closed = True
            self.latest_guard_reason = "router_recovery_gate_closed"
            self.guarded_publisher.publish(Twist())

        def set_controller_paused_confirmed(
            self,
            paused: bool,
            *,
            timeout_s: float | None = None,
            phase: str = "runtime",
        ) -> None:
            """Change pause state only after the controller acknowledges it."""

            if not live:
                raise RuntimeError("live TinyNav output is disabled")
            effective_timeout_s = (
                args.controller_pause_ack_timeout_s
                if timeout_s is None
                else timeout_s
            )
            started = time.monotonic()
            deadline = started + effective_timeout_s
            attempts = 0
            last_error = "service unavailable"
            while time.monotonic() < deadline:
                remaining_s = deadline - time.monotonic()
                if not self.pause_client.wait_for_service(
                    timeout_sec=min(
                        args.controller_pause_retry_s,
                        max(0.0, remaining_s),
                    )
                ):
                    attempts += 1
                    continue
                request = SetBool.Request()
                request.data = bool(paused)
                future = self.pause_client.call_async(request)
                attempts += 1
                response_deadline = min(
                    deadline,
                    time.monotonic() + args.controller_pause_retry_s,
                )
                while (
                    not future.done()
                    and time.monotonic() < response_deadline
                ):
                    time.sleep(0.01)
                if not future.done():
                    future.cancel()
                    last_error = "service response timed out"
                    continue
                try:
                    response = future.result()
                except Exception as exc:  # noqa: BLE001 - bounded local RPC
                    last_error = str(exc)[:256]
                    continue
                if response is not None and bool(response.success):
                    emit(
                        "controller_pause_acknowledged",
                        paused=bool(paused),
                        attempts=attempts,
                        latency_s=round(time.monotonic() - started, 4),
                        phase=phase,
                        detail=str(response.message)[:256],
                    )
                    return
                last_error = (
                    "empty response"
                    if response is None
                    else str(response.message)[:256]
                )
            self.guarded_publisher.publish(Twist())
            self.latest_guard_reason = "controller_pause_ack_timeout"
            emit(
                "controller_pause_ack_timeout",
                paused=bool(paused),
                attempts=attempts,
                timeout_s=effective_timeout_s,
                phase=phase,
                error=last_error,
            )
            raise RuntimeError(
                "controller pause acknowledgement timed out: "
                f"paused={bool(paused)} attempts={attempts} "
                f"error={last_error}"
            )

        def publish_goal(
            self,
            payload: str,
            expires_at_ns: int,
            *,
            authorize_motion: bool = True,
        ) -> None:
            if not live:
                raise RuntimeError("live TinyNav output is disabled")
            self.nav_done = False
            if authorize_motion:
                self.set_controller_paused_confirmed(False)
            message = String()
            message.data = payload
            self.poi_publisher.publish(message)
            if authorize_motion:
                self.authorize(expires_at_ns)

        def request_planner_target_refresh(
            self,
            *,
            decision_id: str,
            path_age_s: float,
            router_waypoint: tuple[float, float],
            all_candidates_in_collision: bool = False,
        ) -> int | None:
            """Request bounded target re-publication without velocity output."""

            if not live or not self.authorized:
                return None
            if (
                all_candidates_in_collision
                and self.planner_collision_refresh_count >= 1
            ):
                return None
            now_ns = time.time_ns()
            if (
                self.last_target_refresh_request_ns > 0
                and (
                    now_ns - self.last_target_refresh_request_ns
                )
                / 1e9
                < args.target_refresh_request_interval_s
            ):
                return None
            self.last_target_refresh_request_ns = now_ns
            self.target_refresh_request_count += 1
            message = String()
            message.data = json.dumps(
                {
                    "schema_version": (
                        TARGET_REFRESH_REQUEST_SCHEMA_VERSION
                    ),
                    "decision_id": decision_id,
                    "requested_at_ns": now_ns,
                    "path_age_s": round(path_age_s, 6),
                    "router_waypoint": [
                        float(router_waypoint[0]),
                        float(router_waypoint[1]),
                    ],
                    "trigger": (
                        "all_candidates_in_collision"
                        if all_candidates_in_collision
                        else "trajectory_missing_or_stale"
                    ),
                },
                separators=(",", ":"),
            )
            self.target_refresh_request_publisher.publish(message)
            if all_candidates_in_collision:
                self.planner_collision_refresh_count += 1
                self.planner_collision_refresh_pending = True
                self.planner_collision_refresh_requested_ns = now_ns
                self.planner_candidate_status_received_ns = 0
                self.planner_collision_since_ns = 0
                self.planner_all_candidates_in_collision = False
            return self.target_refresh_request_count

        def resume_existing_goal(self, expires_at_ns: int) -> None:
            if not live:
                raise RuntimeError("live TinyNav output is disabled")
            if self.authority_started_ns <= 0:
                raise RuntimeError(
                    "router recovery lacks prior local authority"
                )
            self.set_controller_paused_confirmed(False)
            # Keep the independent recovery gate closed until every authority
            # field is restored; clearing it last avoids a transient open gate
            # between the pause acknowledgement and lease renewal.
            self.authority_deadline_ns = expires_at_ns
            self.authorized = True
            self.router_recovery_gate_closed = False

        def renew_authority_while_gate_closed(
            self, expires_at_ns: int
        ) -> None:
            """Extend one verified leg while sensor health keeps output at zero."""

            if not live:
                raise RuntimeError("live TinyNav output is disabled")
            if self.motion_health_pass:
                raise RuntimeError(
                    "bounded recovery renewal requires a closed health gate"
                )
            if expires_at_ns <= time.time_ns():
                raise RuntimeError("bounded recovery renewal already expired")
            if self.authority_started_ns <= 0:
                raise RuntimeError(
                    "bounded recovery renewal lacks prior local authority"
                )
            # Preserve the original authority epoch and reverse-path latch.
            # The 20 Hz gate remains closed because motion_health_pass is
            # false; this only prevents the same authenticated leg and the
            # local router lease from expiring before bounded map recovery.
            self.authority_deadline_ns = expires_at_ns
            self.authorized = True

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
            candidate_status_publishers = self.get_publishers_info_by_topic(
                args.planner_candidate_status_topic
            )
            occupancy_publishers = self.get_publishers_info_by_topic(
                args.occupancy_topic
            )
            reverse_status_publishers = (
                self.get_publishers_info_by_topic(
                    args.reverse_required_topic
                )
                if args.reject_reverse_trajectory
                else ()
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
                "planner_candidate_status_publisher": bool(
                    candidate_status_publishers
                ),
                "online_router_status_publisher": (
                    bool(router_status_publishers)
                    if args.online_buildmap_world
                    else True
                ),
                "reverse_required_status_publisher": (
                    bool(reverse_status_publishers)
                    if args.reject_reverse_trajectory
                    else True
                ),
            }
            return all(checks.values()), json.dumps(
                checks, sort_keys=True, separators=(",", ":")
            )

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
    if live:
        # Fast DDS discovery for a newly created service client can lag behind
        # topic discovery after a controller/receiver lifecycle change. Prime
        # the persistent client with the only safe startup state before this
        # receiver can advertise readiness or receive a GOAL. Runtime unpause
        # acknowledgements then reuse the already discovered connection and
        # retain their short one-second bound.
        node.guarded_publisher.publish(Twist())
        node.set_controller_paused_confirmed(
            True,
            timeout_s=args.controller_pause_startup_timeout_s,
            phase="startup",
        )
    hub = HubV2RobotClient(args.base_url, args.robot_id, token)
    heartbeat_hub = HubV2RobotClient(
        args.base_url,
        args.robot_id,
        token,
        timeout_s=args.heartbeat_request_timeout_s,
    )

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
        if rclpy.ok():
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
            "semantic_terminal_policy": {
                "arrival_radius_m": args.semantic_arrival_radius_m,
                "classification": (
                    "explicit_robot_local_realworld_execution_tolerance"
                ),
                "formal_sr_spl_requires_independent_goal_region": True,
            },
        },
    )
    atomic_write_json(alignment_output, artifact)
    emit(
        "alignment_ready",
        output=str(alignment_output),
        shared_T_robot_map=list(shared_T_robot_map),
        live=live,
    )
    heartbeat_pump = (
        HubHeartbeatPump(
            heartbeat_hub,
            period_s=args.heartbeat_period_s,
        )
        if live
        else None
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
            semantic_arrival_radius_m=args.semantic_arrival_radius_m,
        )
    )
    path = PathAccumulator()
    path_episode_id: str | None = None
    last_decision_id: str | None = None
    active_decision = None
    active_goal = None
    last_feedback_monotonic = 0.0
    goal_issued_ns = 0
    router_recovery_leg_id: str | None = None
    router_recovery_started_ns = 0
    router_recovery_reason = ""
    occupancy_recovery_leg_id: str | None = None
    occupancy_recovery_started_ns = 0
    slam_recovery_leg_id: str | None = None
    slam_recovery_started_ns = 0
    odometry_recovery_leg_id: str | None = None
    odometry_recovery_started_ns = 0
    odometry_slam_recovery_leg_id: str | None = None
    odometry_slam_recovery_started_ns = 0
    progress_watchdog = GoalProgressWatchdog(
        timeout_s=args.no_progress_timeout_s,
        minimum_improvement_m=args.minimum_goal_progress_m,
    )
    heartbeat_result_sequence = 0
    heartbeat_delivery_ready_previous: bool | None = None
    heartbeat_recovery_leg_id: str | None = None
    heartbeat_recovery_started_ns = 0

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
                node.update_motion_health(
                    ready=False, evaluated_ns=time.time_ns()
                )
                if active_decision is not None:
                    node.revoke()
                    active_decision = None
                    active_goal = None
                emit("localization_failed_local_hold", error=str(exc)[:500])
                time.sleep(args.poll_s)
                continue
            path.update(pose[0], pose[1])
            if (
                node.occupancy_received_ns > 0
                and node.occupancy_received_ns
                != node.occupancy_anchor_received_ns
            ):
                # Bind each exact occupancy generation to the base pose seen
                # by the same receiver. Sparse keyframes may then remain
                # spatially valid while stationary, but never beyond the
                # router's independently enforced displacement bound.
                node.occupancy_anchor_received_ns = (
                    node.occupancy_received_ns
                )
                if node.occupancy_anchor_tracking_T_camera is None:
                    node.occupancy_anchor_base_xy = None
                else:
                    occupancy_anchor_pose = robot_map_base_pose(
                        tracking_T_map=current_tracking_T_map,
                        tracking_T_camera=(
                            node.occupancy_anchor_tracking_T_camera
                        ),
                        base_T_camera=base_camera_calibration.matrix,
                    )
                    node.occupancy_anchor_base_xy = (
                        occupancy_anchor_pose[0],
                        occupancy_anchor_pose[1],
                    )
            if active_decision is None:
                heartbeat_recovery_leg_id = None
                heartbeat_recovery_started_ns = 0
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
                odometry_slam_recovery_leg_id = None
                odometry_slam_recovery_started_ns = 0
            now_ns = time.time_ns()
            alignment_shift, alignment_yaw = planar_transform_delta(
                tracking_T_map, current_tracking_T_map
            )
            alignment_stable = (
                alignment_shift <= args.max_alignment_shift_m
                and math.degrees(alignment_yaw) <= args.max_alignment_yaw_deg
            )
            local_fresh, odom_age_s, slam_age_s = local_tracking_freshness(
                now_ns=now_ns,
                odom_received_ns=node.odom_received_ns,
                slam_received_ns=node.slam_received_ns,
                odom_timeout_s=args.local_data_timeout_s,
                slam_timeout_s=args.slam_data_timeout_s,
            )
            platform_fresh = (
                not args.platform_health_topic
                or (
                    node.platform_received_ns > 0
                    and now_ns - node.platform_received_ns
                    <= int(args.local_data_timeout_s * 1e9)
                )
            )
            occupancy_age_s = (
                math.inf
                if node.occupancy_received_ns <= 0
                else max(
                    0.0,
                    (now_ns - node.occupancy_received_ns) / 1e9,
                )
            )
            cached_occupancy_motion_valid = False
            cached_occupancy_motion_m: float | None = None
            if (
                node.occupancy is not None
                and occupancy_age_s > args.occupancy_data_timeout_s
                and args.max_cached_occupancy_motion_m > 0.0
            ):
                (
                    cached_occupancy_motion_valid,
                    cached_occupancy_motion_m,
                ) = cached_map_valid_for_pose(
                    map_age_s=occupancy_age_s,
                    map_timeout_s=args.occupancy_data_timeout_s,
                    map_anchor_base_xy=node.occupancy_anchor_base_xy,
                    current_base_xy=(pose[0], pose[1]),
                    max_cached_map_motion_m=(
                        args.max_cached_occupancy_motion_m
                    ),
                )
            occupancy_fresh = bool(
                node.occupancy is not None
                and (
                    occupancy_age_s <= args.occupancy_data_timeout_s
                    or cached_occupancy_motion_valid
                )
            )
            graph_ready, graph_detail = node.planner_graph_ready()
            odometry_fresh = bool(
                node.odom_received_ns > 0
                and odom_age_s <= args.local_data_timeout_s
            )
            slam_stream_fresh = bool(
                node.slam_received_ns > 0
                and slam_age_s <= args.slam_data_timeout_s
            )
            all_non_slam_health_ready = (
                local_fresh
                and alignment_stable
                and graph_ready
                and platform_fresh
                and node.platform_pass
            )
            all_other_health_ready = (
                all_non_slam_health_ready and node.slam_pass
            )
            ready = all_other_health_ready and occupancy_fresh
            transient_slam_failure = bool(
                not node.slam_pass
                and node.slam_detail in TRANSIENT_SLAM_FAILURES
            )
            occupancy_recovery_candidate = bool(
                active_decision is not None
                and not occupancy_fresh
                and node.occupancy is not None
                and all_non_slam_health_ready
                and (node.slam_pass or transient_slam_failure)
            )
            if occupancy_recovery_candidate:
                if (
                    occupancy_recovery_started_ns <= 0
                    or occupancy_recovery_leg_id
                    != active_decision.leg_id
                ):
                    occupancy_recovery_started_ns = now_ns
                occupancy_recovery_elapsed_s = max(
                    0.0,
                    (now_ns - occupancy_recovery_started_ns) / 1e9,
                )
            else:
                occupancy_recovery_elapsed_s = math.inf
            occupancy_recovery_active = occupancy_recovery_eligible(
                recovery_elapsed_s=occupancy_recovery_elapsed_s,
                recovery_grace_s=args.occupancy_recovery_grace_s,
                all_other_health_ready=all_other_health_ready,
                occupancy_observed=node.occupancy is not None,
            )
            slam_recovery_candidate = bool(
                active_decision is not None
                and not node.slam_pass
                and all_non_slam_health_ready
                and transient_slam_failure
                and (occupancy_fresh or node.occupancy is not None)
            )
            if slam_recovery_candidate:
                slam_recovery_started_ns = inherited_recovery_start_ns(
                    now_ns=now_ns,
                    active_leg_id=active_decision.leg_id,
                    current_started_ns=slam_recovery_started_ns,
                    current_leg_id=slam_recovery_leg_id,
                    handoff_started_ns=(
                        odometry_slam_recovery_started_ns
                    ),
                    handoff_leg_id=odometry_slam_recovery_leg_id,
                )
                slam_recovery_elapsed_s = max(
                    0.0,
                    (now_ns - slam_recovery_started_ns) / 1e9,
                )
            else:
                slam_recovery_elapsed_s = math.inf
            slam_recovery_active = slam_recovery_eligible(
                recovery_elapsed_s=slam_recovery_elapsed_s,
                recovery_grace_s=args.slam_recovery_grace_s,
                slam_detail=node.slam_detail,
                all_non_slam_health_ready=bool(
                    all_non_slam_health_ready and occupancy_fresh
                ),
            )
            all_non_odometry_slam_health_ready = bool(
                slam_stream_fresh
                and occupancy_fresh
                and alignment_stable
                and graph_ready
                and platform_fresh
                and node.platform_pass
            )
            all_non_odometry_health_ready = bool(
                all_non_odometry_slam_health_ready and node.slam_pass
            )
            odometry_recovery_candidate = bool(
                active_decision is not None
                and not odometry_fresh
                and node.odom_received_ns > 0
                and all_non_odometry_slam_health_ready
                and (node.slam_pass or transient_slam_failure)
            )
            if odometry_recovery_candidate:
                if (
                    odometry_recovery_started_ns <= 0
                    or odometry_recovery_leg_id
                    != active_decision.leg_id
                ):
                    odometry_recovery_started_ns = now_ns
                odometry_recovery_elapsed_s = max(
                    0.0,
                    (now_ns - odometry_recovery_started_ns) / 1e9,
                )
            else:
                odometry_recovery_elapsed_s = math.inf
            healthy_slam_odometry_recovery_active = (
                odometry_recovery_eligible(
                    recovery_elapsed_s=odometry_recovery_elapsed_s,
                    recovery_grace_s=args.odometry_recovery_grace_s,
                    all_non_odometry_health_ready=(
                        all_non_odometry_health_ready
                    ),
                    odometry_observed=node.odom_received_ns > 0,
                )
            )
            odometry_slam_recovery_candidate = bool(
                odometry_recovery_candidate and transient_slam_failure
            )
            if odometry_slam_recovery_candidate:
                odometry_slam_recovery_started_ns = (
                    inherited_recovery_start_ns(
                        now_ns=now_ns,
                        active_leg_id=active_decision.leg_id,
                        current_started_ns=(
                            odometry_slam_recovery_started_ns
                        ),
                        current_leg_id=odometry_slam_recovery_leg_id,
                        handoff_started_ns=slam_recovery_started_ns,
                        handoff_leg_id=slam_recovery_leg_id,
                    )
                )
                odometry_slam_recovery_leg_id = active_decision.leg_id
                odometry_slam_recovery_elapsed_s = max(
                    0.0,
                    (
                        now_ns - odometry_slam_recovery_started_ns
                    )
                    / 1e9,
                )
            else:
                odometry_slam_recovery_leg_id = None
                odometry_slam_recovery_started_ns = 0
                odometry_slam_recovery_elapsed_s = math.inf
            odometry_slam_recovery_active = (
                odometry_slam_recovery_candidate
                and odometry_slam_recovery_eligible(
                    odometry_recovery_elapsed_s=(
                        odometry_recovery_elapsed_s
                    ),
                    odometry_recovery_grace_s=(
                        args.odometry_recovery_grace_s
                    ),
                    slam_recovery_elapsed_s=(
                        odometry_slam_recovery_elapsed_s
                    ),
                    slam_recovery_grace_s=args.slam_recovery_grace_s,
                    slam_detail=node.slam_detail,
                    all_non_odometry_slam_health_ready=(
                        all_non_odometry_slam_health_ready
                    ),
                    odometry_observed=node.odom_received_ns > 0,
                )
            )
            odometry_recovery_active = bool(
                healthy_slam_odometry_recovery_active
                or odometry_slam_recovery_active
            )
            combined_sensor_recovery_candidate = bool(
                occupancy_recovery_candidate
                and slam_recovery_candidate
            )
            combined_sensor_recovery_active = bool(
                combined_sensor_recovery_candidate
                and combined_sensor_recovery_eligible(
                    occupancy_recovery_elapsed_s=(
                        occupancy_recovery_elapsed_s
                    ),
                    occupancy_recovery_grace_s=(
                        args.occupancy_recovery_grace_s
                    ),
                    slam_recovery_elapsed_s=slam_recovery_elapsed_s,
                    slam_recovery_grace_s=args.slam_recovery_grace_s,
                    slam_detail=node.slam_detail,
                    all_non_sensor_health_ready=(
                        all_non_slam_health_ready
                    ),
                    occupancy_observed=node.occupancy is not None,
                )
            )
            recovery_kind = closed_gate_recovery_kind(
                occupancy_recovery_active=occupancy_recovery_active,
                slam_recovery_active=slam_recovery_active,
                odometry_recovery_active=odometry_recovery_active,
                combined_sensor_recovery_active=(
                    combined_sensor_recovery_active
                ),
            )
            if (
                occupancy_recovery_leg_id is not None
                and slam_recovery_leg_id is not None
                and occupancy_recovery_leg_id != slam_recovery_leg_id
            ):
                raise RuntimeError(
                    "closed-gate recovery owners disagree on active leg"
                )
            if (
                odometry_recovery_leg_id is not None
                and (
                    occupancy_recovery_leg_id is not None
                    or slam_recovery_leg_id is not None
                )
            ):
                raise RuntimeError(
                    "odometry recovery cannot overlap another recovery owner"
                )
            previous_recovery_kind = closed_gate_recovery_kind(
                occupancy_recovery_active=bool(
                    occupancy_recovery_leg_id is not None
                    and slam_recovery_leg_id is None
                ),
                slam_recovery_active=bool(
                    slam_recovery_leg_id is not None
                    and occupancy_recovery_leg_id is None
                ),
                odometry_recovery_active=bool(
                    odometry_recovery_leg_id is not None
                ),
                combined_sensor_recovery_active=bool(
                    occupancy_recovery_leg_id is not None
                    and slam_recovery_leg_id is not None
                ),
            )
            if (
                previous_recovery_kind is not None
                and recovery_kind is not None
                and previous_recovery_kind != recovery_kind
            ):
                emit(
                    "closed_gate_recovery_handoff",
                    from_kind=previous_recovery_kind,
                    to_kind=recovery_kind,
                    leg_id=(
                        occupancy_recovery_leg_id
                        or slam_recovery_leg_id
                        or odometry_recovery_leg_id
                    ),
                    physical_velocity_gate_closed=True,
                )
                progress_watchdog.reset()
            if recovery_kind == "occupancy":
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
            elif recovery_kind == "slam":
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
            elif recovery_kind == "odometry":
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
            elif recovery_kind == "combined":
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
            sensor_ready = ready
            reported_health = RobotHealth(
                safety_state=SafetyState.READY if ready else SafetyState.HOLD,
                localization_state=(
                    LocalizationState.TRACKING
                    if local_fresh and node.slam_pass and alignment_stable
                    else LocalizationState.LOST
                ),
                estop_engaged=node.platform_estop,
                collision_avoidance_ready=bool(
                    occupancy_fresh and graph_ready
                ),
                motor_controller_ready=bool(
                    graph_ready and platform_fresh and node.platform_pass
                ),
                detail=bounded_protocol_detail(
                    f"{node.slam_detail}; odom_age={odom_age_s:.3f}s/"
                    f"{args.local_data_timeout_s:.3f}s; "
                    f"slam_age={slam_age_s:.3f}s/"
                    f"{args.slam_data_timeout_s:.3f}s; "
                    f"occupancy_age={occupancy_age_s:.3f}s/"
                    f"{args.occupancy_data_timeout_s:.3f}s; "
                    "cached_occupancy_motion="
                    + (
                        "not_active"
                        if cached_occupancy_motion_m is None
                        else (
                            f"{cached_occupancy_motion_m:.3f}m/"
                            f"{args.max_cached_occupancy_motion_m:.3f}m"
                        )
                    )
                    + "; "
                    f"alignment_shift={alignment_shift:.3f}m; "
                    f"alignment_yaw={math.degrees(alignment_yaw):.2f}deg; {graph_detail}; "
                    f"{node.platform_detail}; "
                    + (
                        "Go2 handheld remote retains final local priority"
                        if args.robot_id == "robot-0"
                        else "WATER local status/watchdog retains final authority"
                    )
                ),
            )
            heartbeat_delivery_ready = True
            health = reported_health
            if heartbeat_pump is not None:
                heartbeat_pump.update(reported_health)
                heartbeat_state = heartbeat_pump.snapshot()
                heartbeat_delivery_ready = heartbeat_state.delivered_within(
                    args.health_gate_timeout_s
                )
                if heartbeat_state.result_sequence != heartbeat_result_sequence:
                    heartbeat_result_sequence = heartbeat_state.result_sequence
                    emit(
                        (
                            "heartbeat_delivered"
                            if heartbeat_state.last_result_ok
                            else "heartbeat_delivery_failed"
                        ),
                        result_sequence=heartbeat_state.result_sequence,
                        error=heartbeat_state.last_error,
                    )
                if (
                    heartbeat_delivery_ready
                    != heartbeat_delivery_ready_previous
                ):
                    emit(
                        "heartbeat_delivery_gate_changed",
                        ready=heartbeat_delivery_ready,
                        request_in_flight=(
                            heartbeat_state.request_in_flight
                        ),
                        physical_velocity_gate_closed=(
                            not heartbeat_delivery_ready
                        ),
                    )
                    heartbeat_delivery_ready_previous = (
                        heartbeat_delivery_ready
                    )
                if not heartbeat_delivery_ready:
                    health = reported_health.model_copy(
                        update={
                            "safety_state": SafetyState.HOLD,
                            "detail": bounded_protocol_detail(
                                reported_health.detail
                                + "; heartbeat delivery is not fresh"
                            ),
                        }
                    )
            ready = bool(sensor_ready and heartbeat_delivery_ready)
            heartbeat_recovery_candidate = bool(
                active_decision is not None
                and sensor_ready
                and not heartbeat_delivery_ready
            )
            if heartbeat_recovery_candidate:
                if (
                    heartbeat_recovery_started_ns <= 0
                    or heartbeat_recovery_leg_id
                    != active_decision.leg_id
                ):
                    heartbeat_recovery_started_ns = now_ns
                heartbeat_recovery_elapsed_s = max(
                    0.0,
                    (now_ns - heartbeat_recovery_started_ns) / 1e9,
                )
            else:
                heartbeat_recovery_elapsed_s = math.inf
            heartbeat_recovery_active = (
                heartbeat_delivery_recovery_eligible(
                    recovery_elapsed_s=heartbeat_recovery_elapsed_s,
                    recovery_grace_s=(
                        args.heartbeat_delivery_recovery_grace_s
                    ),
                    sensor_ready=sensor_ready,
                    heartbeat_delivery_ready=heartbeat_delivery_ready,
                )
            )
            node.update_motion_health(
                ready=ready,
                evaluated_ns=now_ns,
                cached_occupancy_motion_valid=(
                    cached_occupancy_motion_valid
                ),
            )
            if (
                heartbeat_delivery_ready
                and heartbeat_recovery_leg_id is not None
            ):
                emit(
                    "heartbeat_delivery_recovery_complete",
                    decision_id=(
                        None
                        if active_decision is None
                        else active_decision.decision_id
                    ),
                    leg_id=heartbeat_recovery_leg_id,
                    recovery_duration_s=round(
                        max(
                            0.0,
                            (
                                now_ns
                                - heartbeat_recovery_started_ns
                            )
                            / 1e9,
                        ),
                        3,
                    ),
                    physical_velocity_gate_closed=not ready,
                )
                heartbeat_recovery_leg_id = None
                heartbeat_recovery_started_ns = 0
                progress_watchdog.reset()
            elif not sensor_ready and heartbeat_recovery_leg_id is not None:
                emit(
                    "heartbeat_delivery_recovery_cancelled",
                    decision_id=(
                        None
                        if active_decision is None
                        else active_decision.decision_id
                    ),
                    leg_id=heartbeat_recovery_leg_id,
                    reason="robot_local_health_not_ready",
                    physical_velocity_gate_closed=True,
                )
                heartbeat_recovery_leg_id = None
                heartbeat_recovery_started_ns = 0
            if (
                sensor_ready
                and occupancy_recovery_leg_id is not None
                and slam_recovery_leg_id is not None
            ):
                emit(
                    "combined_sensor_recovery_complete",
                    decision_id=(
                        None
                        if active_decision is None
                        else active_decision.decision_id
                    ),
                    leg_id=occupancy_recovery_leg_id,
                    occupancy_recovery_duration_s=round(
                        max(
                            0.0,
                            (
                                now_ns
                                - occupancy_recovery_started_ns
                            )
                            / 1e9,
                        ),
                        3,
                    ),
                    slam_recovery_duration_s=round(
                        max(
                            0.0,
                            (now_ns - slam_recovery_started_ns) / 1e9,
                        ),
                        3,
                    ),
                )
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
                progress_watchdog.reset()
            elif sensor_ready and occupancy_recovery_leg_id is not None:
                emit(
                    "occupancy_recovery_complete",
                    decision_id=(
                        None
                        if active_decision is None
                        else active_decision.decision_id
                    ),
                    leg_id=occupancy_recovery_leg_id,
                    occupancy_age_s=round(occupancy_age_s, 3),
                )
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                progress_watchdog.reset()
            elif sensor_ready and slam_recovery_leg_id is not None:
                emit(
                    "slam_recovery_complete",
                    decision_id=(
                        None
                        if active_decision is None
                        else active_decision.decision_id
                    ),
                    leg_id=slam_recovery_leg_id,
                    recovery_duration_s=round(
                        max(
                            0.0,
                            (now_ns - slam_recovery_started_ns) / 1e9,
                        ),
                        3,
                    ),
                )
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
                progress_watchdog.reset()
            elif sensor_ready and odometry_recovery_leg_id is not None:
                emit(
                    "odometry_recovery_complete",
                    decision_id=(
                        None
                        if active_decision is None
                        else active_decision.decision_id
                    ),
                    leg_id=odometry_recovery_leg_id,
                    recovery_duration_s=round(
                        max(
                            0.0,
                            (
                                now_ns
                                - odometry_recovery_started_ns
                            )
                            / 1e9,
                        ),
                        3,
                    ),
                    odometry_age_s=round(odom_age_s, 3),
                )
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
                progress_watchdog.reset()
            if not ready and active_decision is not None:
                if heartbeat_recovery_active:
                    if (
                        heartbeat_recovery_leg_id
                        != active_decision.leg_id
                    ):
                        heartbeat_recovery_leg_id = (
                            active_decision.leg_id
                        )
                        emit(
                            "heartbeat_delivery_recovery_wait",
                            decision_id=active_decision.decision_id,
                            leg_id=active_decision.leg_id,
                            health_gate_timeout_s=(
                                args.health_gate_timeout_s
                            ),
                            recovery_elapsed_s=round(
                                heartbeat_recovery_elapsed_s, 3
                            ),
                            recovery_grace_s=(
                                args.heartbeat_delivery_recovery_grace_s
                            ),
                            physical_velocity_gate_closed=True,
                        )
                        if not post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_HEARTBEAT_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate closed immediately "
                                "for bounded heartbeat-delivery recovery"
                            ),
                        ):
                            node.revoke()
                            active_decision = None
                            active_goal = None
                            heartbeat_recovery_leg_id = None
                            heartbeat_recovery_started_ns = 0
                            progress_watchdog.reset()
                            continue
                        last_feedback_monotonic = time.monotonic()
                elif recovery_kind == "combined":
                    if (
                        occupancy_recovery_leg_id
                        != active_decision.leg_id
                        or slam_recovery_leg_id
                        != active_decision.leg_id
                    ):
                        occupancy_recovery_leg_id = (
                            active_decision.leg_id
                        )
                        slam_recovery_leg_id = active_decision.leg_id
                        emit(
                            "combined_sensor_recovery_wait",
                            decision_id=active_decision.decision_id,
                            leg_id=active_decision.leg_id,
                            slam_detail=node.slam_detail,
                            occupancy_age_s=round(
                                occupancy_age_s, 3
                            ),
                            cached_occupancy_motion_m=(
                                cached_occupancy_motion_m
                            ),
                            max_cached_occupancy_motion_m=(
                                args.max_cached_occupancy_motion_m
                            ),
                            occupancy_recovery_elapsed_s=round(
                                occupancy_recovery_elapsed_s, 3
                            ),
                            occupancy_recovery_grace_s=(
                                args.occupancy_recovery_grace_s
                            ),
                            slam_recovery_elapsed_s=round(
                                slam_recovery_elapsed_s, 3
                            ),
                            slam_recovery_grace_s=(
                                args.slam_recovery_grace_s
                            ),
                            physical_velocity_gate_closed=True,
                        )
                        if not post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_SENSOR_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate closed immediately "
                                "for bounded combined occupancy and "
                                "transient-SLAM recovery"
                            ),
                        ):
                            node.revoke()
                            active_decision = None
                            active_goal = None
                            occupancy_recovery_leg_id = None
                            occupancy_recovery_started_ns = 0
                            slam_recovery_leg_id = None
                            slam_recovery_started_ns = 0
                            progress_watchdog.reset()
                            continue
                        last_feedback_monotonic = time.monotonic()
                elif recovery_kind == "occupancy":
                    if (
                        occupancy_recovery_leg_id
                        != active_decision.leg_id
                    ):
                        occupancy_recovery_leg_id = active_decision.leg_id
                        emit(
                            "occupancy_stale_recovery_wait",
                            decision_id=active_decision.decision_id,
                            leg_id=active_decision.leg_id,
                            occupancy_age_s=round(occupancy_age_s, 3),
                            freshness_timeout_s=(
                                args.occupancy_data_timeout_s
                            ),
                            cached_occupancy_motion_m=(
                                cached_occupancy_motion_m
                            ),
                            max_cached_occupancy_motion_m=(
                                args.max_cached_occupancy_motion_m
                            ),
                            recovery_elapsed_s=round(
                                occupancy_recovery_elapsed_s, 3
                            ),
                            recovery_grace_s=(
                                args.occupancy_recovery_grace_s
                            ),
                            physical_velocity_gate_closed=True,
                        )
                        if not post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_OCCUPANCY_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate closed immediately "
                                "for bounded occupancy recovery"
                            ),
                        ):
                            node.revoke()
                            active_decision = None
                            active_goal = None
                            occupancy_recovery_leg_id = None
                            occupancy_recovery_started_ns = 0
                            progress_watchdog.reset()
                            continue
                        last_feedback_monotonic = time.monotonic()
                elif recovery_kind == "slam":
                    if slam_recovery_leg_id != active_decision.leg_id:
                        slam_recovery_leg_id = active_decision.leg_id
                        emit(
                            "slam_transient_recovery_wait",
                            decision_id=active_decision.decision_id,
                            leg_id=active_decision.leg_id,
                            slam_detail=node.slam_detail,
                            recovery_elapsed_s=round(
                                slam_recovery_elapsed_s, 3
                            ),
                            recovery_grace_s=args.slam_recovery_grace_s,
                            physical_velocity_gate_closed=True,
                        )
                        if not post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_SLAM_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate closed immediately "
                                "for bounded transient SLAM recovery"
                            ),
                        ):
                            node.revoke()
                            active_decision = None
                            active_goal = None
                            slam_recovery_leg_id = None
                            slam_recovery_started_ns = 0
                            progress_watchdog.reset()
                            continue
                        last_feedback_monotonic = time.monotonic()
                elif recovery_kind == "odometry":
                    if (
                        odometry_recovery_leg_id
                        != active_decision.leg_id
                    ):
                        odometry_recovery_leg_id = (
                            active_decision.leg_id
                        )
                        emit(
                            "odometry_stale_recovery_wait",
                            decision_id=active_decision.decision_id,
                            leg_id=active_decision.leg_id,
                            odometry_age_s=round(odom_age_s, 3),
                            slam_detail=node.slam_detail,
                            simultaneous_transient_slam=(
                                odometry_slam_recovery_candidate
                            ),
                            transient_slam_recovery_elapsed_s=(
                                round(
                                    odometry_slam_recovery_elapsed_s,
                                    3,
                                )
                                if math.isfinite(
                                    odometry_slam_recovery_elapsed_s
                                )
                                else None
                            ),
                            transient_slam_recovery_grace_s=(
                                args.slam_recovery_grace_s
                            ),
                            freshness_timeout_s=(
                                args.local_data_timeout_s
                            ),
                            recovery_elapsed_s=round(
                                odometry_recovery_elapsed_s, 3
                            ),
                            recovery_grace_s=(
                                args.odometry_recovery_grace_s
                            ),
                            physical_velocity_gate_closed=True,
                        )
                        if not post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_ODOMETRY_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate closed immediately "
                                "for bounded control-odometry recovery"
                            ),
                        ):
                            node.revoke()
                            active_decision = None
                            active_goal = None
                            odometry_recovery_leg_id = None
                            odometry_recovery_started_ns = 0
                            progress_watchdog.reset()
                            continue
                        last_feedback_monotonic = time.monotonic()
                else:
                    failed_decision = active_decision
                    node.revoke()
                    reason_code = (
                        "HEARTBEAT_DELIVERY_TIMEOUT"
                        if heartbeat_recovery_candidate
                        else (
                            "SENSOR_RECOVERY_TIMEOUT"
                            if combined_sensor_recovery_candidate
                            else (
                                "ODOMETRY_SLAM_RECOVERY_TIMEOUT"
                                if odometry_slam_recovery_candidate
                                else (
                                    "ODOMETRY_STALE_TIMEOUT"
                                    if odometry_recovery_candidate
                                    else (
                                        "OCCUPANCY_STALE_TIMEOUT"
                                        if (
                                            all_other_health_ready
                                            and not occupancy_fresh
                                        )
                                        else (
                                            "SLAM_TRANSIENT_TIMEOUT"
                                            if (
                                                slam_recovery_leg_id
                                                == failed_decision.leg_id
                                                and node.slam_detail
                                                in TRANSIENT_SLAM_FAILURES
                                            )
                                            else "HEALTH_NOT_READY"
                                        )
                                    )
                                )
                            )
                        )
                    )
                    emit(
                        "health_not_ready_local_hold",
                        decision_id=failed_decision.decision_id,
                        reason_code=reason_code,
                        detail=health.detail,
                        checks={
                            "heartbeat_delivery_ready": (
                                heartbeat_delivery_ready
                            ),
                            "heartbeat_recovery_elapsed_s": (
                                heartbeat_recovery_elapsed_s
                                if math.isfinite(
                                    heartbeat_recovery_elapsed_s
                                )
                                else None
                            ),
                            "heartbeat_recovery_grace_s": (
                                args.heartbeat_delivery_recovery_grace_s
                            ),
                            "local_fresh": local_fresh,
                            "slam_pass": node.slam_pass,
                            "slam_recovery_elapsed_s": (
                                slam_recovery_elapsed_s
                                if math.isfinite(slam_recovery_elapsed_s)
                                else None
                            ),
                            "slam_recovery_grace_s": (
                                args.slam_recovery_grace_s
                            ),
                            "combined_sensor_recovery_candidate": (
                                combined_sensor_recovery_candidate
                            ),
                            "odometry_fresh": odometry_fresh,
                            "odometry_recovery_elapsed_s": (
                                odometry_recovery_elapsed_s
                                if math.isfinite(
                                    odometry_recovery_elapsed_s
                                )
                                else None
                            ),
                            "odometry_recovery_grace_s": (
                                args.odometry_recovery_grace_s
                            ),
                            "odometry_slam_recovery_candidate": (
                                odometry_slam_recovery_candidate
                            ),
                            "odometry_slam_recovery_elapsed_s": (
                                odometry_slam_recovery_elapsed_s
                                if math.isfinite(
                                    odometry_slam_recovery_elapsed_s
                                )
                                else None
                            ),
                            "alignment_stable": alignment_stable,
                            "graph_ready": graph_ready,
                            "occupancy_fresh": occupancy_fresh,
                            "occupancy_age_s": (
                                occupancy_age_s
                                if math.isfinite(occupancy_age_s)
                                else None
                            ),
                            "cached_occupancy_motion_valid": (
                                cached_occupancy_motion_valid
                            ),
                            "cached_occupancy_motion_m": (
                                cached_occupancy_motion_m
                            ),
                            "max_cached_occupancy_motion_m": (
                                args.max_cached_occupancy_motion_m
                            ),
                            "occupancy_terminal_age_s": (
                                args.occupancy_data_timeout_s
                                + args.occupancy_recovery_grace_s
                            ),
                            "platform_fresh": platform_fresh,
                            "platform_pass": node.platform_pass,
                        },
                    )
                    post(
                        failed_decision,
                        NavigationStatusV2.REJECTED,
                        reason_code,
                        pose,
                        zero=True,
                        detail=health.detail,
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                    heartbeat_recovery_leg_id = None
                    heartbeat_recovery_started_ns = 0
                    occupancy_recovery_leg_id = None
                    occupancy_recovery_started_ns = 0
                    slam_recovery_leg_id = None
                    slam_recovery_started_ns = 0
                    odometry_recovery_leg_id = None
                    odometry_recovery_started_ns = 0
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
                router_recovery_leg_id = None
                router_recovery_started_ns = 0
                router_recovery_reason = ""
            (
                router_status_received_ns,
                router_state,
                router_reason,
                router_decision_id,
                router_affected_decision_id,
                _router_waypoint,
                _router_route_length_m,
            ) = node.router_status_snapshot()
            if (
                router_recovery_leg_id is not None
                and active_decision is not None
                and active_decision.leg_id == router_recovery_leg_id
                and now_ns < active_decision.expires_at_ns
                and router_status_received_ns >= goal_issued_ns
                and router_state == "NAVIGATING"
                and router_decision_id == active_decision.decision_id
            ):
                recovery_duration_s = (
                    now_ns - router_recovery_started_ns
                ) / 1e9
                node.resume_existing_goal(active_decision.expires_at_ns)
                emit(
                    "online_router_recovered",
                    state=router_state,
                    reason=router_reason,
                    previous_hold_reason=router_recovery_reason,
                    recovery_duration_s=round(recovery_duration_s, 3),
                    decision_id=active_decision.decision_id,
                )
                router_recovery_leg_id = None
                router_recovery_started_ns = 0
                router_recovery_reason = ""
                # Recovery deliberately closes the physical gate.  Give the
                # resumed local planner one complete progress window instead
                # of charging map-recovery time against motion progress.
                progress_watchdog.reset()
            if (
                args.online_buildmap_world
                and active_decision is not None
                and occupancy_recovery_leg_id != active_decision.leg_id
                and slam_recovery_leg_id != active_decision.leg_id
                and odometry_recovery_leg_id != active_decision.leg_id
                and router_status_received_ns >= goal_issued_ns
                and router_state == "HOLD"
                and (
                    router_decision_id == active_decision.decision_id
                    or router_affected_decision_id
                    == active_decision.decision_id
                )
            ):
                held_decision = active_decision
                no_path_waiting_for_replan = (
                    no_known_free_path_requires_replan(
                        (
                            None
                            if active_goal is None
                            else active_goal.target_kind
                        ),
                        router_reason,
                    )
                )
                if router_hold_recovery_eligible(
                    (
                        None
                        if active_goal is None
                        else active_goal.target_kind
                    ),
                    router_reason,
                    receiver_runtime_ready=ready,
                ):
                    if router_recovery_leg_id != held_decision.leg_id:
                        # Close only the independent physical velocity gate.
                        # Keep TinyNav unpaused and preserve the authority epoch
                        # so an overlapping sensor recovery can renew this exact
                        # leg without manufacturing new local authority.
                        node.close_router_recovery_gate()
                        router_recovery_leg_id = held_decision.leg_id
                        router_recovery_started_ns = now_ns
                        router_recovery_reason = router_reason
                        emit(
                            "online_router_recovery_wait",
                            state=router_state,
                            reason=router_reason,
                            grace_s=args.router_recovery_grace_s,
                            receiver_odom_age_s=round(odom_age_s, 3),
                            decision_id=held_decision.decision_id,
                            recovery_class=(
                                "known_free_map_maturation"
                                if no_path_waiting_for_replan
                                else "router_input_lag"
                            ),
                        )
                    recovery_age_s = (
                        now_ns - router_recovery_started_ns
                    ) / 1e9
                    if recovery_age_s > args.router_recovery_grace_s:
                        terminal_reason_code = (
                            "LOCAL_GOAL_UNREACHABLE"
                            if no_path_waiting_for_replan
                            else "LOCAL_ROUTER_HOLD_TIMEOUT"
                        )
                        node.revoke()
                        post(
                            held_decision,
                            NavigationStatusV2.REJECTED,
                            terminal_reason_code,
                            pose,
                            zero=True,
                            goal=(
                                active_goal
                                if no_path_waiting_for_replan
                                else None
                            ),
                            detail=(
                                f"online router state={router_state} "
                                f"reason={router_reason}; "
                                f"recovery_age_s={recovery_age_s:.3f}; "
                                "bounded fresh-map recovery exhausted"
                            ),
                            terminal=True,
                        )
                        emit(
                            (
                                "online_router_no_path_recovery_timeout"
                                if no_path_waiting_for_replan
                                else "online_router_recovery_timeout"
                            ),
                            state=router_state,
                            reason=router_reason,
                            recovery_age_s=round(recovery_age_s, 3),
                            decision_id=held_decision.decision_id,
                            target_kind=(
                                None
                                if active_goal is None
                                else active_goal.target_kind
                            ),
                        )
                        active_decision = None
                        active_goal = None
                        router_recovery_leg_id = None
                        router_recovery_started_ns = 0
                        router_recovery_reason = ""
                elif no_path_waiting_for_replan:
                    # If independent receiver health is not READY, no
                    # map-maturation wait is authorized. The physical gate is
                    # already closed, so reject without weakening local safety.
                    assert active_goal is not None
                    node.revoke()
                    post(
                        held_decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_GOAL_UNREACHABLE",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=(
                            "online router found no known-free path and "
                            "receiver health did not permit bounded recovery "
                            f"for {active_goal.target_kind}"
                        ),
                        terminal=True,
                    )
                    emit(
                        (
                            "frontier_no_path_rejected"
                            if active_goal.target_kind == "FRONTIER_POINT"
                            else "semantic_no_path_rejected"
                        ),
                        state=router_state,
                        reason=router_reason,
                        target_kind=active_goal.target_kind,
                        decision_id=held_decision.decision_id,
                        leg_id=held_decision.leg_id,
                        recovery_permitted=False,
                    )
                    active_decision = None
                    active_goal = None
                    progress_watchdog.reset()
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                else:
                    node.revoke()
                    post(
                        held_decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_ROUTER_HOLD",
                        pose,
                        zero=True,
                        detail=(
                            f"online router state={router_state} "
                            f"reason={router_reason}"
                        ),
                        terminal=True,
                    )
                    emit(
                        "online_router_local_hold",
                        state=router_state,
                        reason=router_reason,
                        decision_id=held_decision.decision_id,
                    )
                    active_decision = None
                    active_goal = None
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""

            try:
                decision = hub.latest_decision()
            except Exception as exc:  # noqa: BLE001 - disconnect revokes authority
                if active_decision is not None:
                    node.revoke()
                    active_decision = None
                    active_goal = None
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
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
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
                continue

            # Bind the path origin before every mode-specific fast path. An
            # episode that starts with coordination HOLD must not report
            # pre-episode drift and then reset on its first GOAL.
            path, path_episode_id = bind_path_to_episode(
                path,
                path_episode_id,
                decision.episode_id,
                pose[0],
                pose[1],
            )

            # HOLD and STOP are high-priority safety commands. They bypass all
            # sensor-recovery and goal-adaptation work, and a failed event POST
            # deliberately leaves the decision unseen so the next cycle retries
            # the zero-velocity acknowledgement.
            if (
                decision.mode.value in {"HOLD", "STOP"}
                and decision.decision_id != last_decision_id
            ):
                node.revoke()
                acknowledged = post(
                    decision,
                    (
                        NavigationStatusV2.HOLDING
                        if decision.mode.value == "HOLD"
                        else NavigationStatusV2.STOPPED
                    ),
                    "HUB_HOLD" if decision.mode.value == "HOLD" else "HUB_STOP",
                    pose,
                    zero=True,
                    detail=decision.reason,
                    terminal=True,
                )
                if acknowledged:
                    last_decision_id = decision.decision_id
                active_decision = None
                active_goal = None
                router_recovery_leg_id = None
                router_recovery_started_ns = 0
                router_recovery_reason = ""
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
                progress_watchdog.reset()
                time.sleep(
                    max(
                        0.0,
                        args.poll_s - (time.monotonic() - cycle_started),
                    )
                )
                continue

            active_combined_sensor_recovery = bool(
                occupancy_recovery_leg_id is not None
                and slam_recovery_leg_id is not None
            )
            active_occupancy_recovery = bool(
                occupancy_recovery_leg_id is not None
                and slam_recovery_leg_id is None
            )
            active_slam_recovery = bool(
                slam_recovery_leg_id is not None
                and occupancy_recovery_leg_id is None
            )
            active_odometry_recovery = bool(
                odometry_recovery_leg_id is not None
            )
            active_recovery_kind = closed_gate_recovery_kind(
                occupancy_recovery_active=active_occupancy_recovery,
                slam_recovery_active=active_slam_recovery,
                odometry_recovery_active=active_odometry_recovery,
                combined_sensor_recovery_active=(
                    active_combined_sensor_recovery
                ),
            )
            active_heartbeat_recovery = bool(
                heartbeat_recovery_leg_id is not None
            )
            if active_heartbeat_recovery:
                if active_recovery_kind is not None:
                    raise RuntimeError(
                        "heartbeat recovery cannot overlap robot-local "
                        "sensor recovery"
                    )
                active_recovery_kind = "heartbeat"
                closed_gate_recovery_leg_id = heartbeat_recovery_leg_id
            else:
                closed_gate_recovery_leg_id = (
                    occupancy_recovery_leg_id
                    if occupancy_recovery_leg_id is not None
                    else (
                        slam_recovery_leg_id
                        if slam_recovery_leg_id is not None
                        else odometry_recovery_leg_id
                    )
                )
            if (
                closed_gate_recovery_leg_id is not None
                and active_decision is not None
                and decision.mode.value == "GOAL"
                and decision.leg_id == closed_gate_recovery_leg_id
                and decision.decision_id != last_decision_id
            ):
                if active_recovery_kind == "heartbeat":
                    renewal_health = (
                        heartbeat_delivery_recovery_renewal_health(
                            health
                        )
                    )
                elif active_recovery_kind == "occupancy":
                    renewal_health = (
                        occupancy_recovery_renewal_health(health)
                    )
                elif active_recovery_kind == "slam":
                    renewal_health = slam_recovery_renewal_health(
                        health
                    )
                elif active_recovery_kind == "odometry":
                    renewal_health = odometry_recovery_renewal_health(
                        health
                    )
                else:
                    renewal_health = (
                        combined_sensor_recovery_renewal_health(
                            health
                        )
                    )
                renewal_result = adapter.evaluate(
                    decision,
                    now_ns=time.time_ns(),
                    health=renewal_health,
                    current_position_robot_map=(pose[0], pose[1], 0.0),
                )
                last_decision_id = decision.decision_id
                if (
                    renewal_result.action != V2AdapterAction.GOAL
                    or renewal_result.local_goal is None
                    or renewal_result.command_preview is None
                    or renewal_result.local_goal != active_goal
                ):
                    failed_decision = active_decision
                    node.revoke()
                    post(
                        failed_decision,
                        NavigationStatusV2.REJECTED,
                        "UNSAFE",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=(
                            f"same-leg {active_recovery_kind} recovery "
                            "renewal failed local "
                            f"validation: {renewal_result.reason_code}"
                        ),
                        terminal=True,
                    )
                    active_decision = None
                    active_goal = None
                    heartbeat_recovery_leg_id = None
                    heartbeat_recovery_started_ns = 0
                    occupancy_recovery_leg_id = None
                    occupancy_recovery_started_ns = 0
                    slam_recovery_leg_id = None
                    slam_recovery_started_ns = 0
                    odometry_recovery_leg_id = None
                    odometry_recovery_started_ns = 0
                    progress_watchdog.reset()
                    emit(
                        RECOVERY_RENEWAL_REJECTED_EVENTS[
                            active_recovery_kind
                        ],
                        pending_decision_id=decision.decision_id,
                        reason_code=renewal_result.reason_code,
                    )
                    time.sleep(
                        max(
                            0.0,
                            args.poll_s
                            - (time.monotonic() - cycle_started),
                        )
                    )
                    continue
                if not post(
                    decision,
                    NavigationStatusV2.RECEIVED,
                    "DECISION_RECEIVED",
                    pose,
                    detail=(
                        "authenticated same-leg renewal parsed during "
                        f"bounded {active_recovery_kind} recovery"
                    ),
                ):
                    node.revoke()
                    active_decision = None
                    active_goal = None
                    heartbeat_recovery_leg_id = None
                    heartbeat_recovery_started_ns = 0
                    occupancy_recovery_leg_id = None
                    occupancy_recovery_started_ns = 0
                    slam_recovery_leg_id = None
                    slam_recovery_started_ns = 0
                    odometry_recovery_leg_id = None
                    odometry_recovery_started_ns = 0
                    progress_watchdog.reset()
                    time.sleep(
                        max(
                            0.0,
                            args.poll_s
                            - (time.monotonic() - cycle_started),
                        )
                    )
                    continue
                try:
                    node.renew_authority_while_gate_closed(
                        decision.expires_at_ns
                    )
                except RuntimeError as exc:
                    # A local authority invariant must fail closed, but it must
                    # not kill the long-lived receiver and strand the Hub
                    # waiting for a HOLD acknowledgement.
                    node.revoke()
                    post(
                        decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_AUTHORITY_LOST",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=str(exc),
                        terminal=True,
                    )
                    emit(
                        "recovery_authority_lost_local_hold",
                        decision_id=decision.decision_id,
                        leg_id=decision.leg_id,
                        recovery_kind=active_recovery_kind,
                        error=str(exc),
                    )
                    active_decision = None
                    active_goal = None
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                    heartbeat_recovery_leg_id = None
                    heartbeat_recovery_started_ns = 0
                    occupancy_recovery_leg_id = None
                    occupancy_recovery_started_ns = 0
                    slam_recovery_leg_id = None
                    slam_recovery_started_ns = 0
                    odometry_recovery_leg_id = None
                    odometry_recovery_started_ns = 0
                    progress_watchdog.reset()
                    time.sleep(
                        max(
                            0.0,
                            args.poll_s
                            - (time.monotonic() - cycle_started),
                        )
                    )
                    continue
                node.publish_goal(
                    renewal_result.command_preview,
                    decision.expires_at_ns,
                    authorize_motion=False,
                )
                goal_issued_ns = time.time_ns()
                active_decision = decision
                active_goal = renewal_result.local_goal
                accepted = post(
                    decision,
                    NavigationStatusV2.ACCEPTED,
                    (
                        "LOCAL_HEARTBEAT_RECOVERY_WAIT"
                        if active_recovery_kind == "heartbeat"
                        else (
                            "LOCAL_OCCUPANCY_RECOVERY_WAIT"
                            if active_recovery_kind == "occupancy"
                            else (
                                "LOCAL_SLAM_RECOVERY_WAIT"
                                if active_recovery_kind == "slam"
                                else (
                                    "LOCAL_ODOMETRY_RECOVERY_WAIT"
                                    if active_recovery_kind
                                    == "odometry"
                                    else "LOCAL_SENSOR_RECOVERY_WAIT"
                                )
                            )
                        )
                    ),
                    pose,
                    zero=True,
                    goal=active_goal,
                    detail=(
                        "same-leg lease renewed while the physical velocity "
                        "gate remains closed for bounded "
                        f"{active_recovery_kind} recovery"
                    ),
                )
                if not accepted:
                    node.revoke()
                    active_decision = None
                    active_goal = None
                    heartbeat_recovery_leg_id = None
                    heartbeat_recovery_started_ns = 0
                    occupancy_recovery_leg_id = None
                    occupancy_recovery_started_ns = 0
                    slam_recovery_leg_id = None
                    slam_recovery_started_ns = 0
                    odometry_recovery_leg_id = None
                    odometry_recovery_started_ns = 0
                    progress_watchdog.reset()
                    emit(
                        RECOVERY_RENEWAL_FEEDBACK_FAILED_EVENTS[
                            active_recovery_kind
                        ],
                        decision_id=decision.decision_id,
                        leg_id=decision.leg_id,
                    )
                else:
                    last_feedback_monotonic = time.monotonic()
                    emit(
                        RECOVERY_LEASE_RENEWED_EVENTS[
                            active_recovery_kind
                        ],
                        decision_id=decision.decision_id,
                        leg_id=decision.leg_id,
                        lease_sequence=decision.lease_sequence,
                        physical_velocity_gate_closed=True,
                    )
                time.sleep(
                    max(
                        0.0,
                        args.poll_s
                        - (time.monotonic() - cycle_started),
                    )
                )
                continue

            goal_published_this_cycle = False
            if decision.decision_id != last_decision_id:
                last_decision_id = decision.decision_id
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
                            progress_watchdog.reset()
                            router_recovery_leg_id = None
                            router_recovery_started_ns = 0
                            router_recovery_reason = ""
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
                        recovery_pending_for_leg = (
                            same_leg
                            and router_recovery_leg_id == decision.leg_id
                        )
                        node.publish_goal(
                            result.command_preview,
                            decision.expires_at_ns,
                            authorize_motion=not recovery_pending_for_leg,
                        )
                        goal_published_this_cycle = True
                        goal_issued_ns = time.time_ns()
                        if not same_leg:
                            remaining_m = max(
                                0.0,
                                math.hypot(
                                    pose[0] - result.local_goal.x,
                                    pose[1] - result.local_goal.y,
                                )
                                - (result.local_goal.arrival_radius_m or 0.0),
                            )
                            progress_watchdog.observe(
                                leg_id=decision.leg_id,
                                remaining_m=remaining_m,
                                now_monotonic=time.monotonic(),
                                position_xy=(pose[0], pose[1]),
                            )
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
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
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
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
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
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""

            if active_decision is not None:
                (
                    trajectory_fresh,
                    trajectory_failed,
                    trajectory_age_s,
                    trajectory_observed_for_authority,
                ) = trajectory_gate_state(
                    now_ns=now_ns,
                    authority_started_ns=node.authority_started_ns,
                    trajectory_received_ns=node.trajectory_received_ns,
                    stale_timeout_s=args.trajectory_stale_timeout_s,
                    start_grace_s=args.trajectory_start_grace_s,
                    recovery_timeout_s=args.trajectory_recovery_timeout_s,
                )
                (
                    collision_gate_closed,
                    collision_terminal,
                    collision_age_s,
                    collision_observed_for_authority,
                ) = node.planner_collision_state(now_ns)
                (
                    _refresh_router_received_ns,
                    refresh_router_state,
                    refresh_router_reason,
                    refresh_router_decision_id,
                    _refresh_router_affected_decision_id,
                    refresh_router_waypoint,
                    _refresh_router_route_length_m,
                ) = node.router_status_snapshot()
                refresh_attempt: int | None = None
                if (
                    now_ns < active_decision.expires_at_ns
                    and planner_target_refresh_eligible(
                        authorized=node.authorized,
                        router_recovery_gate_closed=(
                            node.router_recovery_gate_closed
                        ),
                        trajectory_fresh=trajectory_fresh,
                        trajectory_failed=trajectory_failed,
                        trajectory_age_s=trajectory_age_s,
                        trajectory_stale_timeout_s=(
                            args.trajectory_stale_timeout_s
                        ),
                        router_state=refresh_router_state,
                        router_reason=refresh_router_reason,
                        router_decision_id=refresh_router_decision_id,
                        active_decision_id=active_decision.decision_id,
                        router_waypoint=refresh_router_waypoint,
                        all_candidates_in_collision=(
                            collision_gate_closed
                        ),
                    )
                ):
                    assert refresh_router_waypoint is not None
                    refresh_attempt = node.request_planner_target_refresh(
                        decision_id=active_decision.decision_id,
                        path_age_s=trajectory_age_s,
                        router_waypoint=refresh_router_waypoint,
                        all_candidates_in_collision=(
                            collision_gate_closed
                        ),
                    )
                    if refresh_attempt is not None:
                        emit(
                            "local_planner_target_refresh_requested",
                            decision_id=active_decision.decision_id,
                            leg_id=active_decision.leg_id,
                            request_attempt=refresh_attempt,
                            request_topic=(
                                args.target_refresh_request_topic
                            ),
                            path_age_s=round(trajectory_age_s, 3),
                            router_state=refresh_router_state,
                            router_reason=refresh_router_reason,
                            router_waypoint=list(
                                refresh_router_waypoint
                            ),
                            velocity_gate=(
                                "all_trajectories_in_collision"
                                if collision_gate_closed
                                else "trajectory_missing_or_stale"
                            ),
                        )
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
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                elif (
                    node.authorized
                    and collision_terminal
                    and refresh_attempt is None
                    # A router-owned recovery has its own bounded terminal
                    # verdict and already keeps physical velocity at zero.
                    and router_recovery_leg_id
                    != active_decision.leg_id
                ):
                    failed_decision = active_decision
                    node.revoke()
                    post(
                        failed_decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_GOAL_UNREACHABLE",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=(
                            "TinyNav continuously scored every local "
                            "trajectory candidate as collision; "
                            f"collision_age_s={collision_age_s:.3f}; "
                            "finite_candidates="
                            f"{node.planner_finite_candidate_count}/"
                            f"{node.planner_candidate_count}"
                        ),
                        terminal=True,
                    )
                    emit(
                        "local_planner_all_candidates_in_collision",
                        decision_id=failed_decision.decision_id,
                        leg_id=failed_decision.leg_id,
                        collision_age_s=round(collision_age_s, 3),
                        collision_observed_for_authority=(
                            collision_observed_for_authority
                        ),
                        candidate_count=node.planner_candidate_count,
                        finite_candidate_count=(
                            node.planner_finite_candidate_count
                        ),
                        finite_in_place_candidate_count=(
                            node.planner_finite_in_place_candidate_count
                        ),
                        physical_velocity_gate_closed=True,
                    )
                    active_decision = None
                    active_goal = None
                    progress_watchdog.reset()
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                elif (
                    args.reject_stalled_turn
                    and node.turn_stalled_for_authority
                ):
                    failed_decision = active_decision
                    node.revoke()
                    post(
                        failed_decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_PLANNER_TURN_STALLED",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=(
                            "TinyNav continuous in-place heading recovery "
                            "exceeded its robot-local bounded deadline"
                        ),
                        terminal=True,
                    )
                    emit(
                        "local_planner_turn_stalled",
                        decision_id=failed_decision.decision_id,
                        leg_id=failed_decision.leg_id,
                        turn_stalled_received_ns=(
                            node.turn_stalled_received_ns
                        ),
                        trajectory_pose_count=node.trajectory_pose_count,
                        trajectory_first_xy=node.trajectory_first_xy,
                        trajectory_lookahead_xy=(
                            node.trajectory_lookahead_xy
                        ),
                        latest_raw_cmd=list(node.latest_raw_cmd),
                    )
                    active_decision = None
                    active_goal = None
                    progress_watchdog.reset()
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                elif (
                    args.reject_reverse_trajectory
                    and node.reverse_required_for_authority
                ):
                    failed_decision = active_decision
                    node.revoke()
                    post(
                        failed_decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_PATH_REVERSE_REQUIRED",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=(
                            "TinyNav control lookahead requires reverse "
                            "motion, which this forward-only controller rejects"
                        ),
                        terminal=True,
                    )
                    emit(
                        "local_path_reverse_required",
                        decision_id=failed_decision.decision_id,
                        leg_id=failed_decision.leg_id,
                        reverse_required_received_ns=(
                            node.reverse_required_received_ns
                        ),
                        trajectory_pose_count=node.trajectory_pose_count,
                        trajectory_first_xy=node.trajectory_first_xy,
                        trajectory_lookahead_xy=(
                            node.trajectory_lookahead_xy
                        ),
                        latest_raw_cmd=list(node.latest_raw_cmd),
                    )
                    active_decision = None
                    active_goal = None
                    progress_watchdog.reset()
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
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
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                elif (
                    # Once the bounded republish window expires, a missing
                    # collision-free local path is explicit robot-local
                    # infeasibility for this fixed leg, regardless of whether
                    # its target is semantic or a frontier.  Rejecting the leg
                    # lets the Hub HOLD and start a fresh source/VLM round
                    # immediately; reporting ACCEPTED for a blocked frontier
                    # would incorrectly consume the entire 24/25-tick window
                    # while the physical gate is already closed.
                    node.authorized
                    and node.authority_started_ns > 0
                    and trajectory_failed
                    # A router-owned recovery already keeps physical output
                    # at zero and has its own bounded terminal verdict. Do not
                    # let the shorter trajectory timer preempt that state.
                    and router_recovery_leg_id
                    != active_decision.leg_id
                ):
                    failed_decision = active_decision
                    failure_timeout_s = (
                        args.trajectory_recovery_timeout_s
                        if trajectory_observed_for_authority
                        else args.trajectory_start_grace_s
                    )
                    node.revoke()
                    post(
                        failed_decision,
                        NavigationStatusV2.REJECTED,
                        "LOCAL_PLANNER_PATH_STALE",
                        pose,
                        zero=True,
                        goal=active_goal,
                        detail=(
                            "TinyNav produced no fresh non-empty collision-free "
                            "trajectory; "
                            f"path_age_s={trajectory_age_s:.3f}; "
                            f"failure_timeout_s={failure_timeout_s:.3f}; "
                            f"router={node.router_state}/{node.router_reason}"
                        ),
                        terminal=True,
                    )
                    emit(
                        "local_planner_path_stale",
                        decision_id=failed_decision.decision_id,
                        leg_id=failed_decision.leg_id,
                        path_age_s=trajectory_age_s,
                        failure_timeout_s=failure_timeout_s,
                        trajectory_observed_for_authority=(
                            trajectory_observed_for_authority
                        ),
                        router_state=node.router_state,
                        router_reason=node.router_reason,
                        router_waypoint=node.router_waypoint,
                        latest_raw_cmd=list(node.latest_raw_cmd),
                    )
                    active_decision = None
                    active_goal = None
                    progress_watchdog.reset()
                    router_recovery_leg_id = None
                    router_recovery_started_ns = 0
                    router_recovery_reason = ""
                elif (
                    occupancy_recovery_leg_id
                    == active_decision.leg_id
                    and slam_recovery_leg_id
                    == active_decision.leg_id
                ):
                    if (
                        time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_SENSOR_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate is closed while "
                                "waiting for fresh occupancy and the next "
                                "healthy SLAM report; "
                                f"occupancy_age_s={occupancy_age_s:.3f}; "
                                f"slam_detail={node.slam_detail}; "
                                "occupancy_recovery_elapsed_s="
                                f"{occupancy_recovery_elapsed_s:.3f}; "
                                "slam_recovery_elapsed_s="
                                f"{slam_recovery_elapsed_s:.3f}"
                            ),
                        )
                        last_feedback_monotonic = time.monotonic()
                elif occupancy_recovery_leg_id == active_decision.leg_id:
                    # Occupancy freshness has crossed the hard motion bound,
                    # so the 20 Hz gate is already publishing zero. Preserve
                    # the high-level leg only for the explicit bounded grace.
                    if (
                        time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_OCCUPANCY_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate is closed while "
                                "waiting for a fresh occupancy publication; "
                                f"occupancy_age_s={occupancy_age_s:.3f}; "
                                "recovery_elapsed_s="
                                f"{occupancy_recovery_elapsed_s:.3f}; "
                                "recovery_grace_s="
                                f"{args.occupancy_recovery_grace_s:.3f}"
                            ),
                        )
                        last_feedback_monotonic = time.monotonic()
                elif slam_recovery_leg_id == active_decision.leg_id:
                    if (
                        time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_SLAM_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate is closed while "
                                "waiting for the next healthy SLAM report; "
                                f"slam_detail={node.slam_detail}; "
                                "recovery_elapsed_s="
                                f"{slam_recovery_elapsed_s:.3f}; "
                                "recovery_grace_s="
                                f"{args.slam_recovery_grace_s:.3f}"
                            ),
                        )
                        last_feedback_monotonic = time.monotonic()
                elif (
                    odometry_recovery_leg_id
                    == active_decision.leg_id
                ):
                    if (
                        time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_ODOMETRY_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate is closed while "
                                "waiting for fresh control odometry; "
                                f"odometry_age_s={odom_age_s:.3f}; "
                                f"slam_detail={node.slam_detail}; "
                                "transient_slam_recovery_elapsed_s="
                                + (
                                    f"{odometry_slam_recovery_elapsed_s:.3f}"
                                    if math.isfinite(
                                        odometry_slam_recovery_elapsed_s
                                    )
                                    else "not_active"
                                )
                                + "; "
                                "recovery_elapsed_s="
                                f"{odometry_recovery_elapsed_s:.3f}; "
                                "recovery_grace_s="
                                f"{args.odometry_recovery_grace_s:.3f}"
                            ),
                        )
                        last_feedback_monotonic = time.monotonic()
                elif router_recovery_leg_id == active_decision.leg_id:
                    # The physical velocity gate is closed during bounded
                    # online-map recovery, so this must never be reported as
                    # NAVIGATING.  Still emit fresh ACCEPTED receipts: the Hub
                    # lease monitor otherwise mistakes the intentionally quiet
                    # recovery window for a dead receiver before the local
                    # router's stricter recovery timeout can decide the leg.
                    if (
                        time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        post(
                            active_decision,
                            NavigationStatusV2.ACCEPTED,
                            "LOCAL_ROUTER_RECOVERY_WAIT",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "physical velocity gate is closed while the "
                                "online map catches up; "
                                f"router={node.router_state}/"
                                f"{node.router_reason}"
                            ),
                        )
                        last_feedback_monotonic = time.monotonic()
                elif not goal_published_this_cycle:
                    remaining_m = max(
                        0.0,
                        math.hypot(
                            pose[0] - active_goal.x,
                            pose[1] - active_goal.y,
                        )
                        - (active_goal.arrival_radius_m or 0.0),
                    )
                    stalled, stalled_s = progress_watchdog.observe(
                        leg_id=active_decision.leg_id,
                        remaining_m=remaining_m,
                        now_monotonic=time.monotonic(),
                        position_xy=(pose[0], pose[1]),
                    )
                    if stalled:
                        stalled_decision = active_decision
                        node.revoke()
                        post(
                            stalled_decision,
                            NavigationStatusV2.REJECTED,
                            "LOCAL_PLANNER_NO_PROGRESS",
                            pose,
                            zero=True,
                            goal=active_goal,
                            detail=(
                                "fixed local goal remaining distance did not "
                                f"improve by {args.minimum_goal_progress_m:.3f}m "
                                f"for {stalled_s:.3f}s; "
                                f"remaining_m={remaining_m:.3f}"
                            ),
                            terminal=True,
                        )
                        emit(
                            "local_planner_no_progress",
                            decision_id=stalled_decision.decision_id,
                            leg_id=stalled_decision.leg_id,
                            stalled_s=round(stalled_s, 3),
                            remaining_m=round(remaining_m, 3),
                            goal_distance_m=round(
                                math.hypot(
                                    pose[0] - active_goal.x,
                                    pose[1] - active_goal.y,
                                ),
                                3,
                            ),
                            arrival_radius_m=round(
                                active_goal.arrival_radius_m or 0.0,
                                3,
                            ),
                            local_pose_xy=[
                                round(pose[0], 3),
                                round(pose[1], 3),
                            ],
                        )
                        active_decision = None
                        active_goal = None
                        progress_watchdog.reset()
                        router_recovery_leg_id = None
                        router_recovery_started_ns = 0
                        router_recovery_reason = ""
                    elif (
                        time.monotonic() - last_feedback_monotonic >= 0.5
                    ):
                        planner_active = bool(
                            node.authorized
                            and trajectory_fresh
                        )
                        emit(
                            "control_telemetry",
                            decision_id=active_decision.decision_id,
                            local_pose=[
                                round(pose[0], 4),
                                round(pose[1], 4),
                                round(pose[2], 4),
                            ],
                            local_goal=[
                                round(active_goal.x, 4),
                                round(active_goal.y, 4),
                            ],
                            router_state=node.router_state,
                            router_reason=node.router_reason,
                            router_waypoint=node.router_waypoint,
                            router_route_length_m=node.router_route_length_m,
                            trajectory_pose_count=node.trajectory_pose_count,
                            trajectory_first_xy=node.trajectory_first_xy,
                            trajectory_lookahead_xy=(
                                node.trajectory_lookahead_xy
                            ),
                            reverse_required=(
                                node.reverse_required_for_authority
                            ),
                            turn_stalled=(
                                node.turn_stalled_for_authority
                            ),
                            raw_cmd=list(node.latest_raw_cmd),
                            guard_reason=node.latest_guard_reason,
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
                                "fresh non-empty TinyNav trajectory observed"
                                if planner_active
                                else (
                                    "physical velocity gate is closed while "
                                    "waiting for a fresh collision-free "
                                    "TinyNav trajectory"
                                )
                            ),
                        )
                        last_feedback_monotonic = time.monotonic()
            else:
                progress_watchdog.reset()
                occupancy_recovery_leg_id = None
                occupancy_recovery_started_ns = 0
                slam_recovery_leg_id = None
                slam_recovery_started_ns = 0
                odometry_recovery_leg_id = None
                odometry_recovery_started_ns = 0
            time.sleep(max(0.0, args.poll_s - (time.monotonic() - cycle_started)))
    except KeyboardInterrupt:
        node.revoke()
        emit("receiver_stopped", reason="operator_interrupt")
    except Exception as exc:  # noqa: BLE001 - any receiver fault revokes motion
        exit_code = 4
        node.revoke()
        emit("receiver_fault", error=str(exc)[:1000])
    finally:
        if heartbeat_pump is not None:
            heartbeat_pump.close(timeout_s=2.0)
        ros_executor.shutdown(timeout_sec=2.0)
        ros_spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
