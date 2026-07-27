#!/usr/bin/env python3
"""Deployment wrapper for TinyNav's pinned velocity controller.

The pinned TinyNav controller exposes ``linear_engage_threshold`` but compares
the requested speed against ``min_effective_linear_speed`` instead.  A valid
short trajectory segment can therefore request, for example, 0.078 m/s: above
the intended 0.04 m/s engage threshold but below the 0.10 m/s static-friction
floor.  Its timer then publishes zero forever.

The pinned controller already forbids negative velocity and turns in place
when its lookahead lies behind the robot.  The Focus deployment normally keeps
the stricter reverse-path rejection at 2 cm.  Yunji can explicitly opt into a
bounded rotate-first recovery: publish zero linear velocity, retain one turn
direction, and let the pinned pose/path/depth guards govern the yaw command.
It can also explicitly stabilize a large in-place turn from the current base
pose toward a non-local path point.  That prevents a jittering first path
segment from changing the turn sign on every replan.  If either recovery does
not resolve before the bounded deadline, the existing receiver rejection
remains fail-closed.

Keep the source controller immutable and preserve all of its stale-pose,
stale-path, depth-stop, turn, acceleration and arrival guards.  This deployment
subclass is shared by Yunji and WSJ, while rotate-first remains an explicit
Yunji launcher option.
"""
from __future__ import annotations

import argparse
import logging
import math
import time


MEANINGFUL_REVERSE_SEGMENT_M = 0.02
DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS = 0.35
DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS = 0.10
DEFAULT_ROTATE_FIRST_TIMEOUT_S = 12.0
DEFAULT_STABLE_TURN_LOOKAHEAD_M = 0.35
DEFAULT_STABLE_TURN_MIN_TARGET_M = 0.10
DEFAULT_STABLE_TURN_ENTER_RAD = math.radians(75.0)
DEFAULT_STABLE_TURN_EXIT_RAD = math.radians(35.0)


def path_segment_forward_component(
    first_xy: tuple[float, float],
    second_xy: tuple[float, float],
    *,
    robot_heading_rad: float,
) -> float:
    """Project the first path segment onto the predicted robot forward axis."""

    values = (*first_xy, *second_xy, robot_heading_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("trajectory segment values must be finite")
    dx = second_xy[0] - first_xy[0]
    dy = second_xy[1] - first_xy[1]
    return dx * math.cos(robot_heading_rad) + dy * math.sin(
        robot_heading_rad
    )


def apply_linear_engagement_floor(
    requested_mps: float,
    *,
    engage_threshold_mps: float,
    minimum_effective_mps: float,
) -> float:
    """Raise an intentional small forward command to the executable floor."""

    values = (
        requested_mps,
        engage_threshold_mps,
        minimum_effective_mps,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity thresholds must be finite")
    if (
        engage_threshold_mps < 0.0
        or minimum_effective_mps <= 0.0
        or engage_threshold_mps > minimum_effective_mps
    ):
        raise ValueError("invalid velocity engagement thresholds")
    if engage_threshold_mps <= requested_mps < minimum_effective_mps:
        return minimum_effective_mps
    return requested_mps


def classify_forward_component(
    forward_m: float | None,
    *,
    meaningful_reverse_m: float = MEANINGFUL_REVERSE_SEGMENT_M,
) -> str:
    """Classify a segment before TinyNav quantizes reverse to -0.2 m/s."""

    if not math.isfinite(meaningful_reverse_m) or meaningful_reverse_m <= 0.0:
        raise ValueError("meaningful_reverse_m must be finite and positive")
    if forward_m is None:
        return "unknown"
    if not math.isfinite(forward_m):
        raise ValueError("forward_m must be finite")
    if forward_m < -meaningful_reverse_m:
        return "reject_reverse"
    if forward_m < 0.0:
        return "zero_tiny_reverse"
    return "allow"


def bounded_rotate_first_angular(
    requested_radps: float,
    *,
    latched_direction: int,
    minimum_radps: float = DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS,
    maximum_radps: float = DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS,
) -> float:
    """Return a non-reversing yaw command with a stable turn direction.

    ``requested_radps`` comes from the pinned controller after all of its
    trajectory geometry and arrival handling.  A zero request remains zero so
    this wrapper cannot invent rotation after the pinned controller has
    stopped.  Once recovery begins, ``latched_direction`` prevents a
    near-180-degree heading error from switching sign on successive replans.
    """

    values = (requested_radps, minimum_radps, maximum_radps)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("rotate-first angular values must be finite")
    if minimum_radps <= 0.0 or maximum_radps < minimum_radps:
        raise ValueError("invalid rotate-first angular bounds")
    if latched_direction not in {-1, 0, 1}:
        raise ValueError("latched_direction must be -1, 0 or 1")
    if requested_radps == 0.0:
        return 0.0
    direction = (
        latched_direction
        if latched_direction
        else (1 if requested_radps > 0.0 else -1)
    )
    magnitude = min(
        maximum_radps,
        max(minimum_radps, abs(requested_radps)),
    )
    return float(direction * magnitude)


def stable_path_heading_error(
    robot_xy: tuple[float, float],
    *,
    robot_heading_rad: float,
    path_xy: list[tuple[float, float]],
    lookahead_m: float = DEFAULT_STABLE_TURN_LOOKAHEAD_M,
    minimum_target_m: float = DEFAULT_STABLE_TURN_MIN_TARGET_M,
) -> float | None:
    """Return a stable base-frame heading error to a non-local path point.

    TinyNav's pinned controller derives its large-turn sign from the first
    short path segment.  A replanner can legitimately move that segment across
    the base axis on successive callbacks.  Selecting the first point at least
    ``lookahead_m`` from the *current base pose* keeps the route's local shape
    while filtering that near-pose jitter.  A shorter path uses its farthest
    point, provided it is still geometrically meaningful.
    """

    values = (*robot_xy, robot_heading_rad, lookahead_m, minimum_target_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("stable-turn geometry values must be finite")
    if lookahead_m <= 0.0 or minimum_target_m <= 0.0:
        raise ValueError("stable-turn distances must be positive")
    if minimum_target_m > lookahead_m:
        raise ValueError("minimum_target_m cannot exceed lookahead_m")

    chosen: tuple[float, float] | None = None
    farthest: tuple[float, float] | None = None
    farthest_distance = -1.0
    for point in path_xy:
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError("stable-turn path points must be finite XY pairs")
        dx = point[0] - robot_xy[0]
        dy = point[1] - robot_xy[1]
        distance = math.hypot(dx, dy)
        if distance > farthest_distance:
            farthest = point
            farthest_distance = distance
        if distance >= lookahead_m:
            chosen = point
            break
    if chosen is None:
        if farthest is None or farthest_distance < minimum_target_m:
            return None
        chosen = farthest

    world_bearing = math.atan2(
        chosen[1] - robot_xy[1],
        chosen[0] - robot_xy[0],
    )
    return math.atan2(
        math.sin(world_bearing - robot_heading_rad),
        math.cos(world_bearing - robot_heading_rad),
    )


def large_turn_stabilization_required(
    heading_error_rad: float | None,
    *,
    recovery_active: bool,
    requested_linear_mps: float,
    requested_angular_radps: float,
    enter_rad: float = DEFAULT_STABLE_TURN_ENTER_RAD,
    exit_rad: float = DEFAULT_STABLE_TURN_EXIT_RAD,
) -> bool:
    """Return whether an existing pinned in-place turn needs sign latching."""

    values = (
        requested_linear_mps,
        requested_angular_radps,
        enter_rad,
        exit_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("stable-turn command values must be finite")
    if heading_error_rad is not None and not math.isfinite(heading_error_rad):
        raise ValueError("stable-turn heading error must be finite")
    if not 0.0 < exit_rad < enter_rad <= math.pi:
        raise ValueError("stable-turn angular thresholds are invalid")
    if heading_error_rad is None:
        return False
    if abs(requested_linear_mps) > 1e-9:
        return False
    if abs(requested_angular_radps) <= 1e-9:
        return False
    threshold = exit_rad if recovery_active else enter_rad
    return abs(heading_error_rad) >= threshold


def tiny_reverse_recovery_continuation_required(
    segment_action: str,
    heading_error_rad: float | None,
    *,
    recovery_active: bool,
    rotate_first_enabled: bool,
    paused: bool,
    exit_rad: float = DEFAULT_STABLE_TURN_EXIT_RAD,
) -> bool:
    """Keep an existing turn alive across the near-zero reverse deadband."""

    if segment_action not in {
        "unknown",
        "reject_reverse",
        "zero_tiny_reverse",
        "allow",
    }:
        raise ValueError("unknown forward-component classification")
    if heading_error_rad is not None and not math.isfinite(heading_error_rad):
        raise ValueError("continuation heading error must be finite")
    if not math.isfinite(exit_rad) or not 0.0 < exit_rad <= math.pi:
        raise ValueError("continuation exit angle is invalid")
    return bool(
        recovery_active
        and rotate_first_enabled
        and not paused
        and segment_action == "zero_tiny_reverse"
        and heading_error_rad is not None
        and abs(heading_error_rad) >= exit_rad
    )


def rotate_first_continuation_request(
    requested_radps: float,
    *,
    continuation_required: bool,
    latched_direction: int,
    minimum_radps: float = DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS,
) -> float:
    """Preserve an already-authorized turn when pinned output crosses zero."""

    if not all(
        math.isfinite(value) for value in (requested_radps, minimum_radps)
    ):
        raise ValueError("continuation angular values must be finite")
    if minimum_radps <= 0.0:
        raise ValueError("continuation minimum angular speed must be positive")
    if latched_direction not in {-1, 0, 1}:
        raise ValueError("latched_direction must be -1, 0 or 1")
    if (
        continuation_required
        and requested_radps == 0.0
        and latched_direction == 0
    ):
        raise ValueError("active continuation requires a latched direction")
    if continuation_required and requested_radps == 0.0:
        return float(latched_direction * minimum_radps)
    return requested_radps


def reverse_recovery_expired(
    *,
    started_monotonic: float,
    now_monotonic: float,
    timeout_s: float,
) -> bool:
    """Return whether one continuous rotate-first recovery exceeded its lease."""

    values = (started_monotonic, now_monotonic, timeout_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("reverse recovery timing values must be finite")
    if started_monotonic < 0.0 or now_monotonic < started_monotonic:
        raise ValueError("invalid reverse recovery monotonic timestamps")
    if timeout_s <= 0.0:
        raise ValueError("reverse recovery timeout must be positive")
    return now_monotonic - started_monotonic >= timeout_s


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rotate-first-on-reverse",
        action="store_true",
        help=(
            "replace a meaningful reverse lookahead with bounded in-place "
            "turning; intended for the measured Yunji deployment only"
        ),
    )
    parser.add_argument(
        "--stabilize-large-turn",
        action="store_true",
        help=(
            "latch the sign of an existing large in-place turn using the "
            "current base pose and a non-local path point; intended for the "
            "measured Yunji deployment only"
        ),
    )
    parser.add_argument(
        "--rotate-first-max-angular-radps",
        type=float,
        default=DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS,
    )
    parser.add_argument(
        "--rotate-first-timeout-s",
        type=float,
        default=DEFAULT_ROTATE_FIRST_TIMEOUT_S,
    )
    return parser


def main(args: list[str] | None = None) -> None:
    deployment_args, ros_args = build_parser().parse_known_args(args)
    if (
        not math.isfinite(deployment_args.rotate_first_max_angular_radps)
        or deployment_args.rotate_first_max_angular_radps
        < DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS
    ):
        raise SystemExit(
            "--rotate-first-max-angular-radps must be finite and at least "
            f"{DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS:.2f}"
        )
    if (
        not math.isfinite(deployment_args.rotate_first_timeout_s)
        or deployment_args.rotate_first_timeout_s <= 0.0
    ):
        raise SystemExit(
            "--rotate-first-timeout-s must be finite and positive"
        )

    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist
    from scipy.spatial.transform import Rotation as R
    from std_msgs.msg import Bool
    from tinynav.platforms.cmd_vel_control import CmdVelControlNode

    class FocusCmdVelControlNode(CmdVelControlNode):
        def __init__(self) -> None:
            super().__init__()
            self._last_focus_speed_floor_log = 0.0
            self._last_focus_reverse_log = 0.0
            self._last_focus_rotate_first_log = 0.0
            self._last_focus_tiny_reverse_log = 0.0
            self._focus_rotation_recovery_started: float | None = None
            self._focus_rotation_turn_direction = 0
            self._reverse_required_publisher = self.create_publisher(
                Bool, "/planning/reverse_required", 10
            )

        def _reset_focus_rotation_recovery(self) -> None:
            self._focus_rotation_recovery_started = None
            self._focus_rotation_turn_direction = 0

        def _on_paused(self, message) -> None:
            super()._on_paused(message)
            if bool(message.data):
                self._reset_focus_rotation_recovery()

        @staticmethod
        def _raw_pose_matrix(pose) -> np.ndarray:
            quaternion = pose.orientation
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = R.from_quat(
                [
                    float(quaternion.x),
                    float(quaternion.y),
                    float(quaternion.z),
                    float(quaternion.w),
                ]
            ).as_matrix()
            matrix[:3, 3] = [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ]
            return matrix

        @classmethod
        def _pose_matrix(cls, pose_stamped) -> np.ndarray:
            return cls._raw_pose_matrix(pose_stamped.pose)

        def _control_segment_forward_m(self, message) -> float:
            first = message.poses[0]
            step_index = int(
                min(self.lookahead_steps, len(message.poses) - 1)
            )
            second = message.poses[step_index]
            first_robot = self._pose_matrix(first) @ self.T_robot_to_camera
            second_robot = self._pose_matrix(second) @ self.T_robot_to_camera
            # This is intentionally byte-for-byte equivalent in geometry to
            # the pinned controller's raw_vx calculation:
            # inv(T_robot_1) @ T_robot_2, translation X.  Testing only the
            # world-frame camera-point delta would miss the camera-to-base
            # lever arm and could disagree with the command actually rejected.
            relative = np.linalg.inv(first_robot) @ second_robot
            forward_m = float(relative[0, 3])
            if not math.isfinite(forward_m):
                raise ValueError(
                    "controller trajectory forward component is not finite"
                )
            return forward_m

        def _stable_path_heading_error(self, message) -> float | None:
            current_camera = self._raw_pose_matrix(self.pose.pose.pose)
            current_robot = current_camera @ self.T_robot_to_camera
            path_robot_xy = []
            for pose_stamped in message.poses:
                path_robot = (
                    self._pose_matrix(pose_stamped) @ self.T_robot_to_camera
                )
                path_robot_xy.append(
                    (
                        float(path_robot[0, 3]),
                        float(path_robot[1, 3]),
                    )
                )
            return stable_path_heading_error(
                (
                    float(current_robot[0, 3]),
                    float(current_robot[1, 3]),
                ),
                robot_heading_rad=math.atan2(
                    float(current_robot[1, 0]),
                    float(current_robot[0, 0]),
                ),
                path_xy=path_robot_xy,
            )

        def path_callback(self, message) -> None:
            control_segment_forward_m = None
            stable_heading_error_rad = None
            if message is not None and len(message.poses) >= 2:
                try:
                    control_segment_forward_m = (
                        self._control_segment_forward_m(message)
                    )
                    if self.pose is not None:
                        stable_heading_error_rad = (
                            self._stable_path_heading_error(message)
                        )
                except (
                    AttributeError,
                    TypeError,
                    ValueError,
                    np.linalg.LinAlgError,
                ):
                    control_segment_forward_m = None
                    stable_heading_error_rad = None
            super().path_callback(message)
            if (
                message is None
                or self.pose is None
                or len(message.poses) < 2
            ):
                self._reset_focus_rotation_recovery()
                return
            segment_action = classify_forward_component(
                control_segment_forward_m
            )
            requested_linear = float(self.latest_cmd.linear.x)
            requested_angular = float(self.latest_cmd.angular.z)
            now = time.monotonic()
            recovery_active = (
                self._focus_rotation_recovery_started is not None
            )
            reverse_recovery_requested = bool(
                segment_action == "reject_reverse"
                and deployment_args.rotate_first_on_reverse
                and not self._paused
            )
            large_turn_recovery_requested = bool(
                deployment_args.stabilize_large_turn
                and not self._paused
                and large_turn_stabilization_required(
                    stable_heading_error_rad,
                    recovery_active=recovery_active,
                    requested_linear_mps=requested_linear,
                    requested_angular_radps=requested_angular,
                )
            )
            tiny_reverse_recovery_requested = (
                tiny_reverse_recovery_continuation_required(
                    segment_action,
                    stable_heading_error_rad,
                    recovery_active=recovery_active,
                    rotate_first_enabled=(
                        deployment_args.rotate_first_on_reverse
                    ),
                    paused=self._paused,
                )
            )
            if (
                reverse_recovery_requested
                or large_turn_recovery_requested
                or tiny_reverse_recovery_requested
            ):
                if self._focus_rotation_recovery_started is None:
                    self._focus_rotation_recovery_started = now
                    if (
                        stable_heading_error_rad is not None
                        and abs(stable_heading_error_rad) > 1e-9
                    ):
                        self._focus_rotation_turn_direction = (
                            1 if stable_heading_error_rad > 0.0 else -1
                        )
                    elif requested_angular != 0.0:
                        self._focus_rotation_turn_direction = (
                            1 if requested_angular > 0.0 else -1
                        )
                expired = reverse_recovery_expired(
                    started_monotonic=self._focus_rotation_recovery_started,
                    now_monotonic=now,
                    timeout_s=deployment_args.rotate_first_timeout_s,
                )
                continuation_request = rotate_first_continuation_request(
                    requested_angular,
                    continuation_required=tiny_reverse_recovery_requested,
                    latched_direction=self._focus_rotation_turn_direction,
                )
                rotate_angular = bounded_rotate_first_angular(
                    continuation_request,
                    latched_direction=self._focus_rotation_turn_direction,
                    maximum_radps=(
                        deployment_args.rotate_first_max_angular_radps
                    ),
                )
                if not expired and rotate_angular != 0.0:
                    # Do not publish directly here. The pinned timer keeps
                    # final authority for pause, stale pose/path,
                    # relocalization, depth and acceleration guards.
                    rotate_command = Twist()
                    rotate_command.angular.z = rotate_angular
                    self.latest_cmd = rotate_command
                    self.prev_cmd = Twist()
                    self._reverse_required_publisher.publish(Bool(data=False))
                    if now - self._last_focus_rotate_first_log >= 2.0:
                        elapsed = (
                            now - self._focus_rotation_recovery_started
                        )
                        context = (
                            "reverse_segment"
                            if reverse_recovery_requested
                            else (
                                "tiny_reverse_continuation"
                                if tiny_reverse_recovery_requested
                                else "large_turn"
                            )
                        )
                        heading_text = (
                            "unavailable"
                            if stable_heading_error_rad is None
                            else f"{math.degrees(stable_heading_error_rad):.1f}"
                        )
                        self.get_logger().warning(
                            "Focus TinyNav rotate-first recovery: "
                            f"context={context}, linear=0.000 m/s, "
                            f"angular={rotate_angular:.3f} rad/s, "
                            f"stable_heading_error_deg={heading_text}, "
                            f"forward_component="
                            f"{control_segment_forward_m:.3f} m, "
                            f"elapsed={elapsed:.2f} s"
                        )
                        self._last_focus_rotate_first_log = now
                    return
                # A recovery timeout is a controller failure even when a new
                # short path segment is nominally forward.  Reuse the existing
                # receiver-visible fail-closed rejection channel.
                self._reverse_required_publisher.publish(Bool(data=True))
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                if now - self._last_focus_reverse_log >= 2.0:
                    self.get_logger().warning(
                        "Focus TinyNav rejected unresolved rotate-first "
                        "recovery: bounded timeout expired"
                    )
                    self._last_focus_reverse_log = now
                return

            if (
                segment_action == "reject_reverse"
                and deployment_args.rotate_first_on_reverse
                and self._paused
            ):
                # A path received while navigation is paused must not consume
                # the bounded recovery budget.
                self._reset_focus_rotation_recovery()
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self._reverse_required_publisher.publish(Bool(data=False))
                return
            if segment_action == "reject_reverse":
                # Opt-out or a zero-yaw request retains the original
                # fail-closed receiver rejection.
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=True))
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                if now - self._last_focus_reverse_log >= 2.0:
                    self.get_logger().warning(
                        "Focus TinyNav rejected reverse control segment: "
                        f"forward_component={control_segment_forward_m:.3f} "
                        "m; rotate-first unavailable"
                    )
                    self._last_focus_reverse_log = now
                return

            self._reset_focus_rotation_recovery()
            self._reverse_required_publisher.publish(Bool(data=False))
            if segment_action == "zero_tiny_reverse":
                # A sub-threshold negative component is trajectory jitter, not
                # permission for either translation or recovery rotation.
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                now = time.monotonic()
                if now - self._last_focus_tiny_reverse_log >= 2.0:
                    self.get_logger().info(
                        "Focus TinyNav suppressed tiny reverse segment: "
                        f"forward_component={control_segment_forward_m:.4f} m"
                    )
                    self._last_focus_tiny_reverse_log = now
                return
            requested = float(self.latest_cmd.linear.x)
            floored = apply_linear_engagement_floor(
                requested,
                engage_threshold_mps=float(self.linear_engage_threshold),
                minimum_effective_mps=float(
                    self.min_effective_linear_speed
                ),
            )
            if floored == requested:
                return
            self.latest_cmd.linear.x = floored
            now = time.monotonic()
            if now - self._last_focus_speed_floor_log >= 2.0:
                self.get_logger().info(
                    "Focus TinyNav velocity floor: "
                    f"{requested:.3f} -> {floored:.3f} m/s"
                )
                self._last_focus_speed_floor_log = now

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    rclpy.init(args=ros_args)
    node = FocusCmdVelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
