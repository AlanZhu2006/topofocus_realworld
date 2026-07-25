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
If the trajectory does not return to the forward half-plane before the bounded
deadline, the existing receiver rejection remains fail-closed.

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
            self._focus_reverse_recovery_started: float | None = None
            self._focus_reverse_turn_direction = 0
            self._reverse_required_publisher = self.create_publisher(
                Bool, "/planning/reverse_required", 10
            )

        def _reset_focus_reverse_recovery(self) -> None:
            self._focus_reverse_recovery_started = None
            self._focus_reverse_turn_direction = 0

        def _on_paused(self, message) -> None:
            super()._on_paused(message)
            if bool(message.data):
                self._reset_focus_reverse_recovery()

        @staticmethod
        def _pose_matrix(pose_stamped) -> np.ndarray:
            pose = pose_stamped.pose
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

        def path_callback(self, message) -> None:
            control_segment_forward_m = None
            if message is not None and len(message.poses) >= 2:
                try:
                    control_segment_forward_m = (
                        self._control_segment_forward_m(message)
                    )
                except (TypeError, ValueError):
                    control_segment_forward_m = None
            super().path_callback(message)
            if (
                message is None
                or self.pose is None
                or len(message.poses) < 2
            ):
                return
            segment_action = classify_forward_component(
                control_segment_forward_m
            )
            if segment_action == "reject_reverse":
                now = time.monotonic()
                if (
                    deployment_args.rotate_first_on_reverse
                    and not self._paused
                ):
                    requested_angular = float(self.latest_cmd.angular.z)
                    if self._focus_reverse_recovery_started is None:
                        self._focus_reverse_recovery_started = now
                        if requested_angular != 0.0:
                            self._focus_reverse_turn_direction = (
                                1 if requested_angular > 0.0 else -1
                            )
                    expired = reverse_recovery_expired(
                        started_monotonic=(
                            self._focus_reverse_recovery_started
                        ),
                        now_monotonic=now,
                        timeout_s=deployment_args.rotate_first_timeout_s,
                    )
                    rotate_angular = bounded_rotate_first_angular(
                        requested_angular,
                        latched_direction=(
                            self._focus_reverse_turn_direction
                        ),
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
                        self._reverse_required_publisher.publish(
                            Bool(data=False)
                        )
                        if (
                            now - self._last_focus_rotate_first_log
                            >= 2.0
                        ):
                            elapsed = (
                                now
                                - self._focus_reverse_recovery_started
                            )
                            self.get_logger().warning(
                                "Focus TinyNav rotate-first recovery: "
                                "linear=0.000 m/s, "
                                f"angular={rotate_angular:.3f} rad/s, "
                                f"forward_component="
                                f"{control_segment_forward_m:.3f} m, "
                                f"elapsed={elapsed:.2f} s"
                            )
                            self._last_focus_rotate_first_log = now
                        return
                elif deployment_args.rotate_first_on_reverse:
                    # A path received while navigation is paused must not
                    # consume the bounded recovery budget.
                    self._reset_focus_reverse_recovery()
                    self.latest_cmd = Twist()
                    self.prev_cmd = Twist()
                    self._reverse_required_publisher.publish(
                        Bool(data=False)
                    )
                    return

                # Opt-out, zero-yaw, or timeout all retain the original
                # fail-closed receiver rejection.
                self._reverse_required_publisher.publish(Bool(data=True))
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                if now - self._last_focus_reverse_log >= 2.0:
                    self.get_logger().warning(
                        "Focus TinyNav rejected reverse control segment: "
                        f"forward_component={control_segment_forward_m:.3f} "
                        "m; rotate-first unavailable or expired"
                    )
                    self._last_focus_reverse_log = now
                return
            self._reset_focus_reverse_recovery()
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
