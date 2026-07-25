#!/usr/bin/env python3
"""Deployment wrapper for TinyNav's pinned velocity controller.

The pinned TinyNav controller exposes ``linear_engage_threshold`` but compares
the requested speed against ``min_effective_linear_speed`` instead.  A valid
short trajectory segment can therefore request, for example, 0.078 m/s: above
the intended 0.04 m/s engage threshold but below the 0.10 m/s static-friction
floor.  Its timer then publishes zero forever.

Keep the source controller immutable and preserve all of its stale-pose,
stale-path, depth-stop, turn, acceleration and arrival guards.  This deployment
subclass, shared by Yunji and WSJ, only applies the controller's already-declared
engagement threshold to the freshly computed forward command.
"""
from __future__ import annotations

import logging
import math
import time


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


def main(args: list[str] | None = None) -> None:
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
            self._reverse_required_publisher = self.create_publisher(
                Bool, "/planning/reverse_required", 10
            )

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
            reverse_required = bool(
                control_segment_forward_m is not None
                and control_segment_forward_m < -0.02
            )
            self._reverse_required_publisher.publish(
                Bool(data=reverse_required)
            )
            if reverse_required:
                # The pinned controller's selected lookahead has a negative
                # robot-relative X component. Its current controller forbids
                # reverse but otherwise turns to face that segment, which can
                # rotate a wall-adjacent robot toward the wall. Close the raw
                # command immediately and let the v2 receiver reject/replan
                # this frontier leg with an explicit reason.
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                now = time.monotonic()
                if now - self._last_focus_reverse_log >= 2.0:
                    self.get_logger().warning(
                        "Focus TinyNav rejected reverse control segment: "
                        f"forward_component={control_segment_forward_m:.3f} m"
                    )
                    self._last_focus_reverse_log = now
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
    rclpy.init(args=args)
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
