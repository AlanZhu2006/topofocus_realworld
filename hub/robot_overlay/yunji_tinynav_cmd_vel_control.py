#!/usr/bin/env python3
"""Yunji deployment wrapper for TinyNav's pinned velocity controller.

The pinned TinyNav controller exposes ``linear_engage_threshold`` but compares
the requested speed against ``min_effective_linear_speed`` instead.  A valid
short trajectory segment can therefore request, for example, 0.078 m/s: above
the intended 0.04 m/s engage threshold but below the 0.10 m/s static-friction
floor.  Its timer then publishes zero forever.

Keep the source controller immutable and preserve all of its stale-pose,
stale-path, depth-stop, turn, acceleration and arrival guards.  This deployment
subclass only applies the controller's already-declared engagement threshold to
the freshly computed forward command.
"""
from __future__ import annotations

import logging
import math
import time


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
    import rclpy
    from tinynav.platforms.cmd_vel_control import CmdVelControlNode

    class YunjiCmdVelControlNode(CmdVelControlNode):
        def __init__(self) -> None:
            super().__init__()
            self._last_focus_speed_floor_log = 0.0

        def path_callback(self, message) -> None:
            super().path_callback(message)
            if (
                message is None
                or self.pose is None
                or len(message.poses) < 2
            ):
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
                    "Focus Yunji velocity floor: "
                    f"{requested:.3f} -> {floored:.3f} m/s"
                )
                self._last_focus_speed_floor_log = now

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    rclpy.init(args=args)
    node = YunjiCmdVelControlNode()
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
