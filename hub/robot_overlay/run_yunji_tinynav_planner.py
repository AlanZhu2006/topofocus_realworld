#!/usr/bin/env python3
"""Run the pinned TinyNav local planner with Yunji's measured geometry.

The planner source remains the user's pinned ``go2_tinynav`` revision.  This
deployment wrapper changes only the module-level robot configuration before
constructing ``PlanningNode``; it does not patch or fork the upstream file.
"""
from __future__ import annotations

import argparse
import math


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--body-radius-m",
        type=float,
        default=0.283,
        help=(
            "Yunji circumscribed body radius; 0.283 m is preserved from the "
            "existing robot-local reachability deployment"
        ),
    )
    parser.add_argument(
        "--camera-forward-m",
        type=float,
        default=0.23,
        help="operator-measured base-to-Odin forward offset",
    )
    parser.add_argument(
        "--safety-margin-m",
        type=float,
        default=0.05,
        help="additional planner ESDF margin outside the body",
    )
    return parser


def main() -> int:
    args, ros_args = build_parser().parse_known_args()
    values = (
        args.body_radius_m,
        args.camera_forward_m,
        args.safety_margin_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise SystemExit("Yunji geometry must contain only finite values")
    if (
        not 0.15 <= args.body_radius_m <= 0.60
        or not 0.0 <= args.camera_forward_m <= args.body_radius_m
        or not 0.02 <= args.safety_margin_m <= 0.30
    ):
        raise SystemExit("Yunji planner geometry is outside deployment bounds")

    import rclpy
    from tinynav.core import planning_node

    planning_node.GO2_CONFIG = planning_node.RobotConfig(
        name="yunji-water",
        shape="circle",
        radius=args.body_radius_m,
        camera_x=args.camera_forward_m,
        camera_y=0.0,
        control_x=0.0,
        control_y=0.0,
        safety_radius=args.safety_margin_m,
    )
    # Pass the remaining list explicitly; ``None`` would make rclpy re-read
    # this wrapper's already-consumed geometry flags from sys.argv.
    rclpy.init(args=ros_args)
    node = planning_node.PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
