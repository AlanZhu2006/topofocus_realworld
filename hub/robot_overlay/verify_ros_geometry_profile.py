#!/usr/bin/env python3
"""Verify that a live ROS Image and CameraInfo use one geometry profile.

This is a read-only calibration preflight.  It subscribes to both topics in
the same process so a RealSense reconnect cannot leave a fresh image paired
with TinyNav's cached CameraInfo from the previous stream profile.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

from verify_tinynav_data_plane import validate_geometry_contract


def run(args: argparse.Namespace) -> dict[str, object]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node("focus_ros_geometry_profile_verifier")
    latest: dict[str, Any] = {}
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscriptions = [
        node.create_subscription(
            Image,
            args.image_topic,
            lambda message: latest.__setitem__("image", message),
            qos,
        ),
        node.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            lambda message: latest.__setitem__("camera_info", message),
            qos,
        ),
    ]
    deadline = time.monotonic() + args.timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if {"image", "camera_info"}.issubset(latest):
                geometry = validate_geometry_contract(
                    latest["image"],
                    latest["camera_info"],
                    expected_frame=args.expected_frame,
                )
                return {
                    "schema_version": "focus-ros-geometry-profile-v1",
                    "passed": True,
                    "robot_commands_issued": False,
                    "image_topic": args.image_topic,
                    "camera_info_topic": args.camera_info_topic,
                    "geometry": geometry,
                }
        missing = sorted({"image", "camera_info"} - latest.keys())
        raise TimeoutError(
            "timed out waiting for fresh ROS geometry messages: "
            + ", ".join(missing)
        )
    finally:
        subscriptions.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--expected-frame", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    try:
        report = run(args)
    except Exception as exc:  # noqa: BLE001 - calibration must fail closed
        print(f"ROS geometry profile verification failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
