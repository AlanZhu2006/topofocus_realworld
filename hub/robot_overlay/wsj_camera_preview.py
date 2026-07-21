#!/usr/bin/env python3
"""Standalone, dependency-minimal camera-only preview publisher for wsj.

Deliberately NOT part of focus_ros_sender.py's full observation pipeline:
that pipeline requires a synchronized RGB+depth+pose+intrinsics tuple (real
pose is not optional in the wire protocol), which on wsj means it can only
start once perception_node + map_node have SLAM-relocalized against a
pre-built map -- there is no reason a raw camera preview should be gated
behind that. This subscribes to ONLY the raw color topic and pushes JPEG
frames straight to a foxglove_relay.py instance's /camera/{name} endpoint
the instant each frame arrives -- no synchronization, no pose, no map, no
central mapping pipeline, no relocalization dependency. Runs alongside
run_live_rehearsal.sh (or entirely standalone if only the RealSense driver
is up) without needing perception_node/map_node/pointcloud at all.

Read-only with respect to the robot: subscribes to one topic, never
publishes to it or to anything else. Cannot move the robot -- no cmd_vel,
no planning, no actuation imports at all, matching every other overlay
script in this project.
"""
from __future__ import annotations

import argparse
import time

import cv2
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class WsjCameraPreview(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wsj_camera_preview")
        self.args = args
        self.bridge = CvBridge()
        self.url = f"{args.relay_url}/camera/{args.name}"
        # No requests.Session() -- a fresh connection per push. When
        # relay-url crosses an SSH reverse tunnel, a kept-alive connection
        # idling between pushes has been observed to get silently dropped
        # somewhere in the tunnel (hit this exact failure mode on the Yunji
        # preview script; see audit/TRANSPORT_WSJ_TEST.md for the project's
        # earlier run-in with the same SSH-tunnel keep-alive behavior).
        self.headers = {"X-Robot-Token": args.token, "Content-Type": "image/jpeg", "Connection": "close"}
        self.min_period_s = 1.0 / args.max_rate_hz if args.max_rate_hz > 0 else 0.0
        self.last_push_at = 0.0
        self.frames_seen = 0
        self.frames_pushed = 0
        self.push_failures = 0
        self.create_subscription(Image, args.rgb_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"wsj_camera_preview ready: {args.rgb_topic} -> {self.url} "
            f"(max {args.max_rate_hz} Hz, no sync/pose/map dependency)")

    def on_image(self, msg: Image) -> None:
        self.frames_seen += 1
        now = time.monotonic()
        if now - self.last_push_at < self.min_period_s:
            return
        self.last_push_at = now
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, jpeg = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality])
            if not ok:
                return
            requests.post(self.url, headers=self.headers, data=jpeg.tobytes(), timeout=3.0).raise_for_status()
            self.frames_pushed += 1
        except Exception as exc:  # noqa: BLE001 - one bad push must not kill the subscriber
            self.push_failures += 1
            self.get_logger().warning(f"push failed: {exc}", throttle_duration_sec=5.0)
        if self.frames_pushed % 50 == 0 and self.frames_pushed > 0:
            self.get_logger().info(
                f"seen={self.frames_seen} pushed={self.frames_pushed} failures={self.push_failures}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--relay-url", required=True, help="e.g. http://127.0.0.1:18766")
    parser.add_argument("--name", default="wsj", help="must match the --robot NAME in foxglove_relay.py")
    parser.add_argument("--token", required=True, help="robot-0's token from hub/runtime/tokens.json")
    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--max-rate-hz", type=float, default=5.0,
                         help="throttles pushes even if the driver publishes faster; 0 disables")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args = parser.parse_args()

    rclpy.init()
    node = WsjCameraPreview(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
