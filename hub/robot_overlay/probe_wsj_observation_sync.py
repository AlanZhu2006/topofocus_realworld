#!/usr/bin/env python3
"""Prove that a fresh WSJ DDS reader can assemble one observation tuple.

This is a read-only discriminator for the persistent observation sender.  It
subscribes with the same topics, QoS, RGB cache policy, and approximate
geometry synchronizer as ``focus_ros_sender.py``.  If this fresh participant
receives complete tuples while the persistent sender does not advance, only
the stale sender reader needs replacement; camera/perception tracking must not
be restarted.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import time
from typing import Any, Iterable


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nearest_stamp_skew_s(messages: Iterable[Any], target_ns: int) -> float:
    skews = [abs(stamp_ns(message) - target_ns) / 1e9 for message in messages]
    return min(skews, default=math.inf)


def run_probe(args: argparse.Namespace) -> dict[str, object]:
    import message_filters
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node("focus_wsj_observation_sync_probe")
    rgb_cache: deque[Any] = deque(maxlen=args.rgb_cache_size)
    state: dict[str, object] = {
        "geometry_tuples": 0,
        "valid_observation_tuples": 0,
        "rgb_frames": 0,
        "rgb_info_frames": 0,
        "closest_rgb_skew_s": None,
        "tuple_stamps_ns": [],
    }

    def on_rgb(message: Any) -> None:
        rgb_cache.append(message)
        state["rgb_frames"] = int(state["rgb_frames"]) + 1

    def on_rgb_info(_message: Any) -> None:
        state["rgb_info_frames"] = int(state["rgb_info_frames"]) + 1

    def on_geometry(depth: Any, info: Any, odometry: Any) -> None:
        state["geometry_tuples"] = int(state["geometry_tuples"]) + 1
        depth_ns = stamp_ns(depth)
        stamps = (depth_ns, stamp_ns(info), stamp_ns(odometry))
        tuple_skew_s = (max(stamps) - min(stamps)) / 1e9
        rgb_skew_s = nearest_stamp_skew_s(rgb_cache, depth_ns)
        previous = state["closest_rgb_skew_s"]
        if previous is None or rgb_skew_s < float(previous):
            state["closest_rgb_skew_s"] = rgb_skew_s
        if (
            tuple_skew_s <= args.sync_slop_s
            and rgb_skew_s <= args.latest_rgb_max_skew_s
            and int(state["rgb_info_frames"]) > 0
        ):
            state["valid_observation_tuples"] = (
                int(state["valid_observation_tuples"]) + 1
            )
            stamps_out = state["tuple_stamps_ns"]
            assert isinstance(stamps_out, list)
            stamps_out.append(
                {
                    "depth": depth_ns,
                    "info": stamps[1],
                    "odometry": stamps[2],
                    "tuple_skew_s": tuple_skew_s,
                    "rgb_skew_s": rgb_skew_s,
                }
            )

    subscriptions = [
        node.create_subscription(
            Image, args.rgb_topic, on_rgb, qos_profile_sensor_data
        ),
        node.create_subscription(
            CameraInfo,
            args.rgb_info_topic,
            on_rgb_info,
            qos_profile_sensor_data,
        ),
    ]
    depth_sub = message_filters.Subscriber(
        node,
        Image,
        args.depth_topic,
        qos_profile=qos_profile_sensor_data,
    )
    info_sub = message_filters.Subscriber(
        node,
        CameraInfo,
        args.info_topic,
        qos_profile=qos_profile_sensor_data,
    )
    odometry_sub = message_filters.Subscriber(
        node,
        Odometry,
        args.odometry_topic,
        qos_profile=qos_profile_sensor_data,
    )
    synchronizer = message_filters.ApproximateTimeSynchronizer(
        [depth_sub, info_sub, odometry_sub],
        queue_size=args.sync_queue_size,
        slop=args.sync_slop_s,
    )
    synchronizer.registerCallback(on_geometry)

    started = time.monotonic()
    try:
        while (
            time.monotonic() - started < args.timeout_s
            and int(state["valid_observation_tuples"]) < args.minimum_tuples
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        # Keep explicit references alive through the final spin, then release
        # every reader before shutting down this temporary participant.
        _ = subscriptions, synchronizer
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    passed = int(state["valid_observation_tuples"]) >= args.minimum_tuples
    closest = state["closest_rgb_skew_s"]
    return {
        "schema_version": "focus-wsj-observation-sync-probe-v1",
        "passed": passed,
        "classification": "observed_read_only_fresh_dds_reader",
        "elapsed_s": round(time.monotonic() - started, 3),
        "minimum_tuples": args.minimum_tuples,
        "geometry_tuples": state["geometry_tuples"],
        "valid_observation_tuples": state["valid_observation_tuples"],
        "rgb_frames": state["rgb_frames"],
        "rgb_info_frames": state["rgb_info_frames"],
        "closest_rgb_skew_s": (
            None if closest is None or not math.isfinite(float(closest))
            else round(float(closest), 9)
        ),
        "tuple_stamps_ns": state["tuple_stamps_ns"],
        "robot_commands_issued": False,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rgb-topic", default="/camera/camera/color/image_raw"
    )
    parser.add_argument(
        "--rgb-info-topic", default="/camera/camera/color/camera_info"
    )
    parser.add_argument("--depth-topic", default="/slam/depth")
    parser.add_argument("--info-topic", default="/slam/camera_info")
    parser.add_argument(
        "--odometry-topic", default="/slam/odometry_visual"
    )
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--minimum-tuples", type=int, default=3)
    parser.add_argument("--sync-queue-size", type=int, default=50)
    parser.add_argument("--sync-slop-s", type=float, default=0.05)
    parser.add_argument("--rgb-cache-size", type=int, default=90)
    parser.add_argument("--latest-rgb-max-skew-s", type=float, default=0.05)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    positive = (
        args.timeout_s,
        args.sync_slop_s,
        args.latest_rgb_max_skew_s,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise SystemExit("probe timing bounds must be finite and positive")
    if min(
        args.minimum_tuples,
        args.sync_queue_size,
        args.rgb_cache_size,
    ) <= 0:
        raise SystemExit("probe counts must be positive")
    result = run_probe(args)
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["passed"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
