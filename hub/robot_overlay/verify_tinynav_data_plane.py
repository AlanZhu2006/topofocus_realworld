#!/usr/bin/env python3
"""Verify the live ROS data and command graph before declaring TinyNav ready.

The verifier never publishes a target or velocity. It waits for newly
received odometry, occupancy and router-status messages, then checks that the
only velocity route is controller -> v2 receiver -> guarded chassis bridge.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any


def endpoint_names(endpoints: list[Any]) -> list[str]:
    names = []
    for endpoint in endpoints:
        namespace = str(getattr(endpoint, "node_namespace", "/")).rstrip("/")
        name = str(getattr(endpoint, "node_name", ""))
        names.append(f"{namespace}/{name}" if namespace else f"/{name}")
    return sorted(names)


def require_endpoint(
    names: list[str],
    *,
    description: str,
    contains: str,
    exact_count: int = 1,
) -> None:
    if len(names) != exact_count or any(contains not in name for name in names):
        raise ValueError(
            f"{description} endpoints are not the expected exclusive route: "
            f"{names}"
        )


def validate_occupancy(message: Any, *, frame_id: str) -> dict[str, object]:
    if str(message.header.frame_id) != frame_id:
        raise ValueError(
            f"occupancy frame {message.header.frame_id!r} != {frame_id!r}"
        )
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    values = tuple(int(value) for value in message.data)
    if width <= 0 or height <= 0 or len(values) != width * height:
        raise ValueError("occupancy dimensions/data length are invalid")
    if not 0.0 < resolution <= 0.20:
        raise ValueError(f"occupancy resolution is implausible: {resolution}")
    known = sum(value >= 0 for value in values)
    free = sum(value == 0 for value in values)
    occupied = sum(value > 0 for value in values)
    if known <= 0 or free <= 0:
        raise ValueError("occupancy has no observed free space")
    return {
        "frame_id": frame_id,
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "known_cells": known,
        "free_cells": free,
        "occupied_cells": occupied,
    }


def validate_router_status(message: Any) -> dict[str, object]:
    payload = json.loads(str(message.data))
    state = str(payload.get("state", ""))
    reason = str(payload.get("reason", ""))
    if not state or not reason:
        raise ValueError("router status lacks state/reason")
    return {"state": state, "reason": reason}


def validate_water_status(
    message: Any, *, expected_live: bool
) -> dict[str, object]:
    payload = json.loads(str(message.data))
    if payload.get("schema_version") != "focus-water-cmd-bridge-v1":
        raise ValueError("unexpected WATER bridge status schema")
    if payload.get("live") is not expected_live:
        raise ValueError("WATER bridge live/debug mode mismatch")
    if payload.get("ready") is not True:
        raise ValueError("WATER bridge is not ready")
    water = payload.get("water")
    if not isinstance(water, dict) or water.get("ready") is not True:
        raise ValueError("WATER chassis status is not ready")
    if water.get("estop_engaged") is True:
        raise ValueError("WATER reports an engaged emergency stop")
    if str(water.get("error_code", "")).strip("0"):
        raise ValueError(f"WATER reports error code {water.get('error_code')}")
    if payload.get("command_active") is not False:
        raise ValueError("startup verification requires an inactive command")
    if payload.get("velocity_zero_confirmed") is not True:
        raise ValueError("WATER bridge has not confirmed zero velocity")
    output = payload.get("last_output")
    if (
        not isinstance(output, dict)
        or float(output.get("linear_mps", 1.0)) != 0.0
        or float(output.get("angular_radps", 1.0)) != 0.0
    ):
        raise ValueError("WATER startup output is not zero")
    return {
        "live": expected_live,
        "battery_percent": water.get("battery_percent"),
        "move_status": water.get("move_status"),
        "zero_confirmed": True,
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    node = Node(f"focus_{args.robot_id.replace('-', '_')}_startup_verifier")
    latest: dict[str, Any] = {}
    volatile_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    map_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions = [
        node.create_subscription(
            Odometry,
            args.odom_topic,
            lambda message: latest.__setitem__("odom", message),
            volatile_qos,
        ),
        node.create_subscription(
            OccupancyGrid,
            args.occupancy_topic,
            lambda message: latest.__setitem__("occupancy", message),
            map_qos,
        ),
        node.create_subscription(
            String,
            args.router_status_topic,
            lambda message: latest.__setitem__("router", message),
            volatile_qos,
        ),
    ]
    if args.platform_status_topic:
        subscriptions.append(
            node.create_subscription(
                String,
                args.platform_status_topic,
                lambda message: latest.__setitem__("platform", message),
                volatile_qos,
            )
        )
    for index, topic in enumerate(args.fresh_image_topic):
        key = f"image_{index}"
        subscriptions.append(
            node.create_subscription(
                Image,
                topic,
                lambda message, image_key=key: latest.__setitem__(
                    image_key, message
                ),
                volatile_qos,
            )
        )

    topics = {
        "raw": args.raw_cmd_topic,
        "guarded": args.guarded_cmd_topic,
        "target": args.target_topic,
        "poi": args.poi_topic,
    }

    def graph() -> dict[str, dict[str, list[str]]]:
        return {
            key: {
                "publishers": endpoint_names(
                    node.get_publishers_info_by_topic(topic)
                ),
                "subscriptions": endpoint_names(
                    node.get_subscriptions_info_by_topic(topic)
                ),
            }
            for key, topic in topics.items()
        }

    deadline = time.monotonic() + args.timeout_s
    required_messages = {"odom", "occupancy", "router"}
    if args.platform_status_topic:
        required_messages.add("platform")
    required_messages.update(
        f"image_{index}" for index in range(len(args.fresh_image_topic))
    )
    observed_graph: dict[str, dict[str, list[str]]] = {}
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            observed_graph = graph()
            if not required_messages.issubset(latest):
                continue
            if all(
                observed_graph[key]["publishers"]
                for key in ("raw", "guarded", "target", "poi")
            ) and all(
                observed_graph[key]["subscriptions"]
                for key in ("raw", "target", "poi")
            ):
                if args.robot_id == "robot-0" and args.mode == "debug":
                    if not observed_graph["guarded"]["subscriptions"]:
                        break
                elif observed_graph["guarded"]["subscriptions"]:
                    break
        missing = sorted(required_messages - latest.keys())
        if missing:
            raise TimeoutError(
                "timed out waiting for fresh ROS messages: "
                + ", ".join(missing)
            )

        odom = latest["odom"]
        if str(odom.header.frame_id) != args.frame_id:
            raise ValueError(
                f"odometry frame {odom.header.frame_id!r} != "
                f"{args.frame_id!r}"
            )
        if str(odom.child_frame_id) != args.camera_frame:
            raise ValueError(
                f"odometry child frame {odom.child_frame_id!r} != "
                f"{args.camera_frame!r}"
            )

        require_endpoint(
            observed_graph["raw"]["publishers"],
            description="raw cmd_vel publisher",
            contains="cmd_vel_control_node",
        )
        require_endpoint(
            observed_graph["raw"]["subscriptions"],
            description="raw cmd_vel subscriber",
            contains="focus_v2_",
        )
        require_endpoint(
            observed_graph["guarded"]["publishers"],
            description="guarded cmd_vel publisher",
            contains="focus_v2_",
        )
        expected_guarded_subscriber = (
            None
            if args.robot_id == "robot-0" and args.mode == "debug"
            else (
                "go2_cmd_bridge"
                if args.robot_id == "robot-0"
                else "focus_water_cmd_vel_bridge"
            )
        )
        guarded_subscribers = observed_graph["guarded"]["subscriptions"]
        if expected_guarded_subscriber is None:
            if guarded_subscribers:
                raise ValueError(
                    "WSJ debug mode unexpectedly has a chassis subscriber: "
                    f"{guarded_subscribers}"
                )
        else:
            require_endpoint(
                guarded_subscribers,
                description="guarded chassis subscriber",
                contains=expected_guarded_subscriber,
            )
        require_endpoint(
            observed_graph["target"]["publishers"],
            description="TinyNav target publisher",
            contains="focus_tinynav_buildmap_goal_router",
        )
        require_endpoint(
            observed_graph["target"]["subscriptions"],
            description="TinyNav target subscriber",
            contains="planning_node",
        )
        require_endpoint(
            observed_graph["poi"]["publishers"],
            description="Hub POI publisher",
            contains="focus_v2_",
        )
        require_endpoint(
            observed_graph["poi"]["subscriptions"],
            description="Hub POI subscriber",
            contains="focus_tinynav_buildmap_goal_router",
        )

        report: dict[str, object] = {
            "schema_version": "focus-tinynav-data-plane-verification-v1",
            "robot_id": args.robot_id,
            "mode": args.mode,
            "passed": True,
            "robot_commands_issued": False,
            "odometry": {
                "frame_id": str(odom.header.frame_id),
                "child_frame_id": str(odom.child_frame_id),
            },
            "occupancy": validate_occupancy(
                latest["occupancy"], frame_id=args.frame_id
            ),
            "router": validate_router_status(latest["router"]),
            "fresh_image_topics": list(args.fresh_image_topic),
            "command_graph": observed_graph,
        }
        if args.platform_status_topic:
            report["platform"] = validate_water_status(
                latest["platform"], expected_live=args.mode == "live"
            )
        return report
    finally:
        subscriptions.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", choices=("robot-0", "robot-1"), required=True)
    parser.add_argument("--mode", choices=("debug", "live"), required=True)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--camera-frame", required=True)
    parser.add_argument("--odom-topic", default="/slam/odometry")
    parser.add_argument(
        "--occupancy-topic", default="/semantic_mapping/occupancy_bev"
    )
    parser.add_argument(
        "--router-status-topic", default="/mapping/buildmap_online_status"
    )
    parser.add_argument("--platform-status-topic", default="")
    parser.add_argument(
        "--fresh-image-topic",
        action="append",
        default=[],
        help="image topic that must deliver a new volatile sample",
    )
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/focus_guarded_cmd_vel")
    parser.add_argument("--target-topic", default="/control/target_pose")
    parser.add_argument("--poi-topic", default="/mapping/cmd_pois")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    try:
        report = run(args)
    except Exception as exc:  # noqa: BLE001 - startup must fail closed
        print(f"TinyNav data-plane verification failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
