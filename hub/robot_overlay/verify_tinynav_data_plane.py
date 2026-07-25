#!/usr/bin/env python3
"""Verify the live ROS data and command graph before declaring TinyNav ready.

The verifier never publishes a target or velocity. It waits for newly
received odometry, occupancy and router-status messages, then checks that the
only velocity route is controller -> v2 receiver -> guarded chassis bridge.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any


def endpoint_names(endpoints: list[Any]) -> list[str]:
    names = []
    for endpoint in endpoints:
        namespace = str(getattr(endpoint, "node_namespace", "/")).rstrip("/")
        name = str(getattr(endpoint, "node_name", ""))
        # Fast DDS can expose an endpoint before its participant identity has
        # propagated. Treat that as discovery still in progress; accepting it
        # would make the exclusive-route check fail nondeterministically.
        if (
            not name
            or name == "_NODE_NAME_UNKNOWN_"
            or namespace == "_NODE_NAMESPACE_UNKNOWN_"
        ):
            continue
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


def validate_command_graph(
    observed_graph: dict[str, dict[str, list[str]]],
    *,
    robot_id: str,
    mode: str,
    pre_bridge_full_check: bool = False,
    guarded_only: bool = False,
) -> None:
    """Validate the exclusive command route for the requested startup phase."""

    if not guarded_only:
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
        if robot_id == "robot-0"
        and (mode == "debug" or pre_bridge_full_check)
        else (
            "go2_cmd_bridge"
            if robot_id == "robot-0"
            else "focus_water_cmd_vel_bridge"
        )
    )
    guarded_subscribers = observed_graph["guarded"]["subscriptions"]
    if expected_guarded_subscriber is None:
        if guarded_subscribers:
            raise ValueError(
                "WSJ pre-bridge/debug mode unexpectedly has a chassis "
                f"subscriber: {guarded_subscribers}"
            )
    else:
        require_endpoint(
            guarded_subscribers,
            description="guarded chassis subscriber",
            contains=expected_guarded_subscriber,
        )
    if guarded_only:
        return
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
    return {
        "state": state,
        "reason": reason,
        "decision_id": payload.get("decision_id"),
    }


def message_stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def message_lag_s(message: Any, *, reference_message: Any) -> float:
    """Return source-time lag relative to a fresh message from the same robot.

    Odin1 ROS messages use a device/boot-relative clock rather than Unix wall
    time. Comparing those stamps with ``time.time_ns()`` makes every current
    Yunji map look decades old. The occupancy grid and required fresh image
    are both derived from the same robot sensor stream, so their source-stamp
    delta is the clock-domain-safe freshness contract.
    """
    stamp_ns = message_stamp_ns(message)
    reference_ns = message_stamp_ns(reference_message)
    if stamp_ns <= 0:
        raise ValueError("message timestamp is zero")
    if reference_ns <= 0:
        raise ValueError("reference message timestamp is zero")
    age_s = (reference_ns - stamp_ns) * 1e-9
    if age_s < -1.0:
        raise ValueError(
            "occupancy timestamp is "
            f"{abs(age_s):.3f}s ahead of the fresh image timestamp"
        )
    return max(0.0, age_s)


def planar_position(message: Any) -> tuple[float, float]:
    position = message.pose.pose.position
    x_m = float(position.x)
    y_m = float(position.y)
    if not all(math.isfinite(value) for value in (x_m, y_m)):
        raise ValueError("odometry planar position is not finite")
    return x_m, y_m


def cached_occupancy_start_is_valid(
    *,
    occupancy_age_s: float,
    maximum_age_s: float,
    anchor_xy: tuple[float, float] | None,
    current_xy: tuple[float, float],
    maximum_motion_m: float,
    router_status: dict[str, object],
) -> tuple[bool, float | None]:
    """Bridge a sparse keyframe interval only while HOLD remains stationary."""

    if occupancy_age_s <= maximum_age_s:
        return True, 0.0
    if anchor_xy is None or maximum_motion_m <= 0.0:
        return False, None
    motion_m = math.hypot(
        current_xy[0] - anchor_xy[0],
        current_xy[1] - anchor_xy[1],
    )
    valid = (
        motion_m <= maximum_motion_m
        and router_status.get("state") == "HOLD"
        and router_status.get("reason") == "NO_GOAL"
        and router_status.get("decision_id") is None
    )
    return valid, motion_m


def validate_geometry_contract(
    image: Any,
    camera_info: Any,
    *,
    expected_frame: str,
) -> dict[str, object]:
    dimensions = (int(image.width), int(image.height))
    info_dimensions = (int(camera_info.width), int(camera_info.height))
    if dimensions[0] <= 0 or dimensions[1] <= 0:
        raise ValueError(f"geometry image dimensions are invalid: {dimensions}")
    if info_dimensions != dimensions:
        raise ValueError(
            "geometry image/CameraInfo dimensions differ: "
            f"image={dimensions[0]}x{dimensions[1]}, "
            f"camera_info={info_dimensions[0]}x{info_dimensions[1]}"
        )
    frames = {
        "image": str(image.header.frame_id),
        "camera_info": str(camera_info.header.frame_id),
    }
    invalid_frames = {
        name: frame for name, frame in frames.items() if frame != expected_frame
    }
    if invalid_frames:
        raise ValueError(
            f"geometry frame mismatch; expected {expected_frame!r}, "
            f"got {invalid_frames}"
        )
    intrinsics = tuple(float(value) for value in camera_info.k)
    if len(intrinsics) != 9 or not all(math.isfinite(value) for value in intrinsics):
        raise ValueError("CameraInfo intrinsics are malformed")
    fx = intrinsics[0]
    fy = intrinsics[4]
    cx = intrinsics[2]
    cy = intrinsics[5]
    if fx <= 0 or fy <= 0:
        raise ValueError("CameraInfo focal lengths must be positive")
    if not (-0.5 <= cx <= dimensions[0] - 0.5):
        raise ValueError(f"CameraInfo cx is outside the image: {cx}")
    if not (-0.5 <= cy <= dimensions[1] - 0.5):
        raise ValueError(f"CameraInfo cy is outside the image: {cy}")
    return {
        "frame_id": expected_frame,
        "width": dimensions[0],
        "height": dimensions[1],
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
    }


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
    from sensor_msgs.msg import CameraInfo, Image
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
    subscriptions = []
    if not args.post_bridge_command_check:
        subscriptions.extend(
            [
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
        )
    if args.platform_status_topic and not args.post_bridge_command_check:
        subscriptions.append(
            node.create_subscription(
                String,
                args.platform_status_topic,
                lambda message: latest.__setitem__("platform", message),
                volatile_qos,
            )
        )
    if not args.post_bridge_command_check:
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
    if args.geometry_image_topic and not args.post_bridge_command_check:
        subscriptions.extend(
            [
                node.create_subscription(
                    Image,
                    args.geometry_image_topic,
                    lambda message: latest.__setitem__(
                        "geometry_image", message
                    ),
                    volatile_qos,
                ),
                node.create_subscription(
                    CameraInfo,
                    args.camera_info_topic,
                    lambda message: latest.__setitem__(
                        "camera_info", message
                    ),
                    volatile_qos,
                ),
            ]
        )

    all_topics = {
        "raw": args.raw_cmd_topic,
        "guarded": args.guarded_cmd_topic,
        "target": args.target_topic,
        "poi": args.poi_topic,
    }
    topics = (
        {"guarded": args.guarded_cmd_topic}
        if args.post_bridge_command_check
        else all_topics
    )

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
    if args.geometry_image_topic:
        required_messages.update(("geometry_image", "camera_info"))
    observed_graph: dict[str, dict[str, list[str]]] = {}
    stale_occupancy_age_s: float | None = None
    occupancy_clock_error: str | None = None
    occupancy_anchor_stamp_ns: int | None = None
    occupancy_anchor_xy: tuple[float, float] | None = None
    cached_occupancy_motion_m: float | None = None
    using_cached_occupancy = False
    try:
        if args.post_bridge_command_check:
            last_graph_error: ValueError | None = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                observed_graph = graph()
                try:
                    validate_command_graph(
                        observed_graph,
                        robot_id=args.robot_id,
                        mode=args.mode,
                        guarded_only=True,
                    )
                except ValueError as error:
                    last_graph_error = error
                    continue
                return {
                    "schema_version": (
                        "focus-tinynav-data-plane-verification-v1"
                    ),
                    "robot_id": args.robot_id,
                    "mode": args.mode,
                    "verification_scope": "post_bridge_command_graph",
                    "passed": True,
                    "robot_commands_issued": False,
                    "pre_bridge_full_verification_required": True,
                    "command_graph": observed_graph,
                }
            detail = (
                str(last_graph_error)
                if last_graph_error is not None
                else "no resolved guarded command endpoints"
            )
            raise TimeoutError(
                "timed out waiting for the post-bridge guarded command "
                f"route: {detail}"
            )

        # Full sensor/map validation loop. Keep message collection ahead of
        # expensive graph discovery queries.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if not required_messages.issubset(latest):
                continue
            # Graph introspection can take several seconds on the long-lived
            # TinyNav ROS domain.  Running eight endpoint queries before each
            # spin starves the high-bandwidth Image/CameraInfo callbacks and
            # makes healthy streams look absent.  Receive every required
            # volatile sample first; the exclusive command-route graph is
            # still checked before this verifier can pass.
            observed_graph = graph()
            if args.geometry_image_topic:
                # A profile reconnect can leave TinyNav publishing new-sized
                # images with cached old CameraInfo. Fail immediately instead
                # of waiting for a stale map to time out.
                validate_geometry_contract(
                    latest["geometry_image"],
                    latest["camera_info"],
                    expected_frame=args.camera_frame,
                )
            stale_occupancy_age_s = None
            occupancy_clock_error = None
            cached_occupancy_motion_m = None
            using_cached_occupancy = False
            if args.max_occupancy_age_s > 0:
                fresh_reference = max(
                    (
                        latest[f"image_{index}"]
                        for index in range(len(args.fresh_image_topic))
                    ),
                    key=message_stamp_ns,
                )
                try:
                    stale_occupancy_age_s = message_lag_s(
                        latest["occupancy"],
                        reference_message=fresh_reference,
                    )
                except ValueError as error:
                    # A restarted device clock can briefly put the retained
                    # occupancy sample in a different epoch. Keep waiting for
                    # the new mapper sample, then fail closed at the deadline.
                    occupancy_clock_error = str(error)
                    continue
                occupancy_stamp_ns = message_stamp_ns(latest["occupancy"])
                current_xy = planar_position(latest["odom"])
                if occupancy_stamp_ns != occupancy_anchor_stamp_ns:
                    occupancy_anchor_stamp_ns = occupancy_stamp_ns
                    occupancy_anchor_xy = current_xy
                if stale_occupancy_age_s > args.max_occupancy_age_s:
                    (
                        using_cached_occupancy,
                        cached_occupancy_motion_m,
                    ) = cached_occupancy_start_is_valid(
                        occupancy_age_s=stale_occupancy_age_s,
                        maximum_age_s=args.max_occupancy_age_s,
                        anchor_xy=occupancy_anchor_xy,
                        current_xy=current_xy,
                        maximum_motion_m=(
                            args.max_cached_occupancy_motion_m
                        ),
                        router_status=validate_router_status(
                            latest["router"]
                        ),
                    )
                    if not using_cached_occupancy:
                        continue
            try:
                validate_command_graph(
                    observed_graph,
                    robot_id=args.robot_id,
                    mode=args.mode,
                    pre_bridge_full_check=args.pre_bridge_full_check,
                )
            except ValueError:
                continue
            break
        missing = sorted(required_messages - latest.keys())
        if missing:
            raise TimeoutError(
                "timed out waiting for fresh ROS messages: "
                + ", ".join(missing)
            )
        if (
            stale_occupancy_age_s is not None
            and stale_occupancy_age_s > args.max_occupancy_age_s
            and not using_cached_occupancy
        ):
            raise TimeoutError(
                "occupancy remained stale: "
                f"age={stale_occupancy_age_s:.3f}s, "
                f"limit={args.max_occupancy_age_s:.3f}s"
            )
        if occupancy_clock_error is not None:
            raise TimeoutError(
                "occupancy/source clock did not converge: "
                f"{occupancy_clock_error}"
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

        validate_command_graph(
            observed_graph,
            robot_id=args.robot_id,
            mode=args.mode,
            pre_bridge_full_check=args.pre_bridge_full_check,
        )

        occupancy = validate_occupancy(
            latest["occupancy"], frame_id=args.frame_id
        )
        if args.max_occupancy_age_s > 0:
            fresh_reference = max(
                (
                    latest[f"image_{index}"]
                    for index in range(len(args.fresh_image_topic))
                ),
                key=message_stamp_ns,
            )
            occupancy["age_s"] = message_lag_s(
                latest["occupancy"],
                reference_message=fresh_reference,
            )
            occupancy["maximum_age_s"] = args.max_occupancy_age_s
            occupancy["age_reference"] = "fresh_image_source_timestamp"
            occupancy["freshness_policy"] = (
                "bounded_stationary_cached_map"
                if using_cached_occupancy
                else "strict_source_age"
            )
            if using_cached_occupancy:
                occupancy["cached_motion_m"] = cached_occupancy_motion_m
                occupancy["maximum_cached_motion_m"] = (
                    args.max_cached_occupancy_motion_m
                )
        report: dict[str, object] = {
            "schema_version": "focus-tinynav-data-plane-verification-v1",
            "robot_id": args.robot_id,
            "mode": args.mode,
            "verification_scope": (
                "pre_bridge_full"
                if args.pre_bridge_full_check
                else "full"
            ),
            "passed": True,
            "robot_commands_issued": False,
            "odometry": {
                "frame_id": str(odom.header.frame_id),
                "child_frame_id": str(odom.child_frame_id),
            },
            "occupancy": occupancy,
            "router": validate_router_status(latest["router"]),
            "fresh_image_topics": list(args.fresh_image_topic),
            "command_graph": observed_graph,
        }
        if args.platform_status_topic:
            report["platform"] = validate_water_status(
                latest["platform"], expected_live=args.mode == "live"
            )
        if args.geometry_image_topic:
            report["geometry"] = validate_geometry_contract(
                latest["geometry_image"],
                latest["camera_info"],
                expected_frame=args.camera_frame,
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
    parser.add_argument(
        "--pre-bridge-full-check",
        action="store_true",
        help=(
            "in WSJ live mode, fully validate sensors/maps and require the "
            "guarded chassis subscriber to remain absent"
        ),
    )
    parser.add_argument(
        "--post-bridge-command-check",
        action="store_true",
        help=(
            "after the WSJ Go2 bridge starts, validate only its exclusive "
            "guarded command route; requires a prior full check"
        ),
    )
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
    parser.add_argument(
        "--geometry-image-topic",
        default="",
        help="image whose dimensions/frame must match --camera-info-topic",
    )
    parser.add_argument("--camera-info-topic", default="")
    parser.add_argument(
        "--max-occupancy-age-s",
        type=float,
        default=0.0,
        help="maximum source timestamp age; zero disables this check",
    )
    parser.add_argument(
        "--max-cached-occupancy-motion-m",
        type=float,
        default=0.0,
        help=(
            "allow an older latched grid only while the router reports "
            "HOLD/NO_GOAL and odometry moves no farther than this distance"
        ),
    )
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/focus_guarded_cmd_vel")
    parser.add_argument("--target-topic", default="/control/target_pose")
    parser.add_argument("--poi-topic", default="/mapping/cmd_pois")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    if args.pre_bridge_full_check and args.post_bridge_command_check:
        parser.error(
            "--pre-bridge-full-check and --post-bridge-command-check "
            "are mutually exclusive"
        )
    if args.pre_bridge_full_check and (
        args.robot_id != "robot-0" or args.mode != "live"
    ):
        parser.error(
            "--pre-bridge-full-check is only valid for robot-0 live mode"
        )
    if args.post_bridge_command_check and (
        args.robot_id != "robot-0" or args.mode != "live"
    ):
        parser.error(
            "--post-bridge-command-check is only valid for robot-0 live mode"
        )
    if args.post_bridge_command_check and (
        args.platform_status_topic
        or args.fresh_image_topic
        or args.geometry_image_topic
        or args.camera_info_topic
        or args.max_occupancy_age_s > 0
        or args.max_cached_occupancy_motion_m > 0
    ):
        parser.error(
            "--post-bridge-command-check does not accept sensor/map checks; "
            "run --pre-bridge-full-check first"
        )
    if bool(args.geometry_image_topic) != bool(args.camera_info_topic):
        parser.error(
            "--geometry-image-topic and --camera-info-topic must be used together"
        )
    if args.max_occupancy_age_s < 0:
        parser.error("--max-occupancy-age-s must not be negative")
    if args.max_occupancy_age_s > 0 and not args.fresh_image_topic:
        parser.error(
            "--max-occupancy-age-s requires at least one --fresh-image-topic"
        )
    if not 0.0 <= args.max_cached_occupancy_motion_m <= 2.0:
        parser.error(
            "--max-cached-occupancy-motion-m must be within [0, 2]"
        )
    if (
        args.max_cached_occupancy_motion_m > 0
        and args.max_occupancy_age_s <= 0
    ):
        parser.error(
            "--max-cached-occupancy-motion-m requires "
            "--max-occupancy-age-s"
        )
    try:
        report = run(args)
    except Exception as exc:  # noqa: BLE001 - startup must fail closed
        print(f"TinyNav data-plane verification failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
