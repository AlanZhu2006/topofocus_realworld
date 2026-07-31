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
from pathlib import Path
import sys
import time
from typing import Any


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.geometry import compose_rigid, invert_rigid  # noqa: E402
from focus_hub.v2_robot_runtime import OccupancyGrid2D  # noqa: E402


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


def require_endpoint_set(
    names: list[str],
    *,
    description: str,
    contains_each: tuple[str, ...],
) -> None:
    """Require one distinct endpoint for every named route participant."""

    matches = {
        expected: [name for name in names if expected in name]
        for expected in contains_each
    }
    if (
        len(names) != len(contains_each)
        or any(len(found) != 1 for found in matches.values())
        or any(
            sum(expected in name for expected in contains_each) != 1
            for name in names
        )
    ):
        raise ValueError(
            f"{description} endpoints are not the expected exclusive route: "
            f"{names}"
        )


def validate_command_graph(
    observed_graph: dict[str, dict[str, list[str]]],
    *,
    robot_id: str,
    mode: str,
    pre_bridge_command_check: bool = False,
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
        and (mode == "debug" or pre_bridge_command_check)
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
    # The planner consumes the target to score trajectories. The deployment
    # controller also consumes the same fixed router waypoint so a large-turn
    # recovery cannot alternate direction when the first path poses jitter.
    # Require both named subscribers and reject every additional endpoint.
    require_endpoint_set(
        observed_graph["target"]["subscriptions"],
        description="TinyNav target subscribers",
        contains_each=("planning_node", "cmd_vel_control_node"),
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


def _quaternion_pose_matrix(pose: Any) -> tuple[float, ...]:
    position = pose.position
    orientation = pose.orientation
    x = float(position.x)
    y = float(position.y)
    z = float(position.z)
    qx = float(orientation.x)
    qy = float(orientation.y)
    qz = float(orientation.z)
    qw = float(orientation.w)
    values = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry pose contains a non-finite value")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("odometry orientation quaternion has zero norm")
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    return (
        1 - 2 * (qy * qy + qz * qz),
        2 * (qx * qy - qz * qw),
        2 * (qx * qz + qy * qw),
        x,
        2 * (qx * qy + qz * qw),
        1 - 2 * (qx * qx + qz * qz),
        2 * (qy * qz - qx * qw),
        y,
        2 * (qx * qz - qy * qw),
        2 * (qy * qz + qx * qw),
        1 - 2 * (qx * qx + qy * qy),
        z,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def occupancy_grid_view(message: Any, *, frame_id: str) -> OccupancyGrid2D:
    """Build the exact receiver-side occupancy view from a ROS message."""

    validate_occupancy(message, frame_id=frame_id)
    return OccupancyGrid2D(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution_m=float(message.info.resolution),
        origin_x_m=float(message.info.origin.position.x),
        origin_y_m=float(message.info.origin.position.y),
        data=tuple(int(value) for value in message.data),
    )


def validate_start_reachability(
    occupancy_message: Any,
    odometry_message: Any,
    *,
    frame_id: str,
    base_T_camera: tuple[float, ...],
    clearance_m: float,
    start_snap_radius_m: float,
    start_footprint_override_m: float,
) -> dict[str, object]:
    """Require a non-empty receiver-identical component before arming motion."""

    grid = occupancy_grid_view(occupancy_message, frame_id=frame_id)
    tracking_T_camera = _quaternion_pose_matrix(
        odometry_message.pose.pose
    )
    tracking_T_base = compose_rigid(
        tracking_T_camera, invert_rigid(base_T_camera)
    )
    base_x = float(tracking_T_base[3])
    base_y = float(tracking_T_base[7])
    clearance_cells = math.ceil(clearance_m / grid.resolution_m)
    component = grid.reachable_component(
        base_x,
        base_y,
        clearance_cells=clearance_cells,
        start_snap_radius_m=start_snap_radius_m,
        start_footprint_override_m=start_footprint_override_m,
    )
    base_cell = grid.cell(base_x, base_y)
    report = {
        "base_xy_m": [base_x, base_y],
        "base_inside_grid": base_cell is not None,
        "base_cell": None if base_cell is None else list(base_cell),
        "reachable_component_cells": len(component),
        "clearance_cells": clearance_cells,
        "clearance_m": clearance_m,
        "start_snap_radius_m": start_snap_radius_m,
        "start_footprint_override_m": start_footprint_override_m,
        "grid_x_range_m": [
            grid.origin_x_m,
            grid.origin_x_m + grid.width * grid.resolution_m,
        ],
        "grid_y_range_m": [
            grid.origin_y_m,
            grid.origin_y_m + grid.height * grid.resolution_m,
        ],
    }
    if not component:
        raise ValueError(
            "robot base has no start-connected known-free occupancy "
            f"component: {json.dumps(report, separators=(',', ':'))}"
        )
    return report


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
    Yunji map look decades old. The occupancy grid and required fresh sensor
    witness are both derived from the same robot clock, so their source-stamp
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
            f"{abs(age_s):.3f}s ahead of the fresh sensor timestamp"
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


def validate_camera_info_contract(
    camera_info: Any,
    *,
    expected_frame: str = "",
    expected_width: int = 0,
    expected_height: int = 0,
) -> dict[str, object]:
    dimensions = (int(camera_info.width), int(camera_info.height))
    if dimensions[0] <= 0 or dimensions[1] <= 0:
        raise ValueError(
            f"CameraInfo dimensions are invalid: {dimensions}"
        )
    expected_dimensions = (expected_width, expected_height)
    if any(expected_dimensions) and dimensions != expected_dimensions:
        raise ValueError(
            "CameraInfo dimensions differ from the locked profile: "
            f"camera_info={dimensions[0]}x{dimensions[1]}, "
            f"expected={expected_width}x{expected_height}"
        )
    frame_id = str(camera_info.header.frame_id)
    if expected_frame and frame_id != expected_frame:
        raise ValueError(
            f"geometry frame mismatch; expected {expected_frame!r}, "
            f"got CameraInfo={frame_id!r}"
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
        "frame_id": frame_id,
        "width": dimensions[0],
        "height": dimensions[1],
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
    }


def validate_geometry_contract(
    image: Any,
    camera_info: Any,
    *,
    expected_frame: str,
) -> dict[str, object]:
    dimensions = (int(image.width), int(image.height))
    if dimensions[0] <= 0 or dimensions[1] <= 0:
        raise ValueError(f"geometry image dimensions are invalid: {dimensions}")
    image_frame = str(image.header.frame_id)
    if image_frame != expected_frame:
        raise ValueError(
            f"geometry frame mismatch; expected {expected_frame!r}, "
            f"got image={image_frame!r}"
        )
    return validate_camera_info_contract(
        camera_info,
        expected_frame=expected_frame,
        expected_width=dimensions[0],
        expected_height=dimensions[1],
    )


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
    from focus_hub.base_camera_calibration import (
        load_base_camera_calibration,
    )
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

    base_camera_calibration = None
    if args.require_reachable_start:
        base_camera_calibration = load_base_camera_calibration(
            args.base_camera_calibration_file,
            expected_robot_id=args.robot_id,
            expected_camera_frame=args.camera_frame,
        )

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
    occupancy_updates: list[tuple[int, float]] = []

    def receive_occupancy(message: Any) -> None:
        latest["occupancy"] = message
        stamp_ns = message_stamp_ns(message)
        if not occupancy_updates or (
            occupancy_updates[-1][0] != stamp_ns
        ):
            occupancy_updates.append((stamp_ns, time.monotonic()))
            del occupancy_updates[:-8]

    message_checks_enabled = not (
        args.command_graph_only or args.post_bridge_command_check
    )
    if message_checks_enabled:
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
                    receive_occupancy,
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
    if args.platform_status_topic and message_checks_enabled:
        subscriptions.append(
            node.create_subscription(
                String,
                args.platform_status_topic,
                lambda message: latest.__setitem__("platform", message),
                volatile_qos,
            )
        )
    if message_checks_enabled:
        image_keys_by_topic: dict[str, list[str]] = {}
        for index, topic in enumerate(args.fresh_image_topic):
            image_keys_by_topic.setdefault(topic, []).append(f"image_{index}")
        if args.geometry_image_topic:
            image_keys_by_topic.setdefault(
                args.geometry_image_topic, []
            ).append("geometry_image")
        for topic, keys in image_keys_by_topic.items():
            subscriptions.append(
                node.create_subscription(
                    Image,
                    topic,
                    lambda message, image_keys=tuple(keys): latest.update(
                        {key: message for key in image_keys}
                    ),
                    volatile_qos,
                )
            )
        camera_info_keys_by_topic: dict[str, list[str]] = {}
        for index, topic in enumerate(args.fresh_camera_info_topic):
            camera_info_keys_by_topic.setdefault(topic, []).append(
                f"fresh_camera_info_{index}"
            )
        if args.camera_info_topic:
            camera_info_keys_by_topic.setdefault(
                args.camera_info_topic, []
            ).append("camera_info")
        for topic, keys in camera_info_keys_by_topic.items():
            subscriptions.append(
                node.create_subscription(
                    CameraInfo,
                    topic,
                    lambda message, camera_info_keys=tuple(keys): latest.update(
                        {key: message for key in camera_info_keys}
                    ),
                    volatile_qos,
                )
            )

    all_topics = {
        "raw": args.raw_cmd_topic,
        "guarded": args.guarded_cmd_topic,
        "target": args.target_topic,
        "poi": args.poi_topic,
    }
    if args.sensor_map_only:
        topics = {}
    elif args.post_bridge_command_check:
        topics = {"guarded": args.guarded_cmd_topic}
    else:
        topics = all_topics

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
    required_messages.update(
        f"fresh_camera_info_{index}"
        for index in range(len(args.fresh_camera_info_topic))
    )
    if args.geometry_image_topic:
        required_messages.add("geometry_image")
    if args.camera_info_topic:
        required_messages.add("camera_info")
    fresh_reference_keys = [
        *(f"image_{index}" for index in range(len(args.fresh_image_topic))),
        *(
            f"fresh_camera_info_{index}"
            for index in range(len(args.fresh_camera_info_topic))
        ),
    ]
    observed_graph: dict[str, dict[str, list[str]]] = {}
    stale_occupancy_age_s: float | None = None
    occupancy_clock_error: str | None = None
    occupancy_anchor_stamp_ns: int | None = None
    occupancy_anchor_xy: tuple[float, float] | None = None
    cached_occupancy_motion_m: float | None = None
    using_cached_occupancy = False
    sensor_ready = False
    occupancy_update_report: dict[str, object] | None = None
    reachability_report: dict[str, object] | None = None
    last_start_reachability_error: ValueError | None = None
    occupancy_update_error: str | None = None
    try:
        if args.command_graph_only:
            last_graph_error: ValueError | None = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                observed_graph = graph()
                try:
                    validate_command_graph(
                        observed_graph,
                        robot_id=args.robot_id,
                        mode=args.mode,
                        pre_bridge_command_check=(
                            args.pre_bridge_command_check
                        ),
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
                    "verification_scope": (
                        "pre_bridge_command_graph"
                        if args.pre_bridge_command_check
                        else "command_graph"
                    ),
                    "passed": True,
                    "robot_commands_issued": False,
                    "sensor_map_verification_required": True,
                    "command_graph": observed_graph,
                }
            detail = (
                str(last_graph_error)
                if last_graph_error is not None
                else "no resolved command endpoints"
            )
            raise TimeoutError(
                "timed out waiting for the exclusive command route: "
                f"{detail}"
            )

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
                    "sensor_map_verification_required": True,
                    "pre_bridge_command_verification_required": True,
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
            if not args.sensor_map_only:
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
            elif args.camera_info_topic:
                # WSJ's long-lived Fast DDS domain can drop every sample for a
                # newly-created full-resolution Image subscriber. TinyNav
                # publishes this lightweight CameraInfo from the same visual
                # processing block as /slam/depth, so it is a lossless
                # per-frame liveness and locked-geometry witness.
                validate_camera_info_contract(
                    latest["camera_info"],
                    expected_frame=args.camera_frame,
                    expected_width=args.geometry_width,
                    expected_height=args.geometry_height,
                )
            stale_occupancy_age_s = None
            occupancy_clock_error = None
            cached_occupancy_motion_m = None
            using_cached_occupancy = False
            if args.max_occupancy_age_s > 0:
                fresh_reference = max(
                    (latest[key] for key in fresh_reference_keys),
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
            occupancy_update_error = None
            occupancy_update_report = None
            if args.minimum_occupancy_updates > 1:
                if (
                    len(occupancy_updates)
                    < args.minimum_occupancy_updates
                ):
                    occupancy_update_error = (
                        "received only "
                        f"{len(occupancy_updates)} unique occupancy "
                        f"updates; need {args.minimum_occupancy_updates}"
                    )
                    continue
                selected_updates = occupancy_updates[
                    -args.minimum_occupancy_updates :
                ]
                source_intervals_s = [
                    (right[0] - left[0]) * 1e-9
                    for left, right in zip(
                        selected_updates, selected_updates[1:]
                    )
                ]
                receive_intervals_s = [
                    right[1] - left[1]
                    for left, right in zip(
                        selected_updates, selected_updates[1:]
                    )
                ]
                if any(
                    interval <= 0.0
                    for interval in (
                        *source_intervals_s,
                        *receive_intervals_s,
                    )
                ):
                    occupancy_update_error = (
                        "occupancy source or receive time did not advance"
                    )
                    continue
                maximum_interval_s = max(
                    *source_intervals_s,
                    *receive_intervals_s,
                    0.0,
                )
                occupancy_update_report = {
                    "unique_updates": len(selected_updates),
                    "source_intervals_s": source_intervals_s,
                    "receive_intervals_s": receive_intervals_s,
                    "maximum_interval_s": maximum_interval_s,
                    "maximum_allowed_interval_s": (
                        args.maximum_occupancy_update_interval_s
                    ),
                }
                if (
                    args.maximum_occupancy_update_interval_s > 0.0
                    and maximum_interval_s
                    > args.maximum_occupancy_update_interval_s
                ):
                    occupancy_update_error = (
                        "occupancy update interval remained too slow: "
                        f"{maximum_interval_s:.3f}s > "
                        f"{args.maximum_occupancy_update_interval_s:.3f}s"
                    )
                    continue
            if args.require_reachable_start:
                if base_camera_calibration is None:
                    raise RuntimeError(
                        "reachable-start verification lacks calibration"
                    )
                try:
                    reachability_report = validate_start_reachability(
                        latest["occupancy"],
                        latest["odom"],
                        frame_id=args.frame_id,
                        base_T_camera=base_camera_calibration.matrix,
                        clearance_m=args.reachability_clearance_m,
                        start_snap_radius_m=args.start_snap_radius_m,
                        start_footprint_override_m=(
                            args.start_footprint_override_m
                        ),
                    )
                except ValueError as error:
                    last_start_reachability_error = error
                    continue
                last_start_reachability_error = None
            if not args.sensor_map_only:
                try:
                    validate_command_graph(
                        observed_graph,
                        robot_id=args.robot_id,
                        mode=args.mode,
                    )
                except ValueError:
                    continue
            sensor_ready = True
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
        if not sensor_ready:
            if last_start_reachability_error is not None:
                raise TimeoutError(
                    "start occupancy never became reachable: "
                    f"{last_start_reachability_error}"
                )
            if occupancy_update_error is not None:
                raise TimeoutError(occupancy_update_error)
            raise TimeoutError(
                "sensor/map contract did not converge before the deadline"
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

        if not args.sensor_map_only:
            validate_command_graph(
                observed_graph,
                robot_id=args.robot_id,
                mode=args.mode,
            )

        occupancy = validate_occupancy(
            latest["occupancy"], frame_id=args.frame_id
        )
        if args.max_occupancy_age_s > 0:
            fresh_reference = max(
                (latest[key] for key in fresh_reference_keys),
                key=message_stamp_ns,
            )
            occupancy["age_s"] = message_lag_s(
                latest["occupancy"],
                reference_message=fresh_reference,
            )
            occupancy["maximum_age_s"] = args.max_occupancy_age_s
            occupancy["age_reference"] = "fresh_sensor_source_timestamp"
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
        if occupancy_update_report is not None:
            occupancy["update_contract"] = occupancy_update_report
        report: dict[str, object] = {
            "schema_version": "focus-tinynav-data-plane-verification-v1",
            "robot_id": args.robot_id,
            "mode": args.mode,
            "verification_scope": (
                "sensor_map" if args.sensor_map_only else "full"
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
            "fresh_camera_info_topics": list(
                args.fresh_camera_info_topic
            ),
        }
        if reachability_report is not None:
            report["start_reachability"] = reachability_report
        if not args.sensor_map_only:
            report["command_graph"] = observed_graph
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
        elif args.camera_info_topic:
            report["geometry"] = validate_camera_info_contract(
                latest["camera_info"],
                expected_frame=args.camera_frame,
                expected_width=args.geometry_width,
                expected_height=args.geometry_height,
            )
            report["geometry"]["verification_source"] = (
                "fresh_camera_info_profile"
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
        "--sensor-map-only",
        action="store_true",
        help=(
            "validate odometry, images, geometry, occupancy and router state "
            "without querying the command graph"
        ),
    )
    parser.add_argument(
        "--command-graph-only",
        action="store_true",
        help=(
            "validate the full exclusive command graph without creating "
            "sensor/image subscriptions"
        ),
    )
    parser.add_argument(
        "--pre-bridge-command-check",
        action="store_true",
        help=(
            "with --command-graph-only in WSJ live mode, require the guarded "
            "chassis subscriber to remain absent"
        ),
    )
    parser.add_argument(
        "--post-bridge-command-check",
        action="store_true",
        help=(
            "after the WSJ Go2 bridge starts, validate only its exclusive "
            "guarded command route; requires prior sensor/map and command checks"
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
        "--fresh-camera-info-topic",
        action="append",
        default=[],
        help=(
            "lightweight CameraInfo topic that must deliver a new volatile "
            "sample; its source stamp may be used for map freshness"
        ),
    )
    parser.add_argument(
        "--geometry-image-topic",
        default="",
        help="image whose dimensions/frame must match --camera-info-topic",
    )
    parser.add_argument("--camera-info-topic", default="")
    parser.add_argument(
        "--geometry-width",
        type=int,
        default=0,
        help="locked CameraInfo width; zero accepts any positive width",
    )
    parser.add_argument(
        "--geometry-height",
        type=int,
        default=0,
        help="locked CameraInfo height; zero accepts any positive height",
    )
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
    parser.add_argument(
        "--minimum-occupancy-updates",
        type=int,
        default=1,
        help=(
            "unique source-stamped grids required before readiness; values "
            "above one reject a retained single sample"
        ),
    )
    parser.add_argument(
        "--maximum-occupancy-update-interval-s",
        type=float,
        default=0.0,
        help=(
            "maximum interval across the required recent grids; zero "
            "disables the interval check"
        ),
    )
    parser.add_argument(
        "--require-reachable-start",
        action="store_true",
        help=(
            "require the measured robot base to have a non-empty known-free "
            "component under the receiver's exact start policy"
        ),
    )
    parser.add_argument(
        "--base-camera-calibration-file",
        type=Path,
        default=Path(),
    )
    parser.add_argument(
        "--reachability-clearance-m", type=float, default=0.05
    )
    parser.add_argument("--start-snap-radius-m", type=float, default=0.0)
    parser.add_argument(
        "--start-footprint-override-m", type=float, default=0.0
    )
    parser.add_argument("--raw-cmd-topic", default="/cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/focus_guarded_cmd_vel")
    parser.add_argument("--target-topic", default="/control/target_pose")
    parser.add_argument("--poi-topic", default="/mapping/cmd_pois")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    scope_count = sum(
        (
            args.sensor_map_only,
            args.command_graph_only,
            args.post_bridge_command_check,
        )
    )
    if scope_count > 1:
        parser.error(
            "--sensor-map-only, --command-graph-only and "
            "--post-bridge-command-check are mutually exclusive"
        )
    if args.pre_bridge_command_check and not args.command_graph_only:
        parser.error(
            "--pre-bridge-command-check requires --command-graph-only"
        )
    if args.pre_bridge_command_check and (
        args.robot_id != "robot-0" or args.mode != "live"
    ):
        parser.error(
            "--pre-bridge-command-check is only valid for robot-0 live mode"
        )
    if args.post_bridge_command_check and (
        args.robot_id != "robot-0" or args.mode != "live"
    ):
        parser.error(
            "--post-bridge-command-check is only valid for robot-0 live mode"
        )
    if (args.command_graph_only or args.post_bridge_command_check) and (
        args.platform_status_topic
        or args.fresh_image_topic
        or args.fresh_camera_info_topic
        or args.geometry_image_topic
        or args.camera_info_topic
        or args.geometry_width != 0
        or args.geometry_height != 0
        or args.max_occupancy_age_s > 0
        or args.max_cached_occupancy_motion_m > 0
    ):
        parser.error(
            "command-graph-only checks do not accept sensor/map options; "
            "run --sensor-map-only first"
        )
    if args.geometry_image_topic and not args.camera_info_topic:
        parser.error(
            "--geometry-image-topic requires --camera-info-topic"
        )
    if bool(args.geometry_width) != bool(args.geometry_height):
        parser.error(
            "--geometry-width and --geometry-height must be used together"
        )
    if args.geometry_width < 0 or args.geometry_height < 0:
        parser.error("locked geometry dimensions must not be negative")
    if (
        (args.geometry_width or args.geometry_height)
        and not args.camera_info_topic
    ):
        parser.error(
            "locked geometry dimensions require --camera-info-topic"
        )
    if args.max_occupancy_age_s < 0:
        parser.error("--max-occupancy-age-s must not be negative")
    if args.minimum_occupancy_updates <= 0:
        parser.error("--minimum-occupancy-updates must be positive")
    if args.maximum_occupancy_update_interval_s < 0.0:
        parser.error(
            "--maximum-occupancy-update-interval-s must not be negative"
        )
    if (
        args.maximum_occupancy_update_interval_s > 0.0
        and args.minimum_occupancy_updates < 2
    ):
        parser.error(
            "--maximum-occupancy-update-interval-s requires at least two "
            "occupancy updates"
        )
    if args.max_occupancy_age_s > 0 and not (
        args.fresh_image_topic or args.fresh_camera_info_topic
    ):
        parser.error(
            "--max-occupancy-age-s requires a fresh Image or CameraInfo topic"
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
    if any(
        value < 0.0
        for value in (
            args.reachability_clearance_m,
            args.start_snap_radius_m,
            args.start_footprint_override_m,
        )
    ):
        parser.error("reachable-start distances must not be negative")
    if args.require_reachable_start and not (
        args.base_camera_calibration_file.is_file()
    ):
        parser.error(
            "--require-reachable-start needs a readable "
            "--base-camera-calibration-file"
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
