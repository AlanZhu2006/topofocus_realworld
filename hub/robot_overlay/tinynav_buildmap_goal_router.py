#!/usr/bin/env python3
"""Route an expiring Hub POI over a fresh online TinyNav occupancy map.

This deployment overlay replaces only the saved-map ``MapNode`` boundary:

* input: versioned POI JSON on ``/mapping/cmd_pois``;
* map: live ``nav_msgs/OccupancyGrid`` with unknown cells preserved;
* output: a bounded intermediate pose on ``/control/target_pose`` for
  TinyNav's unchanged local planner.

It never imports a robot SDK and never publishes a velocity command. Unknown
cells and occupied cells are blocked. Goal expiry, stale pose/map input, an
unreachable goal, or process shutdown clears TinyNav's target locally.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import heapq
import json
import math
import sys
from pathlib import Path
import threading
import time
from typing import Any


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.base_camera_calibration import (  # noqa: E402
    BaseCameraCalibration,
    load_base_camera_calibration,
)
from focus_hub.geometry import compose_rigid, invert_rigid  # noqa: E402
from focus_hub.v2_robot_runtime import OccupancyGrid2D  # noqa: E402


@dataclass(frozen=True)
class OnlineGoal:
    decision_id: str
    leg_id: str
    lease_sequence: int
    expires_at_ns: int
    x: float
    y: float
    z: float
    yaw_rad: float
    arrival_radius_m: float
    target_kind: str


@dataclass(frozen=True)
class RoutePlan:
    cells: tuple[tuple[int, int], ...]
    target_cell: tuple[int, int]
    length_m: float
    start_snap_distance_m: float = 0.0
    reaches_arrival_region: bool = True
    remaining_goal_distance_m: float = 0.0


def quaternion_pose_matrix(pose: Any) -> tuple[float, ...]:
    """Convert a ROS-like pose to a validated row-major rigid transform."""

    position = pose.position
    quaternion = pose.orientation
    qx, qy, qz, qw = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("pose quaternion has zero/non-finite norm")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    x, y, z = float(position.x), float(position.y), float(position.z)
    if not all(math.isfinite(value) for value in (x, y, z, qx, qy, qz, qw)):
        raise ValueError("pose contains a non-finite value")
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


def tracking_T_base_from_camera_pose(
    tracking_T_camera: tuple[float, ...],
    base_T_camera: tuple[float, ...],
) -> tuple[float, ...]:
    """Use the measured mount to recover base pose from camera odometry."""

    return compose_rigid(tracking_T_camera, invert_rigid(base_T_camera))


def cached_map_valid_for_pose(
    *,
    map_age_s: float,
    map_timeout_s: float,
    map_anchor_base_xy: tuple[float, float] | None,
    current_base_xy: tuple[float, float] | None,
    max_cached_map_motion_m: float,
) -> tuple[bool, float | None]:
    """Keep a latched world map only within a bounded base displacement."""

    if not all(
        math.isfinite(value)
        for value in (
            map_age_s,
            map_timeout_s,
            max_cached_map_motion_m,
        )
    ):
        raise ValueError("map-age gate contains a non-finite value")
    if map_age_s < 0 or map_timeout_s <= 0 or max_cached_map_motion_m < 0:
        raise ValueError("map-age gate values are outside their valid range")
    if map_age_s <= map_timeout_s:
        return True, 0.0
    if map_anchor_base_xy is None or current_base_xy is None:
        return False, None
    displacement_m = math.hypot(
        current_base_xy[0] - map_anchor_base_xy[0],
        current_base_xy[1] - map_anchor_base_xy[1],
    )
    return displacement_m <= max_cached_map_motion_m, displacement_m


def parse_goal_payload(raw: str, *, now_ns: int) -> OnlineGoal:
    """Strictly parse the single-POI JSON emitted by ``V2GoalAdapter``."""

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError("POI payload is not valid JSON") from exc
    if not isinstance(payload, dict) or len(payload) != 1:
        raise ValueError("POI payload must contain exactly one goal")
    entry = next(iter(payload.values()))
    if not isinstance(entry, dict):
        raise ValueError("POI entry must be an object")
    if entry.get("source") != "focus_hub_v2":
        raise ValueError("POI source is not focus_hub_v2")
    target_kind = str(entry.get("target_kind", "")).strip()
    if target_kind not in {"FRONTIER_POINT", "SEMANTIC_REGION"}:
        raise ValueError("POI target_kind is unsupported")
    position = entry.get("position")
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError("POI position must contain x, y and z")
    try:
        x, y, z = (float(value) for value in position)
        yaw_rad = float(entry.get("yaw_rad", 0.0))
        lease_sequence = int(entry["lease_sequence"])
        expires_at_ns = int(entry["expires_at_ns"])
        arrival_radius_m = float(entry.get("arrival_radius_m", 0.35))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("POI numeric fields are malformed") from exc
    if not all(
        math.isfinite(value) for value in (x, y, z, yaw_rad, arrival_radius_m)
    ):
        raise ValueError("POI contains a non-finite value")
    if not 0.10 <= arrival_radius_m <= 2.0:
        raise ValueError("POI arrival radius is outside [0.10, 2.0] m")
    if expires_at_ns <= now_ns:
        raise ValueError("POI lease is already expired")
    if lease_sequence < 0:
        raise ValueError("POI lease sequence is negative")
    decision_id = str(entry.get("decision_id", "")).strip()
    leg_id = str(entry.get("leg_id", "")).strip()
    if not decision_id or not leg_id:
        raise ValueError("POI decision_id and leg_id are required")
    return OnlineGoal(
        decision_id=decision_id,
        leg_id=leg_id,
        lease_sequence=lease_sequence,
        expires_at_ns=expires_at_ns,
        x=x,
        y=y,
        z=z,
        yaw_rad=yaw_rad,
        arrival_radius_m=arrival_radius_m,
        target_kind=target_kind,
    )


def is_seamless_lease_renewal(current: OnlineGoal, replacement: OnlineGoal) -> bool:
    """Accept only a newer lease for the exact same local navigation leg."""

    return (
        replacement.leg_id == current.leg_id
        and replacement.target_kind == current.target_kind
        and replacement.lease_sequence > current.lease_sequence
        and replacement.expires_at_ns > current.expires_at_ns
        and all(
            math.isclose(first, second, abs_tol=1e-6)
            for first, second in (
                (replacement.x, current.x),
                (replacement.y, current.y),
                (replacement.z, current.z),
                (replacement.yaw_rad, current.yaw_rad),
                (replacement.arrival_radius_m, current.arrival_radius_m),
            )
        )
    )


def _heuristic_m(
    grid: OccupancyGrid2D,
    cell: tuple[int, int],
    goal_x: float,
    goal_y: float,
    arrival_radius_m: float,
) -> float:
    x_m, y_m = grid.cell_center(*cell)
    return max(0.0, math.hypot(x_m - goal_x, y_m - goal_y) - arrival_radius_m)


def _is_arrival_cell(
    grid: OccupancyGrid2D,
    cell: tuple[int, int],
    goal_x: float,
    goal_y: float,
    arrival_radius_m: float,
) -> bool:
    x_m, y_m = grid.cell_center(*cell)
    return math.hypot(x_m - goal_x, y_m - goal_y) <= arrival_radius_m


def plan_route(
    grid: OccupancyGrid2D,
    *,
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
    arrival_radius_m: float,
    clearance_cells: int,
    start_snap_radius_m: float = 0.0,
    start_footprint_override_m: float = 0.0,
    allow_partial_progress: bool = False,
    minimum_progress_m: float = 0.10,
) -> RoutePlan | None:
    """A* through known-free cells, optionally ending at an online-map edge."""

    if clearance_cells < 0:
        raise ValueError("clearance_cells must be non-negative")
    if (
        not math.isfinite(start_snap_radius_m)
        or start_snap_radius_m < 0
        or not math.isfinite(start_footprint_override_m)
        or start_footprint_override_m < 0
    ):
        raise ValueError(
            "start distances must be finite and non-negative"
        )
    if not math.isfinite(arrival_radius_m) or arrival_radius_m <= 0:
        raise ValueError("arrival_radius_m must be finite and positive")
    if not math.isfinite(minimum_progress_m) or minimum_progress_m < 0:
        raise ValueError("minimum_progress_m must be finite and non-negative")
    start = grid.cell(start_x, start_y)
    if start is None or not grid.free_with_clearance(
        *start, clearance_cells=clearance_cells
    ):
        if start_snap_radius_m <= 0:
            return None
        start = grid.nearest_clearance_seed(
            start_x,
            start_y,
            clearance_cells=clearance_cells,
            max_distance_m=start_snap_radius_m,
            start_footprint_override_m=start_footprint_override_m,
        )
        if start is None:
            return None
    start_center = grid.cell_center(*start)
    start_snap_distance_m = math.hypot(
        start_center[0] - start_x, start_center[1] - start_y
    )

    frontier: list[tuple[float, float, tuple[int, int]]] = [
        (
            _heuristic_m(
                grid, start, goal_x, goal_y, arrival_radius_m
            ),
            0.0,
            start,
        )
    ]
    best_cost = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    target: tuple[int, int] | None = None
    best_partial = start
    best_partial_distance_m = math.hypot(
        start_center[0] - goal_x, start_center[1] - goal_y
    )
    reaches_arrival_region = False
    directions = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )

    while frontier:
        _score, cost, current = heapq.heappop(frontier)
        if cost > best_cost.get(current, math.inf) + 1e-12:
            continue
        if _is_arrival_cell(
            grid, current, goal_x, goal_y, arrival_radius_m
        ):
            target = current
            reaches_arrival_region = True
            break
        current_center = grid.cell_center(*current)
        current_distance_m = math.hypot(
            current_center[0] - goal_x, current_center[1] - goal_y
        )
        if (
            current_distance_m < best_partial_distance_m - 1e-12
            or (
                math.isclose(
                    current_distance_m,
                    best_partial_distance_m,
                    abs_tol=1e-12,
                )
                and cost > best_cost.get(best_partial, 0.0)
            )
        ):
            best_partial = current
            best_partial_distance_m = current_distance_m
        row, column = current
        for delta_row, delta_column, step_cells in directions:
            candidate = (row + delta_row, column + delta_column)
            if not grid.free_with_clearance(
                *candidate, clearance_cells=clearance_cells
            ):
                continue
            if delta_row and delta_column:
                # Never cut diagonally through the corner of an obstacle.
                if not grid.free_with_clearance(
                    row + delta_row,
                    column,
                    clearance_cells=clearance_cells,
                ) or not grid.free_with_clearance(
                    row,
                    column + delta_column,
                    clearance_cells=clearance_cells,
                ):
                    continue
            candidate_cost = cost + step_cells * grid.resolution_m
            if candidate_cost >= best_cost.get(candidate, math.inf) - 1e-12:
                continue
            best_cost[candidate] = candidate_cost
            parent[candidate] = current
            estimate = candidate_cost + _heuristic_m(
                grid, candidate, goal_x, goal_y, arrival_radius_m
            )
            heapq.heappush(frontier, (estimate, candidate_cost, candidate))

    if target is None:
        start_distance_m = math.hypot(
            start_center[0] - goal_x, start_center[1] - goal_y
        )
        progress_m = start_distance_m - best_partial_distance_m
        if (
            not allow_partial_progress
            or best_partial == start
            or progress_m < minimum_progress_m
        ):
            return None
        target = best_partial
    cells = [target]
    while cells[-1] != start:
        cells.append(parent[cells[-1]])
    cells.reverse()
    return RoutePlan(
        cells=tuple(cells),
        target_cell=target,
        length_m=float(best_cost[target]),
        start_snap_distance_m=start_snap_distance_m,
        reaches_arrival_region=reaches_arrival_region,
        remaining_goal_distance_m=max(
            0.0,
            math.hypot(
                grid.cell_center(*target)[0] - goal_x,
                grid.cell_center(*target)[1] - goal_y,
            )
            - arrival_radius_m,
        ),
    )


def select_lookahead(
    grid: OccupancyGrid2D,
    plan: RoutePlan,
    *,
    lookahead_m: float,
) -> tuple[float, float]:
    if not math.isfinite(lookahead_m) or lookahead_m <= 0:
        raise ValueError("lookahead_m must be finite and positive")
    if not plan.cells:
        raise ValueError("route contains no cells")
    previous = grid.cell_center(*plan.cells[0])
    selected = previous
    distance = 0.0
    for cell in plan.cells[1:]:
        current = grid.cell_center(*cell)
        distance += math.hypot(current[0] - previous[0], current[1] - previous[1])
        selected = current
        previous = current
        if distance >= lookahead_m:
            break
    return selected


def occupancy_from_ros(message: Any, *, expected_frame: str) -> OccupancyGrid2D:
    orientation = message.info.origin.orientation
    if (
        abs(float(orientation.x)) > 1e-3
        or abs(float(orientation.y)) > 1e-3
        or abs(float(orientation.z)) > 1e-3
        or abs(float(orientation.w) - 1.0) > 1e-3
    ):
        raise ValueError("rotated OccupancyGrid origin is unsupported")
    if message.header.frame_id != expected_frame:
        raise ValueError(
            f"occupancy frame {message.header.frame_id!r} is not {expected_frame!r}"
        )
    return OccupancyGrid2D(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution_m=float(message.info.resolution),
        origin_x_m=float(message.info.origin.position.x),
        origin_y_m=float(message.info.origin.position.y),
        data=tuple(int(value) for value in message.data),
    )


def run_ros(
    args: argparse.Namespace,
    base_camera_calibration: BaseCameraCalibration,
) -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    class OnlineGoalRouter(Node):
        def __init__(self) -> None:
            super().__init__("focus_tinynav_buildmap_goal_router")
            # A* replanning and large OccupancyGrid conversion can each
            # consume the Python executor.  Give odometry its own callback
            # group so a healthy stream cannot age into a false
            # ODOMETRY_STALE hold while a new grid is being materialized.
            self.odom_callback_group = MutuallyExclusiveCallbackGroup()
            self.occupancy_callback_group = MutuallyExclusiveCallbackGroup()
            self.control_callback_group = MutuallyExclusiveCallbackGroup()
            self.sensor_lock = threading.Lock()
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            status_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                args.cmd_pois_topic,
                self.on_goal,
                10,
                callback_group=self.control_callback_group,
            )
            self.create_subscription(
                OccupancyGrid,
                args.occupancy_topic,
                self.on_occupancy,
                map_qos,
                callback_group=self.occupancy_callback_group,
            )
            self.create_subscription(
                Odometry,
                args.odom_topic,
                self.on_odom,
                20,
                callback_group=self.odom_callback_group,
            )
            self.target_publisher = self.create_publisher(
                Odometry, args.target_pose_topic, 10
            )
            self.poi_change_publisher = self.create_publisher(
                Odometry, args.poi_change_topic, 10
            )
            self.nav_done_publisher = self.create_publisher(
                Bool, args.nav_done_topic, 10
            )
            self.progress_publisher = self.create_publisher(
                String, args.progress_topic, 10
            )
            self.path_publisher = self.create_publisher(
                RosPath, args.global_plan_topic, 10
            )
            self.status_publisher = self.create_publisher(
                String, args.status_topic, status_qos
            )
            self.goal: OnlineGoal | None = None
            self.grid: OccupancyGrid2D | None = None
            self.grid_received_monotonic = 0.0
            self.grid_anchor_base_xy: tuple[float, float] | None = None
            self.odom: Odometry | None = None
            self.tracking_T_base: tuple[float, ...] | None = None
            self.odom_received_monotonic = 0.0
            self.target_active = False
            self.last_plan_monotonic = 0.0
            self.initial_route_length_m: float | None = None
            self.last_status: tuple[str, str] | None = None
            self.last_status_fields: dict[str, object] = {}
            self.last_status_publish_monotonic = 0.0
            self.create_timer(
                0.1,
                self.tick,
                callback_group=self.control_callback_group,
            )
            self.create_timer(
                min(0.5, args.status_heartbeat_s / 2.0),
                self.publish_status_heartbeat,
                callback_group=self.control_callback_group,
            )
            self.publish_status("HOLD", "NO_GOAL")

        def publish_status(self, state: str, reason: str, **fields: object) -> None:
            signature = (state, reason)
            now = time.monotonic()
            if signature != self.last_status:
                self.last_status_fields = {}
            if fields:
                self.last_status_fields = dict(fields)
            if (
                signature == self.last_status
                and not fields
                and now - self.last_status_publish_monotonic
                < args.status_heartbeat_s
            ):
                return
            self.last_status = signature
            self.last_status_publish_monotonic = now
            message = String()
            message.data = json.dumps(
                {
                    "t_ns": time.time_ns(),
                    "state": state,
                    "reason": reason,
                    "decision_id": self.goal.decision_id if self.goal else None,
                    **self.last_status_fields,
                },
                separators=(",", ":"),
            )
            self.status_publisher.publish(message)

        def publish_status_heartbeat(self) -> None:
            if self.last_status is not None:
                self.publish_status(*self.last_status)

        def clear_target(
            self,
            reason: str,
            *,
            discard_goal: bool,
            **fields: object,
        ) -> None:
            affected_decision_id = (
                self.goal.decision_id if self.goal is not None else None
            )
            if self.target_active:
                reset = Odometry()
                reset.header.stamp = self.get_clock().now().to_msg()
                reset.header.frame_id = args.frame_id
                self.poi_change_publisher.publish(reset)
            self.target_active = False
            self.initial_route_length_m = None
            if discard_goal:
                self.goal = None
            self.publish_status(
                "HOLD",
                reason,
                affected_decision_id=affected_decision_id,
                **fields,
            )

        def on_goal(self, message: String) -> None:
            try:
                goal = parse_goal_payload(message.data, now_ns=time.time_ns())
            except ValueError as exc:
                self.clear_target("INVALID_OR_EXPIRED_GOAL", discard_goal=True)
                self.get_logger().warning(str(exc))
                return
            if self.goal is not None and self.goal.decision_id == goal.decision_id:
                return
            if self.goal is not None and self.goal.leg_id == goal.leg_id:
                if not is_seamless_lease_renewal(self.goal, goal):
                    self.clear_target(
                        "INVALID_LEASE_RENEWAL", discard_goal=True
                    )
                    return
                self.goal = goal
                self.publish_status("ACCEPTED", "LEASE_RENEWED")
                return
            self.clear_target("GOAL_REPLACED", discard_goal=True)
            self.goal = goal
            self.initial_route_length_m = None
            self.last_plan_monotonic = 0.0
            self.nav_done_publisher.publish(Bool(data=False))
            self.publish_status("ACCEPTED", "FRESH_VERSIONED_GOAL")

        def on_occupancy(self, message: OccupancyGrid) -> None:
            try:
                grid = occupancy_from_ros(
                    message, expected_frame=args.frame_id
                )
                received_monotonic = time.monotonic()
                with self.sensor_lock:
                    self.grid = grid
                    self.grid_received_monotonic = received_monotonic
                    self.grid_anchor_base_xy = (
                        None
                        if self.tracking_T_base is None
                        else (
                            self.tracking_T_base[3],
                            self.tracking_T_base[7],
                        )
                    )
            except ValueError as exc:
                with self.sensor_lock:
                    self.grid = None
                    self.grid_anchor_base_xy = None
                self.get_logger().warning(str(exc))

        def on_odom(self, message: Odometry) -> None:
            if message.header.frame_id != args.frame_id:
                self.get_logger().warning(
                    f"odometry frame {message.header.frame_id!r} is not "
                    f"{args.frame_id!r}"
                )
                return
            try:
                tracking_T_camera = quaternion_pose_matrix(message.pose.pose)
                tracking_T_base = tracking_T_base_from_camera_pose(
                    tracking_T_camera,
                    base_camera_calibration.matrix,
                )
            except ValueError:
                return
            received_monotonic = time.monotonic()
            with self.sensor_lock:
                self.odom = message
                self.tracking_T_base = tracking_T_base
                self.odom_received_monotonic = received_monotonic
                if self.grid is not None and self.grid_anchor_base_xy is None:
                    self.grid_anchor_base_xy = (
                        tracking_T_base[3],
                        tracking_T_base[7],
                    )

        def publish_route(
            self, plan: RoutePlan, grid: OccupancyGrid2D
        ) -> None:
            message = RosPath()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = args.frame_id
            for row, column in plan.cells:
                x_m, y_m = grid.cell_center(row, column)
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = x_m
                pose.pose.position.y = y_m
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
            self.path_publisher.publish(message)

        def publish_target(
            self, x_m: float, y_m: float, odom: Odometry
        ) -> None:
            message = Odometry()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = args.frame_id
            message.child_frame_id = "camera"
            message.pose.pose.position.x = x_m
            message.pose.pose.position.y = y_m
            message.pose.pose.position.z = float(
                odom.pose.pose.position.z
            )
            message.pose.pose.orientation = odom.pose.pose.orientation
            self.target_publisher.publish(message)
            self.target_active = True

        def publish_progress(self, plan: RoutePlan) -> None:
            if self.goal is None:
                return
            if self.initial_route_length_m is None:
                self.initial_route_length_m = max(
                    plan.length_m, self.goal.arrival_radius_m
                )
            total = self.initial_route_length_m
            covered = max(0.0, total - plan.length_m)
            message = String()
            message.data = json.dumps(
                {
                    "decision_id": self.goal.decision_id,
                    "percent": round(min(100.0, 100.0 * covered / total), 1),
                    "path_remaining_m": round(plan.length_m, 3),
                    "path_total_m": round(total, 3),
                    "planner": "tinynav-buildmap-online-a-star-v1",
                },
                separators=(",", ":"),
            )
            self.progress_publisher.publish(message)

        def tick(self) -> None:
            goal = self.goal
            if goal is None:
                return
            if time.time_ns() >= goal.expires_at_ns:
                self.clear_target("LEASE_EXPIRED", discard_goal=True)
                return
            now = time.monotonic()
            with self.sensor_lock:
                odom = self.odom
                tracking_T_base = self.tracking_T_base
                odom_received_monotonic = self.odom_received_monotonic
                grid = self.grid
                grid_received_monotonic = self.grid_received_monotonic
                grid_anchor_base_xy = self.grid_anchor_base_xy
            odom_age_s = (
                None
                if odom is None or tracking_T_base is None
                else max(0.0, now - odom_received_monotonic)
            )
            if (
                odom is None
                or tracking_T_base is None
                or odom_age_s is None
                or odom_age_s > args.input_timeout_s
            ):
                self.clear_target(
                    "ODOMETRY_STALE",
                    discard_goal=False,
                    odom_age_s=(
                        None
                        if odom_age_s is None
                        else round(odom_age_s, 3)
                    ),
                    input_timeout_s=args.input_timeout_s,
                )
                return
            if grid is None:
                self.clear_target("OCCUPANCY_MISSING", discard_goal=False)
                return
            base_x = tracking_T_base[3]
            base_y = tracking_T_base[7]
            map_age_s = now - grid_received_monotonic
            cached_map_valid, cached_map_motion_m = cached_map_valid_for_pose(
                map_age_s=map_age_s,
                map_timeout_s=args.map_timeout_s,
                map_anchor_base_xy=grid_anchor_base_xy,
                current_base_xy=(base_x, base_y),
                max_cached_map_motion_m=args.max_cached_map_motion_m,
            )
            if not cached_map_valid:
                self.clear_target(
                    (
                        "OCCUPANCY_STALE_NO_POSE_ANCHOR"
                        if cached_map_motion_m is None
                        else "OCCUPANCY_STALE_AFTER_MOTION"
                    ),
                    discard_goal=False,
                    occupancy_age_s=round(map_age_s, 3),
                    cached_map_motion_m=(
                        None
                        if cached_map_motion_m is None
                        else round(cached_map_motion_m, 3)
                    ),
                    max_cached_map_motion_m=round(
                        args.max_cached_map_motion_m, 3
                    ),
                    map_timeout_s=round(args.map_timeout_s, 3),
                )
                return
            using_cached_map = map_age_s > args.map_timeout_s
            distance = math.hypot(base_x - goal.x, base_y - goal.y)
            if distance <= goal.arrival_radius_m:
                self.clear_target("ARRIVED", discard_goal=True)
                self.nav_done_publisher.publish(Bool(data=True))
                self.publish_status("ARRIVED", "ARRIVAL_RADIUS_REACHED")
                return
            if now - self.last_plan_monotonic < args.plan_period_s:
                return
            self.last_plan_monotonic = now
            clearance_cells = max(
                0, math.ceil(args.clearance_m / grid.resolution_m)
            )
            plan_started = time.monotonic()
            plan = plan_route(
                grid,
                start_x=base_x,
                start_y=base_y,
                goal_x=goal.x,
                goal_y=goal.y,
                arrival_radius_m=goal.arrival_radius_m,
                clearance_cells=clearance_cells,
                start_snap_radius_m=args.start_snap_radius_m,
                start_footprint_override_m=(
                    args.start_footprint_override_m
                ),
                allow_partial_progress=goal.target_kind
                in {"FRONTIER_POINT", "SEMANTIC_REGION"},
                minimum_progress_m=args.minimum_partial_progress_m,
            )
            plan_duration_s = time.monotonic() - plan_started
            if plan is None:
                self.clear_target(
                    "NO_KNOWN_FREE_PATH",
                    discard_goal=False,
                    plan_duration_s=round(plan_duration_s, 3),
                )
                return
            waypoint_x, waypoint_y = select_lookahead(
                grid, plan, lookahead_m=args.lookahead_m
            )
            self.publish_route(plan, grid)
            self.publish_target(waypoint_x, waypoint_y, odom)
            self.publish_progress(plan)
            self.publish_status(
                "NAVIGATING",
                (
                    (
                        "ONLINE_PATH_READY"
                        if plan.reaches_arrival_region
                        else "ONLINE_PARTIAL_PATH_READY"
                    )
                    + ("_CACHED_MAP" if using_cached_map else "")
                ),
                route_length_m=round(plan.length_m, 3),
                remaining_goal_distance_m=round(
                    plan.remaining_goal_distance_m, 3
                ),
                start_snap_distance_m=round(
                    plan.start_snap_distance_m, 3
                ),
                start_footprint_override_m=round(
                    args.start_footprint_override_m, 3
                ),
                odom_age_s=round(odom_age_s, 3),
                plan_duration_s=round(plan_duration_s, 3),
                waypoint=[round(waypoint_x, 3), round(waypoint_y, 3)],
                occupancy_age_s=round(map_age_s, 3),
                cached_map_motion_m=(
                    None
                    if cached_map_motion_m is None
                    else round(cached_map_motion_m, 3)
                ),
            )

    rclpy.init()
    node = OnlineGoalRouter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    exit_code = 0
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.clear_target("OPERATOR_INTERRUPT", discard_goal=True)
    except Exception as exc:  # noqa: BLE001 - any router fault must clear target
        exit_code = 3
        node.clear_target("ROUTER_FAULT", discard_goal=True)
        node.get_logger().error(str(exc))
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument(
        "--base-camera-calibration-file",
        type=Path,
        required=True,
        help="measured base_link_T_camera artifact for robot-0",
    )
    parser.add_argument("--cmd-pois-topic", default="/mapping/cmd_pois")
    parser.add_argument(
        "--occupancy-topic", default="/semantic_mapping/occupancy_bev"
    )
    parser.add_argument("--odom-topic", default="/slam/odometry")
    parser.add_argument("--target-pose-topic", default="/control/target_pose")
    parser.add_argument("--poi-change-topic", default="/mapping/poi_change")
    parser.add_argument("--nav-done-topic", default="/mapping/nav_done")
    parser.add_argument("--progress-topic", default="/mapping/nav_progress")
    parser.add_argument("--global-plan-topic", default="/mapping/global_plan")
    parser.add_argument(
        "--status-topic", default="/mapping/buildmap_online_status"
    )
    parser.add_argument("--lookahead-m", type=float, default=1.0)
    parser.add_argument("--clearance-m", type=float, default=0.05)
    parser.add_argument(
        "--minimum-partial-progress-m", type=float, default=0.10
    )
    parser.add_argument(
        "--start-snap-radius-m",
        type=float,
        default=0.35,
        help=(
            "maximum known-free centerline advance used only when the fresh "
            "map boundary invalidates the current footprint"
        ),
    )
    parser.add_argument(
        "--start-footprint-override-m",
        type=float,
        default=0.18,
        help=(
            "bounded measured-base footprint through which a non-free online "
            "map start may connect to a genuinely clearance-safe seed"
        ),
    )
    parser.add_argument("--input-timeout-s", type=float, default=1.0)
    parser.add_argument("--map-timeout-s", type=float, default=6.0)
    parser.add_argument(
        "--max-cached-map-motion-m",
        type=float,
        default=0.10,
        help=(
            "a transient-local occupancy may remain valid beyond the time "
            "limit only while the measured base stays within this distance; "
            "the deployment launcher aligns it to one source keyframe plus "
            "one occupancy cell"
        ),
    )
    parser.add_argument("--plan-period-s", type=float, default=0.5)
    parser.add_argument("--status-heartbeat-s", type=float, default=1.0)
    args = parser.parse_args()
    if (
        args.lookahead_m <= 0
        or args.clearance_m < 0
        or args.minimum_partial_progress_m < 0
        or args.start_snap_radius_m < 0
        or args.start_footprint_override_m < 0
        or args.input_timeout_s <= 0
        or args.map_timeout_s <= 0
        or args.max_cached_map_motion_m < 0
        or args.plan_period_s <= 0
        or args.status_heartbeat_s <= 0
    ):
        parser.error("distances and timeouts must be positive")
    base_camera_calibration = load_base_camera_calibration(
        args.base_camera_calibration_file,
        expected_robot_id="robot-0",
        expected_camera_frame="camera",
    )
    return run_ros(args, base_camera_calibration)


if __name__ == "__main__":
    raise SystemExit(main())
