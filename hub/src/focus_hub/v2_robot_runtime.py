"""Shared, robot-local runtime pieces for the v2 TinyNav/WATER receivers.

This module deliberately contains no ROS publisher and no velocity command.
The two deployment overlays own their robot-specific planner/control boundary.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import json
import math
import socket
import time
import urllib.error
import urllib.request
import uuid

from .models import HeartbeatAck, LocalizationState, RobotHealth, SafetyState
from .transport_v2 import (
    HighLevelDecisionV2,
    NavigationEventAckV2,
    NavigationEventV2,
    NavigationStatusV2,
    ResolvedLocalGoalV2,
    SemanticRegionTargetV2,
)
from .v2_goal_adapter import LocalHighLevelGoal


class HubV2RobotClient:
    """Small stdlib-only authenticated v2 polling/event client."""

    def __init__(
        self,
        base_url: str,
        robot_id: str,
        token: str,
        *,
        timeout_s: float = 3.0,
    ) -> None:
        if not token:
            raise ValueError("robot token is empty")
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.token = token
        self.timeout_s = timeout_s

    def _request(
        self,
        method: str,
        path: str,
        *,
        body: dict[str, object] | None = None,
    ) -> tuple[int, bytes]:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            method=method,
            headers={
                "X-Robot-Token": self.token,
                "Accept": "application/json",
                **({"Content-Type": "application/json"} if data is not None else {}),
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return int(response.status), response.read()
        except urllib.error.HTTPError as exc:
            payload = exc.read()
            if exc.code == 204:
                return 204, payload
            raise RuntimeError(
                f"Hub {method} {path} failed: HTTP {exc.code} "
                f"{payload.decode('utf-8', 'replace')[:300]}"
            ) from exc

    def latest_decision(self) -> HighLevelDecisionV2 | None:
        status, payload = self._request(
            "GET", f"/v2/robots/{self.robot_id}/decisions/latest"
        )
        if status == 204 or not payload:
            return None
        return HighLevelDecisionV2.model_validate_json(payload)

    def post_event(self, event: NavigationEventV2) -> NavigationEventAckV2:
        if event.robot_id != self.robot_id:
            raise ValueError("event robot ID differs from client")
        _status, payload = self._request(
            "POST",
            f"/v2/robots/{self.robot_id}/navigation-events",
            body=event.model_dump(mode="json"),
        )
        return NavigationEventAckV2.model_validate_json(payload)

    def post_heartbeat(self, health: RobotHealth) -> HeartbeatAck:
        _status, payload = self._request(
            "POST",
            f"/v1/robots/{self.robot_id}/heartbeat",
            body={
                "protocol_version": "1.0",
                "robot_id": self.robot_id,
                "sent_time_ns": time.time_ns(),
                "health": health.model_dump(mode="json"),
            },
        )
        return HeartbeatAck.model_validate_json(payload)


class WaterTcpClient:
    """One-request-per-connection WATER API client matching the vendor guide."""

    def __init__(self, host: str, port: int = 31001, timeout_s: float = 3.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s

    def request(self, path: str, **params: object) -> dict[str, object]:
        request_id = uuid.uuid4().hex[:12]
        query = "&".join(f"{key}={value}" for key, value in params.items())
        line = f"{path}?uuid={request_id}" + (f"&{query}" if query else "") + "\n"
        with socket.create_connection(
            (self.host, self.port), timeout=self.timeout_s
        ) as connection:
            connection.settimeout(self.timeout_s)
            connection.sendall(line.encode("utf-8"))
            reader = connection.makefile("rb")
            for _ in range(40):
                raw = reader.readline()
                if not raw:
                    raise ConnectionError("WATER TCP API closed the connection")
                message = json.loads(raw)
                if (
                    isinstance(message, dict)
                    and message.get("type") == "response"
                    and message.get("uuid") == request_id
                ):
                    return message
        raise TimeoutError(f"no matching WATER response for {path}")


def require_water_ok(response: dict[str, object], *, command: str) -> dict[str, object]:
    status = str(response.get("status", ""))
    error_message = str(response.get("error_message", ""))
    if status.upper() != "OK" or error_message:
        raise RuntimeError(
            f"WATER {command} rejected: status={status!r} error={error_message!r}"
        )
    return response


def parse_water_current_pose(status: dict[str, object]) -> tuple[float, float, float]:
    raw = status.get("current_pose")
    if isinstance(raw, dict):
        values = (raw.get("x"), raw.get("y"), raw.get("theta", raw.get("yaw")))
    elif isinstance(raw, (list, tuple)) and len(raw) >= 3:
        values = (raw[0], raw[1], raw[2])
    elif isinstance(raw, str):
        parts = raw.split(",")
        if len(parts) < 3:
            raise ValueError("WATER current_pose string has fewer than three values")
        values = (parts[0], parts[1], parts[2])
    else:
        raise ValueError("WATER robot_status has no recognized current_pose")
    pose = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in pose):
        raise ValueError("WATER current_pose contains a non-finite value")
    return pose  # type: ignore[return-value]


def water_move_state(status: dict[str, object]) -> str:
    """Normalize only states needed by the receiver; unknown remains unknown."""

    value = str(status.get("move_status", "")).strip().lower()
    if value in {"running", "moving", "executing", "working"}:
        return "ACTIVE"
    if value in {"succeeded", "success", "completed", "finished"}:
        return "ARRIVED"
    if value in {"failed", "failure", "aborted", "error"}:
        return "FAILED"
    if value in {"canceled", "cancelled", "idle", "none", "stopped"}:
        return "ZERO"
    return "UNKNOWN"


def water_robot_health(
    status: dict[str, object],
    *,
    odometry_fresh: bool,
) -> RobotHealth:
    estop = bool(status.get("estop_state") or status.get("hard_estop_state"))
    error_code = str(status.get("error_code", "00000000"))
    error_free = error_code in {"0", "00000000", "", "None", "none"}
    ready = odometry_fresh and not estop and error_free
    battery_raw = status.get("power_percent")
    battery = None if battery_raw is None else float(battery_raw)
    return RobotHealth(
        safety_state=(
            SafetyState.ESTOP
            if estop
            else (SafetyState.READY if ready else SafetyState.HOLD)
        ),
        localization_state=(
            LocalizationState.TRACKING if odometry_fresh else LocalizationState.LOST
        ),
        estop_engaged=estop,
        collision_avoidance_ready=ready,
        motor_controller_ready=ready,
        battery_percent=battery,
        detail=(
            f"local WATER status error_code={error_code} "
            f"move_status={status.get('move_status')} odometry_fresh={odometry_fresh}"
        ),
    )


@dataclass
class PathAccumulator:
    max_step_m: float = 2.0
    length_m: float = 0.0
    last_xy: tuple[float, float] | None = None
    rejected_jumps: int = 0

    def update(self, x: float, y: float) -> float:
        if not all(math.isfinite(value) for value in (x, y)):
            raise ValueError("path pose contains a non-finite value")
        current = (float(x), float(y))
        if self.last_xy is not None:
            step = math.hypot(current[0] - self.last_xy[0], current[1] - self.last_xy[1])
            if step <= self.max_step_m:
                self.length_m += step
            else:
                self.rejected_jumps += 1
        self.last_xy = current
        return self.length_m


@dataclass(frozen=True)
class OccupancyGrid2D:
    """Minimal ROS OccupancyGrid view used for robot-local reachability.

    Only value 0 is traversible. Unknown and occupied cells remain blocked;
    the receiver never converts uncertainty into free space.
    """

    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    data: tuple[int, ...]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("occupancy dimensions must be positive")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("occupancy resolution must be finite and positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy payload length differs from dimensions")

    def cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        column = math.floor((x_m - self.origin_x_m) / self.resolution_m)
        row = math.floor((y_m - self.origin_y_m) / self.resolution_m)
        if not (0 <= row < self.height and 0 <= column < self.width):
            return None
        return row, column

    def _free_with_clearance(self, row: int, column: int, clearance: int) -> bool:
        for check_row in range(row - clearance, row + clearance + 1):
            for check_column in range(column - clearance, column + clearance + 1):
                if not (
                    0 <= check_row < self.height
                    and 0 <= check_column < self.width
                ):
                    return False
                if self.data[check_row * self.width + check_column] != 0:
                    return False
        return True

    def free_with_clearance(
        self,
        row: int,
        column: int,
        *,
        clearance_cells: int = 1,
    ) -> bool:
        """Return whether a cell and its square clearance footprint are free."""

        if clearance_cells < 0:
            raise ValueError("clearance_cells must be non-negative")
        if not (0 <= row < self.height and 0 <= column < self.width):
            return False
        return self._free_with_clearance(row, column, clearance_cells)

    def cell_center(self, row: int, column: int) -> tuple[float, float]:
        if not (0 <= row < self.height and 0 <= column < self.width):
            raise ValueError("occupancy cell is outside the grid")
        return (
            self.origin_x_m + (column + 0.5) * self.resolution_m,
            self.origin_y_m + (row + 0.5) * self.resolution_m,
        )

    def nearest_clearance_seed(
        self,
        start_x_m: float,
        start_y_m: float,
        *,
        clearance_cells: int = 1,
        max_distance_m: float,
        start_footprint_override_m: float = 0.0,
    ) -> tuple[int, int] | None:
        """Find the nearest clearance-safe seed through known-free cells.

        A fresh forward-looking online map can place the robot's free start
        cell beside the unknown map boundary.  In that case footprint
        clearance fails even though the observed centerline immediately ahead
        is free.  This helper may advance the graph-search seed by a tightly
        bounded distance.

        ``start_footprint_override_m`` additionally permits the search to
        leave a non-free start through a tightly bounded disk centered on the
        measured base.  This mirrors the source agent's current-pose
        traversible override without changing the occupancy grid: the returned
        seed must still be genuinely known-free with the requested clearance,
        and no occupied/unknown cell outside the measured footprint is crossed.
        """

        if clearance_cells < 0:
            raise ValueError("clearance_cells must be non-negative")
        if (
            not math.isfinite(max_distance_m)
            or max_distance_m < 0
            or not math.isfinite(start_footprint_override_m)
            or start_footprint_override_m < 0
        ):
            raise ValueError("seed distances must be finite and non-negative")
        start = self.cell(start_x_m, start_y_m)
        if start is None:
            return None
        if (
            self.data[start[0] * self.width + start[1]] != 0
            and start_footprint_override_m <= 0
        ):
            return None

        reached = {start}
        pending = deque([start])
        best: tuple[float, int, int] | None = None
        max_distance_sq = max_distance_m * max_distance_m
        footprint_distance_sq = (
            start_footprint_override_m * start_footprint_override_m
        )
        while pending:
            row, column = pending.popleft()
            center_x, center_y = self.cell_center(row, column)
            distance_sq = (
                (center_x - start_x_m) ** 2
                + (center_y - start_y_m) ** 2
            )
            if (
                distance_sq <= max_distance_sq + 1e-12
                and self._free_with_clearance(
                    row, column, clearance_cells
                )
            ):
                candidate = (distance_sq, row, column)
                if best is None or candidate < best:
                    best = candidate
            for candidate_cell in (
                (row - 1, column),
                (row + 1, column),
                (row, column - 1),
                (row, column + 1),
            ):
                if candidate_cell in reached:
                    continue
                check_row, check_column = candidate_cell
                if not (
                    0 <= check_row < self.height
                    and 0 <= check_column < self.width
                ):
                    continue
                check_value = self.data[
                    check_row * self.width + check_column
                ]
                check_x, check_y = self.cell_center(
                    check_row, check_column
                )
                inside_measured_footprint = (
                    (check_x - start_x_m) ** 2
                    + (check_y - start_y_m) ** 2
                    <= footprint_distance_sq + 1e-12
                )
                if check_value != 0 and not inside_measured_footprint:
                    continue
                reached.add(candidate_cell)
                pending.append(candidate_cell)
        if best is None:
            return None
        return best[1], best[2]

    def reachable_component(
        self,
        start_x_m: float,
        start_y_m: float,
        *,
        clearance_cells: int = 1,
        start_snap_radius_m: float = 0.0,
        start_footprint_override_m: float = 0.0,
    ) -> frozenset[tuple[int, int]]:
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
        start = self.cell(start_x_m, start_y_m)
        if start is None or not self._free_with_clearance(
            *start, clearance_cells
        ):
            if start_snap_radius_m <= 0:
                return frozenset()
            start = self.nearest_clearance_seed(
                start_x_m,
                start_y_m,
                clearance_cells=clearance_cells,
                max_distance_m=start_snap_radius_m,
                start_footprint_override_m=start_footprint_override_m,
            )
            if start is None:
                return frozenset()
        reached = {start}
        pending = deque([start])
        while pending:
            row, column = pending.popleft()
            for candidate in (
                (row - 1, column),
                (row + 1, column),
                (row, column - 1),
                (row, column + 1),
            ):
                if candidate in reached:
                    continue
                if not (
                    0 <= candidate[0] < self.height
                    and 0 <= candidate[1] < self.width
                ):
                    continue
                if not self._free_with_clearance(
                    candidate[0], candidate[1], clearance_cells
                ):
                    continue
                reached.add(candidate)
                pending.append(candidate)
        return frozenset(reached)

    def point_in_component(
        self,
        x_m: float,
        y_m: float,
        component: frozenset[tuple[int, int]],
    ) -> bool:
        cell = self.cell(x_m, y_m)
        return cell is not None and cell in component

    def component_within_radius(
        self,
        x_m: float,
        y_m: float,
        radius_m: float,
        component: frozenset[tuple[int, int]],
    ) -> bool:
        """Return whether a reachable cell center intersects an arrival disk."""

        if not math.isfinite(radius_m) or radius_m < 0:
            raise ValueError("radius_m must be finite and non-negative")
        radius_sq = radius_m * radius_m
        for row, column in component:
            center_x, center_y = self.cell_center(row, column)
            if (center_x - x_m) ** 2 + (center_y - y_m) ** 2 <= radius_sq:
                return True
        return False


def navigation_event(
    decision: HighLevelDecisionV2,
    *,
    status: NavigationStatusV2,
    reason_code: str,
    local_pose: tuple[float, float, float],
    path_length_m: float,
    velocity_zero_confirmed: bool,
    local_goal: LocalHighLevelGoal | None = None,
    detail: str = "",
    terminal: bool = False,
    event_id: str | None = None,
    observed_at_ns: int | None = None,
    adapter_name: str = "robot-local-v2-point-adapter",
) -> NavigationEventV2:
    resolved = None
    if (
        status == NavigationStatusV2.ACCEPTED
        and isinstance(decision.target, SemanticRegionTargetV2)
        and local_goal is not None
    ):
        resolved = ResolvedLocalGoalV2(
            frame_id=local_goal.frame_id,
            x=local_goal.x,
            y=local_goal.y,
            yaw_rad=local_goal.yaw_rad,
            source_region_sha256=decision.target.region.payload_sha256,
            arrival_radius_m=float(local_goal.arrival_radius_m or 0.5),
            adapter_name=adapter_name,
            adapter_version="1",
        )
    return NavigationEventV2(
        robot_id=decision.robot_id,
        scene_id=decision.scene_id,
        episode_id=decision.episode_id,
        decision_batch_id=decision.decision_batch_id,
        leg_id=decision.leg_id,
        decision_id=decision.decision_id,
        lease_sequence=decision.lease_sequence,
        event_id=event_id or f"{decision.robot_id}-{time.time_ns()}-{uuid.uuid4().hex[:6]}",
        status=status,
        reason_code=reason_code,
        observed_at_ns=observed_at_ns or time.time_ns(),
        local_pose={
            "frame_id": local_goal.frame_id if local_goal is not None else f"{decision.robot_id}/map",
            "x": local_pose[0],
            "y": local_pose[1],
            "yaw_rad": local_pose[2],
        },
        path_length_m_from_episode_start=path_length_m,
        velocity_zero_confirmed=velocity_zero_confirmed,
        terminal_observation_sequence=(
            decision.input_observations[decision.robot_id].sequence if terminal else None
        ),
        resolved_local_goal=resolved,
        detail=detail[:512],
    )
