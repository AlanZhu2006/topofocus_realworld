"""Robot-local dry-run adapter for v2 high-level decisions.

The adapter validates and reduces a high-level target to either TinyNav POI
JSON or a WATER ``/api/move`` request preview.  It has no ROS publisher,
socket, HTTP client or actuator import and therefore cannot move a robot.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math

import cv2
import numpy as np

from .geometry import invert_rigid, transform_point
from .models import CommandMode, RobotHealth
from .transport_v2 import (
    FrontierPointTargetV2,
    HighLevelDecisionV2,
    SemanticRegionTargetV2,
)


class V2AdapterAction(str, Enum):
    GOAL = "GOAL"
    HOLD = "HOLD"
    STOP = "STOP"


@dataclass(frozen=True)
class V2GoalAdapterConfig:
    robot_id: str
    transform_version: str
    shared_frame_calibration_id: str
    shared_T_robot_map: tuple[float, ...]
    output_kind: str
    local_frame_id: str
    max_goal_distance_m: float = 8.0
    allow_unreachable_semantic_projection: bool = False

    def __post_init__(self) -> None:
        if self.output_kind not in {"tinynav_poi", "water_move"}:
            raise ValueError("output_kind must be tinynav_poi or water_move")
        if len(self.shared_T_robot_map) != 16:
            raise ValueError("shared_T_robot_map must contain 16 values")
        if not self.shared_frame_calibration_id:
            raise ValueError("shared_frame_calibration_id is required")
        if not self.local_frame_id:
            raise ValueError("local_frame_id is required")
        if not math.isfinite(self.max_goal_distance_m) or self.max_goal_distance_m <= 0:
            raise ValueError("max_goal_distance_m must be finite and positive")


@dataclass(frozen=True)
class LocalHighLevelGoal:
    frame_id: str
    x: float
    y: float
    z: float
    yaw_rad: float
    target_kind: str
    source_region_sha256: str | None = None
    arrival_radius_m: float | None = None


@dataclass(frozen=True)
class V2AdapterResult:
    action: V2AdapterAction
    reason_code: str
    detail: str
    local_goal: LocalHighLevelGoal | None = None
    command_preview: str | None = None


def wrap_to_pi(angle: float) -> float:
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi and angle > 0 else wrapped


class V2GoalAdapter:
    """Fail-closed v2 decision reducer shared by the two robot receivers."""

    def __init__(self, config: V2GoalAdapterConfig) -> None:
        self.config = config
        self._robot_map_T_shared = invert_rigid(config.shared_T_robot_map)
        self._shared_yaw_from_robot_map = math.atan2(
            config.shared_T_robot_map[4], config.shared_T_robot_map[0]
        )
        self._highest_map_version = -1
        self._highest_execution_epoch = -1
        self._latest_lease_by_leg: dict[str, tuple[int, str]] = {}
        self._stop_latched = False

    @property
    def stop_latched(self) -> bool:
        return self._stop_latched

    def local_operator_reset_stop(self) -> None:
        """Only an authenticated robot-local operator path may call this."""
        self._stop_latched = False

    def evaluate(
        self,
        decision: HighLevelDecisionV2,
        *,
        now_ns: int,
        health: RobotHealth,
        current_position_robot_map: tuple[float, float, float],
        is_local_goal_reachable: Callable[[float, float], bool] | None = None,
    ) -> V2AdapterResult:
        if decision.robot_id != self.config.robot_id:
            return self._hold("ROBOT_ID_MISMATCH", "decision is for another robot")

        # STOP can only reduce authority and is honored before coordinate,
        # health, time and ordering checks.
        if decision.mode == CommandMode.STOP:
            self._stop_latched = True
            self._record_order(decision)
            return V2AdapterResult(
                V2AdapterAction.STOP,
                "LOCAL_STOP_LATCHED",
                "v2 STOP latched locally",
            )
        if self._stop_latched:
            return V2AdapterResult(
                V2AdapterAction.STOP,
                "LOCAL_STOP_LATCHED",
                "local STOP latch is active",
            )
        if decision.expires_at_ns <= now_ns:
            return self._hold("EXPIRED", "decision lease expired")
        if decision.coordination.execution_epoch < self._highest_execution_epoch:
            return self._hold("OUT_OF_ORDER", "execution epoch moved backward")
        previous_lease = self._latest_lease_by_leg.get(decision.leg_id)
        if previous_lease is not None:
            previous_sequence, previous_decision_id = previous_lease
            if decision.decision_id == previous_decision_id:
                if decision.lease_sequence != previous_sequence:
                    return self._hold("OUT_OF_ORDER", "duplicate decision changed lease sequence")
            elif decision.lease_sequence != previous_sequence + 1:
                return self._hold("OUT_OF_ORDER", "lease sequence did not increase exactly once")
        elif decision.lease_sequence != 0:
            return self._hold("OUT_OF_ORDER", "new leg did not start at lease zero")
        if decision.map_provenance.map_version < self._highest_map_version:
            return self._hold("MAP_VERSION_REGRESSION", "map version moved backward")
        if decision.map_provenance.transform_version != self.config.transform_version:
            return self._hold("TRANSFORM_MISMATCH", "transform version mismatch")
        if (
            decision.map_provenance.shared_frame_calibration_id
            != self.config.shared_frame_calibration_id
        ):
            return self._hold("CALIBRATION_MISMATCH", "shared calibration mismatch")

        self._record_order(decision)
        if decision.mode == CommandMode.HOLD:
            return V2AdapterResult(V2AdapterAction.HOLD, "HUB_HOLD", decision.reason)
        if self.config.robot_id not in decision.coordination.active_robot_ids:
            return self._hold("UNSAFE", "robot is absent from active_robot_ids")
        if not health.ready_for_goal():
            return self._hold("HEALTH_NOT_READY", "local health does not permit motion")
        if decision.target is None:
            return self._hold("UNSAFE", "GOAL has no target")

        if isinstance(decision.target, FrontierPointTargetV2):
            local_goal = self._frontier_goal(decision.target)
        elif isinstance(decision.target, SemanticRegionTargetV2):
            try:
                local_goal = self._semantic_goal(
                    decision.target,
                    current_position_robot_map=current_position_robot_map,
                    is_local_goal_reachable=is_local_goal_reachable,
                )
            except ValueError as exc:
                return self._hold("REGION_ARTIFACT_INVALID", str(exc))
            if local_goal is None:
                return self._hold(
                    "UNREACHABLE",
                    "no locally reachable candidate in the semantic arrival region",
                )
        else:  # pragma: no cover - discriminated Pydantic union is exhaustive
            return self._hold("UNSAFE", "unsupported v2 target kind")

        distance = math.hypot(
            local_goal.x - current_position_robot_map[0],
            local_goal.y - current_position_robot_map[1],
        )
        if distance > self.config.max_goal_distance_m:
            return self._hold(
                "DISTANCE_LIMIT",
                f"goal is {distance:.2f}m away, above "
                f"{self.config.max_goal_distance_m:.2f}m limit",
            )
        command_preview = self._command_preview(decision, local_goal)
        return V2AdapterResult(
            V2AdapterAction.GOAL,
            "LOCAL_GOAL_READY_DRY_RUN",
            "fresh high-level target reduced locally; preview only, never sent",
            local_goal=local_goal,
            command_preview=command_preview,
        )

    def _frontier_goal(self, target: FrontierPointTargetV2) -> LocalHighLevelGoal:
        point = transform_point(
            self._robot_map_T_shared,
            (target.pose.x, target.pose.y, target.pose.z),
        )
        yaw = wrap_to_pi(target.pose.yaw_rad - self._shared_yaw_from_robot_map)
        return LocalHighLevelGoal(
            frame_id=self.config.local_frame_id,
            x=point[0],
            y=point[1],
            z=point[2],
            yaw_rad=yaw,
            target_kind=target.kind,
            arrival_radius_m=(
                target.source_goal_dilation_cells
                * 0.05
            ),
        )

    def _semantic_goal(
        self,
        target: SemanticRegionTargetV2,
        *,
        current_position_robot_map: tuple[float, float, float],
        is_local_goal_reachable: Callable[[float, float], bool] | None,
    ) -> LocalHighLevelGoal | None:
        payload = target.region.png_bytes()
        mask = cv2.imdecode(
            np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        expected_shape = (target.region.height, target.region.width)
        if mask is None or mask.ndim != 2 or mask.dtype != np.uint8:
            raise ValueError("semantic region PNG did not decode as uint8 grayscale")
        if mask.shape != expected_shape:
            raise ValueError("semantic region decoded shape mismatch")
        values = np.unique(mask)
        if not set(int(value) for value in values).issubset({0, 255}):
            raise ValueError("semantic region contains non-binary pixels")
        raw = mask == 255
        if int(raw.sum()) != target.region.component_size_cells:
            raise ValueError("semantic region component_size_cells mismatch")

        radius = target.source_goal_dilation_cells
        yy, xx = np.ogrid[-radius : radius + 1, -radius : radius + 1]
        disk = ((xx * xx + yy * yy) <= radius * radius).astype(np.uint8)
        dilated = cv2.dilate(raw.astype(np.uint8), disk) > 0
        approach = dilated & ~raw
        candidates = approach if np.any(approach) else dilated
        rows, columns = np.nonzero(candidates)
        if rows.size == 0:
            raise ValueError("semantic arrival region is empty")

        x_shared = (
            target.region.origin_xy_m[0]
            + (columns.astype(np.float64) + 0.5) * target.region.resolution_m
        )
        y_shared = (
            target.region.origin_xy_m[1]
            + (rows.astype(np.float64) + 0.5) * target.region.resolution_m
        )
        matrix = np.asarray(self._robot_map_T_shared, dtype=np.float64).reshape(4, 4)
        shared_points = np.stack(
            [x_shared, y_shared, np.zeros_like(x_shared), np.ones_like(x_shared)],
            axis=0,
        )
        local_points = matrix @ shared_points
        distance_sq = (
            (local_points[0] - current_position_robot_map[0]) ** 2
            + (local_points[1] - current_position_robot_map[1]) ** 2
        )
        selected_index: int | None = None
        for index_value in np.argsort(distance_sq):
            index = int(index_value)
            x_local = float(local_points[0, index])
            y_local = float(local_points[1, index])
            if is_local_goal_reachable is None or not is_local_goal_reachable(
                x_local, y_local
            ):
                continue
            selected_index = index
            break
        if (
            selected_index is None
            and self.config.allow_unreachable_semantic_projection
        ):
            selected_index = int(np.argmin(distance_sq))
        if selected_index is None:
            return None
        x_local = float(local_points[0, selected_index])
        y_local = float(local_points[1, selected_index])
        centroid = transform_point(
            self._robot_map_T_shared,
            (target.display_centroid.x, target.display_centroid.y, 0.0),
        )
        yaw = math.atan2(centroid[1] - y_local, centroid[0] - x_local)
        return LocalHighLevelGoal(
            frame_id=self.config.local_frame_id,
            x=x_local,
            y=y_local,
            z=float(local_points[2, selected_index]),
            yaw_rad=wrap_to_pi(yaw),
            target_kind=target.kind,
            source_region_sha256=target.region.payload_sha256,
            arrival_radius_m=radius * target.region.resolution_m,
        )

    def _command_preview(
        self,
        decision: HighLevelDecisionV2,
        goal: LocalHighLevelGoal,
    ) -> str:
        if self.config.output_kind == "tinynav_poi":
            payload = {
                "0": {
                    "id": 0,
                    "name": "focus_hub_goal",
                    "position": [goal.x, goal.y, goal.z],
                    "yaw_rad": goal.yaw_rad,
                    "source": "focus_hub_v2",
                    "target_kind": goal.target_kind,
                    "decision_id": decision.decision_id,
                    "leg_id": decision.leg_id,
                    "lease_sequence": decision.lease_sequence,
                    "map_version": decision.map_provenance.map_version,
                    "transform_version": decision.map_provenance.transform_version,
                    "expires_at_ns": decision.expires_at_ns,
                    "arrival_radius_m": (
                        goal.arrival_radius_m
                        if goal.arrival_radius_m is not None
                        else 0.35
                    ),
                }
            }
            return json.dumps(payload, separators=(",", ":"))
        request_uuid = hashlib.sha256(decision.decision_id.encode("utf-8")).hexdigest()[:12]
        return (
            f"/api/move?location={goal.x:.4f},{goal.y:.4f},{goal.yaw_rad:.4f}"
            f"&uuid={request_uuid}"
        )

    def _record_order(self, decision: HighLevelDecisionV2) -> None:
        self._highest_map_version = max(
            self._highest_map_version, decision.map_provenance.map_version
        )
        self._highest_execution_epoch = max(
            self._highest_execution_epoch,
            decision.coordination.execution_epoch,
        )
        self._latest_lease_by_leg[decision.leg_id] = (
            decision.lease_sequence,
            decision.decision_id,
        )

    @staticmethod
    def _hold(reason_code: str, detail: str) -> V2AdapterResult:
        return V2AdapterResult(V2AdapterAction.HOLD, reason_code, detail)
