from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum

from .geometry import invert_rigid, transform_point
from .models import CommandMode, Decision, DecisionAckStatus, RobotHealth


class GuardAction(str, Enum):
    GOAL = "GOAL"
    HOLD = "HOLD"
    STOP = "STOP"


@dataclass(frozen=True)
class GoalGuardConfig:
    robot_id: str
    transform_version: str
    shared_T_robot_map: tuple[float, ...]
    max_goal_distance_m: float = 8.0


@dataclass(frozen=True)
class GuardResult:
    action: GuardAction
    ack_status: DecisionAckStatus
    detail: str
    poi_json: str | None = None


class GoalGuard:
    """Robot-side fail-closed reduction from hub decisions to TinyNav POI JSON."""

    def __init__(self, config: GoalGuardConfig) -> None:
        self.config = config
        self._robot_map_T_shared = invert_rigid(config.shared_T_robot_map)
        self._highest_map_version = -1
        self._latest_issued_at_ns = -1
        self._stop_latched = False

    @property
    def stop_latched(self) -> bool:
        return self._stop_latched

    def local_operator_reset_stop(self) -> None:
        """This must only be called from an authenticated local operator path."""
        self._stop_latched = False

    def evaluate(
        self,
        decision: Decision,
        *,
        now_ns: int,
        health: RobotHealth,
        current_position_robot_map: tuple[float, float, float],
    ) -> GuardResult:
        if decision.robot_id != self.config.robot_id:
            return self._hold(DecisionAckStatus.REJECTED_UNSAFE, "decision is for another robot")

        # STOP is safe to honor even if a coordinate calibration is unavailable.
        if decision.mode == CommandMode.STOP:
            self._stop_latched = True
            self._record_order(decision)
            return GuardResult(GuardAction.STOP, DecisionAckStatus.ACCEPTED, "STOP latched locally")
        if self._stop_latched:
            return GuardResult(GuardAction.STOP, DecisionAckStatus.REJECTED_UNSAFE, "local STOP latch is active")

        if decision.expires_at_ns <= now_ns:
            return self._hold(DecisionAckStatus.REJECTED_EXPIRED, "decision expired")
        if decision.issued_at_ns < self._latest_issued_at_ns:
            return self._hold(DecisionAckStatus.REJECTED_OUT_OF_ORDER, "decision issue time moved backward")
        if decision.map_version < self._highest_map_version:
            return self._hold(DecisionAckStatus.REJECTED_MAP_VERSION, "decision map version moved backward")
        if decision.transform_version != self.config.transform_version:
            return self._hold(DecisionAckStatus.REJECTED_TRANSFORM, "transform version mismatch")

        self._record_order(decision)
        if decision.mode == CommandMode.HOLD:
            return GuardResult(GuardAction.HOLD, DecisionAckStatus.ACCEPTED, decision.reason, poi_json="{}")
        if not health.ready_for_goal():
            return self._hold(DecisionAckStatus.REJECTED_HEALTH, "local health does not permit motion")
        if decision.target is None:
            return self._hold(DecisionAckStatus.REJECTED_UNSAFE, "GOAL has no target")

        point = transform_point(
            self._robot_map_T_shared,
            (decision.target.x, decision.target.y, decision.target.z),
        )
        distance = math.hypot(
            point[0] - current_position_robot_map[0],
            point[1] - current_position_robot_map[1],
        )
        if distance > self.config.max_goal_distance_m:
            return self._hold(
                DecisionAckStatus.REJECTED_UNSAFE,
                f"goal is {distance:.2f}m away, above {self.config.max_goal_distance_m:.2f}m limit",
            )

        payload = {
            "0": {
                "id": 0,
                "name": "focus_hub_goal",
                "position": [point[0], point[1], point[2]],
                "source": "focus_hub",
                "decision_id": decision.decision_id,
                "map_version": decision.map_version,
                "transform_version": decision.transform_version,
                "expires_at_ns": decision.expires_at_ns,
            }
        }
        return GuardResult(
            GuardAction.GOAL,
            DecisionAckStatus.ACCEPTED,
            "fresh high-level goal accepted",
            poi_json=json.dumps(payload, separators=(",", ":")),
        )

    def _record_order(self, decision: Decision) -> None:
        self._highest_map_version = max(self._highest_map_version, decision.map_version)
        self._latest_issued_at_ns = max(self._latest_issued_at_ns, decision.issued_at_ns)

    @staticmethod
    def _hold(status: DecisionAckStatus, detail: str) -> GuardResult:
        return GuardResult(GuardAction.HOLD, status, detail, poi_json="{}")

