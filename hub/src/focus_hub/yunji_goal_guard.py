"""Dry-run command guard for a Yunji WATER chassis.

Same fail-closed contract as `goal_guard.py` (GoalGuard, built for the wsj
Unitree Go2 / TinyNav POI format): expiry, ordering, map/transform version,
health, distance are all checked identically, because the safety envelope
does not change with robot type. The only robot-specific part is the final
step — instead of a TinyNav POI JSON, an accepted GOAL is reduced to a
dry-run WATER TCP API move request string
(`/api/move?location=x,y,theta&uuid=...`), matching the vendor-documented
`/api/move` contract in
`~/workspace/tinynav/yunji-water-robot/docs/vendor/yunji_water_development_guide.md`.

This module NEVER sends that string anywhere — it only constructs it. Wiring
it to a live `/api/move` call is future work that requires hardware-in-the-
loop testing with an operator present (see audit/YUNJI_WATER_SENDER.md).
"""
from __future__ import annotations

import math
import uuid
from dataclasses import dataclass
from enum import Enum

from .geometry import invert_rigid, transform_point
from .models import CommandMode, Decision, DecisionAckStatus, RobotHealth


class YunjiGuardAction(str, Enum):
    GOAL = "GOAL"
    HOLD = "HOLD"
    STOP = "STOP"


@dataclass(frozen=True)
class YunjiGoalGuardConfig:
    robot_id: str
    transform_version: str
    shared_T_robot_map: tuple[float, ...]
    max_goal_distance_m: float = 8.0


@dataclass(frozen=True)
class YunjiGuardResult:
    action: YunjiGuardAction
    ack_status: DecisionAckStatus
    detail: str
    move_request: str | None = None


class YunjiGoalGuard:
    """Fail-closed reduction from hub decisions to a dry-run WATER move request."""

    def __init__(self, config: YunjiGoalGuardConfig) -> None:
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
    ) -> YunjiGuardResult:
        if decision.robot_id != self.config.robot_id:
            return self._hold(DecisionAckStatus.REJECTED_UNSAFE, "decision is for another robot")

        if decision.mode == CommandMode.STOP:
            self._stop_latched = True
            self._record_order(decision)
            return YunjiGuardResult(YunjiGuardAction.STOP, DecisionAckStatus.ACCEPTED, "STOP latched locally")
        if self._stop_latched:
            return YunjiGuardResult(
                YunjiGuardAction.STOP, DecisionAckStatus.REJECTED_UNSAFE, "local STOP latch is active")

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
            return YunjiGuardResult(YunjiGuardAction.HOLD, DecisionAckStatus.ACCEPTED, decision.reason)
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

        # `point` above already applies the full shared_T_robot_map rotation
        # (via transform_point on x/y/z), but yaw_rad below is passed through
        # from the shared-frame decision unrotated — WATER's /api/move yaw is
        # in this guard's own robot-map frame, and nothing here applies
        # shared_T_robot_map's rotation to it the way it's applied to
        # position. This is a real, currently-unaddressed gap, not fixed by
        # the 2026-07-19 sender pose-source/extrinsics work (that only
        # changed how the sender computes and calibrates poses, not this
        # guard's yaw handling) — dormant while shared_T_robot_map is still
        # a placeholder/identity, but must be fixed before a real
        # calibration with non-trivial rotation is used for live commands.
        # See hub/tools/calibrate_shared_frame.py and
        # audit/YUNJI_WATER_SENDER.md.
        move_request = (
            f"/api/move?location={point[0]:.4f},{point[1]:.4f},{decision.target.yaw_rad:.4f}"
            f"&uuid={uuid.uuid4().hex[:12]}"
        )
        return YunjiGuardResult(
            YunjiGuardAction.GOAL,
            DecisionAckStatus.ACCEPTED,
            "fresh high-level goal accepted (dry-run, never sent)",
            move_request=move_request,
        )

    def _record_order(self, decision: Decision) -> None:
        self._highest_map_version = max(self._highest_map_version, decision.map_version)
        self._latest_issued_at_ns = max(self._latest_issued_at_ns, decision.issued_at_ns)

    @staticmethod
    def _hold(status: DecisionAckStatus, detail: str) -> YunjiGuardResult:
        return YunjiGuardResult(YunjiGuardAction.HOLD, status, detail)
