from __future__ import annotations

import json
import os
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from .models import CommandMode, Decision, ObservationMetadata, RobotHealth


class RegistryError(ValueError):
    status_code = 400


class UnknownRobot(RegistryError):
    status_code = 404


class OutOfOrderObservation(RegistryError):
    status_code = 409


class ClockViolation(RegistryError):
    status_code = 422


class TransformViolation(RegistryError):
    status_code = 422


class UnsafeDecision(RegistryError):
    status_code = 409


@dataclass(frozen=True)
class RobotPolicy:
    transform_version: str
    allow_goal: bool = False


@dataclass
class RobotState:
    last_sequence: int = -1
    last_payload_digest: str = ""
    last_observation: ObservationMetadata | None = None
    last_received_at_ns: int = 0
    map_version: int = 0
    latest_decision: Decision | None = None
    last_heartbeat: RobotHealth | None = None
    last_heartbeat_sent_ns: int = 0
    last_heartbeat_received_at_ns: int = 0


@dataclass(frozen=True)
class AcceptResult:
    status: str
    received_at_ns: int
    map_version: int
    previous_sequence: int
    previous_digest: str
    previous_observation: ObservationMetadata | None
    previous_received_at_ns: int


class HubRegistry:
    def __init__(
        self,
        policies: dict[str, RobotPolicy],
        *,
        max_observation_age_ns: int = 3_000_000_000,
        max_future_skew_ns: int = 250_000_000,
        max_health_age_ns: int = 3_000_000_000,
        max_heartbeat_age_ns: int = 2_000_000_000,
        state_path: "Path | None" = None,
    ) -> None:
        self._policies = dict(policies)
        self._states = {robot_id: RobotState() for robot_id in policies}
        self._max_observation_age_ns = max_observation_age_ns
        self._max_future_skew_ns = max_future_skew_ns
        self._max_health_age_ns = max_health_age_ns
        self._max_heartbeat_age_ns = max_heartbeat_age_ns
        self._lock = threading.RLock()
        self._state_path = state_path
        self._load_persisted_state()

    def _load_persisted_state(self) -> None:
        """Restore sequence/map-version continuity across hub restarts.

        Only ordering state is persisted; the last observation itself is not,
        so a restarted hub correctly refuses GOAL publishes until a fresh
        observation arrives (fail-closed).
        """
        if self._state_path is None or not self._state_path.is_file():
            return
        raw = json.loads(self._state_path.read_text(encoding="utf-8"))
        for robot_id, entry in raw.get("robots", {}).items():
            state = self._states.get(robot_id)
            if state is None:
                continue
            state.last_sequence = int(entry["last_sequence"])
            state.last_payload_digest = str(entry.get("last_payload_digest", ""))
            state.map_version = int(entry.get("map_version", 0))

    def _persist_state(self) -> None:
        if self._state_path is None:
            return
        snapshot = {
            "robots": {
                robot_id: {
                    "last_sequence": state.last_sequence,
                    "last_payload_digest": state.last_payload_digest,
                    "map_version": state.map_version,
                }
                for robot_id, state in self._states.items()
            }
        }
        self._state_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self._state_path.with_suffix(".tmp")
        temp.write_text(json.dumps(snapshot, sort_keys=True), encoding="utf-8")
        os.replace(temp, self._state_path)

    @property
    def robot_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._states))

    def _state(self, robot_id: str) -> RobotState:
        try:
            return self._states[robot_id]
        except KeyError as exc:
            raise UnknownRobot(f"unknown robot_id: {robot_id}") from exc

    def accept_observation(
        self,
        metadata: ObservationMetadata,
        payload_digest: str,
        *,
        now_ns: int | None = None,
    ) -> AcceptResult:
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            state = self._state(metadata.robot_id)
            policy = self._policies[metadata.robot_id]
            age_ns = now_ns - metadata.capture_time_ns
            if age_ns > self._max_observation_age_ns:
                raise ClockViolation(f"observation is stale by {age_ns / 1e9:.3f}s")
            if age_ns < -self._max_future_skew_ns:
                raise ClockViolation(f"observation is {-age_ns / 1e9:.3f}s in the future")
            if policy.transform_version != "UNSET" and metadata.pose.transform_version != policy.transform_version:
                raise TransformViolation(
                    f"transform_version {metadata.pose.transform_version!r} does not match "
                    f"configured {policy.transform_version!r}"
                )
            if metadata.sequence < state.last_sequence:
                raise OutOfOrderObservation(
                    f"sequence {metadata.sequence} is older than {state.last_sequence}"
                )
            if metadata.sequence == state.last_sequence:
                if payload_digest != state.last_payload_digest:
                    raise OutOfOrderObservation("same sequence was reused with different content")
                return AcceptResult(
                    "duplicate",
                    state.last_received_at_ns,
                    state.map_version,
                    state.last_sequence,
                    state.last_payload_digest,
                    state.last_observation,
                    state.last_received_at_ns,
                )

            result = AcceptResult(
                "accepted",
                now_ns,
                state.map_version,
                state.last_sequence,
                state.last_payload_digest,
                state.last_observation,
                state.last_received_at_ns,
            )
            state.last_sequence = metadata.sequence
            state.last_payload_digest = payload_digest
            state.last_observation = metadata
            state.last_received_at_ns = now_ns
            self._persist_state()
            return result

    def accept_heartbeat(
        self,
        robot_id: str,
        health: RobotHealth,
        sent_time_ns: int,
        *,
        now_ns: int | None = None,
    ) -> int:
        """Records a lightweight, RGBD-independent health ping.

        Deliberately NOT persisted across restarts (like last_observation)
        so a restarted hub stays fail-closed until fresh data of either kind
        arrives. Returns received_at_ns.
        """
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            state = self._state(robot_id)
            age_ns = now_ns - sent_time_ns
            if age_ns > self._max_heartbeat_age_ns:
                raise ClockViolation(f"heartbeat is stale by {age_ns / 1e9:.3f}s")
            if age_ns < -self._max_future_skew_ns:
                raise ClockViolation(f"heartbeat is {-age_ns / 1e9:.3f}s in the future")
            if sent_time_ns < state.last_heartbeat_sent_ns:
                # A reordered/delayed heartbeat arrived after a fresher one
                # already landed — accept it (still valid telemetry) but
                # don't let it regress the freshness clock.
                return now_ns
            state.last_heartbeat = health
            state.last_heartbeat_sent_ns = sent_time_ns
            state.last_heartbeat_received_at_ns = now_ns
            return now_ns

    def _freshest_health(self, state: RobotState) -> tuple[RobotHealth | None, int]:
        """Returns (health, age_source_received_at_ns) from whichever of the
        observation stream or the independent heartbeat channel is newer.
        """
        observation_health = state.last_observation.health if state.last_observation else None
        if state.last_heartbeat_received_at_ns > state.last_received_at_ns:
            return state.last_heartbeat, state.last_heartbeat_received_at_ns
        return observation_health, state.last_received_at_ns

    def rollback_observation(
        self,
        metadata: ObservationMetadata,
        payload_digest: str,
        accepted: AcceptResult,
    ) -> None:
        if accepted.status != "accepted":
            return
        with self._lock:
            state = self._state(metadata.robot_id)
            if state.last_sequence != metadata.sequence or state.last_payload_digest != payload_digest:
                return
            state.last_sequence = accepted.previous_sequence
            state.last_payload_digest = accepted.previous_digest
            state.last_observation = accepted.previous_observation
            state.last_received_at_ns = accepted.previous_received_at_ns

    def advance_map_version(self) -> int:
        with self._lock:
            version = max((state.map_version for state in self._states.values()), default=0) + 1
            for state in self._states.values():
                state.map_version = version
            self._persist_state()
            return version

    def publish_decision(self, decision: Decision, *, now_ns: int | None = None) -> None:
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            state = self._state(decision.robot_id)
            policy = self._policies[decision.robot_id]
            if decision.issued_at_ns > now_ns + self._max_future_skew_ns:
                raise ClockViolation("decision issue time is in the future")
            if decision.expires_at_ns <= now_ns:
                raise ClockViolation("cannot publish an already expired decision")
            if decision.map_version != state.map_version:
                raise UnsafeDecision("decision map_version is not the current map version")
            if decision.mode == CommandMode.GOAL:
                observation = state.last_observation
                if not policy.allow_goal:
                    raise UnsafeDecision("GOAL output is disabled for this robot")
                if observation is None:
                    raise UnsafeDecision("robot has no accepted observation")
                freshest_health, health_received_at_ns = self._freshest_health(state)
                if now_ns - health_received_at_ns > self._max_health_age_ns:
                    raise UnsafeDecision("robot health is stale")
                if observation.mapping_only:
                    raise UnsafeDecision("latest observation is mapping_only")
                if freshest_health is None or not freshest_health.ready_for_goal():
                    raise UnsafeDecision("robot health does not permit a GOAL")
                if observation.base_T_camera is None:
                    raise UnsafeDecision("base_T_camera calibration is absent")
                if decision.transform_version != observation.pose.transform_version:
                    raise TransformViolation("decision and observation transform versions differ")
            state.latest_decision = decision

    def effective_decision(self, robot_id: str, *, now_ns: int | None = None) -> Decision:
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            state = self._state(robot_id)
            if state.latest_decision is not None and state.latest_decision.expires_at_ns > now_ns:
                return state.latest_decision
            observation = state.last_observation
            transform_version = observation.pose.transform_version if observation else self._policies[robot_id].transform_version
            return Decision(
                robot_id=robot_id,
                decision_id=f"fallback-hold-{uuid.uuid4()}",
                mode=CommandMode.HOLD,
                map_version=state.map_version,
                transform_version=transform_version,
                issued_at_ns=now_ns,
                expires_at_ns=now_ns + 1_000_000_000,
                target=None,
                reason="no fresh safe decision",
            )

    def snapshot(self, robot_id: str) -> RobotState:
        with self._lock:
            state = self._state(robot_id)
            return RobotState(**vars(state))
