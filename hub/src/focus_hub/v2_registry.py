"""In-memory fail-closed registry for approved v2 demo decisions/events.

The v1 observation registry remains authoritative for identity, freshness,
health, calibration and policy.  This layer adds atomic two-robot decisions,
independent leases and idempotent feedback without changing v1 behavior.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time

from .models import CommandMode
from .registry import (
    ClockViolation,
    HubRegistry,
    OutOfOrderObservation,
    RobotPolicy,
    TransformViolation,
    UnsafeDecision,
)
from .transport_v2 import (
    DecisionBatchV2,
    HighLevelDecisionV2,
    NavigationEventV2,
    NavigationStatusV2,
    SemanticRegionTargetV2,
)


@dataclass
class _V2RobotState:
    latest_decision: HighLevelDecisionV2 | None = None
    decisions_by_id: dict[str, HighLevelDecisionV2] = field(default_factory=dict)
    latest_by_leg: dict[str, HighLevelDecisionV2] = field(default_factory=dict)
    event_digests: dict[str, str] = field(default_factory=dict)
    latest_event: NavigationEventV2 | None = None
    latest_event_received_at_ns: int = 0
    episode_path_lengths: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class V2EventAcceptResult:
    status: str
    received_at_ns: int


@dataclass(frozen=True)
class V2NavigationState:
    latest_decision: HighLevelDecisionV2 | None
    latest_event: NavigationEventV2 | None
    latest_event_received_at_ns: int


class V2DecisionRegistry:
    def __init__(
        self,
        observation_registry: HubRegistry,
        policies: dict[str, RobotPolicy],
        *,
        max_future_skew_ns: int = 250_000_000,
        max_health_age_ns: int = 3_000_000_000,
        renewal_event_age_ns: int = 2_000_000_000,
        max_input_age_ns: int = 30_000_000_000,
        max_input_skew_ns: int = 5_000_000_000,
    ) -> None:
        self._observations = observation_registry
        self._policies = dict(policies)
        self._states = {
            robot_id: _V2RobotState() for robot_id in observation_registry.robot_ids
        }
        self._max_future_skew_ns = max_future_skew_ns
        self._max_health_age_ns = max_health_age_ns
        self._renewal_event_age_ns = renewal_event_age_ns
        self._max_input_age_ns = max_input_age_ns
        self._max_input_skew_ns = max_input_skew_ns
        self._decision_ids: set[str] = set()
        self._lock = threading.RLock()

    @property
    def robot_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._states))

    def _state(self, robot_id: str) -> _V2RobotState:
        try:
            return self._states[robot_id]
        except KeyError as exc:
            from .registry import UnknownRobot

            raise UnknownRobot(f"unknown robot_id: {robot_id}") from exc

    @staticmethod
    def _freshest_health(observation_state):
        """Return command-receiver health once that authority has appeared.

        RGB-D observations can arrive after a receiver heartbeat, but their
        embedded health only describes the sensing path.  Falling back to that
        health would let an unrelated camera frame replace the command
        receiver's READY/TRACKING state.  A stale receiver heartbeat therefore
        remains stale (and fails closed) instead of falling back to a newer
        observation.
        """
        if observation_state.last_heartbeat_received_at_ns > 0:
            return (
                observation_state.last_heartbeat,
                observation_state.last_heartbeat_received_at_ns,
            )
        observation = observation_state.last_observation
        return (
            None if observation is None else observation.health,
            observation_state.last_received_at_ns,
        )

    @staticmethod
    def _renewal_identity(decision: HighLevelDecisionV2) -> dict[str, object]:
        return {
            "profile": decision.profile,
            "robot_id": decision.robot_id,
            "scene_id": decision.scene_id,
            "episode_id": decision.episode_id,
            "round_index": decision.round_index,
            "source_step": decision.source_step,
            "decision_batch_id": decision.decision_batch_id,
            "leg_id": decision.leg_id,
            "mode": decision.mode,
            "goal_category": decision.goal_category,
            "input_observations": decision.input_observations,
            "map_provenance": decision.map_provenance,
            "target": decision.target,
            "reason": decision.reason,
        }

    def _validate_lease_order(
        self,
        decision: HighLevelDecisionV2,
        state: _V2RobotState,
        *,
        now_ns: int,
    ) -> bool:
        previous = state.latest_by_leg.get(decision.leg_id)
        if previous is None:
            if decision.lease_sequence != 0:
                raise OutOfOrderObservation("a new v2 leg must start at lease_sequence 0")
            return False
        if decision.lease_sequence != previous.lease_sequence + 1:
            raise OutOfOrderObservation("v2 lease_sequence must increase by exactly one")
        if self._renewal_identity(decision) != self._renewal_identity(previous):
            raise UnsafeDecision("a v2 renewal changed target or provenance; use a new leg_id")
        latest_event = state.latest_event
        if latest_event is None or latest_event.leg_id != decision.leg_id:
            raise UnsafeDecision("v2 renewal requires fresh feedback for the same leg")
        if now_ns - state.latest_event_received_at_ns > self._renewal_event_age_ns:
            raise UnsafeDecision("v2 renewal feedback is stale")
        if latest_event.status not in {
            NavigationStatusV2.RECEIVED,
            NavigationStatusV2.ACCEPTED,
            NavigationStatusV2.NAVIGATING,
        }:
            raise UnsafeDecision(
                f"v2 renewal blocked by robot status {latest_event.status.value}"
            )
        return True

    def _validate_decision(
        self,
        decision: HighLevelDecisionV2,
        state: _V2RobotState,
        observation_states: dict[str, object],
        *,
        now_ns: int,
    ) -> None:
        if decision.decision_id in self._decision_ids:
            raise OutOfOrderObservation("v2 decision_id was already used")
        is_renewal = self._validate_lease_order(decision, state, now_ns=now_ns)

        # A correctly authenticated STOP can only reduce authority.  It is
        # deliberately accepted even when its clock/map/calibration is stale.
        if decision.mode == CommandMode.STOP:
            return
        if decision.issued_at_ns > now_ns + self._max_future_skew_ns:
            raise ClockViolation("v2 decision issue time is in the future")
        if decision.expires_at_ns <= now_ns:
            raise ClockViolation("cannot publish an expired v2 GOAL/HOLD")

        observation_state = observation_states[decision.robot_id]
        if decision.mode == CommandMode.HOLD:
            return
        if decision.map_provenance.map_version != observation_state.map_version:
            raise UnsafeDecision("v2 map_version is not current")
        if set(decision.input_observations) != set(self._states):
            raise UnsafeDecision("v2 input_observations must cover both configured robots")
        if not is_renewal:
            input_capture_times_ns: list[int] = []
            for robot_id, identity in decision.input_observations.items():
                source_state = observation_states[robot_id]
                source_record = source_state.observation_history.get(identity.sequence)
                if source_record is None:
                    raise UnsafeDecision(
                        f"{robot_id} input sequence is not in accepted observation history"
                    )
                source_observation = source_record.metadata
                if identity.capture_time_ns != source_observation.capture_time_ns:
                    raise UnsafeDecision(f"{robot_id} input capture time differs")
                if identity.payload_sha256 != source_record.payload_digest:
                    raise UnsafeDecision(f"{robot_id} input payload digest differs")
                age_ns = now_ns - identity.capture_time_ns
                if age_ns > self._max_input_age_ns:
                    raise ClockViolation(f"{robot_id} v2 decision input is stale")
                if age_ns < -self._max_future_skew_ns:
                    raise ClockViolation(f"{robot_id} v2 decision input is in the future")
                input_capture_times_ns.append(identity.capture_time_ns)
            if (
                max(input_capture_times_ns) - min(input_capture_times_ns)
                > self._max_input_skew_ns
            ):
                raise ClockViolation("v2 decision inputs exceed cross-robot skew limit")

        policy = self._policies[decision.robot_id]
        observation = observation_state.last_observation
        if not policy.allow_goal:
            raise UnsafeDecision("GOAL output is disabled for this robot")
        if observation is None:
            raise UnsafeDecision("robot has no accepted observation")
        health, health_received_at_ns = self._freshest_health(observation_state)
        if now_ns - health_received_at_ns > self._max_health_age_ns:
            raise UnsafeDecision(f"{decision.robot_id} robot health is stale")
        if observation.mapping_only:
            raise UnsafeDecision("latest observation is mapping_only")
        if health is None:
            raise UnsafeDecision(
                f"{decision.robot_id} health does not permit a GOAL: health is absent"
            )
        if not health.ready_for_goal():
            raise UnsafeDecision(
                f"{decision.robot_id} health does not permit a GOAL: "
                f"safety={health.safety_state.value} "
                f"localization={health.localization_state.value} "
                f"estop={health.estop_engaged} "
                f"collision_avoidance={health.collision_avoidance_ready} "
                f"motor={health.motor_controller_ready} "
                f"detail={health.detail!r}"
            )
        if observation.base_T_camera is None:
            raise UnsafeDecision("base_T_camera calibration is absent")
        if decision.map_provenance.transform_version != observation.pose.transform_version:
            raise TransformViolation("v2 decision and observation transforms differ")
    def publish_batch(
        self,
        batch: DecisionBatchV2,
        *,
        now_ns: int | None = None,
    ) -> None:
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            batch_robot_ids = {decision.robot_id for decision in batch.decisions}
            if batch_robot_ids != set(self._states):
                raise UnsafeDecision("v2 batch must contain every configured robot exactly once")
            observation_states = {
                robot_id: self._observations.snapshot(robot_id)
                for robot_id in self._states
            }
            for decision in batch.decisions:
                self._validate_decision(
                    decision,
                    self._state(decision.robot_id),
                    observation_states,
                    now_ns=now_ns,
                )
            # Commit only after the complete concurrent pair passes.
            for decision in batch.decisions:
                state = self._state(decision.robot_id)
                state.latest_decision = decision
                state.latest_by_leg[decision.leg_id] = decision
                state.decisions_by_id[decision.decision_id] = decision
                self._decision_ids.add(decision.decision_id)

    def effective_decision(
        self,
        robot_id: str,
        *,
        now_ns: int | None = None,
    ) -> HighLevelDecisionV2 | None:
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            decision = self._state(robot_id).latest_decision
            if decision is None:
                return None
            if decision.mode == CommandMode.STOP or decision.expires_at_ns > now_ns:
                return decision
            return None

    def navigation_state(self, robot_id: str) -> V2NavigationState:
        with self._lock:
            state = self._state(robot_id)
            return V2NavigationState(
                latest_decision=state.latest_decision,
                latest_event=state.latest_event,
                latest_event_received_at_ns=state.latest_event_received_at_ns,
            )

    def accept_event(
        self,
        event: NavigationEventV2,
        payload_digest: str,
        *,
        now_ns: int | None = None,
    ) -> V2EventAcceptResult:
        now_ns = time.time_ns() if now_ns is None else now_ns
        with self._lock:
            state = self._state(event.robot_id)
            previous_digest = state.event_digests.get(event.event_id)
            if previous_digest is not None:
                if previous_digest != payload_digest:
                    raise OutOfOrderObservation(
                        "v2 event_id was reused with different content"
                    )
                return V2EventAcceptResult("duplicate", now_ns)
            if event.observed_at_ns > now_ns + self._max_future_skew_ns:
                raise ClockViolation("v2 navigation event is in the future")
            decision = state.decisions_by_id.get(event.decision_id)
            if decision is None:
                raise UnsafeDecision("v2 event references an unknown decision")
            identity_pairs = (
                (event.scene_id, decision.scene_id, "scene_id"),
                (event.episode_id, decision.episode_id, "episode_id"),
                (event.decision_batch_id, decision.decision_batch_id, "decision_batch_id"),
                (event.leg_id, decision.leg_id, "leg_id"),
                (event.lease_sequence, decision.lease_sequence, "lease_sequence"),
            )
            for actual, expected, label in identity_pairs:
                if actual != expected:
                    raise UnsafeDecision(f"v2 event {label} differs from decision")
            if event.status in {
                NavigationStatusV2.NAVIGATING,
                NavigationStatusV2.ARRIVED,
            } and decision.mode != CommandMode.GOAL:
                raise UnsafeDecision(f"{event.status.value} requires a GOAL decision")
            if event.status == NavigationStatusV2.STOPPED and decision.mode != CommandMode.STOP:
                raise UnsafeDecision("STOPPED requires a STOP decision")
            if event.resolved_local_goal is not None:
                if not isinstance(decision.target, SemanticRegionTargetV2):
                    raise UnsafeDecision("resolved semantic goal references a non-semantic decision")
                if (
                    event.resolved_local_goal.source_region_sha256
                    != decision.target.region.payload_sha256
                ):
                    raise UnsafeDecision("resolved semantic goal hash differs from decision")
            previous_path = state.episode_path_lengths.get(event.episode_id, 0.0)
            if event.path_length_m_from_episode_start + 1e-9 < previous_path:
                raise OutOfOrderObservation("episode path length moved backward")
            observation_state = self._observations.snapshot(event.robot_id)
            if (
                event.terminal_observation_sequence is not None
                and event.terminal_observation_sequence > observation_state.last_sequence
            ):
                raise UnsafeDecision("terminal observation sequence was not uploaded")

            state.event_digests[event.event_id] = payload_digest
            state.latest_event = event
            state.latest_event_received_at_ns = now_ns
            state.episode_path_lengths[event.episode_id] = (
                event.path_length_m_from_episode_start
            )
            return V2EventAcceptResult("accepted", now_ns)
