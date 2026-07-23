from __future__ import annotations

import pytest

from focus_hub.models import CommandMode, Decision, GoalPose, RobotHealth
from focus_hub.registry import (
    ClockViolation,
    HubRegistry,
    OutOfOrderObservation,
    RobotPolicy,
    TransformViolation,
    UnsafeDecision,
)


def make_health(ready: bool) -> RobotHealth:
    return RobotHealth.model_validate({
        "safety_state": "READY" if ready else "UNKNOWN",
        "localization_state": "TRACKING" if ready else "UNKNOWN",
        "estop_engaged": False,
        "collision_avoidance_ready": ready,
        "motor_controller_ready": ready,
    })


def make_goal_decision(*, now, map_version=0, transform="calib-test-v1",
                       issued_at_ns=None, expires_at_ns=None, decision_id="decision-1"):
    issued_at_ns = now if issued_at_ns is None else issued_at_ns
    expires_at_ns = now + 1_000_000_000 if expires_at_ns is None else expires_at_ns
    return Decision(
        robot_id="robot-0",
        decision_id=decision_id,
        mode=CommandMode.GOAL,
        map_version=map_version,
        transform_version=transform,
        issued_at_ns=issued_at_ns,
        expires_at_ns=expires_at_ns,
        target=GoalPose(x=1, y=2, yaw_rad=0),
        reason="test",
    )


def make_registry(*, allow_goal: bool = False) -> HubRegistry:
    return HubRegistry({"robot-0": RobotPolicy("calib-test-v1", allow_goal=allow_goal)})


def test_sequence_is_monotonic_and_retry_is_idempotent(observation_factory):
    now = 2_000_000_000_000
    registry = make_registry()
    first = observation_factory(sequence=10, now_ns=now)
    assert registry.accept_observation(first, "digest-a", now_ns=now).status == "accepted"
    assert registry.accept_observation(first, "digest-a", now_ns=now).status == "duplicate"

    with pytest.raises(OutOfOrderObservation):
        registry.accept_observation(first, "changed", now_ns=now)
    with pytest.raises(OutOfOrderObservation):
        registry.accept_observation(observation_factory(sequence=9, now_ns=now), "old", now_ns=now)


def test_stale_observation_is_rejected(observation_factory):
    now = 20_000_000_000
    registry = make_registry()
    observation = observation_factory(now_ns=now)
    with pytest.raises(ClockViolation):
        registry.accept_observation(observation, "digest", now_ns=now + 4_000_000_000)


def test_no_decision_fails_closed_to_hold():
    registry = make_registry()
    decision = registry.effective_decision("robot-0", now_ns=10_000_000_000)
    assert decision.mode == CommandMode.HOLD
    assert decision.target is None


def test_goal_requires_policy_calibration_and_ready_health(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    mapping_only = observation_factory(now_ns=now, mapping_only=True, health_ready=False)
    registry.accept_observation(mapping_only, "digest", now_ns=now)
    decision = Decision(
        robot_id="robot-0",
        decision_id="decision-1",
        mode=CommandMode.GOAL,
        map_version=0,
        transform_version="calib-test-v1",
        issued_at_ns=now,
        expires_at_ns=now + 1_000_000_000,
        target=GoalPose(x=1, y=2, yaw_rad=0),
        reason="test",
    )
    with pytest.raises(UnsafeDecision):
        registry.publish_decision(decision, now_ns=now)

    ready = observation_factory(sequence=1, now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest-2", now_ns=now)
    registry.publish_decision(decision, now_ns=now)
    assert registry.effective_decision("robot-0", now_ns=now).decision_id == "decision-1"


# --- G5 fault-injection: hub-side (registry) rejection paths ---------------


def test_future_clock_observation_is_rejected(observation_factory):
    now = 20_000_000_000
    registry = make_registry()
    observation = observation_factory(now_ns=now)
    # capture_time_ns is now-100ms in the fixture; pretend the server clock
    # is far behind, so the observation looks like it arrived from the future.
    with pytest.raises(ClockViolation):
        registry.accept_observation(observation, "digest", now_ns=now - 1_000_000_000)


def test_duplicate_sequence_content_conflict_is_409(observation_factory):
    now = 20_000_000_000
    registry = make_registry()
    first = observation_factory(sequence=5, now_ns=now)
    registry.accept_observation(first, "digest-a", now_ns=now)
    with pytest.raises(OutOfOrderObservation) as excinfo:
        registry.accept_observation(first, "digest-b", now_ns=now)
    assert excinfo.value.status_code == 409


def test_publish_future_issued_decision_is_rejected(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    decision = make_goal_decision(now=now, issued_at_ns=now + 10_000_000_000,
                                  expires_at_ns=now + 20_000_000_000)
    with pytest.raises(ClockViolation, match="future"):
        registry.publish_decision(decision, now_ns=now)


def test_publish_already_expired_decision_is_rejected(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    decision = make_goal_decision(now=now, issued_at_ns=now - 5_000_000_000,
                                  expires_at_ns=now - 1_000_000_000)
    with pytest.raises(ClockViolation, match="expired"):
        registry.publish_decision(decision, now_ns=now)


def test_publish_stale_map_version_is_rejected(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    registry.advance_map_version()  # current version is now 1
    decision = make_goal_decision(now=now, map_version=0)
    with pytest.raises(UnsafeDecision, match="map_version"):
        registry.publish_decision(decision, now_ns=now)


def test_goal_blocked_when_robot_health_is_stale(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    decision = make_goal_decision(now=now, expires_at_ns=now + 10_000_000_000_000)
    stale_now = now + 4_000_000_000  # past the 3 s max_health_age_ns default
    with pytest.raises(UnsafeDecision, match="stale"):
        registry.publish_decision(decision, now_ns=stale_now)


def test_goal_blocked_when_decision_transform_differs_from_observation(observation_factory):
    now = 20_000_000_000
    registry = HubRegistry({"robot-0": RobotPolicy("UNSET", allow_goal=True)})
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    decision = make_goal_decision(now=now, transform="a-different-calibration")
    with pytest.raises(TransformViolation):
        registry.publish_decision(decision, now_ns=now)


# --- independent 2Hz heartbeat channel --------------------------------------


def test_heartbeat_rescues_a_decision_that_would_otherwise_fail_on_stale_health(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    decision = make_goal_decision(now=now, expires_at_ns=now + 10_000_000_000_000)

    later = now + 4_000_000_000  # past the 3s max_health_age_ns default
    with pytest.raises(UnsafeDecision, match="stale"):
        registry.publish_decision(decision, now_ns=later)

    # A heartbeat posted just before `later` keeps health fresh even though
    # the full RGBD observation stream itself went quiet.
    registry.accept_heartbeat("robot-0", make_health(True), later - 500_000_000, now_ns=later)
    registry.publish_decision(decision, now_ns=later)  # no longer raises


def test_runtime_readiness_uses_newer_receiver_heartbeat(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    observation = observation_factory(
        now_ns=now,
        mapping_only=False,
        health_ready=False,
    )
    registry.accept_observation(observation, "digest", now_ns=now)

    blocked = registry.runtime_readiness("robot-0", now_ns=now)
    assert blocked["ready_for_goal"] is False
    assert "HEALTH_NOT_READY" in blocked["blockers"]
    assert blocked["health_source"] == "observation"

    heartbeat_now = now + 100_000_000
    registry.accept_heartbeat(
        "robot-0",
        make_health(True),
        heartbeat_now,
        now_ns=heartbeat_now,
    )
    ready = registry.runtime_readiness(
        "robot-0", now_ns=heartbeat_now + 10_000_000
    )
    assert ready["ready_for_goal"] is True
    assert ready["blockers"] == []
    assert ready["health_source"] == "heartbeat"


def test_unhealthy_heartbeat_blocks_goal_even_over_a_healthy_observation(observation_factory):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    ready = observation_factory(now_ns=now, mapping_only=False, health_ready=True)
    registry.accept_observation(ready, "digest", now_ns=now)
    decision = make_goal_decision(now=now, expires_at_ns=now + 10_000_000_000_000)
    registry.publish_decision(decision, now_ns=now)  # healthy observation alone is enough

    # A fresher heartbeat reporting NOT ready must override the stale-but-
    # still-within-window "healthy" snapshot cached from the observation.
    registry.accept_heartbeat("robot-0", make_health(False), now + 100_000_000, now_ns=now + 100_000_000)
    with pytest.raises(UnsafeDecision, match="does not permit"):
        registry.publish_decision(decision, now_ns=now + 100_000_000)


def test_fresh_receiver_heartbeat_is_not_overridden_by_later_mapping_health(
    observation_factory,
):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    first = observation_factory(
        sequence=0,
        now_ns=now,
        mapping_only=False,
        health_ready=False,
    )
    registry.accept_observation(first, "digest-0", now_ns=now)
    heartbeat_now = now + 100_000_000
    registry.accept_heartbeat(
        "robot-0",
        make_health(True),
        heartbeat_now,
        now_ns=heartbeat_now,
    )

    # A command-capable RGB-D frame can land between the receiver's 2 Hz
    # heartbeats. Its mapping health must not cause a transient GOAL rejection.
    later_observation = observation_factory(
        sequence=1,
        now_ns=heartbeat_now + 100_000_000,
        mapping_only=False,
        health_ready=False,
    )
    registry.accept_observation(
        later_observation,
        "digest-1",
        now_ns=heartbeat_now + 100_000_000,
    )
    publish_now = heartbeat_now + 110_000_000
    readiness = registry.runtime_readiness("robot-0", now_ns=publish_now)
    assert readiness["ready_for_goal"] is True
    assert readiness["health_source"] == "heartbeat"
    registry.publish_decision(
        make_goal_decision(now=publish_now),
        now_ns=publish_now,
    )


def test_stale_receiver_heartbeat_never_falls_back_to_newer_observation(
    observation_factory,
):
    now = 20_000_000_000
    registry = make_registry(allow_goal=True)
    first = observation_factory(
        sequence=0,
        now_ns=now,
        mapping_only=False,
        health_ready=True,
    )
    registry.accept_observation(first, "digest-0", now_ns=now)
    registry.accept_heartbeat(
        "robot-0",
        make_health(True),
        now + 100_000_000,
        now_ns=now + 100_000_000,
    )

    later = now + 3_200_000_000
    fresh_observation = observation_factory(
        sequence=1,
        now_ns=later,
        mapping_only=False,
        health_ready=True,
    )
    registry.accept_observation(fresh_observation, "digest-1", now_ns=later)
    readiness = registry.runtime_readiness("robot-0", now_ns=later)
    assert readiness["ready_for_goal"] is False
    assert readiness["health_source"] == "heartbeat"
    assert "HEALTH_STALE" in readiness["blockers"]
    with pytest.raises(UnsafeDecision, match="stale"):
        registry.publish_decision(
            make_goal_decision(now=later),
            now_ns=later,
        )


def test_heartbeat_rejects_stale_and_future(observation_factory):
    now = 20_000_000_000
    registry = make_registry()
    with pytest.raises(ClockViolation):
        registry.accept_heartbeat("robot-0", make_health(True), now - 3_000_000_000, now_ns=now)
    with pytest.raises(ClockViolation):
        registry.accept_heartbeat("robot-0", make_health(True), now + 1_000_000_000, now_ns=now)


def test_reordered_heartbeat_does_not_regress_freshness(observation_factory):
    now = 20_000_000_000
    registry = make_registry()
    registry.accept_heartbeat("robot-0", make_health(True), now, now_ns=now)
    # An older heartbeat arrives late (network reordering); it must not
    # overwrite the freshness clock or the health value of the newer one.
    registry.accept_heartbeat("robot-0", make_health(False), now - 200_000_000, now_ns=now + 10_000_000)
    state = registry.snapshot("robot-0")
    assert state.last_heartbeat.ready_for_goal() is True
    assert state.last_heartbeat_sent_ns == now


def test_heartbeat_requires_known_robot():
    registry = make_registry()
    with pytest.raises(Exception):
        registry.accept_heartbeat("robot-does-not-exist", make_health(True), 1, now_ns=1)
