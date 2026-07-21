from __future__ import annotations

from focus_hub.goal_guard import GoalGuard, GoalGuardConfig, GuardAction
from focus_hub.models import CommandMode, Decision, GoalPose

from conftest import IDENTITY


def make_decision(*, mode=CommandMode.GOAL, now=10_000_000_000, map_version=1,
                  transform="calib-test-v1", expires_at_ns=None):
    return Decision(
        robot_id="robot-0",
        decision_id=f"decision-{map_version}-{now}",
        mode=mode,
        map_version=map_version,
        transform_version=transform,
        issued_at_ns=now,
        expires_at_ns=(now + 1_000_000_000) if expires_at_ns is None else expires_at_ns,
        target=GoalPose(x=1, y=2, z=0, yaw_rad=0) if mode == CommandMode.GOAL else None,
        reason="test",
    )


def make_guard():
    return GoalGuard(
        GoalGuardConfig(
            robot_id="robot-0",
            transform_version="calib-test-v1",
            shared_T_robot_map=IDENTITY,
        )
    )


def test_expired_goal_becomes_hold(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(now=10_000_000_000),
        now_ns=12_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD
    assert result.poi_json == "{}"


def test_transform_mismatch_becomes_hold(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(transform="wrong"),
        now_ns=10_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD


def test_ready_goal_reduces_to_legacy_poi(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(),
        now_ns=10_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.GOAL
    assert '"position":[1.0,2.0,0.0]' in result.poi_json


def test_stop_is_latched_until_local_reset(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    stop = guard.evaluate(
        make_decision(mode=CommandMode.STOP),
        now_ns=10_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert stop.action == GuardAction.STOP
    blocked = guard.evaluate(
        make_decision(map_version=2),
        now_ns=10_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert blocked.action == GuardAction.STOP
    guard.local_operator_reset_stop()
    accepted = guard.evaluate(
        make_decision(map_version=2),
        now_ns=10_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert accepted.action == GuardAction.GOAL


# --- G5 fault-injection: robot-side (GoalGuard) rejection paths ------------


def test_stop_latches_even_with_wrong_transform_version(observation_factory):
    """STOP must never be blockable by a stale/wrong calibration version."""
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    stop = guard.evaluate(
        make_decision(mode=CommandMode.STOP, transform="not-the-configured-one"),
        now_ns=10_000_000_000,
        health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert stop.action == GuardAction.STOP


def test_decision_for_another_robot_is_rejected(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    decision = make_decision()
    decision = decision.model_copy(update={"robot_id": "robot-1"})
    result = guard.evaluate(
        decision, now_ns=10_000_000_000, health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD
    assert result.ack_status.value == "REJECTED_UNSAFE"


def test_out_of_order_issued_at_becomes_hold(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    far_future_expiry = 1_000_000_000_000
    newer = guard.evaluate(
        make_decision(now=20_000_000_000, map_version=2, expires_at_ns=far_future_expiry),
        now_ns=20_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert newer.action == GuardAction.GOAL
    older = guard.evaluate(
        make_decision(now=10_000_000_000, map_version=2, expires_at_ns=far_future_expiry),
        now_ns=20_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert older.action == GuardAction.HOLD
    assert older.ack_status.value == "REJECTED_OUT_OF_ORDER"


def test_map_version_regression_becomes_hold(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    guard.evaluate(
        make_decision(map_version=5), now_ns=10_000_000_000, health=health,
        current_position_robot_map=(0, 0, 0),
    )
    regressed = guard.evaluate(
        make_decision(map_version=3), now_ns=10_000_000_000, health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert regressed.action == GuardAction.HOLD
    assert regressed.ack_status.value == "REJECTED_MAP_VERSION"


def test_estop_engaged_blocks_goal(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    health = health.model_copy(update={"estop_engaged": True})
    result = guard.evaluate(
        make_decision(), now_ns=10_000_000_000, health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD
    assert result.ack_status.value == "REJECTED_HEALTH"


def test_degraded_localization_blocks_goal(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    health = health.model_copy(update={"localization_state": "DEGRADED"})
    result = guard.evaluate(
        make_decision(), now_ns=10_000_000_000, health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD
    assert result.ack_status.value == "REJECTED_HEALTH"


def test_goal_beyond_max_distance_becomes_hold(observation_factory):
    guard = GoalGuard(
        GoalGuardConfig(
            robot_id="robot-0",
            transform_version="calib-test-v1",
            shared_T_robot_map=IDENTITY,
            max_goal_distance_m=1.0,
        )
    )
    health = observation_factory(mapping_only=False, health_ready=True).health
    # make_decision's target is (1, 2, 0); robot starts far from it.
    result = guard.evaluate(
        make_decision(), now_ns=10_000_000_000, health=health,
        current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD
    assert result.ack_status.value == "REJECTED_UNSAFE"
    assert "away" in result.detail


def test_hold_mode_passes_through_accepted(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(mode=CommandMode.HOLD),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert result.action == GuardAction.HOLD
    assert result.ack_status.value == "ACCEPTED"

