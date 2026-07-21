from __future__ import annotations

from focus_hub.models import CommandMode, Decision, GoalPose
from focus_hub.yunji_goal_guard import YunjiGoalGuard, YunjiGoalGuardConfig, YunjiGuardAction

from conftest import IDENTITY


def make_decision(*, mode=CommandMode.GOAL, now=10_000_000_000, map_version=1,
                  transform="calib-test-v1", expires_at_ns=None):
    return Decision(
        robot_id="robot-1",
        decision_id=f"decision-{map_version}-{now}",
        mode=mode,
        map_version=map_version,
        transform_version=transform,
        issued_at_ns=now,
        expires_at_ns=(now + 1_000_000_000) if expires_at_ns is None else expires_at_ns,
        target=GoalPose(x=1, y=2, z=0, yaw_rad=0.5) if mode == CommandMode.GOAL else None,
        reason="test",
    )


def make_guard(max_goal_distance_m: float = 8.0) -> YunjiGoalGuard:
    return YunjiGoalGuard(
        YunjiGoalGuardConfig(
            robot_id="robot-1",
            transform_version="calib-test-v1",
            shared_T_robot_map=IDENTITY,
            max_goal_distance_m=max_goal_distance_m,
        )
    )


def test_expired_goal_becomes_hold(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(now=10_000_000_000),
        now_ns=12_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert result.action == YunjiGuardAction.HOLD
    assert result.move_request is None


def test_transform_mismatch_becomes_hold(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(transform="wrong"),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert result.action == YunjiGuardAction.HOLD
    assert result.ack_status.value == "REJECTED_TRANSFORM"


def test_ready_goal_produces_dry_run_move_request(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert result.action == YunjiGuardAction.GOAL
    assert result.move_request.startswith("/api/move?location=1.0000,2.0000,0.5000")
    assert "&uuid=" in result.move_request


def test_stop_is_latched_until_local_reset(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    stop = guard.evaluate(
        make_decision(mode=CommandMode.STOP),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert stop.action == YunjiGuardAction.STOP
    blocked = guard.evaluate(
        make_decision(map_version=2),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert blocked.action == YunjiGuardAction.STOP
    assert blocked.ack_status.value == "REJECTED_UNSAFE"
    guard.local_operator_reset_stop()
    accepted = guard.evaluate(
        make_decision(map_version=2),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert accepted.action == YunjiGuardAction.GOAL


def test_stop_latches_even_with_wrong_transform_version(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    stop = guard.evaluate(
        make_decision(mode=CommandMode.STOP, transform="not-the-configured-one"),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0),
    )
    assert stop.action == YunjiGuardAction.STOP


def test_decision_for_another_robot_is_rejected(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    decision = make_decision().model_copy(update={"robot_id": "robot-0"})
    result = guard.evaluate(
        decision, now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0))
    assert result.action == YunjiGuardAction.HOLD
    assert result.ack_status.value == "REJECTED_UNSAFE"


def test_unhealthy_robot_blocks_goal(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    health = health.model_copy(update={"estop_engaged": True})
    result = guard.evaluate(
        make_decision(), now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0))
    assert result.action == YunjiGuardAction.HOLD
    assert result.ack_status.value == "REJECTED_HEALTH"


def test_goal_beyond_max_distance_becomes_hold(observation_factory):
    guard = make_guard(max_goal_distance_m=1.0)
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(), now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0))
    assert result.action == YunjiGuardAction.HOLD
    assert result.ack_status.value == "REJECTED_UNSAFE"
    assert "away" in result.detail


def test_hold_mode_passes_through_accepted(observation_factory):
    guard = make_guard()
    health = observation_factory(mapping_only=False, health_ready=True).health
    result = guard.evaluate(
        make_decision(mode=CommandMode.HOLD),
        now_ns=10_000_000_000, health=health, current_position_robot_map=(0, 0, 0))
    assert result.action == YunjiGuardAction.HOLD
    assert result.ack_status.value == "ACCEPTED"


def test_move_request_is_never_sent_only_constructed():
    """Documentation-as-test: this module has no HTTP/socket import at all,
    so it is structurally incapable of calling the real WATER API."""
    import focus_hub.yunji_goal_guard as module

    assert "socket" not in dir(module)
    assert "requests" not in dir(module)
    assert "urllib" not in dir(module)
