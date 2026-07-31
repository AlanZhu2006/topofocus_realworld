from __future__ import annotations

from focus_hub.v2_episode_control import (
    next_coordination_batch,
    recoverable_local_path_failure,
    scope_initial_coordination_batch,
)

from test_v2_registry import make_batch, ready_registries


def test_arrived_robot_holds_without_restarting_other_goal(observation_factory):
    observations, _registry, digests, now = ready_registries(observation_factory)
    first = make_batch(observations, digests, now=now)

    second = next_coordination_batch(
        first,
        active_robot_ids=("robot-1",),
        execution_epoch=2,
        issued_at_ns=now + 1_000_000_000,
        expires_at_ns=now + 9_000_000_000,
        identity_token="test",
    )

    wsj, yunji = second.decisions
    assert wsj.mode.value == "HOLD"
    assert wsj.target is None
    assert wsj.leg_id != first.decisions[0].leg_id
    assert yunji.mode.value == "GOAL"
    assert yunji.leg_id == first.decisions[1].leg_id
    assert yunji.lease_sequence == 1
    assert yunji.target == first.decisions[1].target
    assert tuple(yunji.coordination.active_robot_ids) == ("robot-1",)


def test_empty_active_set_produces_two_holds(observation_factory):
    observations, _registry, digests, now = ready_registries(observation_factory)
    first = make_batch(observations, digests, now=now)
    terminal = next_coordination_batch(
        first,
        active_robot_ids=(),
        execution_epoch=3,
        issued_at_ns=now + 1,
        expires_at_ns=now + 1_000_000_001,
        identity_token="terminal",
    )
    assert [decision.mode.value for decision in terminal.decisions] == ["HOLD", "HOLD"]
    assert tuple(terminal.decisions[0].coordination.active_robot_ids) == ()


def test_initial_readiness_isolation_preserves_ready_lease_zero_goal(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = make_batch(observations, digests, now=now)

    scoped = scope_initial_coordination_batch(
        candidate,
        active_robot_ids=("robot-1",),
        identity_token="readiness",
    )

    robot_0, robot_1 = scoped.decisions
    assert robot_0.mode.value == "HOLD"
    assert robot_0.target is None
    assert robot_0.lease_sequence == 0
    assert robot_0.leg_id != candidate.decisions[0].leg_id
    assert robot_1.mode.value == "GOAL"
    assert robot_1.target == candidate.decisions[1].target
    assert robot_1.leg_id == candidate.decisions[1].leg_id
    assert robot_1.decision_id == candidate.decisions[1].decision_id
    assert robot_1.lease_sequence == 0
    assert tuple(robot_1.coordination.active_robot_ids) == ("robot-1",)
    assert candidate.decisions[0].mode.value == "GOAL"


def test_initial_readiness_isolation_can_hold_both_candidates(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    candidate = make_batch(observations, digests, now=now)
    scoped = scope_initial_coordination_batch(
        candidate,
        active_robot_ids=(),
        identity_token="all-hold",
    )

    assert [item.mode.value for item in scoped.decisions] == ["HOLD", "HOLD"]
    assert all(item.lease_sequence == 0 for item in scoped.decisions)
    assert tuple(scoped.decisions[0].coordination.active_robot_ids) == ()


def test_only_explicit_local_path_failure_is_recoverable(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = make_batch(observations, digests, now=now)
    frontier = batch.decisions[1]

    assert recoverable_local_path_failure(
        frontier,
        {
            "status": "REJECTED",
            "reason_code": "LOCAL_PATH_REVERSE_REQUIRED",
        },
    )
    assert recoverable_local_path_failure(
        frontier,
        {
            "status": "REJECTED",
            "reason_code": "LOCAL_PLANNER_TURN_STALLED",
        },
    )
    assert recoverable_local_path_failure(
        frontier,
        {
            "status": "REJECTED",
            "reason_code": "LOCAL_PLANNER_NO_PROGRESS",
        },
    )
    semantic = batch.decisions[0]
    assert recoverable_local_path_failure(
        semantic,
        {
            "status": "REJECTED",
            "reason_code": "LOCAL_PLANNER_PATH_STALE",
        },
    )
    assert recoverable_local_path_failure(
        frontier,
        {"status": "REJECTED", "reason_code": "DISTANCE_LIMIT"},
    )
    assert not recoverable_local_path_failure(
        frontier,
        {"status": "LOCAL_ESTOP", "reason_code": "LOCAL_ESTOP"},
    )
    assert not recoverable_local_path_failure(
        frontier,
        {"status": "REJECTED", "reason_code": "TRANSFORM_MISMATCH"},
    )
