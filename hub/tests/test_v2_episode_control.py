from __future__ import annotations

from focus_hub.v2_episode_control import next_coordination_batch

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
