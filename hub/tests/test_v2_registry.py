from __future__ import annotations

import hashlib

import pytest

from focus_hub.models import RobotHealth
from focus_hub.registry import ClockViolation, HubRegistry, RobotPolicy, UnsafeDecision
from focus_hub.transport_v2 import DecisionBatchV2, HighLevelDecisionV2, NavigationEventV2
from focus_hub.v2_registry import V2DecisionRegistry


ROBOTS = ("robot-0", "robot-1")
TRANSFORM = "calib-test-v1"


def make_health(ready: bool, *, detail: str = "") -> RobotHealth:
    return RobotHealth.model_validate({
        "safety_state": "READY" if ready else "UNKNOWN",
        "localization_state": "TRACKING" if ready else "UNKNOWN",
        "estop_engaged": False,
        "collision_avoidance_ready": ready,
        "motor_controller_ready": ready,
        "detail": detail,
    })


def ready_registries(observation_factory, *, second_mapping_only: bool = False):
    policies = {
        robot_id: RobotPolicy(TRANSFORM, allow_goal=True) for robot_id in ROBOTS
    }
    observations = HubRegistry(policies)
    now = 20_000_000_000
    digests: dict[str, str] = {}
    for index, robot_id in enumerate(ROBOTS):
        digest = hashlib.sha256(f"payload-{robot_id}".encode()).hexdigest()
        metadata = observation_factory(
            robot_id=robot_id,
            sequence=index + 4,
            now_ns=now,
            mapping_only=second_mapping_only and robot_id == "robot-1",
            health_ready=not (second_mapping_only and robot_id == "robot-1"),
        )
        observations.accept_observation(metadata, digest, now_ns=now)
        digests[robot_id] = digest
    return observations, V2DecisionRegistry(observations, policies), digests, now


def make_batch(
    observations: HubRegistry,
    digests: dict[str, str],
    *,
    now: int,
    lease_sequence: int = 0,
    decision_suffix: str = "0",
    modes: tuple[str, str] = ("GOAL", "GOAL"),
    active_robot_ids: tuple[str, ...] = ROBOTS,
    issued_at_ns: int | None = None,
    expires_at_ns: int | None = None,
) -> DecisionBatchV2:
    issued_at_ns = now if issued_at_ns is None else issued_at_ns
    expires_at_ns = now + 8_000_000_000 if expires_at_ns is None else expires_at_ns
    inputs = {
        robot_id: {
            "sequence": observations.snapshot(robot_id).last_sequence,
            "capture_time_ns": observations.snapshot(robot_id).last_observation.capture_time_ns,
            "payload_sha256": digests[robot_id],
        }
        for robot_id in ROBOTS
    }
    decisions = []
    for index, (robot_id, mode) in enumerate(zip(ROBOTS, modes)):
        target = None
        if mode == "GOAL":
            target = {
                "kind": "FRONTIER_POINT",
                "frontier_id": f"frontier-{index}",
                "source_goal_dilation_cells": 10,
                "pose": {
                    "frame_id": "shared_world",
                    "x": index + 1.0,
                    "y": index + 2.0,
                    "z": 0.0,
                    "yaw_rad": 0.25,
                },
            }
        decisions.append(HighLevelDecisionV2.model_validate({
            "robot_id": robot_id,
            "scene_id": "scene-1",
            "episode_id": "scene-1-trial-1",
            "round_index": 1,
            "source_step": 24,
            "decision_batch_id": "batch-1",
            "leg_id": f"leg-{robot_id}",
            "decision_id": f"decision-{robot_id}-{decision_suffix}",
            "lease_sequence": lease_sequence,
            "mode": mode,
            "coordination": {
                "execution_epoch": 1,
                "active_robot_ids": list(active_robot_ids),
            },
            "goal_category": "chair",
            "input_observations": inputs,
            "map_provenance": {
                "map_version": observations.snapshot(robot_id).map_version,
                "map_snapshot_sha256": ("c" if index == 0 else "d") * 64,
                "map_format_version": "focus-hub-central-map-v3",
                "frame_id": "shared_world",
                "resolution_m": 0.05,
                "transform_version": TRANSFORM,
                "shared_frame_calibration_id": "shared-board-v1",
            },
            "issued_at_ns": issued_at_ns,
            "expires_at_ns": expires_at_ns,
            "target": target,
            "reason": "test concurrent goal",
        }))
    return DecisionBatchV2(decisions=tuple(decisions))


def make_event(decision: HighLevelDecisionV2, *, now: int, event_id: str) -> NavigationEventV2:
    return NavigationEventV2.model_validate({
        "robot_id": decision.robot_id,
        "scene_id": decision.scene_id,
        "episode_id": decision.episode_id,
        "decision_batch_id": decision.decision_batch_id,
        "leg_id": decision.leg_id,
        "decision_id": decision.decision_id,
        "lease_sequence": decision.lease_sequence,
        "event_id": event_id,
        "status": "NAVIGATING",
        "reason_code": "LOCAL_PLANNER_ACTIVE",
        "observed_at_ns": now,
        "local_pose": {
            "frame_id": f"{decision.robot_id}/map",
            "x": 0,
            "y": 0,
            "yaw_rad": 0,
        },
        "path_length_m_from_episode_start": 0.2,
        "velocity_zero_confirmed": False,
    })


def test_two_goals_publish_and_expire_independently(observation_factory):
    observations, registry, digests, now = ready_registries(observation_factory)
    batch = make_batch(observations, digests, now=now)
    registry.publish_batch(batch, now_ns=now)
    for decision in batch.decisions:
        assert registry.effective_decision(
            decision.robot_id, now_ns=now + 1
        ).decision_id == decision.decision_id
        assert registry.effective_decision(
            decision.robot_id, now_ns=now + 9_000_000_000
        ) is None


def test_frozen_inputs_remain_valid_while_new_frames_arrive(observation_factory):
    observations, registry, digests, now = ready_registries(observation_factory)
    frozen_batch = make_batch(observations, digests, now=now)

    for index, robot_id in enumerate(ROBOTS):
        newer = observation_factory(
            robot_id=robot_id,
            sequence=100 + index,
            now_ns=now + 1_000_000_000,
            mapping_only=False,
            health_ready=True,
        )
        observations.accept_observation(
            newer,
            hashlib.sha256(f"new-{robot_id}".encode()).hexdigest(),
            now_ns=now + 1_000_000_000,
        )

    registry.publish_batch(frozen_batch, now_ns=now + 1_000_000_000)
    assert registry.effective_decision(
        "robot-0", now_ns=now + 1_000_000_001
    ).decision_id == "decision-robot-0-0"


def test_receiver_heartbeat_remains_authoritative_after_newer_rgbd_health(
    observation_factory,
):
    observations, registry, digests, now = ready_registries(observation_factory)
    frozen_batch = make_batch(observations, digests, now=now)
    observations.accept_heartbeat(
        "robot-0",
        make_health(True, detail="command receiver ready"),
        now,
        now_ns=now,
    )

    newer_rgbd = observation_factory(
        robot_id="robot-0",
        sequence=100,
        now_ns=now + 1_000_000_000,
        mapping_only=False,
        health_ready=False,
    )
    observations.accept_observation(
        newer_rgbd,
        hashlib.sha256(b"newer-unready-rgbd").hexdigest(),
        now_ns=now + 1_000_000_000,
    )

    registry.publish_batch(frozen_batch, now_ns=now + 1_000_000_000)
    assert registry.effective_decision(
        "robot-0", now_ns=now + 1_000_000_001
    ).decision_id == "decision-robot-0-0"


def test_stale_receiver_heartbeat_never_falls_back_to_fresh_rgbd_health(
    observation_factory,
):
    observations, registry, digests, now = ready_registries(observation_factory)
    batch = make_batch(
        observations,
        digests,
        now=now,
        modes=("GOAL", "HOLD"),
        active_robot_ids=("robot-0",),
    )
    observations.accept_heartbeat(
        "robot-0",
        make_health(True, detail="command receiver ready"),
        now,
        now_ns=now,
    )

    publish_at = now + 3_100_000_000
    newer_rgbd = observation_factory(
        robot_id="robot-0",
        sequence=100,
        now_ns=publish_at,
        mapping_only=False,
        health_ready=True,
    )
    observations.accept_observation(
        newer_rgbd,
        hashlib.sha256(b"fresh-ready-rgbd").hexdigest(),
        now_ns=publish_at,
    )

    with pytest.raises(
        UnsafeDecision,
        match=r"robot-0 robot health is stale",
    ):
        registry.publish_batch(batch, now_ns=publish_at)


def test_v2_health_rejection_identifies_robot_and_receiver_state(
    observation_factory,
):
    observations, registry, digests, now = ready_registries(observation_factory)
    batch = make_batch(
        observations,
        digests,
        now=now,
        modes=("GOAL", "HOLD"),
        active_robot_ids=("robot-0",),
    )
    observations.accept_heartbeat(
        "robot-0",
        make_health(False, detail="local planner not ready"),
        now,
        now_ns=now,
    )

    with pytest.raises(UnsafeDecision) as excinfo:
        registry.publish_batch(batch, now_ns=now)
    message = str(excinfo.value)
    assert "robot-0 health does not permit a GOAL" in message
    assert "safety=UNKNOWN" in message
    assert "localization=UNKNOWN" in message
    assert "detail='local planner not ready'" in message


def test_frozen_inputs_expire_after_bounded_history_window(observation_factory):
    observations, registry, digests, now = ready_registries(observation_factory)
    raw = make_batch(observations, digests, now=now).model_dump(mode="json")
    publish_at = now + 31_000_000_000
    for decision in raw["decisions"]:
        decision["issued_at_ns"] = publish_at
        decision["expires_at_ns"] = publish_at + 8_000_000_000
    stale_batch = DecisionBatchV2.model_validate(raw)

    with pytest.raises(ClockViolation, match="input is stale"):
        registry.publish_batch(stale_batch, now_ns=publish_at)


def test_batch_commit_is_atomic_when_one_robot_is_mapping_only(observation_factory):
    observations, registry, digests, now = ready_registries(
        observation_factory, second_mapping_only=True
    )
    batch = make_batch(observations, digests, now=now)
    with pytest.raises(UnsafeDecision, match="mapping_only"):
        registry.publish_batch(batch, now_ns=now)
    assert registry.effective_decision("robot-0", now_ns=now) is None
    assert registry.effective_decision("robot-1", now_ns=now) is None


def test_renewal_requires_fresh_feedback_from_each_robot(observation_factory):
    observations, registry, digests, now = ready_registries(observation_factory)
    first = make_batch(observations, digests, now=now)
    registry.publish_batch(first, now_ns=now)
    renewal = make_batch(
        observations,
        digests,
        now=now,
        lease_sequence=1,
        decision_suffix="1",
        issued_at_ns=now + 1_000_000_000,
        expires_at_ns=now + 9_000_000_000,
    )
    with pytest.raises(UnsafeDecision, match="fresh feedback"):
        registry.publish_batch(renewal, now_ns=now + 1_000_000_000)

    for index, decision in enumerate(first.decisions):
        event = make_event(decision, now=now + 500_000_000, event_id=f"event-{index}")
        result = registry.accept_event(
            event,
            hashlib.sha256(event.model_dump_json().encode()).hexdigest(),
            now_ns=now + 500_000_000,
        )
        assert result.status == "accepted"
    registry.publish_batch(renewal, now_ns=now + 1_000_000_000)
    assert registry.effective_decision(
        "robot-0", now_ns=now + 1_000_000_001
    ).lease_sequence == 1


def test_one_robot_holds_while_other_renews_without_restarting_leg(
    observation_factory,
):
    observations, registry, digests, now = ready_registries(observation_factory)
    first = make_batch(observations, digests, now=now)
    registry.publish_batch(first, now_ns=now)

    arrived_raw = make_event(
        first.decisions[0], now=now + 500_000_000, event_id="arrived-0"
    ).model_dump(mode="json")
    arrived_raw["status"] = "ARRIVED"
    arrived_raw["reason_code"] = "LOCAL_PLANNER_ARRIVED"
    arrived_raw["velocity_zero_confirmed"] = True
    arrived = NavigationEventV2.model_validate(arrived_raw)
    navigating = make_event(
        first.decisions[1], now=now + 500_000_000, event_id="navigating-1"
    )
    for event in (arrived, navigating):
        registry.accept_event(
            event,
            hashlib.sha256(event.model_dump_json().encode()).hexdigest(),
            now_ns=now + 500_000_000,
        )

    raw = make_batch(
        observations,
        digests,
        now=now,
        lease_sequence=1,
        decision_suffix="1",
        modes=("HOLD", "GOAL"),
        active_robot_ids=("robot-1",),
        issued_at_ns=now + 1_000_000_000,
        expires_at_ns=now + 9_000_000_000,
    ).model_dump(mode="json")
    # The arrived robot starts a new HOLD leg. The still-moving robot keeps
    # its existing GOAL leg and lease sequence, so its local planner does not
    # restart merely because the other robot arrived first.
    raw["decisions"][0]["leg_id"] = "hold-leg-robot-0"
    raw["decisions"][0]["lease_sequence"] = 0
    second = DecisionBatchV2.model_validate(raw)

    registry.publish_batch(second, now_ns=now + 1_000_000_000)
    assert registry.effective_decision(
        "robot-0", now_ns=now + 1_000_000_001
    ).mode.value == "HOLD"
    active = registry.effective_decision(
        "robot-1", now_ns=now + 1_000_000_001
    )
    assert active.mode.value == "GOAL"
    assert active.leg_id == first.decisions[1].leg_id
    assert active.lease_sequence == 1


def test_navigation_event_retry_is_idempotent(observation_factory):
    observations, registry, digests, now = ready_registries(observation_factory)
    batch = make_batch(observations, digests, now=now)
    registry.publish_batch(batch, now_ns=now)
    event = make_event(batch.decisions[0], now=now + 1, event_id="event-0")
    digest = hashlib.sha256(event.model_dump_json().encode()).hexdigest()
    assert registry.accept_event(event, digest, now_ns=now + 1).status == "accepted"
    assert registry.accept_event(event, digest, now_ns=now + 2).status == "duplicate"
