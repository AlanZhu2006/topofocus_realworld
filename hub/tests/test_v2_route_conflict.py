from __future__ import annotations

import pytest

from focus_hub.transport_v2 import DecisionBatchV2
from focus_hub.v2_route_conflict import apply_route_conflict_guard

from test_v2_registry import make_batch, ready_registries


def with_route_targets(
    batch: DecisionBatchV2,
    targets: dict[str, tuple[float, float]],
) -> DecisionBatchV2:
    raw = batch.model_dump(mode="json")
    for decision in raw["decisions"]:
        target = targets[decision["robot_id"]]
        decision["target"]["pose"]["x"] = target[0]
        decision["target"]["pose"]["y"] = target[1]
    return DecisionBatchV2.model_validate(raw)


def test_observed_crossing_routes_are_serialized(observation_factory):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_route_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (-1.289703371828395, 4.977325218102752),
            "robot-1": (2.110296628171607, 4.577325218102754),
        },
    )

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={
            "robot-0": (0.3362092841198523, -0.19971329635089863),
            "robot-1": (-0.9426940506797459, -0.1337700123489402),
        },
        minimum_separation_m=0.9,
    )

    assert report["status"] == "serialized_route_corridor_conflict"
    assert report["minimum_predicted_separation_m"] == pytest.approx(0.0)
    assert report["effective_active_robot_ids"] == ["robot-0"]
    assert report["suppressed_robot_ids"] == ["robot-1"]
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "HOLD"]
    assert guarded.decisions[0].target == candidate.decisions[0].target
    assert guarded.decisions[1].target is None


def test_conflict_prefers_robot_with_stronger_current_goal_evidence(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_route_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (-1.0, 4.0),
            "robot-1": (1.0, 4.0),
        },
    )

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={
            "robot-0": (0.0, 0.0),
            "robot-1": (0.2, 0.0),
        },
        minimum_separation_m=0.9,
        goal_evidence_by_robot={
            "robot-0": 0.56,
            "robot-1": 0.88,
        },
    )

    assert report["status"] == "serialized_route_corridor_conflict"
    assert report["effective_active_robot_ids"] == ["robot-1"]
    assert report["serialized_leader_robot_id"] == "robot-1"
    assert report["serialized_leader_priority_source"] == (
        "current_goal_detector_evidence"
    )
    assert report["goal_evidence_by_robot"] == {
        "robot-0": 0.56,
        "robot-1": 0.88,
    }
    assert [item.mode.value for item in guarded.decisions] == ["HOLD", "GOAL"]


def test_well_separated_parallel_routes_remain_concurrent(observation_factory):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_route_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (0.0, 5.0), "robot-1": (3.0, 5.0)},
    )

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={"robot-0": (0.0, 0.0), "robot-1": (3.0, 0.0)},
        minimum_separation_m=0.9,
    )

    assert report["status"] == "concurrent_corridors_clear"
    assert report["minimum_predicted_separation_m"] == pytest.approx(3.0)
    assert report["effective_active_robot_ids"] == ["robot-0", "robot-1"]
    assert guarded == candidate


def test_clear_close_start_with_strictly_diverging_routes_remains_concurrent(
    observation_factory,
):
    """Reproduce Scene 04 Formal 02's false route-conflict classification."""

    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_route_targets(
        make_batch(observations, digests, now=now),
        {
            "robot-0": (-1.5358560621465678, 2.041995729863027),
            "robot-1": (6.1641439378534315, 0.7419957298630262),
        },
    )

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={
            "robot-0": (0.15844079826624444, -0.28401190831029444),
            "robot-1": (0.9333089424626231, -0.01714636231466149),
        },
        minimum_separation_m=0.9,
        minimum_current_separation_m=0.69,
    )

    pair = report["pairwise"][0]
    assert report["status"] == (
        "concurrent_routes_separating_from_clear_start"
    )
    assert report["minimum_predicted_separation_m"] == pytest.approx(
        0.8195351490509037
    )
    assert pair["start_separation_m"] == pytest.approx(
        report["minimum_predicted_separation_m"]
    )
    assert pair["current_separation_clear"] is True
    assert pair["route_introduces_closer_approach"] is False
    assert pair["initial_proximity_only"] is True
    assert pair["conflict"] is False
    assert report["effective_active_robot_ids"] == ["robot-0", "robot-1"]
    assert guarded == candidate


def test_diverging_routes_still_serialize_when_current_pose_is_too_close(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = with_route_targets(
        make_batch(observations, digests, now=now),
        {"robot-0": (-3.0, 0.0), "robot-1": (3.2, 0.0)},
    )

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={"robot-0": (0.0, 0.0), "robot-1": (0.2, 0.0)},
        minimum_separation_m=0.9,
        minimum_current_separation_m=0.69,
    )

    pair = report["pairwise"][0]
    assert report["status"] == "serialized_route_corridor_conflict"
    assert pair["current_separation_clear"] is False
    assert pair["initial_proximity_only"] is False
    assert pair["conflict"] is True
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "HOLD"]


def test_missing_shared_pose_fails_closed_to_serial_execution(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = make_batch(observations, digests, now=now)

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={"robot-0": (0.0, 0.0)},
        minimum_separation_m=0.9,
        priority_index=1,
    )

    assert report["status"] == "serialized_missing_shared_pose"
    assert report["missing_shared_pose_robot_ids"] == ["robot-1"]
    assert report["effective_active_robot_ids"] == ["robot-0"]
    assert [item.mode.value for item in guarded.decisions] == ["GOAL", "HOLD"]


def test_all_missing_shared_poses_hold_both_robots(observation_factory):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    candidate = make_batch(observations, digests, now=now)

    guarded, report = apply_route_conflict_guard(
        candidate,
        shared_start_xy={},
        minimum_separation_m=0.9,
    )

    assert report["status"] == "blocked_missing_all_shared_poses"
    assert report["effective_active_robot_ids"] == []
    assert [item.mode.value for item in guarded.decisions] == ["HOLD", "HOLD"]
