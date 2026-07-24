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
