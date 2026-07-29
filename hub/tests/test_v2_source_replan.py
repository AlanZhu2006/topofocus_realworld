from __future__ import annotations

from focus_hub.transport_v2 import DecisionBatchV2
from focus_hub.v2_source_replan import (
    SOURCE_STAGNANT_REPLAN_M,
    NavigationFailureMemory,
    evaluate_source_replan,
)

from test_v2_registry import make_batch, ready_registries


def with_source_frontiers(
    batch: DecisionBatchV2,
) -> DecisionBatchV2:
    raw = batch.model_dump(mode="json")
    values = {
        "robot-0": ("A", 1.0, 2.0),
        "robot-1": ("B", 2.0, 3.0),
    }
    for decision in raw["decisions"]:
        frontier_id, x_m, y_m = values[decision["robot_id"]]
        decision["target"]["frontier_id"] = frontier_id
        decision["target"]["pose"]["x"] = x_m
        decision["target"]["pose"]["y"] = y_m
    return DecisionBatchV2.model_validate(raw)


def frontier(
    frontier_id: str,
    x_m: float,
    y_m: float,
) -> dict[str, object]:
    return {
        "frontier_id": frontier_id,
        "x_m": x_m,
        "y_m": y_m,
    }


def shadow_manifest() -> dict[str, object]:
    return {
        "robots": [
            {
                "robot_id": "robot-0",
                "candidate_frontiers": [
                    frontier("A", 1.0, 2.0),
                    frontier("B", 2.0, 3.0),
                    frontier("C", 4.0, 2.0),
                    frontier("D", 4.0, 4.0),
                ],
                "choice_probabilities": {
                    "A": 0.50,
                    "B": 0.05,
                    "C": 0.30,
                    "D": 0.15,
                },
                "final_shadow_selection": {
                    "kind": "frontier",
                    "frontier_id": "A",
                },
            },
            {
                "robot_id": "robot-1",
                "candidate_frontiers": [
                    frontier("B", 2.0, 3.0),
                    frontier("C", 4.0, 2.0),
                    frontier("D", 4.0, 4.0),
                ],
                "choice_probabilities": {
                    "B": 0.60,
                    "C": 0.10,
                    "D": 0.30,
                },
                "final_shadow_selection": {
                    "kind": "frontier",
                    "frontier_id": "B",
                },
            },
        ]
    }


def memory() -> NavigationFailureMemory:
    return NavigationFailureMemory(
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        shared_frame_calibration_id="shared-board-v1",
    )


def test_local_rejection_blocks_same_target_and_preserves_robot_score_order(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )
    state = memory()
    update = state.record_frontier_failure(
        batch.decisions[0],
        reason_code="LOCAL_GOAL_UNREACHABLE",
        failure_robot_xy_m=(0.0, 0.0),
        recorded_at_ns=now,
        event={
            "event_id": "failure-a",
            "status": "REJECTED",
            "reason_code": "LOCAL_GOAL_UNREACHABLE",
            "observed_at_ns": now + 1,
        },
    )

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=shadow_manifest(),
        memory=state,
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (1.0, 2.0),
        },
    )

    assert update["status"] == "recorded_new_failure"
    assert state.entries[0]["evidence_classifications"] == [
        "observed_robot_local_rejection_event"
    ]
    assert state.entries[0]["last_event"]["observed_at_ns"] == now + 1
    assert rejected == frozenset({"robot-0"})
    assert [
        item["frontier_id"] for item in fallbacks["robot-0"]
    ] == ["C", "D"]
    assert [
        item["frontier_id"] for item in fallbacks["robot-1"]
    ] == ["D", "C"]
    check = report["checks"]["robot-0"]
    assert check["current_target_rejected"] is True
    assert check["current_memory_matches"][0]["match_kind"] == (
        "near_same_source_target"
    )
    excluded = {
        item["frontier_id"]: item["excluded_reason"]
        for item in check["excluded_fallback_candidates"]
    }
    assert excluded == {
        "A": "already_assigned_in_guard_input",
        "B": "already_assigned_in_guard_input",
    }


def test_same_blocked_sector_matches_shifted_label_but_material_move_expires_it(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )
    raw = batch.model_dump(mode="json")
    raw["decisions"][0]["target"]["pose"]["x"] = 3.0
    raw["decisions"][0]["target"]["pose"]["y"] = 0.0
    failed = DecisionBatchV2.model_validate(raw).decisions[0]
    state = memory()
    state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_PATH_STALE",
        failure_robot_xy_m=(0.0, 0.0),
        recorded_at_ns=now,
    )

    shifted = state.matching_entries(
        robot_id="robot-0",
        target_xy_m=(4.0, 0.5),
        robot_xy_m=(0.0, 0.0),
    )
    relocated = state.matching_entries(
        robot_id="robot-0",
        target_xy_m=(4.0, 0.5),
        robot_xy_m=(2.0, 0.0),
    )

    assert shifted[0]["match_kind"] == "same_blocked_approach_sector"
    assert relocated == []


def test_observed_failure_heading_defines_blocked_sector(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )
    raw = batch.model_dump(mode="json")
    raw["decisions"][0]["target"]["pose"]["x"] = 3.0
    raw["decisions"][0]["target"]["pose"]["y"] = 0.0
    failed = DecisionBatchV2.model_validate(raw).decisions[0]
    state = memory()
    state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_PATH_STALE",
        failure_robot_xy_m=(0.0, 0.0),
        failure_heading_deg=90.0,
        recorded_at_ns=now,
    )

    heading_sector = state.matching_entries(
        robot_id="robot-0",
        target_xy_m=(0.0, 4.0),
        robot_xy_m=(0.0, 0.0),
    )
    old_target_sector = state.matching_entries(
        robot_id="robot-0",
        target_xy_m=(4.0, 1.0),
        robot_xy_m=(0.0, 0.0),
    )

    assert heading_sector[0]["match_kind"] == (
        "same_blocked_approach_sector"
    )
    assert heading_sector[0]["blocked_bearing_source"] == (
        "observed_failure_base_heading"
    )
    assert old_target_sector == []


def test_transient_router_timeout_does_not_poison_spatial_target(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )
    state = memory()

    update = state.record_frontier_failure(
        batch.decisions[0],
        reason_code="LOCAL_ROUTER_HOLD_TIMEOUT",
        failure_robot_xy_m=(0.0, 1.0),
        recorded_at_ns=now,
    )

    assert update["status"] == "ignored_non_spatial_failure"
    assert state.entries == []


def test_source_stationary_threshold_is_two_and_a_half_cells():
    assert SOURCE_STAGNANT_REPLAN_M == 0.125


def test_source_stationary_evidence_is_not_labelled_observed(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )
    state = memory()

    state.record_frontier_failure(
        batch.decisions[0],
        reason_code="CROSS_ROUND_SOURCE_STALL",
        failure_robot_xy_m=(0.0, 1.0),
        recorded_at_ns=now,
    )

    assert state.entries[0]["evidence_classifications"] == [
        "source_derived_from_observed_shared_boundary_poses"
    ]
    assert state.to_dict()["classification"].startswith("mixed provenance")


def test_source_stationary_rule_rejects_current_frontier_even_if_vlm_changed_it(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=shadow_manifest(),
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (1.0, 2.0),
        },
        source_stationary_robot_ids=frozenset({"robot-0"}),
    )

    assert rejected == frozenset({"robot-0"})
    assert report["checks"]["robot-0"]["source_stationary_replan"] is True
    assert [
        item["frontier_id"] for item in fallbacks["robot-0"]
    ] == ["C", "D"]


def test_history_fallback_uses_source_history_score_order(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(
        observation_factory
    )
    batch = with_source_frontiers(
        make_batch(observations, digests, now=now)
    )
    raw = batch.model_dump(mode="json")
    raw["decisions"][0]["target"]["frontier_id"] = "history-0"
    batch = DecisionBatchV2.model_validate(raw)
    manifest = shadow_manifest()
    robot_0 = manifest["robots"][0]
    robot_0["final_shadow_selection"] = {
        "kind": "history",
        "history_index": 0,
    }
    robot_0["candidate_history_nodes"] = [
        {
            "frontier_id": "history-0",
            "history_index": 0,
            "x_m": 1.0,
            "y_m": 2.0,
            "history_score": 4.0,
        },
        {
            "frontier_id": "history-1",
            "history_index": 1,
            "x_m": 3.0,
            "y_m": 3.0,
            "history_score": 7.0,
        },
        {
            "frontier_id": "history-2",
            "history_index": 2,
            "x_m": 4.0,
            "y_m": 3.0,
            "history_score": 5.0,
        },
    ]

    _rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (1.0, 2.0),
        },
    )

    assert [
        item["frontier_id"] for item in fallbacks["robot-0"]
    ] == ["history-1", "history-2"]
    assert report["checks"]["robot-0"]["fallback_ranking_source"] == (
        "source_history_score_descending_first_max"
    )
