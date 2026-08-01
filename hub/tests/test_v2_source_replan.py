from __future__ import annotations

import pytest

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
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
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
    assert [item["frontier_id"] for item in fallbacks["robot-0"]] == ["C", "D"]
    assert [item["frontier_id"] for item in fallbacks["robot-1"]] == ["D", "C"]
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


def test_inactive_hold_robot_has_no_physical_fallback_candidates(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    raw = with_source_frontiers(make_batch(observations, digests, now=now)).model_dump(
        mode="json"
    )
    for decision in raw["decisions"]:
        decision["coordination"]["active_robot_ids"] = ["robot-0"]
        if decision["robot_id"] == "robot-1":
            decision["mode"] = "HOLD"
            decision["target"] = None
    batch = DecisionBatchV2.model_validate(raw)

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=shadow_manifest(),
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (1.0, 2.0),
        },
    )

    assert rejected == frozenset()
    assert set(fallbacks) == {"robot-0"}
    assert report["checks"]["robot-1"]["active"] is False
    assert [
        item["frontier_id"]
        for item in report["checks"]["robot-1"]["accepted_fallback_candidates"]
    ] == ["B", "D", "C"]


def test_same_blocked_sector_matches_shifted_label_but_material_move_expires_it(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    raw = batch.model_dump(mode="json")
    raw["decisions"][0]["target"]["pose"]["x"] = 3.0
    raw["decisions"][0]["target"]["pose"]["y"] = 0.0
    failed = DecisionBatchV2.model_validate(raw).decisions[0]
    state = memory()
    state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_NO_PROGRESS",
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
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    raw = batch.model_dump(mode="json")
    raw["decisions"][0]["target"]["pose"]["x"] = 3.0
    raw["decisions"][0]["target"]["pose"]["y"] = 0.0
    failed = DecisionBatchV2.model_validate(raw).decisions[0]
    state = memory()
    state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_NO_PROGRESS",
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

    assert heading_sector[0]["match_kind"] == ("same_blocked_approach_sector")
    assert heading_sector[0]["blocked_bearing_source"] == (
        "observed_failure_base_heading"
    )
    assert old_target_sector == []


@pytest.mark.parametrize(
    "reason_code",
    [
        "LOCAL_PATH_REVERSE_REQUIRED",
        "LOCAL_ROUTER_HOLD_TIMEOUT",
        "LOCAL_PLANNER_PATH_STALE",
        "LOCAL_PLANNER_TURN_STALLED",
    ],
)
def test_one_transient_local_stack_failure_does_not_poison_spatial_target(
    observation_factory,
    reason_code,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    state = memory()

    update = state.record_frontier_failure(
        batch.decisions[0],
        reason_code=reason_code,
        failure_robot_xy_m=(0.0, 1.0),
        recorded_at_ns=now,
    )

    assert update["status"] == "recorded_transient_failure_pending"
    assert update["occurrence_count"] == 1
    assert state.entries == []
    assert state.transient_entries[0]["status"] == "pending"


def test_distance_limit_remains_non_spatial_even_when_reported_repeatedly(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    state = memory()

    for offset in range(2):
        update = state.record_frontier_failure(
            batch.decisions[0],
            reason_code="DISTANCE_LIMIT",
            failure_robot_xy_m=(0.0, 1.0),
            recorded_at_ns=now + offset,
        )
        assert update["status"] == "ignored_non_spatial_failure"

    assert state.entries == []
    assert state.transient_entries == []


def test_repeated_colocated_transient_failure_escalates_to_spatial_memory(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    failed = batch.decisions[0]
    state = memory()

    first = state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_PATH_STALE",
        failure_robot_xy_m=(0.0, 1.0),
        recorded_at_ns=now,
    )
    second = state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_TURN_STALLED",
        failure_robot_xy_m=(0.02, 1.01),
        recorded_at_ns=now + 1,
    )

    assert first["status"] == "recorded_transient_failure_pending"
    assert second["status"] == "escalated_repeated_transient_failure"
    assert second["occurrence_count"] == 2
    assert len(state.entries) == 1
    assert state.entries[0]["reason_codes"] == [
        "PERSISTENT_LOCAL_STACK_FAILURE",
        "LOCAL_PLANNER_PATH_STALE",
        "LOCAL_PLANNER_TURN_STALLED",
    ]
    target = failed.target
    assert target is not None
    matches = state.matching_entries(
        robot_id=failed.robot_id,
        target_xy_m=(target.pose.x, target.pose.y),
        robot_xy_m=(0.02, 1.01),
    )
    assert matches[0]["entry_id"] == second["entry_id"]


def test_relocated_transient_failure_starts_a_new_pending_approach(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    failed = batch.decisions[0]
    state = memory()

    state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_PATH_STALE",
        failure_robot_xy_m=(0.0, 0.0),
        recorded_at_ns=now,
    )
    update = state.record_frontier_failure(
        failed,
        reason_code="LOCAL_PLANNER_PATH_STALE",
        failure_robot_xy_m=(2.0, 0.0),
        recorded_at_ns=now + 1,
    )

    assert update["status"] == "recorded_transient_failure_pending"
    assert state.entries == []
    assert len(state.transient_entries) == 2


def test_source_stationary_threshold_is_two_and_a_half_cells():
    assert SOURCE_STAGNANT_REPLAN_M == 0.125


def test_source_stationary_evidence_is_not_labelled_observed(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
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
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))

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
    assert [item["frontier_id"] for item in fallbacks["robot-0"]] == ["C", "D"]


def test_spatially_completed_relabelled_frontier_uses_source_history_fallbacks(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    raw = with_source_frontiers(make_batch(observations, digests, now=now)).model_dump(
        mode="json"
    )
    robot_1_decision = next(
        item for item in raw["decisions"] if item["robot_id"] == "robot-1"
    )
    robot_1_decision["target"]["frontier_id"] = "D"
    robot_1_decision["target"]["pose"]["x"] = 1.20
    robot_1_decision["target"]["pose"]["y"] = 2.00
    batch = DecisionBatchV2.model_validate(raw)
    manifest = shadow_manifest()
    robot_1 = manifest["robots"][1]
    robot_1["candidate_frontiers"] = [
        frontier("D", 1.20, 2.00),
    ]
    robot_1["choice_probabilities"] = {"D": 1.0}
    robot_1["final_shadow_selection"] = {
        "kind": "frontier",
        "frontier_id": "D",
    }
    robot_1["candidate_history_nodes"] = [
        {
            "frontier_id": "history-0",
            "history_index": 0,
            "x_m": 3.0,
            "y_m": 2.0,
            "history_score": 4.0,
        },
        {
            "frontier_id": "history-1",
            "history_index": 1,
            "x_m": 4.0,
            "y_m": 2.0,
            "history_score": 7.0,
        },
    ]

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (1.0, 2.0),
        },
    )

    assert rejected == frozenset({"robot-1"})
    check = report["checks"]["robot-1"]
    assert check["current_frontier_arrival_already_satisfied"] is True
    assert check["current_target_rejected"] is True
    assert check["source_frontier_arrival_radius_m"] == 0.5
    assert [item["frontier_id"] for item in fallbacks["robot-1"]] == [
        "history-1",
        "history-0",
    ]
    assert check["fallback_ranking_source"].endswith(
        "_then_source_history_score_descending"
    )


def test_exhausted_frontiers_skip_completed_point_and_use_source_history(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
    state = memory()
    robot_1 = batch.decisions[1]
    state.record_frontier_failure(
        robot_1,
        reason_code="LOCAL_GOAL_UNREACHABLE",
        failure_robot_xy_m=(1.0, 2.0),
        recorded_at_ns=now,
    )
    failed_c_raw = robot_1.model_dump(mode="json")
    failed_c_raw["target"]["frontier_id"] = "C"
    failed_c_raw["target"]["pose"]["x"] = 4.0
    failed_c_raw["target"]["pose"]["y"] = 2.0
    state.record_frontier_failure(
        type(robot_1).model_validate(failed_c_raw),
        reason_code="LOCAL_GOAL_UNREACHABLE",
        failure_robot_xy_m=(1.0, 2.0),
        recorded_at_ns=now + 1,
    )
    manifest = shadow_manifest()
    result = manifest["robots"][1]
    result["candidate_frontiers"] = [
        frontier("B", 2.0, 3.0),
        frontier("C", 4.0, 2.0),
        frontier("D", 1.2, 2.0),
    ]
    result["choice_probabilities"] = {"B": 0.6, "C": 0.3, "D": 0.1}
    result["candidate_history_nodes"] = [
        {
            "frontier_id": "history-0",
            "history_index": 0,
            "x_m": 1.0,
            "y_m": 5.0,
            "history_score": 4.0,
        },
        {
            "frontier_id": "history-1",
            "history_index": 1,
            "x_m": -2.0,
            "y_m": 4.0,
            "history_score": 7.0,
        },
    ]

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=state,
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (1.0, 2.0),
        },
    )

    assert rejected == frozenset({"robot-1"})
    assert [item["frontier_id"] for item in fallbacks["robot-1"]] == [
        "history-1",
        "history-0",
    ]
    excluded = {
        item["frontier_id"]: item["excluded_reason"]
        for item in report["checks"]["robot-1"]["excluded_fallback_candidates"]
    }
    assert excluded["C"] == "navigation_failure_memory_match"
    assert excluded["D"] == "source_arrival_already_satisfied"


def test_history_fallback_uses_source_history_score_order(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    batch = with_source_frontiers(make_batch(observations, digests, now=now))
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

    assert [item["frontier_id"] for item in fallbacks["robot-0"]] == [
        "history-1",
        "history-2",
    ]
    assert report["checks"]["robot-0"]["fallback_ranking_source"] == (
        "source_history_score_descending_first_max"
    )


def test_long_cross_round_reversal_prefers_source_ranked_forward_alternative(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    raw = with_source_frontiers(make_batch(observations, digests, now=now)).model_dump(
        mode="json"
    )
    robot_1 = next(item for item in raw["decisions"] if item["robot_id"] == "robot-1")
    robot_1["target"]["pose"]["x"] = -7.461922679552056
    robot_1["target"]["pose"]["y"] = 5.323087209529522
    batch = DecisionBatchV2.model_validate(raw)
    manifest = shadow_manifest()
    result = manifest["robots"][1]
    result["candidate_frontiers"] = [
        frontier("A", 3.5880773204479457, -0.17691279047047814),
        frontier("B", -7.461922679552056, 5.323087209529522),
        frontier("C", 3.8380773204479457, 6.823087209529522),
    ]
    result["choice_probabilities"] = {
        "A": 0.0,
        "B": 0.6513548701194006,
        "C": 0.34864512988059937,
    }

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (-0.43306242095306674, 1.9786871151028098),
        },
        progress_vector_by_robot={
            "robot-0": (0.0, 2.0),
            "robot-1": (1.3060030586835352, 2.2172778810007925),
        },
    )

    assert rejected == frozenset({"robot-1"})
    assert [item["frontier_id"] for item in fallbacks["robot-1"]] == ["C"]
    check = report["checks"]["robot-1"]
    assert check["backtrack_redirected"] is True
    assert check["current_backtrack_check"]["severe_backtrack"] is True
    assert check["current_backtrack_check"]["angle_deg"] == pytest.approx(
        95.05303173007502
    )
    assert check["non_backtracking_fallback_count"] == 1


def test_backtracking_remains_available_when_source_has_no_forward_alternative(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    raw = with_source_frontiers(make_batch(observations, digests, now=now)).model_dump(
        mode="json"
    )
    robot_1 = next(item for item in raw["decisions"] if item["robot_id"] == "robot-1")
    robot_1["target"]["pose"]["x"] = -5.0
    robot_1["target"]["pose"]["y"] = 0.0
    batch = DecisionBatchV2.model_validate(raw)
    manifest = shadow_manifest()
    result = manifest["robots"][1]
    result["candidate_frontiers"] = [frontier("B", -5.0, 0.0)]
    result["choice_probabilities"] = {"B": 1.0}

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (0.0, 0.0),
        },
        progress_vector_by_robot={"robot-1": (2.0, 0.0)},
    )

    assert rejected == frozenset()
    assert fallbacks["robot-1"] == []
    check = report["checks"]["robot-1"]
    assert check["current_backtrack_check"]["severe_backtrack"] is True
    assert check["backtrack_redirected"] is False


def test_rear_target_outside_arrival_disk_has_no_two_meter_exemption(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    raw = with_source_frontiers(make_batch(observations, digests, now=now)).model_dump(
        mode="json"
    )
    robot_1 = next(item for item in raw["decisions"] if item["robot_id"] == "robot-1")
    robot_1["target"]["pose"]["x"] = -0.9
    robot_1["target"]["pose"]["y"] = 0.0
    batch = DecisionBatchV2.model_validate(raw)
    manifest = shadow_manifest()
    result = manifest["robots"][1]
    result["candidate_frontiers"] = [
        frontier("B", -0.9, 0.0),
        frontier("C", 2.0, 0.0),
    ]
    result["choice_probabilities"] = {"B": 0.8, "C": 0.2}

    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (0.0, 0.0),
        },
        progress_vector_by_robot={"robot-1": (1.0, 0.0)},
    )

    assert rejected == frozenset({"robot-1"})
    assert [item["frontier_id"] for item in fallbacks["robot-1"]] == [
        "C"
    ]
    check = report["checks"]["robot-1"]
    assert check["current_backtrack_check"]["target_distance_m"] == pytest.approx(
        0.9
    )
    assert check["current_backtrack_check"]["severe_backtrack"] is True
    assert report["backtrack_policy"]["minimum_target_distance_m"] == pytest.approx(
        0.5
    )


def test_failure_fallback_preserves_source_rank_but_tries_forward_first(
    observation_factory,
):
    observations, _registry, digests, now = ready_registries(observation_factory)
    raw = with_source_frontiers(make_batch(observations, digests, now=now)).model_dump(
        mode="json"
    )
    robot_1 = next(item for item in raw["decisions"] if item["robot_id"] == "robot-1")
    robot_1["target"]["pose"]["x"] = 2.0
    robot_1["target"]["pose"]["y"] = 0.0
    batch = DecisionBatchV2.model_validate(raw)
    manifest = shadow_manifest()
    result = manifest["robots"][1]
    result["candidate_frontiers"] = [
        frontier("B", 2.0, 0.0),
        frontier("rear", -3.0, 0.0),
        frontier("forward", 3.0, 0.5),
    ]
    result["choice_probabilities"] = {
        "B": 0.6,
        "rear": 0.3,
        "forward": 0.1,
    }
    rejected, fallbacks, report = evaluate_source_replan(
        batch,
        shadow_manifest=manifest,
        memory=memory(),
        robot_xy_by_robot={
            "robot-0": (0.0, 1.0),
            "robot-1": (0.0, 0.0),
        },
        source_stationary_robot_ids=frozenset({"robot-1"}),
        progress_vector_by_robot={"robot-1": (1.0, 0.0)},
    )

    assert rejected == frozenset({"robot-1"})
    candidates = fallbacks["robot-1"]
    assert [item["frontier_id"] for item in candidates] == [
        "forward",
        "rear",
    ]
    assert [item["source_rank"] for item in candidates] == [2, 1]
    assert [item["execution_priority_rank"] for item in candidates] == [
        0,
        1,
    ]
    assert [item["backtrack_priority_rank"] for item in candidates] == [
        0,
        1,
    ]
    assert report["checks"]["robot-1"]["direction_priority_applied"] is True
