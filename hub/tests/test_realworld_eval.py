from __future__ import annotations

import hashlib

import pytest

from focus_hub.realworld_eval import (
    RealworldEpisodeResult,
    RealworldExperimentResults,
    score_episode,
    score_experiment,
    validate_result_evidence,
)


def robot_result(
    robot_id: str,
    *,
    verified: bool,
    actual_m: float = 5.0,
    shortest_m: float = 4.0,
) -> dict[str, object]:
    evidence = {
        "path": f"hub/runtime/test/{robot_id}.json",
        "size_bytes": 1,
        "sha256": "0" * 64,
        "classification": "observed",
    }
    return {
        "robot_id": robot_id,
        "start_xy": {"x_m": 0.0, "y_m": 0.0},
        "stop_xy": {"x_m": 3.0, "y_m": 0.0},
        "actual_path_length_m": actual_m,
        "shortest_path_length_m": shortest_m,
        "local_planner_stopped": verified,
        "reached_goal_region": verified,
        "independent_target_verified": verified,
        "trajectory_evidence": evidence,
        "shortest_path_evidence": {
            **evidence,
            "classification": "source-derived_from_observed",
        },
        "terminal_evidence": evidence if verified else None,
    }


def episode_payload(
    scene_id: str,
    trial_index: int,
    *,
    successful: bool,
    intervention: bool = False,
) -> dict[str, object]:
    return {
        "scene_id": scene_id,
        "trial_index": trial_index,
        "target_category": "chair",
        "termination": "operator_intervention" if intervention else "completed",
        "operator_intervention": intervention,
        "robots": [
            robot_result("robot-0", verified=successful),
            robot_result("robot-1", verified=False),
        ],
    }


def test_episode_reports_standard_and_exact_source_compatible_spl():
    episode = RealworldEpisodeResult.model_validate(
        episode_payload("scene_1", 1, successful=True)
    )
    score = score_episode(episode)

    assert score["success"] is True
    assert score["sr"] == 1.0
    assert score["standard_multi_spl"] == pytest.approx(0.8)
    assert score["source_compatible_multi_spl"] == pytest.approx(0.6)


def test_operator_intervention_invalidates_autonomous_metrics():
    episode = RealworldEpisodeResult.model_validate(
        episode_payload(
            "scene_1", 1, successful=True, intervention=True
        )
    )
    score = score_episode(episode)

    assert score["success"] is False
    assert score["standard_multi_spl"] == 0.0
    assert score["source_compatible_multi_spl"] == 0.0


def test_four_by_five_experiment_aggregates_sr_and_spl():
    episodes = [
        episode_payload(
            f"scene_{scene_index}",
            trial_index,
            successful=trial_index <= 3,
        )
        for scene_index in range(1, 5)
        for trial_index in range(1, 6)
    ]
    results = RealworldExperimentResults.model_validate(
        {"experiment_id": "demo_1", "episodes": episodes}
    )
    report = score_experiment(results)

    assert report["status"] == "complete"
    assert report["overall"]["episodes"] == 20
    assert report["overall"]["successes"] == 12
    assert report["overall"]["sr"] == pytest.approx(0.6)
    assert report["overall"]["standard_multi_spl_mean"] == pytest.approx(0.48)
    assert report["overall"]["source_compatible_multi_spl_mean"] == pytest.approx(0.36)


def test_incomplete_experiment_is_rejected_unless_explicitly_requested():
    results = RealworldExperimentResults.model_validate(
        {
            "experiment_id": "demo_partial",
            "episodes": [episode_payload("scene_1", 1, successful=False)],
        }
    )
    with pytest.raises(ValueError, match="expected 4 scenes"):
        score_experiment(results)

    report = score_experiment(results, allow_incomplete=True)
    assert report["status"] == "incomplete"
    assert report["shape_errors"]


def test_result_evidence_is_verified_against_workspace_bytes(tmp_path):
    payload = b"observed evidence"
    relative = "hub/runtime/test/evidence.json"
    path = tmp_path / relative
    path.parent.mkdir(parents=True)
    path.write_bytes(payload)
    evidence = {
        "path": relative,
        "size_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "classification": "observed",
    }
    episode = episode_payload("scene_1", 1, successful=True)
    for robot in episode["robots"]:
        robot["trajectory_evidence"] = evidence
        robot["shortest_path_evidence"] = {
            **evidence,
            "classification": "source-derived_from_observed",
        }
        if robot["independent_target_verified"]:
            robot["terminal_evidence"] = evidence
    results = RealworldExperimentResults.model_validate(
        {"experiment_id": "evidence_test", "episodes": [episode]}
    )

    report = validate_result_evidence(results, workspace=tmp_path)
    assert report["status"] == "all_referenced_evidence_verified"
    assert report["reference_count"] == 5
    assert report["unique_file_count"] == 1
