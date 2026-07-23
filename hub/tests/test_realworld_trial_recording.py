from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


TOOLS = Path(__file__).resolve().parents[1] / "tools"


def load_tool(name: str):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def navigation_event(status: str, *, path_m: float) -> dict[str, object]:
    return {
        "status": status,
        "episode_start_local_pose": {"x": 1.0, "y": 2.0, "yaw_rad": 0.0},
        "local_pose": {"x": 2.0, "y": 2.5, "yaw_rad": 0.1},
        "path_length_m_from_episode_start": path_m,
        "velocity_zero_confirmed": status == "ARRIVED",
        "terminal_observation_sequence": 42,
    }


def test_arrival_evidence_survives_following_hold():
    module = load_tool("run_v2_supervised_episode")
    selected: dict[str, dict[str, object]] = {}

    module.update_evaluation_events(
        selected,
        {
            "robot-0": {
                "latest_event": navigation_event("ARRIVED", path_m=2.5)
            }
        },
    )
    module.update_evaluation_events(
        selected,
        {
            "robot-0": {
                "latest_event": navigation_event("HOLDING", path_m=2.5)
            }
        },
    )
    seed = module.evaluation_seed_from_events(selected)

    assert seed["robot-0"]["latest_navigation_status"] == "ARRIVED"
    assert seed["robot-0"]["local_planner_stopped"] is True
    assert seed["robot-0"]["actual_path_length_m"] == 2.5


def test_trial_recorder_writes_incomplete_auditable_progress(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_tool("record_realworld_trial")
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    report = tmp_path / "episode_report.json"
    survey_0 = tmp_path / "survey-0.json"
    survey_1 = tmp_path / "survey-1.json"
    terminal_0 = tmp_path / "terminal-0.jpg"
    survey_0.write_text('{"shortest_m": 2.0}\n')
    survey_1.write_text('{"shortest_m": 3.0}\n')
    terminal_0.write_bytes(b"terminal evidence")
    report.write_text(
        json.dumps(
            {
                "schema_version": "focus-v2-supervised-episode-run-v1",
                "scene_id": "scene01",
                "episode_id": "scene01-run01",
                "target_category": "chair",
                "live_goal_publication_enabled": True,
                "operator_confirmation": "OPERATOR_PRESENT_AND_ROBOTS_CLEAR",
                "robot_velocity_commands_sent_by_hub": False,
                "evaluation_seed": {
                    "robot-0": {
                        **navigation_event("ARRIVED", path_m=2.5),
                        "local_planner_stopped": True,
                        "stop_local_pose": {
                            "x": 2.0,
                            "y": 2.5,
                            "yaw_rad": 0.1,
                        },
                        "actual_path_length_m": 2.5,
                    },
                    "robot-1": {
                        **navigation_event("HOLDING", path_m=1.0),
                        "local_planner_stopped": False,
                        "stop_local_pose": {
                            "x": 0.5,
                            "y": 0.5,
                            "yaw_rad": 0.0,
                        },
                        "actual_path_length_m": 1.0,
                    },
                },
            }
        )
        + "\n"
    )
    results = tmp_path / "results.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "record_realworld_trial.py",
            "--episode-report",
            str(report),
            "--results",
            str(results),
            "--experiment-id",
            "demo",
            "--trial-index",
            "1",
            "--termination",
            "completed",
            "--robot-0-shortest-m",
            "2.0",
            "--robot-0-shortest-evidence",
            str(survey_0),
            "--robot-0-reached-goal-region",
            "yes",
            "--robot-0-target-verified",
            "yes",
            "--robot-0-terminal-evidence",
            str(terminal_0),
            "--robot-1-shortest-m",
            "3.0",
            "--robot-1-shortest-evidence",
            str(survey_1),
            "--robot-1-reached-goal-region",
            "no",
            "--robot-1-target-verified",
            "no",
        ],
    )

    assert module.main() == 0
    metrics = json.loads((tmp_path / "results_metrics.json").read_text())
    assert metrics["status"] == "incomplete"
    assert metrics["overall"]["successes"] == 1
    saved = json.loads(results.read_text())
    assert saved["episodes"][0]["robots"][0]["terminal_evidence"]["sha256"]


def test_trial_recorder_rejects_non_boolean_stop_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_tool("record_realworld_trial")
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    report = tmp_path / "report.json"
    survey = tmp_path / "survey.json"
    report.write_text("{}\n")
    survey.write_text("{}\n")
    seed = {
        "episode_start_local_pose": {"x": 0.0, "y": 0.0},
        "stop_local_pose": {"x": 1.0, "y": 0.0},
        "actual_path_length_m": 1.0,
        "local_planner_stopped": "false",
    }

    with pytest.raises(ValueError, match="not boolean"):
        module.robot_result(
            "robot-0",
            seed,
            episode_report=report,
            shortest_m=1.0,
            shortest_evidence=survey,
            reached_goal_region=False,
            target_verified=False,
            terminal_evidence=None,
        )
