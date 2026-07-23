#!/usr/bin/env python3
"""Append one supervised physical trial and immediately emit SR/SPL progress.

The controller report supplies robot-local start/stop poses, path length and
planner STOP evidence.  The operator must separately supply the surveyed
shortest-path files, goal-region judgments and terminal target evidence.
Those facts cannot be inferred safely from VLM output or ARRIVED alone.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub/src"))

from focus_hub.realworld_eval import (  # noqa: E402
    REALWORLD_RESULTS_SCHEMA_VERSION,
    RealworldEpisodeResult,
    RealworldExperimentResults,
    ResultEvidence,
    RobotEpisodeResult,
    XYPoint,
    score_experiment,
    validate_result_evidence,
)
from focus_hub.shadow_coordination import sha256_file  # noqa: E402


def workspace_path(value: Path) -> Path:
    path = value.expanduser()
    resolved = path.resolve() if path.is_absolute() else (WORKSPACE / path).resolve()
    if not resolved.is_relative_to(WORKSPACE.resolve()):
        raise ValueError(f"evidence must remain in the workspace: {resolved}")
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return resolved


def evidence(path: Path, classification: str) -> ResultEvidence:
    resolved = workspace_path(path)
    return ResultEvidence(
        path=str(resolved.relative_to(WORKSPACE.resolve())),
        size_bytes=resolved.stat().st_size,
        sha256=sha256_file(resolved),
        classification=classification,
    )


def atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def yes(value: str) -> bool:
    return value == "yes"


def require_number(
    value: object,
    label: str,
    *,
    non_negative: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a numeric JSON value")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    if non_negative and number < 0.0:
        raise ValueError(f"{label} must be non-negative")
    return number


def validate_episode_report(report: dict[str, object]) -> None:
    if report.get("schema_version") != "focus-v2-supervised-episode-run-v1":
        raise ValueError("episode report has the wrong schema")
    if report.get("live_goal_publication_enabled") is not True:
        raise ValueError("episode report was not a live publication")
    if report.get("operator_confirmation") != "OPERATOR_PRESENT_AND_ROBOTS_CLEAR":
        raise ValueError("episode report lacks the live operator confirmation")
    if report.get("robot_velocity_commands_sent_by_hub") is not False:
        raise ValueError("episode report violates the high-level-only Hub contract")
    if not isinstance(report.get("episode_id"), str) or not report["episode_id"]:
        raise ValueError("episode report lacks episode_id")


def robot_result(
    robot_id: str,
    seed: dict[str, object],
    *,
    episode_report: Path,
    shortest_m: float,
    shortest_evidence: Path,
    reached_goal_region: bool,
    target_verified: bool,
    terminal_evidence: Path | None,
) -> RobotEpisodeResult:
    start = seed.get("episode_start_local_pose")
    stop = seed.get("stop_local_pose")
    if not isinstance(start, dict) or not isinstance(stop, dict):
        raise ValueError(f"{robot_id} report lacks start/stop pose evidence")
    actual_path = require_number(
        seed.get("actual_path_length_m"),
        f"{robot_id} actual path length",
        non_negative=True,
    )
    if not isinstance(seed.get("local_planner_stopped"), bool):
        raise ValueError(f"{robot_id} planner STOP evidence is not boolean")
    terminal = (
        None
        if terminal_evidence is None
        else evidence(terminal_evidence, "observed")
    )
    return RobotEpisodeResult(
        robot_id=robot_id,
        start_xy=XYPoint(
            x_m=require_number(start.get("x"), f"{robot_id} start x"),
            y_m=require_number(start.get("y"), f"{robot_id} start y"),
        ),
        stop_xy=XYPoint(
            x_m=require_number(stop.get("x"), f"{robot_id} stop x"),
            y_m=require_number(stop.get("y"), f"{robot_id} stop y"),
        ),
        actual_path_length_m=actual_path,
        shortest_path_length_m=shortest_m,
        local_planner_stopped=seed.get("local_planner_stopped") is True,
        reached_goal_region=reached_goal_region,
        independent_target_verified=target_verified,
        trajectory_evidence=evidence(
            episode_report, "source-derived_from_observed"
        ),
        shortest_path_evidence=evidence(
            shortest_evidence, "source-derived_from_observed"
        ),
        terminal_evidence=terminal,
    )


def add_robot_arguments(parser: argparse.ArgumentParser, index: int) -> None:
    prefix = f"--robot-{index}"
    parser.add_argument(f"{prefix}-shortest-m", type=float, required=True)
    parser.add_argument(
        f"{prefix}-shortest-evidence", type=Path, required=True
    )
    parser.add_argument(
        f"{prefix}-reached-goal-region",
        choices=("yes", "no"),
        required=True,
    )
    parser.add_argument(
        f"{prefix}-target-verified",
        choices=("yes", "no"),
        required=True,
    )
    parser.add_argument(f"{prefix}-terminal-evidence", type=Path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-report", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--metrics-output", type=Path)
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--trial-index", type=int, required=True)
    parser.add_argument(
        "--termination",
        choices=(
            "completed",
            "timeout",
            "wrong_target",
            "operator_intervention",
            "system_failure",
            "collision",
            "aborted",
        ),
        required=True,
    )
    parser.add_argument("--notes", default="")
    parser.add_argument("--expected-scenes", type=int, default=4)
    parser.add_argument("--expected-trials-per-scene", type=int, default=5)
    add_robot_arguments(parser, 0)
    add_robot_arguments(parser, 1)
    args = parser.parse_args()

    try:
        report_path = workspace_path(args.episode_report)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if not isinstance(report, dict):
            raise ValueError("episode report must be a JSON object")
        validate_episode_report(report)
        scene_id = str(report.get("scene_id", ""))
        target_category = str(report.get("target_category", ""))
        seeds = report.get("evaluation_seed")
        if not scene_id or not target_category or not isinstance(seeds, dict):
            raise ValueError("episode report lacks evaluation identity/seed")
        robots = []
        for index, robot_id in enumerate(("robot-0", "robot-1")):
            seed = seeds.get(robot_id)
            if not isinstance(seed, dict):
                raise ValueError(f"episode report lacks {robot_id} seed")
            terminal_path = getattr(
                args, f"robot_{index}_terminal_evidence"
            )
            verified = yes(getattr(args, f"robot_{index}_target_verified"))
            if verified and terminal_path is None:
                raise ValueError(
                    f"{robot_id} verified target requires terminal evidence"
                )
            robots.append(
                robot_result(
                    robot_id,
                    seed,
                    episode_report=report_path,
                    shortest_m=getattr(args, f"robot_{index}_shortest_m"),
                    shortest_evidence=getattr(
                        args, f"robot_{index}_shortest_evidence"
                    ),
                    reached_goal_region=yes(
                        getattr(args, f"robot_{index}_reached_goal_region")
                    ),
                    target_verified=verified,
                    terminal_evidence=terminal_path,
                )
            )
        episode = RealworldEpisodeResult(
            scene_id=scene_id,
            trial_index=args.trial_index,
            target_category=target_category,
            termination=args.termination,
            operator_intervention=(
                args.termination == "operator_intervention"
            ),
            robots=tuple(robots),
            notes=args.notes,
        )
        results_path = (
            args.results.expanduser().resolve()
            if args.results.is_absolute()
            else (WORKSPACE / args.results).resolve()
        )
        if not results_path.is_relative_to(WORKSPACE.resolve()):
            raise ValueError("results file must remain inside the workspace")
        if results_path.exists():
            current = RealworldExperimentResults.model_validate_json(
                results_path.read_text(encoding="utf-8")
            )
            if current.experiment_id != args.experiment_id:
                raise ValueError("experiment ID differs from existing results")
            episodes = (*current.episodes, episode)
        else:
            episodes = (episode,)
        updated = RealworldExperimentResults(
            schema_version=REALWORLD_RESULTS_SCHEMA_VERSION,
            experiment_id=args.experiment_id,
            mode="supervised_autonomy",
            episodes=episodes,
        )
        validate_result_evidence(updated, workspace=WORKSPACE)
        metrics = score_experiment(
            updated,
            expected_scenes=args.expected_scenes,
            expected_trials_per_scene=args.expected_trials_per_scene,
            allow_incomplete=True,
        )
        metrics["records_file"] = str(results_path)
        if args.metrics_output is None:
            metrics_path = results_path.with_name(
                f"{results_path.stem}_metrics.json"
            )
        else:
            raw_metrics = args.metrics_output.expanduser()
            metrics_path = (
                raw_metrics.resolve()
                if raw_metrics.is_absolute()
                else (WORKSPACE / raw_metrics).resolve()
            )
        if not metrics_path.is_relative_to(WORKSPACE.resolve()):
            raise ValueError("metrics output must remain inside the workspace")
        atomic_write_json(
            results_path, updated.model_dump(mode="json")
        )
        atomic_write_json(metrics_path, metrics)
    except (
        FileExistsError,
        FileNotFoundError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        parser.error(str(exc))
    print(
        json.dumps(
            {
                "results": str(results_path),
                "metrics": str(metrics_path),
                "scene_id": scene_id,
                "trial_index": args.trial_index,
                "progress_status": metrics["status"],
                "overall": metrics["overall"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
