"""SR/SPL accounting for supervised two-robot real-world episodes.

The scorer keeps standard SPL separate from the exact source-compatible
quantity.  Focus_realworld calls its start-to-stop displacement ratio SPL;
the standard ObjectNav definition instead uses a pre-surveyed shortest path
to a valid goal region.  Reporting both avoids silently changing the source
or silently relabelling its non-standard numerator.
"""
from __future__ import annotations

from collections import Counter, defaultdict
import math
from pathlib import Path
from statistics import fmean, pstdev
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .models import ROBOT_ID_PATTERN, SHA256_PATTERN, StrictModel
from .shadow_coordination import sha256_file
from .source_episode import SOURCE_HM3D_OBJECTNAV_GOALS


REALWORLD_RESULTS_SCHEMA_VERSION = "focus-realworld-demo-results-v1"
TERMINATIONS = (
    "completed",
    "timeout",
    "wrong_target",
    "operator_intervention",
    "system_failure",
    "collision",
    "aborted",
)


class XYPoint(StrictModel):
    x_m: float
    y_m: float

    @field_validator("x_m", "y_m")
    @classmethod
    def finite_coordinate(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("coordinates must be finite")
        return value


class ResultEvidence(StrictModel):
    path: str = Field(min_length=1, max_length=1024)
    size_bytes: int = Field(gt=0)
    sha256: str = Field(pattern=SHA256_PATTERN)
    classification: Literal[
        "observed",
        "source-derived_from_observed",
    ]


class RobotEpisodeResult(StrictModel):
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    start_xy: XYPoint
    stop_xy: XYPoint
    actual_path_length_m: float = Field(ge=0.0)
    shortest_path_length_m: float = Field(ge=0.0)
    local_planner_stopped: bool
    reached_goal_region: bool
    independent_target_verified: bool
    trajectory_evidence: ResultEvidence
    shortest_path_evidence: ResultEvidence
    terminal_evidence: ResultEvidence | None

    @field_validator("actual_path_length_m", "shortest_path_length_m")
    @classmethod
    def finite_distance(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("path lengths must be finite")
        return value

    @model_validator(mode="after")
    def require_terminal_evidence(self) -> "RobotEpisodeResult":
        if self.independent_target_verified and self.terminal_evidence is None:
            raise ValueError(
                "independent target verification requires terminal evidence"
            )
        return self


class RealworldEpisodeResult(StrictModel):
    scene_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{0,63}$")
    trial_index: int = Field(ge=1, le=1000)
    target_category: str = Field(min_length=1, max_length=64)
    termination: Literal[
        "completed",
        "timeout",
        "wrong_target",
        "operator_intervention",
        "system_failure",
        "collision",
        "aborted",
    ]
    operator_intervention: bool
    robots: tuple[RobotEpisodeResult, ...] = Field(min_length=2, max_length=2)
    notes: str = Field(default="", max_length=1024)

    @model_validator(mode="after")
    def validate_episode(self) -> "RealworldEpisodeResult":
        if self.target_category not in SOURCE_HM3D_OBJECTNAV_GOALS:
            raise ValueError(
                f"unsupported HPC ObjectNav target {self.target_category!r}"
            )
        robot_ids = [item.robot_id for item in self.robots]
        if set(robot_ids) != {"robot-0", "robot-1"} or len(set(robot_ids)) != 2:
            raise ValueError("an episode needs exactly robot-0 and robot-1")
        if self.operator_intervention != (
            self.termination == "operator_intervention"
        ):
            raise ValueError(
                "operator_intervention must agree with the termination label"
            )
        return self


class RealworldExperimentResults(StrictModel):
    schema_version: Literal["focus-realworld-demo-results-v1"] = (
        REALWORLD_RESULTS_SCHEMA_VERSION
    )
    experiment_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{0,127}$")
    mode: Literal["supervised_autonomy"] = "supervised_autonomy"
    episodes: tuple[RealworldEpisodeResult, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_results(self) -> "RealworldExperimentResults":
        identities = [(item.scene_id, item.trial_index) for item in self.episodes]
        if len(identities) != len(set(identities)):
            raise ValueError("scene/trial identities must be unique")
        scene_targets: dict[str, str] = {}
        for episode in self.episodes:
            previous = scene_targets.setdefault(
                episode.scene_id, episode.target_category
            )
            if previous != episode.target_category:
                raise ValueError("one scene cannot change target category across trials")
        return self


def _robot_success(
    episode: RealworldEpisodeResult,
    robot: RobotEpisodeResult,
) -> bool:
    return (
        episode.termination == "completed"
        and not episode.operator_intervention
        and robot.local_planner_stopped
        and robot.reached_goal_region
        and robot.independent_target_verified
    )


def _standard_spl(success: bool, shortest_m: float, actual_m: float) -> float:
    if not success:
        return 0.0
    if shortest_m <= 1e-3:
        return 1.0 if actual_m <= 1e-3 else 0.0
    return shortest_m / max(shortest_m, actual_m)


def _source_compatible_spl(
    success: bool,
    displacement_m: float,
    actual_m: float,
) -> float:
    if not success:
        return 0.0
    if actual_m <= 1e-3:
        return 1.0
    return min(displacement_m / actual_m, 1.0)


def score_episode(episode: RealworldEpisodeResult) -> dict[str, object]:
    robot_scores: list[dict[str, object]] = []
    for robot in episode.robots:
        success = _robot_success(episode, robot)
        displacement_m = math.hypot(
            robot.stop_xy.x_m - robot.start_xy.x_m,
            robot.stop_xy.y_m - robot.start_xy.y_m,
        )
        robot_scores.append(
            {
                "robot_id": robot.robot_id,
                "success": success,
                "actual_path_length_m": robot.actual_path_length_m,
                "shortest_path_length_m": robot.shortest_path_length_m,
                "start_to_stop_displacement_m": displacement_m,
                "standard_spl": _standard_spl(
                    success,
                    robot.shortest_path_length_m,
                    robot.actual_path_length_m,
                ),
                "source_compatible_spl": _source_compatible_spl(
                    success,
                    displacement_m,
                    robot.actual_path_length_m,
                ),
            }
        )
    episode_success = any(bool(item["success"]) for item in robot_scores)
    return {
        "scene_id": episode.scene_id,
        "trial_index": episode.trial_index,
        "target_category": episode.target_category,
        "termination": episode.termination,
        "operator_intervention": episode.operator_intervention,
        "success": episode_success,
        "sr": float(episode_success),
        # Exact source aggregation: best successful agent for the episode.
        "standard_multi_spl": max(
            float(item["standard_spl"]) for item in robot_scores
        ),
        "source_compatible_multi_spl": max(
            float(item["source_compatible_spl"]) for item in robot_scores
        ),
        "robots": robot_scores,
    }


def _metric_summary(scored: list[dict[str, object]]) -> dict[str, object]:
    sr_values = [float(item["sr"]) for item in scored]
    standard = [float(item["standard_multi_spl"]) for item in scored]
    source_compatible = [
        float(item["source_compatible_multi_spl"]) for item in scored
    ]
    return {
        "episodes": len(scored),
        "successes": int(sum(sr_values)),
        "sr": fmean(sr_values),
        "standard_multi_spl_mean": fmean(standard),
        "standard_multi_spl_population_std": pstdev(standard),
        "source_compatible_multi_spl_mean": fmean(source_compatible),
        "source_compatible_multi_spl_population_std": pstdev(source_compatible),
        "termination_counts": dict(
            sorted(Counter(str(item["termination"]) for item in scored).items())
        ),
    }


def score_experiment(
    results: RealworldExperimentResults,
    *,
    expected_scenes: int = 4,
    expected_trials_per_scene: int = 5,
    allow_incomplete: bool = False,
) -> dict[str, object]:
    scored = [score_episode(item) for item in results.episodes]
    by_scene: dict[str, list[dict[str, object]]] = defaultdict(list)
    for item in scored:
        by_scene[str(item["scene_id"])].append(item)

    shape_errors: list[str] = []
    if len(by_scene) != expected_scenes:
        shape_errors.append(
            f"expected {expected_scenes} scenes, observed {len(by_scene)}"
        )
    for scene_id, items in sorted(by_scene.items()):
        trials = sorted(int(item["trial_index"]) for item in items)
        expected = list(range(1, expected_trials_per_scene + 1))
        if trials != expected:
            shape_errors.append(
                f"scene {scene_id} trials are {trials}, expected {expected}"
            )
    if shape_errors and not allow_incomplete:
        raise ValueError("; ".join(shape_errors))

    return {
        "schema_version": "focus-realworld-demo-metrics-v1",
        "experiment_id": results.experiment_id,
        "mode": results.mode,
        "status": "complete" if not shape_errors else "incomplete",
        "shape_errors": shape_errors,
        "metric_contract": {
            "success": (
                "any robot completed a local-planner STOP inside the surveyed "
                "goal region with independent target verification; operator "
                "intervention invalidates autonomous success"
            ),
            "episode_aggregation": "max per-robot SPL, matching Focus_realworld main.py",
            "standard_spl_numerator": "pre-surveyed shortest path to valid goal region",
            "source_compatible_spl_numerator": "start-to-stop Euclidean displacement",
            "classification": "source-derived metric adapter over observed real-world records",
        },
        "overall": _metric_summary(scored),
        "scenes": {
            scene_id: _metric_summary(
                sorted(items, key=lambda item: int(item["trial_index"]))
            )
            for scene_id, items in sorted(by_scene.items())
        },
        "episodes": sorted(
            scored,
            key=lambda item: (str(item["scene_id"]), int(item["trial_index"])),
        ),
    }


def _resolved_workspace_path(workspace: Path, relative_path: str) -> Path:
    candidate = Path(relative_path)
    if candidate.is_absolute():
        raise ValueError(f"evidence path must be workspace-relative: {relative_path}")
    root = workspace.resolve()
    resolved = (root / candidate).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"evidence path escapes workspace: {relative_path}")
    return resolved


def validate_result_evidence(
    results: RealworldExperimentResults,
    *,
    workspace: Path,
) -> dict[str, object]:
    """Verify every trajectory, survey and terminal-evidence byte reference."""

    unique: dict[tuple[str, int, str], ResultEvidence] = {}
    for episode in results.episodes:
        for robot in episode.robots:
            references = {
                "trajectory": robot.trajectory_evidence,
                "shortest_path": robot.shortest_path_evidence,
            }
            if robot.terminal_evidence is not None:
                references["terminal"] = robot.terminal_evidence
            for role, evidence in references.items():
                key = (episode.scene_id, episode.trial_index, f"{robot.robot_id}:{role}")
                unique[key] = evidence

    total_bytes = 0
    verified_paths: set[str] = set()
    for evidence in unique.values():
        path = _resolved_workspace_path(workspace, evidence.path)
        if not path.is_file():
            raise FileNotFoundError(f"missing result evidence: {path}")
        observed_size = path.stat().st_size
        if observed_size != evidence.size_bytes:
            raise ValueError(
                f"evidence size drift for {evidence.path}: expected "
                f"{evidence.size_bytes}, observed {observed_size}"
            )
        if sha256_file(path) != evidence.sha256:
            raise ValueError(f"evidence hash drift for {evidence.path}")
        if evidence.path not in verified_paths:
            total_bytes += observed_size
            verified_paths.add(evidence.path)

    return {
        "status": "all_referenced_evidence_verified",
        "reference_count": len(unique),
        "unique_file_count": len(verified_paths),
        "unique_file_bytes": total_bytes,
        "classification": "observed and source-derived evidence identities verified",
    }
