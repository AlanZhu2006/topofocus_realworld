from __future__ import annotations

import json
from pathlib import Path

import pytest

from focus_hub.public_baseline import (
    BaselineValidationError,
    DEFAULT_MANIFEST,
    validate_public_baseline,
)


WORKSPACE = Path(__file__).resolve().parents[2]


def test_repository_public_baseline_matches_file_contracts() -> None:
    summary = validate_public_baseline(WORKSPACE)
    assert summary.baseline_id == "rtx4090-go2-orin-nx-dual-robot-v1"
    assert summary.file_count >= 8
    assert summary.total_bytes > 0


def test_public_baseline_rejects_workspace_escape(tmp_path: Path) -> None:
    manifest = json.loads(
        (WORKSPACE / DEFAULT_MANIFEST).read_text(encoding="utf-8")
    )
    manifest["file_contracts"][0]["path"] = "../outside"
    candidate = tmp_path / "baseline.json"
    candidate.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(BaselineValidationError, match="escapes the workspace"):
        validate_public_baseline(WORKSPACE, candidate)


def test_public_baseline_locks_guarded_velocity_topic(
    tmp_path: Path,
) -> None:
    manifest = json.loads(
        (WORKSPACE / DEFAULT_MANIFEST).read_text(encoding="utf-8")
    )
    manifest["control_contract"]["guarded_velocity_topic"] = "/cmd_vel"
    candidate = tmp_path / "baseline.json"
    candidate.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(
        BaselineValidationError,
        match="guarded_velocity_topic",
    ):
        validate_public_baseline(WORKSPACE, candidate)


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("default_mode", "apply"),
        ("starts_robot_processes", True),
        ("downloads_simulator_data", True),
    ),
)
def test_public_baseline_locks_cleanroom_safety(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    manifest = json.loads(
        (WORKSPACE / DEFAULT_MANIFEST).read_text(encoding="utf-8")
    )
    manifest["software"]["cleanroom_install"][field] = value
    candidate = tmp_path / "baseline.json"
    candidate.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(BaselineValidationError, match=field):
        validate_public_baseline(WORKSPACE, candidate)


@pytest.mark.parametrize(
    ("section", "field", "value"),
    (
        ("controller", "continuous_turn_timeout_s", 0.0),
        (
            "semantic_execution_confirmation",
            "component_area_alone_grants_execution_authority",
            True,
        ),
        (
            "semantic_execution_confirmation",
            "semantic_map_reinforcement",
            True,
        ),
        (
            "frontier_goal_continuity",
            "physical_completion_distance_m",
            1.25,
        ),
    ),
)
def test_public_baseline_locks_scene03_execution_repairs(
    tmp_path: Path,
    section: str,
    field: str,
    value: object,
) -> None:
    manifest = json.loads(
        (WORKSPACE / DEFAULT_MANIFEST).read_text(encoding="utf-8")
    )
    manifest["planning"][section][field] = value
    candidate = tmp_path / "baseline.json"
    candidate.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(BaselineValidationError, match=field):
        validate_public_baseline(WORKSPACE, candidate)
