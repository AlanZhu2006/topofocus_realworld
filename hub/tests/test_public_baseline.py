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
