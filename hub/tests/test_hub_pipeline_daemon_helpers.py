from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace


def load_daemon_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "tools"
        / "hub_pipeline_daemon.py"
    )
    name = "focus_test_hub_pipeline_daemon"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_map_snapshot_revision_does_not_change_with_wall_clock():
    daemon = load_daemon_module()
    pipeline = SimpleNamespace(
        last_observation_sequence=42,
        frames_processed=7,
        mapping_blocked_reason=None,
    )

    first = daemon.map_snapshot_revision(pipeline)
    second = daemon.map_snapshot_revision(pipeline)

    assert first == second == (42, 7, None)


def test_map_snapshot_revision_changes_for_input_map_or_latch():
    daemon = load_daemon_module()
    pipeline = SimpleNamespace(
        last_observation_sequence=42,
        frames_processed=7,
        mapping_blocked_reason=None,
    )
    baseline = daemon.map_snapshot_revision(pipeline)

    pipeline.last_observation_sequence = 43
    assert daemon.map_snapshot_revision(pipeline) != baseline
    baseline = daemon.map_snapshot_revision(pipeline)

    pipeline.frames_processed = 8
    assert daemon.map_snapshot_revision(pipeline) != baseline
    baseline = daemon.map_snapshot_revision(pipeline)

    pipeline.mapping_blocked_reason = "pose discontinuity"
    assert daemon.map_snapshot_revision(pipeline) != baseline
