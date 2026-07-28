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


def test_snapshot_is_persisted_during_replay_only_when_due(
    tmp_path, monkeypatch
):
    daemon = load_daemon_module()
    pipeline = SimpleNamespace(
        last_observation_sequence=42,
        frames_processed=7,
        mapping_blocked_reason=None,
    )
    writes = []
    monkeypatch.setattr(
        daemon,
        "write_map_snapshot",
        lambda observed, out_dir: writes.append((observed, out_dir)),
    )

    last_at, revision = daemon.persist_map_snapshot_if_due(
        pipeline,
        tmp_path,
        3.0,
        10.0,
        None,
        now=12.9,
    )
    assert writes == []
    assert (last_at, revision) == (10.0, None)

    last_at, revision = daemon.persist_map_snapshot_if_due(
        pipeline,
        tmp_path,
        3.0,
        last_at,
        revision,
        now=13.0,
    )
    assert writes == [(pipeline, tmp_path)]
    assert (last_at, revision) == (13.0, (42, 7, None))

    last_at, revision = daemon.persist_map_snapshot_if_due(
        pipeline,
        tmp_path,
        3.0,
        last_at,
        revision,
        now=20.0,
    )
    assert len(writes) == 1
    assert (last_at, revision) == (13.0, (42, 7, None))

    pipeline.last_observation_sequence = 43
    last_at, revision = daemon.persist_map_snapshot_if_due(
        pipeline,
        tmp_path,
        3.0,
        last_at,
        revision,
        now=20.0,
    )
    assert len(writes) == 2
    assert (last_at, revision) == (20.0, (43, 7, None))


def test_cold_replay_streams_decoded_observations():
    source = (
        Path(__file__).resolve().parents[1]
        / "tools"
        / "hub_pipeline_daemon.py"
    ).read_text(encoding="utf-8")

    assert "new = list(iter_spooled_observations" not in source
    assert "for observation in iter_spooled_observations(" in source
    assert "saw_new_observation = False" in source
    assert "saw_new_observation = True" in source
    assert "if not saw_new_observation:" in source
    assert "if not new:" not in source
