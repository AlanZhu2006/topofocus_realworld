from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
import pytest


def load_scene_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "live_vlm_scene.py"
    name = "focus_test_live_vlm_scene"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def write_ready_tree(module, tmp_path, observation):
    snapshot_dir = tmp_path / "map"
    spool = tmp_path / "spool"
    snapshot_dir.mkdir()
    source_dir = spool / observation.robot_id / f"{observation.sequence:020d}"
    source_dir.mkdir(parents=True)
    (source_dir / "metadata.json").write_text(
        observation.model_dump_json(), encoding="utf-8"
    )
    np.savez_compressed(
        snapshot_dir / "central_map.npz",
        grid=np.zeros((17, 10, 10), dtype=np.float32),
        origin_xy_m=np.array([0.0, 0.0]),
        resolution_m=np.array(0.05),
        frame_id=np.asarray("shared_world"),
        transform_version=np.asarray(observation.pose.transform_version),
        shared_frame_calibration_id=np.asarray("shared-test-v1"),
        map_format_version=np.asarray("focus-hub-central-map-v3"),
    )
    status = {
        "mapping_blocked_reason": None,
        "shared_frame_calibration_id": "shared-test-v1",
        "transform_version": observation.pose.transform_version,
    }
    summary = {
        **status,
        "semantic_mapping": {
            "yolo_reinforcement": {
                "enabled": True,
                "last_sequence": observation.sequence,
            }
        },
    }
    (snapshot_dir / "live_status.json").write_text(
        json.dumps(status), encoding="utf-8"
    )
    (snapshot_dir / "map_summary.json").write_text(
        json.dumps(summary), encoding="utf-8"
    )
    return module.RobotSpec(observation.robot_id, "robot", snapshot_dir), spool


def test_inspect_ready_input_enforces_new_sequence_and_contract(
    tmp_path, observation_factory
):
    module = load_scene_module()
    observation = observation_factory(sequence=42, now_ns=time.time_ns())
    spec, spool = write_ready_tree(module, tmp_path, observation)

    ready = module.inspect_ready_input(
        spec,
        spool=spool,
        calibration_id="shared-test-v1",
        previous_sequence=41,
    )

    assert ready is not None
    assert ready.sequence == 42
    assert ready.transform_version == "calib-test-v1"
    assert module.inspect_ready_input(
        spec,
        spool=spool,
        calibration_id="shared-test-v1",
        previous_sequence=42,
    ) is None


def test_inspect_ready_input_aborts_on_mapping_block(tmp_path, observation_factory):
    module = load_scene_module()
    observation = observation_factory(sequence=7)
    spec, spool = write_ready_tree(module, tmp_path, observation)
    status_path = spec.snapshot_dir / "live_status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["mapping_blocked_reason"] = "ground drift"
    status_path.write_text(json.dumps(status), encoding="utf-8")

    with pytest.raises(module.SceneSafetyAbort, match="ground drift"):
        module.inspect_ready_input(
            spec,
            spool=spool,
            calibration_id="shared-test-v1",
            previous_sequence=None,
        )


def test_input_timing_requires_fresh_and_synchronized():
    module = load_scene_module()
    now = 100_000_000_000
    fresh = [
        module.ReadyInput("robot-0", "a", 1, 98_000_000_000, "t0", "a" * 64),
        module.ReadyInput("robot-1", "b", 2, 96_000_000_000, "t1", "b" * 64),
    ]
    result = module.input_timing(
        fresh,
        now_ns=now,
        max_input_age_s=5.0,
        max_sync_skew_s=3.0,
    )
    assert result["fresh"] is True
    assert result["synchronized"] is True

    stale = module.input_timing(
        fresh,
        now_ns=110_000_000_000,
        max_input_age_s=5.0,
        max_sync_skew_s=1.0,
    )
    assert stale["fresh"] is False
    assert stale["synchronized"] is False


def test_round_input_exposes_hash_for_transaction_lock(tmp_path, observation_factory):
    module = load_scene_module()
    observation = observation_factory(sequence=88, now_ns=time.time_ns())
    spec, spool = write_ready_tree(module, tmp_path, observation)

    ready = module.inspect_ready_input(
        spec,
        spool=spool,
        calibration_id="shared-test-v1",
        previous_sequence=87,
    )

    assert ready is not None
    assert len(ready.map_sha256) == 64
    assert ready.map_sha256 == module.sha256_file(
        spec.snapshot_dir / "central_map.npz"
    )
