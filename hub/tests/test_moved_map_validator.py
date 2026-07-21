from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest


def load_validator_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "validate_moved_map_run.py"
    name = "focus_test_moved_map_validator"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def pose_at(x_m: float) -> np.ndarray:
    pose = np.eye(4)
    pose[0, 3] = x_m
    return pose


def test_trajectory_metrics_use_real_capture_timestamps_and_keyframe_gate():
    validator = load_validator_module()
    poses = [pose_at(value) for value in (0.0, 0.2, 0.4, 0.6, 0.8)]
    sequences = [100, 101, 102, 103, 104]
    timestamps = [index * 1_000_000_000 for index in range(5)]

    metrics, accepted, jumps = validator.trajectory_metrics(
        poses, sequences, timestamps
    )

    assert metrics["xy_path_length_m"] == pytest.approx(0.8)
    assert metrics["max_adjacent_translation_m"] == pytest.approx(0.2)
    assert accepted[0] == 100
    assert accepted[-1] == 104
    assert jumps == []


def test_load_trajectory_rejects_wrong_transform_version(tmp_path):
    validator = load_validator_module()
    directory = tmp_path / "robot-0" / f"{7:020d}"
    directory.mkdir(parents=True)
    (directory / "metadata.json").write_text(
        json.dumps({
            "capture_time_ns": 123,
            "pose": {
                "transform_version": "wrong-v2",
                "shared_T_camera": {
                    "parent_frame": "shared_world",
                    "matrix": np.eye(4).reshape(-1).tolist(),
                },
            },
        }),
        encoding="utf-8",
    )

    poses, sequences, timestamps, provenance, errors = validator.load_trajectory(
        tmp_path,
        "robot-0",
        7,
        7,
        "expected-v1",
        "shared_world",
    )

    assert poses == []
    assert sequences == []
    assert timestamps == []
    assert provenance == []
    assert errors[0]["sequence"] == 7
