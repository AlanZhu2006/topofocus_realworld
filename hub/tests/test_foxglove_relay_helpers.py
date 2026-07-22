from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


def load_relay_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "foxglove_relay.py"
    name = "focus_test_foxglove_relay"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def make_source(module, tmp_path):
    return module.RobotSource(
        robot_id="robot-0",
        name="wsj",
        snapshot_dir=tmp_path,
        camera_channel=None,
        map_channel=None,
        geometry_channel=None,
        pose_channel=None,
        status_channel=None,
    )


def test_pose_scene_trail_deduplicates_small_motion_and_resets_on_frame_change(tmp_path):
    relay = load_relay_module()
    source = make_source(relay, tmp_path)

    assert relay.update_pose_scene(
        source, {"last_camera_xy_m": [1.0, 2.0], "frame_id": "frame-a"}
    ) is not None
    relay.update_pose_scene(
        source, {"last_camera_xy_m": [1.01, 2.0], "frame_id": "frame-a"}
    )
    assert source.trajectory_xy_m == [(1.0, 2.0)]

    relay.update_pose_scene(
        source, {"last_camera_xy_m": [1.20, 2.0], "frame_id": "frame-a"}
    )
    assert source.trajectory_xy_m == [(1.0, 2.0), (1.2, 2.0)]

    relay.update_pose_scene(
        source, {"last_camera_xy_m": [4.0, 5.0], "frame_id": "frame-b"}
    )
    assert source.trajectory_xy_m == [(4.0, 5.0)]


def test_pose_scene_rejects_missing_or_nonfinite_xy(tmp_path):
    relay = load_relay_module()
    source = make_source(relay, tmp_path)

    assert relay.update_pose_scene(source, {}) is None
    assert relay.update_pose_scene(
        source, {"last_camera_xy_m": [float("nan"), 0.0], "frame_id": "world"}
    ) is None


def test_fusion_loop_publishes_geometry_semantics_and_evidence_status(
    monkeypatch, tmp_path
):
    relay = load_relay_module()
    grid = np.zeros((17, 2, 2), dtype=np.float32)
    grid[0, 0, 0] = 1.0
    grid[1, :, :] = 1.0
    grid[2, 1, 1] = 1.0
    snapshot = relay.MapSnapshot(
        grid=grid,
        origin_xy_m=(0.0, 0.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version="test-transform",
        shared_frame_calibration_id="test-calibration",
        map_format_version="focus-hub-central-map-v2",
    )

    monkeypatch.setattr(relay, "load_grid_npz", lambda _path: snapshot)
    monkeypatch.setattr(relay.time, "monotonic", lambda: 8.0)
    monkeypatch.setattr(
        relay,
        "grid_to_message",
        lambda *_args, **kwargs: kwargs["view"],
    )

    class StopLoop(Exception):
        pass

    monkeypatch.setattr(
        relay.time,
        "sleep",
        lambda _seconds: (_ for _ in ()).throw(StopLoop()),
    )

    class Channel:
        def __init__(self):
            self.messages = []

        def log(self, message):
            self.messages.append(message)

    semantic = Channel()
    geometry = Channel()
    status = Channel()
    sources = [make_source(relay, tmp_path), make_source(relay, tmp_path)]

    with pytest.raises(StopLoop):
        relay.fusion_poll_loop(
            sources,
            semantic,
            geometry,
            status,
            relay.HM3D_CATEGORY_NAMES,
            interval_s=8.0,
            downsample=1,
        )

    assert semantic.messages == ["semantic"]
    assert geometry.messages == ["geometry"]
    assert len(status.messages) == 1
    status_text = repr(status.messages[0])
    assert "calibration=test-calibration" in status_text
    assert "explored=4" in status_text
    assert "obstacles=1" in status_text
    assert "semantic_evidence=1" in status_text
