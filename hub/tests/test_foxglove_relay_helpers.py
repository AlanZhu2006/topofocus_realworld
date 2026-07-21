from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


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
