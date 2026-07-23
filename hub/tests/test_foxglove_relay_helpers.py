from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

from focus_hub.shadow_coordination import build_shadow_target_payload


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


def test_pose_scene_prefers_calibrated_robot_pose_over_camera_fallback(tmp_path):
    relay = load_relay_module()
    source = make_source(relay, tmp_path)

    scene = relay.update_pose_scene(
        source,
        {
            "frame_id": "shared_world",
            "last_robot_xy_m": [1.0, 2.0],
            "last_robot_heading_deg": 90.0,
            "robot_trajectory_xy_m": [[0.5, 2.0], [1.0, 2.0]],
            "last_camera_xy_m": [9.0, 9.0],
            "trajectory_xy_m": [[9.0, 9.0]],
        },
    )

    assert source.last_pose_xy_m == (1.0, 2.0)
    assert source.last_heading_deg == 90.0
    assert source.trajectory_xy_m == [(0.5, 2.0), (1.0, 2.0)]
    encoded = repr(scene)
    assert 'id: "wsj-robot-trail"' in encoded
    assert 'text: "wsj base"' in encoded


def test_snapshot_freshness_uses_integrated_capture_not_file_mtime(tmp_path):
    relay = load_relay_module()
    map_path = tmp_path / "central_map.npz"
    map_path.write_bytes(b"snapshot")

    freshness = relay.snapshot_freshness(
        {
            "last_capture_time_ns": 9_000_000_000,
            "last_map_capture_time_ns": 8_000_000_000,
            "last_observation_sequence": 12,
            "last_map_sequence": 11,
        },
        map_path,
        now_ns=10_000_000_000,
    )

    assert freshness["input_capture_age_s"] == pytest.approx(1.0)
    assert freshness["map_content_age_s"] == pytest.approx(2.0)
    assert freshness["map_content_age_source"] == (
        "integrated_keyframe_capture_time"
    )
    assert freshness["last_observation_sequence"] == 12
    assert freshness["last_map_sequence"] == 11


def test_snapshot_freshness_labels_legacy_capture_fallback(tmp_path):
    relay = load_relay_module()

    freshness = relay.snapshot_freshness(
        {"last_capture_time_ns": 8_000_000_000},
        tmp_path / "missing-map.npz",
        now_ns=10_000_000_000,
    )

    assert freshness["map_content_age_s"] == pytest.approx(2.0)
    assert freshness["map_content_age_source"] == (
        "legacy_last_input_capture_time_fallback"
    )
    assert freshness["snapshot_file_age_s"] is None


def test_semantic_scene_paints_every_chair_cell_without_bounding_box(tmp_path):
    relay = load_relay_module()
    grid = np.zeros((17, 20, 20), dtype=np.float32)
    grid[2, 4:6, 7:9] = 1.0
    grid[2, 15, 15] = 1.0
    snapshot = relay.MapSnapshot(
        grid=grid,
        origin_xy_m=(-1.0, -2.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version="test-transform",
        shared_frame_calibration_id="test-calibration",
        map_format_version="focus-hub-central-map-v3",
    )

    scene = relay.semantic_scene_from_snapshot(
        "wsj", snapshot, relay.HM3D_CATEGORY_NAMES
    )

    encoded = repr(scene)
    assert 'id: "wsj-semantic-objects"' in encoded
    assert encoded.count("CubePrimitive {") == 5
    assert encoded.count("TextPrimitive {") == 1
    assert "size: Some(Vector3 { x: 0.05, y: 0.05, z: 0.07" in encoded
    assert "color: Some(Color { r: 0.901960" in encoded
    assert 'text: "chair"' in encoded
    assert "position: Some(Vector3 { x: -0.6, y: -1.75, z: 0.16" in encoded


def test_shadow_target_scene_is_expiring_and_explicitly_non_authoritative(tmp_path):
    relay = load_relay_module()
    grid = np.zeros((17, 20, 20), dtype=np.float32)
    snapshot = relay.MapSnapshot(
        grid=grid,
        origin_xy_m=(-1.0, -2.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version="test-transform",
        shared_frame_calibration_id="test-calibration",
        map_format_version="focus-hub-central-map-v3",
    )
    payload = build_shadow_target_payload(
        robot_id="robot-0",
        frontier_id="B",
        goal_category="chair",
        target_xy_m=(1.25, 2.5),
        yaw_rad=0.0,
        snapshot=snapshot,
        created_at_ns=1_000_000_000,
        expires_at_ns=6_000_000_000,
        run_manifest="/tmp/shadow_manifest.json",
        map_snapshot_sha256="a" * 64,
    )

    scene = relay.shadow_target_scene_from_payload(
        "wsj", "robot-0", snapshot, payload, now_ns=2_000_000_000
    )

    encoded = repr(scene)
    assert 'id: "wsj-vlm-shadow-target"' in encoded
    assert encoded.count("CylinderPrimitive {") == 1
    assert "lifetime: Some(Duration { sec: 4, nsec: 0 })" in encoded
    assert 'text: "SHADOW B · chair · NO MOTION"' in encoded


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
    overview = Channel()
    fusion_state = relay.FusionSourceState(enabled=True)
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
            fused_overview_channel=overview,
            fusion_state=fusion_state,
        )

    assert semantic.messages == ["semantic"]
    assert geometry.messages == ["geometry"]
    assert len(overview.messages) == 1
    assert fusion_state.semantic_overview_ready is True
    assert fusion_state.calibration_id == "test-calibration"
    assert fusion_state.last_error is None
    assert fusion_state.last_published_at_ns is not None
    assert len(status.messages) == 1
    status_text = repr(status.messages[0])
    assert "calibration=test-calibration" in status_text
    assert "explored=4" in status_text
    assert "obstacles=1" in status_text
    assert "semantic_evidence=1" in status_text
