from __future__ import annotations

import json

import numpy as np

from focus_hub.central_mapping import MapperConfig
from focus_hub.pipeline import SpoolMappingPipeline


def _make_pipeline() -> SpoolMappingPipeline:
    # save() never touches self.segmenter, so a real RedNetSegmenter (GPU
    # checkpoint) isn't needed to exercise it.
    K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
    return SpoolMappingPipeline(
        None,
        K,
        MapperConfig(),
        (0.0, 0.0),
        0.0,
        expected_transform_version="test-transform-v1",
        robot_id="robot-0",
        shared_frame_calibration_id="test-calibration-v1",
    )


def test_save_writes_a_loadable_npz_with_no_stray_tmp_files(tmp_path):
    """Regression test: np.savez_compressed silently appends .npz to any
    path that doesn't already end with it, so a temp filename like
    'central_map.npz.tmp' actually gets written as
    'central_map.npz.tmp.npz' -- the subsequent os.replace() then raises
    FileNotFoundError looking for a file that was never created. This
    crashed a live hub_pipeline_daemon.py process for real (2026-07-20)."""
    pipeline = _make_pipeline()

    pipeline.save(tmp_path)

    assert (tmp_path / "central_map.npz").is_file()
    assert (tmp_path / "map_summary.json").is_file()
    leftover = [
        p.name
        for p in tmp_path.iterdir()
        if p.name not in {"central_map.npz", "map_summary.json"}
    ]
    assert leftover == [], f"stray temp files left behind: {leftover}"

    with np.load(tmp_path / "central_map.npz") as data:
        assert data["grid"].shape == pipeline.mapper.map.grid.shape
        assert float(data["resolution_m"]) == pipeline.mapper.config.resolution_m
        assert str(data["frame_id"].item()) == "shared_world"
        assert str(data["transform_version"].item()) == "test-transform-v1"
        assert str(data["shared_frame_calibration_id"].item()) == "test-calibration-v1"
        assert str(data["map_format_version"].item()) == "focus-hub-central-map-v3"
        assert str(data["floor_source"].item()) == "caller_provided_unverified"
        np.testing.assert_allclose(data["floor_plane_coefficients"], [0.0, 0.0, 0.0])
        assert str(data["obstacle_fusion_mode"].item()) == "max"
        np.testing.assert_allclose(data["obstacle_band_m"], [0.25, 1.5])
        assert str(data["semantic_fusion_mode"].item()) == "max"
        assert int(data["semantic_min_hits"]) == 1
        assert int(data["semantic_winner_margin_hits"]) == 0
        assert str(data["semantic_fusion"].item()) == "rednet_mp3d40"
        assert str(data["semantic_yolo_model_sha256"].item()) == ""

    summary = json.loads((tmp_path / "map_summary.json").read_text())
    assert summary["obstacle_band_m"] == [0.25, 1.5]
    assert summary["ground_drift_frames"] == 0
    assert summary["ground_drift_streak"] == 0
    assert summary["ground_guard"]["consecutive_frames_to_latch"] == 3
    assert summary["semantic_mapping"]["pixel_segmenter"]["backend"] == (
        "rednet_mp3d40"
    )
    assert summary["semantic_mapping"]["yolo_reinforcement"]["enabled"] is False
    assert summary["semantic_cells"] == 0


def test_save_can_be_called_repeatedly(tmp_path):
    """The daemon calls save() on a recurring timer -- a second call must
    not collide with or be blocked by the first call's temp file."""
    pipeline = _make_pipeline()

    pipeline.save(tmp_path)
    pipeline.save(tmp_path)
    pipeline.save(tmp_path)

    assert (tmp_path / "central_map.npz").is_file()
    leftover = [
        p.name
        for p in tmp_path.iterdir()
        if p.name not in {"central_map.npz", "map_summary.json"}
    ]
    assert leftover == []
