from __future__ import annotations

import numpy as np

from focus_hub.central_mapping import MapperConfig
from focus_hub.pipeline import SpoolMappingPipeline


def _make_pipeline() -> SpoolMappingPipeline:
    # save() never touches self.segmenter, so a real RedNetSegmenter (GPU
    # checkpoint) isn't needed to exercise it.
    K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
    return SpoolMappingPipeline(None, K, MapperConfig(), (0.0, 0.0), 0.0)


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
    leftover = [p.name for p in tmp_path.iterdir() if p.name not in {"central_map.npz", "map_summary.json"}]
    assert leftover == [], f"stray temp files left behind: {leftover}"

    with np.load(tmp_path / "central_map.npz") as data:
        assert data["grid"].shape == pipeline.mapper.map.grid.shape
        assert float(data["resolution_m"]) == pipeline.mapper.config.resolution_m


def test_save_can_be_called_repeatedly(tmp_path):
    """The daemon calls save() on a recurring timer -- a second call must
    not collide with or be blocked by the first call's temp file."""
    pipeline = _make_pipeline()

    pipeline.save(tmp_path)
    pipeline.save(tmp_path)
    pipeline.save(tmp_path)

    assert (tmp_path / "central_map.npz").is_file()
    leftover = [p.name for p in tmp_path.iterdir() if p.name not in {"central_map.npz", "map_summary.json"}]
    assert leftover == []
