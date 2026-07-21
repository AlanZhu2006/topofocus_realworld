from __future__ import annotations

import hashlib
import json

import numpy as np
import pytest

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES
from focus_hub.tinynav_occupancy import (
    load_tinynav_occupancy,
    project_tinynav_occupancy,
    write_hub_snapshot,
)


def _write_native(record, grid, meta=(10.0, 20.0, -1.0, 0.1)):
    record.mkdir()
    np.save(record / "occupancy_grid.npy", grid)
    np.save(record / "occupancy_meta.npy", np.asarray(meta, dtype=np.float32))


def test_projection_preserves_native_states_and_transposes_xy(tmp_path):
    native_grid = np.zeros((2, 3, 2), dtype=np.uint8)
    native_grid[0, 1, 0] = 1
    native_grid[1, 2, 1] = 2
    record = tmp_path / "record"
    _write_native(record, native_grid)

    native = load_tinynav_occupancy(record)
    hub_grid = project_tinynav_occupancy(native)

    assert hub_grid.shape == (2 + len(HM3D_CATEGORY_NAMES), 3, 2)
    assert hub_grid[1, 1, 0] == 1.0  # native [x=0,y=1] -> Hub [row=1,col=0]
    assert hub_grid[0, 1, 0] == 0.0
    assert hub_grid[1, 2, 1] == 1.0  # occupied is also explored
    assert hub_grid[0, 2, 1] == 1.0
    assert np.count_nonzero(hub_grid[2:]) == 0
    assert native.origin_xyz_m == pytest.approx((10.0, 20.0, -1.0))
    assert native.resolution_m == pytest.approx(0.1)


def test_snapshot_contains_frame_contract_and_source_hashes(tmp_path):
    native_grid = np.zeros((2, 2, 2), dtype=np.uint8)
    native_grid[0, 0, 0] = 1
    native_grid[1, 0, 1] = 2
    record = tmp_path / "record"
    _write_native(record, native_grid)
    out = tmp_path / "out"

    summary = write_hub_snapshot(
        load_tinynav_occupancy(record),
        out,
        robot_id="robot-0",
        frame_id="wsj_tinynav_world",
        transform_version="wsj-buildmap-test-v1",
    )

    with np.load(out / "central_map.npz", allow_pickle=False) as data:
        assert data["grid"].shape == (2 + len(HM3D_CATEGORY_NAMES), 2, 2)
        assert data["origin_xy_m"].tolist() == pytest.approx([10.0, 20.0])
        assert data["frame_id"].item() == "wsj_tinynav_world"
        assert data["transform_version"].item() == "wsj-buildmap-test-v1"
    persisted = json.loads((out / "map_summary.json").read_text())
    assert persisted == summary
    assert summary["free_cells"] == 1
    assert summary["occupied_cells"] == 1
    for filename in ("occupancy_grid.npy", "occupancy_meta.npy"):
        source = record / filename
        assert summary["source_files"][filename]["size_bytes"] == source.stat().st_size
        assert summary["source_files"][filename]["sha256"] == hashlib.sha256(
            source.read_bytes()
        ).hexdigest()

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        write_hub_snapshot(
            load_tinynav_occupancy(record), out,
            robot_id="robot-0", frame_id="x", transform_version="v2",
        )


@pytest.mark.parametrize(
    ("grid", "meta", "message"),
    [
        (np.zeros((2, 2), dtype=np.uint8), (0, 0, 0, 0.1), "non-empty 3-D"),
        (np.full((2, 2, 2), 3, dtype=np.uint8), (0, 0, 0, 0.1), "outside"),
        (np.zeros((2, 2, 2), dtype=np.float32), (0, 0, 0, 0.1), "integer dtype"),
        (np.zeros((2, 2, 2), dtype=np.uint8), (0, 0, 0), "origin_x,y,z,resolution"),
        (np.zeros((2, 2, 2), dtype=np.uint8), (0, 0, 0, 0), "positive resolution"),
    ],
)
def test_invalid_native_artifacts_fail_closed(tmp_path, grid, meta, message):
    record = tmp_path / "record"
    _write_native(record, grid, meta)
    with pytest.raises(ValueError, match=message):
        load_tinynav_occupancy(record)

