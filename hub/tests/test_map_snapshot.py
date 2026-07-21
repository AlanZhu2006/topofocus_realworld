from __future__ import annotations

import numpy as np
import pytest

from focus_hub.map_snapshot import load_map_snapshot, validate_fusion_contract


def write_snapshot(path, *, frame="shared_world", transform="robot-v1", calibration=None):
    values = {
        "grid": np.zeros((17, 4, 5), dtype=np.float32),
        "origin_xy_m": np.array([-1.0, 2.0]),
        "resolution_m": np.array(0.05),
    }
    if frame is not None:
        values["frame_id"] = np.asarray(frame)
    if transform is not None:
        values["transform_version"] = np.asarray(transform)
    if calibration is not None:
        values["shared_frame_calibration_id"] = np.asarray(calibration)
    np.savez_compressed(path, **values)


def test_legacy_snapshot_requires_explicit_per_robot_override(tmp_path):
    path = tmp_path / "legacy.npz"
    write_snapshot(path, frame=None, transform=None)

    with pytest.raises(ValueError, match="legacy map snapshot"):
        load_map_snapshot(path)

    loaded = load_map_snapshot(path, allow_legacy=True)
    assert loaded is not None
    assert loaded.legacy_contract
    assert loaded.frame_id == "shared_world"


def test_fusion_requires_same_explicit_calibration_id(tmp_path):
    first_path = tmp_path / "first.npz"
    second_path = tmp_path / "second.npz"
    write_snapshot(first_path, transform="wsj-v3")
    write_snapshot(second_path, transform="yunji-v1")
    first = load_map_snapshot(first_path)
    second = load_map_snapshot(second_path)
    assert first is not None and second is not None

    with pytest.raises(ValueError, match="shared_frame_calibration_id"):
        validate_fusion_contract([first, second])

    write_snapshot(first_path, transform="wsj-v3", calibration="survey-20260722-v1")
    write_snapshot(second_path, transform="yunji-v1", calibration="survey-20260722-v1")
    first = load_map_snapshot(first_path)
    second = load_map_snapshot(second_path)
    assert first is not None and second is not None
    frame, resolution, calibration = validate_fusion_contract([first, second])
    assert frame == "shared_world"
    assert resolution == pytest.approx(0.05)
    assert calibration == "survey-20260722-v1"


def test_fusion_rejects_calibration_mismatch(tmp_path):
    first_path = tmp_path / "first.npz"
    second_path = tmp_path / "second.npz"
    write_snapshot(first_path, calibration="calibration-a")
    write_snapshot(second_path, calibration="calibration-b")
    snapshots = [load_map_snapshot(first_path), load_map_snapshot(second_path)]

    with pytest.raises(ValueError, match="calibration mismatch"):
        validate_fusion_contract([item for item in snapshots if item is not None])
