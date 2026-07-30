from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from focus_hub.tinynav_map_contract import (
    REQUIRED_MAP_FILES,
    build_saved_map_manifest,
    validate_saved_map_manifest,
)


ROOT = Path(__file__).parents[2]
TOOL = (
    ROOT / "hub" / "robot_overlay" / "derive_tinynav_reanchored_map.py"
)


def pose(x: float, y: float, yaw_deg: float = 0.0) -> np.ndarray:
    yaw = np.deg2rad(yaw_deg)
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:2, :2] = ((cosine, -sine), (sine, cosine))
    matrix[:2, 3] = (x, y)
    return matrix


def make_parent(tmp_path: Path) -> tuple[Path, Path]:
    parent = tmp_path / "parent"
    parent.mkdir()
    poses = {
        100: pose(1.0, 2.0, 15.0),
        200: pose(2.0, 2.0, 15.0),
        300: pose(8.0, 5.0, 80.0),
        400: pose(9.0, 3.0, 40.0),
        500: pose(10.0, 0.0, -20.0),
        600: pose(11.0, 0.0, -20.0),
    }
    np.save(parent / "poses.npy", poses, allow_pickle=True)
    for index, name in enumerate(REQUIRED_MAP_FILES):
        if name != "poses.npy":
            (parent / name).write_bytes(f"{index}:{name}".encode())
    manifest = build_saved_map_manifest(parent, created_at_ns=1)
    manifest_path = parent / "focus_saved_map_manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    return parent, manifest_path


def test_reanchored_map_keeps_prefix_and_corrected_tail(tmp_path) -> None:
    parent, parent_manifest = make_parent(tmp_path)
    output = tmp_path / "reanchored"
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL),
            "--parent-map",
            str(parent),
            "--parent-manifest",
            str(parent_manifest),
            "--output-map",
            str(output),
            "--retain-through-stamp-ns",
            "200",
            "--append-from-stamp-ns",
            "500",
            "--target-anchor-stamp-ns",
            "100",
            "--source-anchor-stamp-ns",
            "600",
            "--anchor-reason",
            "operator_reported_same_test_pose",
            "--minimum-prefix-keyframes",
            "2",
            "--minimum-appended-keyframes",
            "2",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    parent_poses = np.load(
        parent / "poses.npy", allow_pickle=True
    ).item()
    poses = np.load(output / "poses.npy", allow_pickle=True).item()
    assert tuple(poses) == (100, 200, 500, 600)
    assert np.allclose(poses[100], parent_poses[100])
    assert np.allclose(poses[200], parent_poses[200])
    assert np.allclose(poses[600], parent_poses[100])
    correction = parent_poses[100] @ np.linalg.inv(parent_poses[600])
    assert np.allclose(poses[500], correction @ parent_poses[500])

    contract = validate_saved_map_manifest(
        output / "focus_saved_map_manifest.json",
        map_directory=output,
    )
    derivation = contract["derivation"]
    assert derivation["selection"]["retained_prefix_keyframes"] == 2
    assert derivation["selection"]["appended_tail_keyframes"] == 2
    assert derivation["selection"]["discarded_transition_keyframes"] == 2
    assert (
        derivation["anchor"]["status"]
        == "source_derived_from_operator_reported_same_physical_pose"
    )
    assert derivation["robot_commands_issued"] is False
    assert os.stat(parent / "features.db").st_ino == os.stat(
        output / "features.db"
    ).st_ino


def test_reanchored_map_refuses_overlapping_selection(tmp_path) -> None:
    parent, parent_manifest = make_parent(tmp_path)
    output = tmp_path / "reanchored"
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL),
            "--parent-map",
            str(parent),
            "--parent-manifest",
            str(parent_manifest),
            "--output-map",
            str(output),
            "--retain-through-stamp-ns",
            "500",
            "--append-from-stamp-ns",
            "500",
            "--target-anchor-stamp-ns",
            "100",
            "--source-anchor-stamp-ns",
            "600",
            "--anchor-reason",
            "invalid_overlap",
            "--minimum-prefix-keyframes",
            "2",
            "--minimum-appended-keyframes",
            "2",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 2
    assert "must not overlap" in result.stderr
    assert not output.exists()
