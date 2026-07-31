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
TOOL = ROOT / "hub" / "robot_overlay" / "derive_tinynav_reference_map.py"


def make_parent(tmp_path: Path) -> tuple[Path, Path]:
    parent = tmp_path / "parent"
    parent.mkdir()
    poses = {
        stamp: np.eye(4, dtype=np.float64)
        for stamp in (100, 200, 300, 400)
    }
    np.save(parent / "poses.npy", poses, allow_pickle=True)
    for index, name in enumerate(REQUIRED_MAP_FILES):
        if name != "poses.npy":
            (parent / name).write_bytes(f"{index}:{name}".encode())
    manifest = build_saved_map_manifest(parent, created_at_ns=1)
    manifest_path = parent / "focus_saved_map_manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    return parent, manifest_path


def test_reference_map_keeps_reviewed_prefix_and_provenance(tmp_path) -> None:
    parent, parent_manifest = make_parent(tmp_path)
    output = tmp_path / "reference"
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
            "--cutoff-stamp-ns",
            "250",
            "--cutoff-reason",
            "observed_test_boundary",
            "--minimum-keyframes",
            "2",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    poses = np.load(output / "poses.npy", allow_pickle=True).item()
    assert tuple(poses) == (100, 200)
    contract = validate_saved_map_manifest(
        output / "focus_saved_map_manifest.json",
        map_directory=output,
    )
    assert contract["derivation"]["parent_keyframes"] == 4
    assert contract["derivation"]["retained_keyframes"] == 2
    assert contract["derivation"]["robot_commands_issued"] is False
    assert os.stat(parent / "depths.db").st_ino == os.stat(
        output / "depths.db"
    ).st_ino


def test_reference_map_refuses_cutoff_that_excludes_nothing(tmp_path) -> None:
    parent, parent_manifest = make_parent(tmp_path)
    output = tmp_path / "reference"
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
            "--cutoff-stamp-ns",
            "400",
            "--cutoff-reason",
            "bad_boundary",
            "--minimum-keyframes",
            "2",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 2
    assert "exclude at least one" in result.stderr
    assert not output.exists()
