from __future__ import annotations

import json

import pytest

from focus_hub.tinynav_map_contract import (
    REQUIRED_MAP_FILES,
    build_saved_map_manifest,
    validate_saved_map_manifest,
)


def make_map(tmp_path):
    directory = tmp_path / "map"
    directory.mkdir()
    for index, name in enumerate(REQUIRED_MAP_FILES):
        (directory / name).write_bytes(f"{index}:{name}".encode())
    return directory


def test_saved_map_manifest_round_trip(tmp_path) -> None:
    directory = make_map(tmp_path)
    source = tmp_path / "map_node.py"
    source.write_text("source")
    payload = build_saved_map_manifest(
        directory,
        source_files=[source],
        created_at_ns=123,
    )
    manifest = directory / "focus_saved_map_manifest.json"
    manifest.write_text(json.dumps(payload))

    validated = validate_saved_map_manifest(
        manifest,
        map_directory=directory,
    )

    assert validated["created_at_ns"] == 123
    assert validated["result_status"] == (
        "source_derived_from_observed_finalized_map_files"
    )
    assert validated["robot_commands_issued"] is False
    assert validated["runtime_sources"]["map_node.py"]["size_bytes"] == 6
    assert len(validated["map_snapshot_sha256"]) == 64


def test_saved_map_manifest_detects_content_change(tmp_path) -> None:
    directory = make_map(tmp_path)
    payload = build_saved_map_manifest(directory)
    manifest = directory / "focus_saved_map_manifest.json"
    manifest.write_text(json.dumps(payload))
    (directory / "poses.npy").write_bytes(b"tampered but longer")

    with pytest.raises(ValueError, match="size mismatch"):
        validate_saved_map_manifest(manifest, map_directory=directory)


def test_saved_map_manifest_rejects_missing_required_file(tmp_path) -> None:
    directory = make_map(tmp_path)
    (directory / "sdf_map.npy").unlink()

    with pytest.raises(ValueError, match="regular non-symlink"):
        build_saved_map_manifest(directory)
