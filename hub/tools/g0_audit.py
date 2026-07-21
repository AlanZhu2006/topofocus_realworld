#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit the selected Focus workspace for G0")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--full-hash", action="store_true", help="also hash every GLM cache blob")
    args = parser.parse_args()
    workspace = args.workspace.resolve()

    required = {
        "focus_source": workspace / "source/Focus_realworld/main.py",
        "rednet_source": workspace / "dependencies/RedNet/RedNet_model.py",
        "habitat_reference": workspace / "dependencies/habitat-lab/habitat/__init__.py",
        "rednet_checkpoint": workspace / "artifacts/checkpoints/rednet_semmap_mp3d_40.pth",
        "yolo_checkpoint": workspace / "artifacts/vision/yolov10m.pt",
        "clip_checkpoint": workspace / "artifacts/vision/ViT-B-32.pt",
        "glm_cache": workspace / "artifacts/models/hf_cache/hub/models--THUDM--glm-4v-9b",
    }
    missing = [name for name, path in required.items() if not path.exists()]

    parse_failures: list[dict[str, str]] = []
    python_count = 0
    for root_name in ("source", "dependencies", "hub"):
        for path in (workspace / root_name).rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            python_count += 1
            try:
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            except (OSError, UnicodeError, SyntaxError) as exc:
                parse_failures.append({"path": str(path.relative_to(workspace)), "error": str(exc)})

    standalone_hashes = {}
    for name in ("rednet_checkpoint", "yolo_checkpoint", "clip_checkpoint"):
        path = required[name]
        if path.is_file():
            standalone_hashes[name] = {"bytes": path.stat().st_size, "sha256": sha256(path)}

    glm = required["glm_cache"]
    snapshot = glm / "snapshots/3376fea6e54db68587a89bf1ac27a6889bafb867"
    shards = sorted(snapshot.glob("model-*-of-00015.safetensors")) if snapshot.is_dir() else []
    links = list(snapshot.iterdir()) if snapshot.is_dir() else []
    broken_links = [str(path.relative_to(workspace)) for path in links if path.is_symlink() and not path.exists()]
    blob_hash_failures: list[dict[str, str]] = []
    blob_hashes: dict[str, str] = {}
    if args.full_hash and glm.is_dir():
        for path in sorted((glm / "blobs").iterdir()):
            if not path.is_file():
                continue
            actual = sha256(path)
            blob_hashes[path.name] = actual
            if len(path.name) == 64 and path.name != actual:
                blob_hash_failures.append({"blob": path.name, "actual": actual})

    forbidden_present = []
    for pattern in ("overlay-*.ext3", "*.sif", "objectnav_hm3d_v2.zip"):
        forbidden_present.extend(str(path.relative_to(workspace)) for path in workspace.rglob(pattern))

    result = {
        "workspace": str(workspace),
        "required": {name: str(path.relative_to(workspace)) for name, path in required.items()},
        "missing": missing,
        "python_files_parsed": python_count,
        "parse_failures": parse_failures,
        "standalone_artifacts": standalone_hashes,
        "glm_snapshot": {
            "revision": "3376fea6e54db68587a89bf1ac27a6889bafb867",
            "model_shards": len(shards),
            "broken_links": broken_links,
            "blob_hash_failures": blob_hash_failures,
            "blob_hashes": blob_hashes,
        },
        "deliberate_exclusions_found": sorted(forbidden_present),
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    passed = (
        not missing
        and not parse_failures
        and len(shards) == 15
        and not broken_links
        and not blob_hash_failures
        and not forbidden_present
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

