#!/usr/bin/env python3
"""Fetch only the checksum-pinned real-world model artifacts.

This deliberately excludes HM3D, simulator scenes, overlays, SIF images,
recorded bags, and maps.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.request


WORKSPACE = Path(__file__).resolve().parents[2]
GLM_REVISION = "3376fea6e54db68587a89bf1ac27a6889bafb867"
SEGFORMER_REVISION = "489d5cd81a0b59fab9b7ea758d3548ebe99677da"

DIRECT_ARTIFACTS = (
    {
        "name": "YOLOv10m",
        "url": (
            "https://github.com/ultralytics/assets/releases/download/"
            "v8.3.0/yolov10m.pt"
        ),
        "path": "artifacts/vision/yolov10m.pt",
        "bytes": 33_643_667,
        "sha256": (
            "6dc78f7a88591cec1e8716b8f5c7e3aefa9206684f025d202be34439ccb329a0"
        ),
    },
    {
        "name": "OpenAI CLIP ViT-B/32",
        "url": (
            "https://openaipublic.azureedge.net/clip/models/"
            "40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/"
            "ViT-B-32.pt"
        ),
        "path": "artifacts/vision/ViT-B-32.pt",
        "bytes": 353_976_522,
        "sha256": (
            "40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af"
        ),
    },
)

REDNET = {
    "name": "RedNet HM3D semantic-map checkpoint",
    "google_drive_id": "1U0dS44DIPZ22nTjw0RfO431zV-lMPcvv",
    "source_page": (
        "https://drive.google.com/file/d/"
        "1U0dS44DIPZ22nTjw0RfO431zV-lMPcvv/view"
    ),
    "path": "artifacts/checkpoints/rednet_semmap_mp3d_40.pth",
    "bytes": 656_550_984,
    "sha256": (
        "f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995"
    ),
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(path: Path, record: dict[str, object]) -> None:
    if not path.is_file():
        raise RuntimeError(f"missing {record['name']}: {path}")
    observed_size = path.stat().st_size
    observed_digest = sha256_file(path)
    if observed_size != record["bytes"] or observed_digest != record["sha256"]:
        raise RuntimeError(
            f"artifact identity mismatch for {path}: "
            f"bytes={observed_size}/{record['bytes']} "
            f"sha256={observed_digest}/{record['sha256']}"
        )


def download_direct(workspace: Path, record: dict[str, object]) -> None:
    destination = workspace / str(record["path"])
    if destination.exists():
        verify(destination, record)
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=destination.parent,
        prefix=f".{destination.name}.",
        suffix=".partial",
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
    try:
        request = urllib.request.Request(
            str(record["url"]),
            headers={"User-Agent": "TopoFocus-cleanroom/1"},
        )
        with urllib.request.urlopen(request, timeout=60) as response:
            with temporary.open("wb") as output:
                while block := response.read(8 * 1024 * 1024):
                    output.write(block)
        verify(temporary, record)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def download_rednet(workspace: Path) -> None:
    destination = workspace / str(REDNET["path"])
    if destination.exists():
        verify(destination, REDNET)
        return
    try:
        import gdown
    except ImportError as exc:
        raise RuntimeError(
            "gdown is required only for the source-linked RedNet checkpoint"
        ) from exc
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=destination.parent,
        prefix=f".{destination.name}.",
        suffix=".partial",
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
    try:
        result = gdown.download(
            id=str(REDNET["google_drive_id"]),
            output=str(temporary),
            quiet=False,
        )
        if result is None:
            raise RuntimeError(
                f"RedNet download failed; source page: {REDNET['source_page']}"
            )
        verify(temporary, REDNET)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def download_huggingface(workspace: Path) -> None:
    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise RuntimeError("huggingface_hub is required for model snapshots") from exc

    cache_dir = workspace / "artifacts/models/hf_cache"
    snapshot_download(
        repo_id="THUDM/glm-4v-9b",
        revision=GLM_REVISION,
        cache_dir=cache_dir,
    )
    segformer_dir = workspace / "artifacts/vision/segformer_b0_ade20k_hf"
    snapshot_download(
        repo_id="nvidia/segformer-b0-finetuned-ade-512-512",
        revision=SEGFORMER_REVISION,
        local_dir=segformer_dir,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--accept-model-licenses",
        action="store_true",
        help="confirm that the operator reviewed each upstream model license",
    )
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()

    plan = {
        "schema_version": "focus-cleanroom-model-fetch-v1",
        "workspace": str(workspace),
        "direct_artifacts": list(DIRECT_ARTIFACTS),
        "rednet": REDNET,
        "huggingface": [
            {"repo": "THUDM/glm-4v-9b", "revision": GLM_REVISION},
            {
                "repo": "nvidia/segformer-b0-finetuned-ade-512-512",
                "revision": SEGFORMER_REVISION,
            },
        ],
        "deliberate_exclusions": [
            "HM3D and simulator scenes",
            "ObjectNav datasets",
            "overlays and SIF images",
            "robot bags, maps, and runtime state",
        ],
        "physical_commands_sent": False,
    }
    print(json.dumps(plan, indent=2, sort_keys=True))
    if not args.apply:
        print("PLAN_ONLY=true")
        return 0
    if not args.accept_model_licenses:
        parser.error("--apply requires --accept-model-licenses")

    for record in DIRECT_ARTIFACTS:
        download_direct(workspace, record)
    download_rednet(workspace)
    download_huggingface(workspace)
    print("CLEANROOM_MODEL_FETCH_COMPLETE=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
