#!/usr/bin/env python3
"""Fetch or verify only checksum-pinned real-world model artifacts.

The tool deliberately excludes HM3D/simulator scenes, ObjectNav datasets,
overlays, SIF images, robot bags, maps, and runtime state. Plan mode is the
default and does not access the network.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import urllib.request
from typing import Any

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
        "sha256": ("6dc78f7a88591cec1e8716b8f5c7e3aefa9206684f025d202be34439ccb329a0"),
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
        "sha256": ("40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af"),
    },
)

REDNET = {
    "name": "RedNet HM3D semantic-map checkpoint",
    "google_drive_id": "1U0dS44DIPZ22nTjw0RfO431zV-lMPcvv",
    "source_page": (
        "https://drive.google.com/file/d/" "1U0dS44DIPZ22nTjw0RfO431zV-lMPcvv/view"
    ),
    "path": "artifacts/checkpoints/rednet_semmap_mp3d_40.pth",
    "bytes": 656_550_984,
    "sha256": ("f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995"),
}

SEGFORMER = {
    "name": "SegFormer-B0 ADE20K",
    "repo": "nvidia/segformer-b0-finetuned-ade-512-512",
    "revision": SEGFORMER_REVISION,
    "path": "artifacts/vision/segformer_b0_ade20k_hf",
    "files": (
        {
            "name": "SegFormer config",
            "path": "config.json",
            "bytes": 6_884,
            "sha256": (
                "209caa9091e4632f7c8883c11170cd08ad29af68b23c09590aa4a5befb1a2a7f"
            ),
        },
        {
            "name": "SegFormer preprocessor config",
            "path": "preprocessor_config.json",
            "bytes": 271,
            "sha256": (
                "8039d1d210abaa7117ad78e58cdfd6141a2ec72c03dae891b3cd76737e422c6c"
            ),
        },
        {
            "name": "SegFormer weights",
            "path": "model.safetensors",
            "bytes": 15_036_944,
            "sha256": (
                "6ae39addd01de6b1b8bde2cf677d43a5cd733424b8d186de3f95d1c51fee23f9"
            ),
        },
    ),
}

GLM = {
    "name": "GLM-4V-9B",
    "repo": "THUDM/glm-4v-9b",
    "revision": GLM_REVISION,
    "path": "artifacts/models/hf_cache/hub/models--THUDM--glm-4v-9b",
    "blob_count": 23,
    "blob_bytes": 27_815_609_781,
    "snapshot_entries": 23,
    "model_shards": 15,
}

GLM_BLOBS = {
    "03056c3ed47256a63021621daa4fe1d4b90bedda4b44c8e988193aced9f19c98": (
        1_815_217_672,
        "03056c3ed47256a63021621daa4fe1d4b90bedda4b44c8e988193aced9f19c98",
    ),
    "1bff55c70098795914510320a7c994d8d8c108b9e2b95c429cddee5fcef3e5d4": (
        1_968_291_952,
        "1bff55c70098795914510320a7c994d8d8c108b9e2b95c429cddee5fcef3e5d4",
    ),
    "1eb45e9f635e09a3ea4b3ee9db6ae612fce6537dd82d20acc927eb8244f21858": (
        810_746_296,
        "1eb45e9f635e09a3ea4b3ee9db6ae612fce6537dd82d20acc927eb8244f21858",
    ),
    "2f4670bc2cbce7ca12b5eeba459afe0e5e100ec3": (
        2_568,
        "e083211065590aae444fe1cf7a487f022b4f642195fe54d4a8538c04c57e0a62",
    ),
    "36bf2fd8ba6b480964bf724a9027a7ffbf2930fd3bb2244d6df2469a12b7f02c": (
        1_982_747_048,
        "36bf2fd8ba6b480964bf724a9027a7ffbf2930fd3bb2244d6df2469a12b7f02c",
    ),
    "3eaa7d6b4e4cea74cf7c4837cbc0671f4fd5f401": (
        110_646,
        "884016f2fd101012ca6230ace46c9af302c8ef02ac0917fc3637fae10207a359",
    ),
    "4cca1b773dacf4cb8e6ecfe9097d5156c1549f4b": (
        17_672,
        "5232a12f947127d2812059b332ccf57ff26ba127ddc5517b75232d57052a00a1",
    ),
    "5413314e163920b6a3563a5a8b5016204171cf78f00c60426eec503fa12273ee": (
        1_815_217_640,
        "5413314e163920b6a3563a5a8b5016204171cf78f00c60426eec503fa12273ee",
    ),
    "5a493598071550244b2ee7f26118f3edec2150b9dfa967929a99052ac83fe716": (
        2_623_634,
        "5a493598071550244b2ee7f26118f3edec2150b9dfa967929a99052ac83fe716",
    ),
    "5f069db54d8e01d0c8126abccd5eba80ad1fd9f1ecbc9c83d422609d0a971a23": (
        1_968_291_912,
        "5f069db54d8e01d0c8126abccd5eba80ad1fd9f1ecbc9c83d422609d0a971a23",
    ),
    "7263d40c35f58a39c3d01bd35c7b43e7fc38a29e5cf18d4cd54baafcd5995312": (
        1_927_406_992,
        "7263d40c35f58a39c3d01bd35c7b43e7fc38a29e5cf18d4cd54baafcd5995312",
    ),
    "7c865a6bbb37362a13cefa732193ae6c5c1151106b61bfe6d7fd730778aca808": (
        1_968_291_952,
        "7c865a6bbb37362a13cefa732193ae6c5c1151106b61bfe6d7fd730778aca808",
    ),
    "a55c2cb3be400a256b441946fa9d8b6402fe6f7708467325569ed23f2e530ef8": (
        1_927_406_992,
        "a55c2cb3be400a256b441946fa9d8b6402fe6f7708467325569ed23f2e530ef8",
    ),
    "c08d3cc18a5fb1c9dd2e0d90e334bad0a38db4fdfc2604e5165f8fac58e61499": (
        1_971_932_440,
        "c08d3cc18a5fb1c9dd2e0d90e334bad0a38db4fdfc2604e5165f8fac58e61499",
    ),
    "cba45f7b0bda438f74b95a9beb5bfc9d0bd80015c2b67d85f585be85a0e91b7c": (
        1_815_217_672,
        "cba45f7b0bda438f74b95a9beb5bfc9d0bd80015c2b67d85f585be85a0e91b7c",
    ),
    "cc69c30f0d14ebef1afde2077ea4b11ec23764be": (
        3_221,
        "be092c7785c8208ab6ad9258c0f90618a23c6f17de164f2b6bbd324e50be2d10",
    ),
    "cf41fa02b2427522b31b719cea68a160309c840f": (
        1_771,
        "7c80de82168f1fac04cd97c7c200fb618ce16a90ccd87cc8c662f429a3b2550e",
    ),
    "d14427429d21c245eabd966d145069247cf7c721905b18181faae4e7ec90966c": (
        1_982_747_048,
        "d14427429d21c245eabd966d145069247cf7c721905b18181faae4e7ec90966c",
    ),
    "df12f8ce20711ef2f3aa93f9c5c75b47bdb31ce5a4baff967267a63fd63e9483": (
        1_957_054_176,
        "df12f8ce20711ef2f3aa93f9c5c75b47bdb31ce5a4baff967267a63fd63e9483",
    ),
    "e583e560b38e6b69e3b6b6b0d6ac1d10bc4a63534fa48021eb7089feb0b9f7f8": (
        1_957_054_280,
        "e583e560b38e6b69e3b6b6b0d6ac1d10bc4a63534fa48021eb7089feb0b9f7f8",
    ),
    "ec92b0780c5119743aaf061a010afa37a3b4590b": (
        57_435,
        "a7e3ea33ea4a4b675e2a9b927f827b973937f8acf20261550a90aef3776704dd",
    ),
    "ee34c991cf3b847af4a8ef067716038fa10b89da": (
        7_002,
        "76ceab149963e47fb12c7a94b68dca526e182b4246494b95492e9c3b84c36a57",
    ),
    "f5f7c4461ec292196389492c8ba97de591d9eaed6f214608f3ee875e0a5e3442": (
        1_945_161_760,
        "f5f7c4461ec292196389492c8ba97de591d9eaed6f214608f3ee875e0a5e3442",
    ),
}

GLM_SNAPSHOT_BLOBS = {
    "config.json": "cf41fa02b2427522b31b719cea68a160309c840f",
    "configuration_chatglm.py": "2f4670bc2cbce7ca12b5eeba459afe0e5e100ec3",
    "model-00001-of-00015.safetensors": (
        "f5f7c4461ec292196389492c8ba97de591d9eaed6f214608f3ee875e0a5e3442"
    ),
    "model-00002-of-00015.safetensors": (
        "5413314e163920b6a3563a5a8b5016204171cf78f00c60426eec503fa12273ee"
    ),
    "model-00003-of-00015.safetensors": (
        "5f069db54d8e01d0c8126abccd5eba80ad1fd9f1ecbc9c83d422609d0a971a23"
    ),
    "model-00004-of-00015.safetensors": (
        "7263d40c35f58a39c3d01bd35c7b43e7fc38a29e5cf18d4cd54baafcd5995312"
    ),
    "model-00005-of-00015.safetensors": (
        "cba45f7b0bda438f74b95a9beb5bfc9d0bd80015c2b67d85f585be85a0e91b7c"
    ),
    "model-00006-of-00015.safetensors": (
        "7c865a6bbb37362a13cefa732193ae6c5c1151106b61bfe6d7fd730778aca808"
    ),
    "model-00007-of-00015.safetensors": (
        "a55c2cb3be400a256b441946fa9d8b6402fe6f7708467325569ed23f2e530ef8"
    ),
    "model-00008-of-00015.safetensors": (
        "03056c3ed47256a63021621daa4fe1d4b90bedda4b44c8e988193aced9f19c98"
    ),
    "model-00009-of-00015.safetensors": (
        "1bff55c70098795914510320a7c994d8d8c108b9e2b95c429cddee5fcef3e5d4"
    ),
    "model-00010-of-00015.safetensors": (
        "c08d3cc18a5fb1c9dd2e0d90e334bad0a38db4fdfc2604e5165f8fac58e61499"
    ),
    "model-00011-of-00015.safetensors": (
        "df12f8ce20711ef2f3aa93f9c5c75b47bdb31ce5a4baff967267a63fd63e9483"
    ),
    "model-00012-of-00015.safetensors": (
        "d14427429d21c245eabd966d145069247cf7c721905b18181faae4e7ec90966c"
    ),
    "model-00013-of-00015.safetensors": (
        "e583e560b38e6b69e3b6b6b0d6ac1d10bc4a63534fa48021eb7089feb0b9f7f8"
    ),
    "model-00014-of-00015.safetensors": (
        "36bf2fd8ba6b480964bf724a9027a7ffbf2930fd3bb2244d6df2469a12b7f02c"
    ),
    "model-00015-of-00015.safetensors": (
        "1eb45e9f635e09a3ea4b3ee9db6ae612fce6537dd82d20acc927eb8244f21858"
    ),
    "model.safetensors.index.json": ("3eaa7d6b4e4cea74cf7c4837cbc0671f4fd5f401"),
    "modeling_chatglm.py": "ec92b0780c5119743aaf061a010afa37a3b4590b",
    "tokenization_chatglm.py": "4cca1b773dacf4cb8e6ecfe9097d5156c1549f4b",
    "tokenizer.model": (
        "5a493598071550244b2ee7f26118f3edec2150b9dfa967929a99052ac83fe716"
    ),
    "tokenizer_config.json": "cc69c30f0d14ebef1afde2077ea4b11ec23764be",
    "visual.py": "ee34c991cf3b847af4a8ef067716038fa10b89da",
}

EXCLUSIONS = (
    "HM3D and simulator scenes",
    "ObjectNav datasets",
    "overlays and SIF images",
    "robot bags, maps, and runtime state",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def artifact_record(
    path: Path,
    expected: dict[str, Any],
) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"missing {expected['name']}: {path}")
    observed_size = path.stat().st_size
    observed_digest = sha256_file(path)
    if observed_size != expected["bytes"] or observed_digest != expected["sha256"]:
        raise RuntimeError(
            f"artifact identity mismatch for {path}: "
            f"bytes={observed_size}/{expected['bytes']} "
            f"sha256={observed_digest}/{expected['sha256']}"
        )
    return {
        "name": expected["name"],
        "path": str(path),
        "bytes": observed_size,
        "sha256": observed_digest,
        "classification": "observed checksum verification",
    }


def download_direct(workspace: Path, record: dict[str, Any]) -> None:
    destination = workspace / record["path"]
    if destination.exists():
        artifact_record(destination, record)
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
            record["url"],
            headers={"User-Agent": "TopoFocus-cleanroom/1"},
        )
        with urllib.request.urlopen(request, timeout=60) as response:
            with temporary.open("wb") as output:
                while block := response.read(8 * 1024 * 1024):
                    output.write(block)
        artifact_record(temporary, record)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def download_rednet(workspace: Path) -> None:
    destination = workspace / REDNET["path"]
    if destination.exists():
        artifact_record(destination, REDNET)
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
            id=REDNET["google_drive_id"],
            output=str(temporary),
            quiet=False,
        )
        if result is None:
            raise RuntimeError(
                f"RedNet download failed; source page: {REDNET['source_page']}"
            )
        artifact_record(temporary, REDNET)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def download_huggingface(workspace: Path) -> None:
    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise RuntimeError("huggingface_hub is required for model snapshots") from exc

    # The extra "hub" component intentionally matches HF_HOME/hub layout and
    # the runtime path locked by manifests/artifacts.json.
    cache_dir = workspace / "artifacts/models/hf_cache/hub"
    snapshot_download(
        repo_id=GLM["repo"],
        revision=GLM_REVISION,
        cache_dir=cache_dir,
    )
    segformer_dir = workspace / SEGFORMER["path"]
    snapshot_download(
        repo_id=SEGFORMER["repo"],
        revision=SEGFORMER_REVISION,
        local_dir=segformer_dir,
        allow_patterns=[item["path"] for item in SEGFORMER["files"]],
    )


def verify_glm(workspace: Path) -> dict[str, Any]:
    model_root = workspace / GLM["path"]
    snapshot = model_root / "snapshots" / GLM_REVISION
    blobs = model_root / "blobs"
    if not snapshot.is_dir() or not blobs.is_dir():
        raise RuntimeError(f"missing pinned GLM cache: {model_root}")

    snapshot_entries = sorted(snapshot.iterdir())
    broken_links = [
        str(path)
        for path in snapshot_entries
        if path.is_symlink() and not path.exists()
    ]
    if broken_links:
        raise RuntimeError(f"broken GLM snapshot links: {broken_links}")
    if len(snapshot_entries) != GLM["snapshot_entries"]:
        raise RuntimeError(
            "GLM snapshot entry count mismatch: "
            f"{len(snapshot_entries)}/{GLM['snapshot_entries']}"
        )
    snapshot_names = {path.name for path in snapshot_entries}
    if snapshot_names != set(GLM_SNAPSHOT_BLOBS):
        raise RuntimeError(
            "GLM snapshot file set mismatch: "
            f"observed={sorted(snapshot_names)} "
            f"expected={sorted(GLM_SNAPSHOT_BLOBS)}"
        )
    for path in snapshot_entries:
        expected_blob = GLM_SNAPSHOT_BLOBS[path.name]
        if not path.is_symlink():
            raise RuntimeError(f"GLM snapshot entry is not a symlink: {path}")
        if path.resolve().parent != blobs or path.resolve().name != expected_blob:
            raise RuntimeError(
                f"GLM snapshot mapping mismatch for {path.name}: "
                f"{path.resolve().name}/{expected_blob}"
            )
    shards = sorted(snapshot.glob("model-*-of-00015.safetensors"))
    if len(shards) != GLM["model_shards"]:
        raise RuntimeError(
            f"GLM shard count mismatch: {len(shards)}/{GLM['model_shards']}"
        )

    blob_paths = sorted(path for path in blobs.iterdir() if path.is_file())
    blob_bytes = sum(path.stat().st_size for path in blob_paths)
    if len(blob_paths) != GLM["blob_count"]:
        raise RuntimeError(
            f"GLM blob count mismatch: {len(blob_paths)}/{GLM['blob_count']}"
        )
    if blob_bytes != GLM["blob_bytes"]:
        raise RuntimeError(f"GLM blob bytes mismatch: {blob_bytes}/{GLM['blob_bytes']}")
    if {path.name for path in blob_paths} != set(GLM_BLOBS):
        raise RuntimeError("GLM blob name set does not match the public lock")

    blob_records: list[dict[str, Any]] = []
    for path in blob_paths:
        expected_size, expected_digest = GLM_BLOBS[path.name]
        digest = sha256_file(path)
        if path.stat().st_size != expected_size or digest != expected_digest:
            raise RuntimeError(
                f"locked GLM blob mismatch for {path.name}: "
                f"bytes={path.stat().st_size}/{expected_size} "
                f"sha256={digest}/{expected_digest}"
            )
        blob_records.append(
            {
                "name": path.name,
                "path": str(path),
                "bytes": path.stat().st_size,
                "sha256": digest,
                "content_addressed": bool(re.fullmatch(r"[0-9a-f]{64}", path.name)),
            }
        )
    return {
        "name": GLM["name"],
        "path": str(model_root),
        "revision": GLM_REVISION,
        "snapshot_entries": len(snapshot_entries),
        "model_shards": len(shards),
        "blob_count": len(blob_records),
        "blob_bytes": blob_bytes,
        "blobs": blob_records,
        "classification": "observed full-cache checksum verification",
    }


def verify_all(workspace: Path) -> dict[str, Any]:
    standalone = [
        artifact_record(workspace / record["path"], record)
        for record in (*DIRECT_ARTIFACTS, REDNET)
    ]
    segformer_root = workspace / SEGFORMER["path"]
    segformer = [
        artifact_record(segformer_root / record["path"], record)
        for record in SEGFORMER["files"]
    ]
    return {
        "schema_version": "focus-cleanroom-model-provenance-v1",
        "workspace": str(workspace),
        "standalone_artifacts": standalone,
        "segformer": {
            "repository": SEGFORMER["repo"],
            "revision": SEGFORMER_REVISION,
            "files": segformer,
        },
        "glm": verify_glm(workspace),
        "deliberate_exclusions": list(EXCLUSIONS),
        "physical_commands_sent": False,
        "robot_connections_opened": False,
        "verified": True,
    }


def write_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--apply", action="store_true")
    operation.add_argument("--verify", action="store_true")
    parser.add_argument(
        "--accept-model-licenses",
        action="store_true",
        help="confirm that the operator reviewed each upstream model license",
    )
    parser.add_argument("--write-provenance", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()

    plan = {
        "schema_version": "focus-cleanroom-model-fetch-v2",
        "workspace": str(workspace),
        "direct_artifacts": list(DIRECT_ARTIFACTS),
        "rednet": REDNET,
        "segformer": SEGFORMER,
        "glm": GLM,
        "deliberate_exclusions": list(EXCLUSIONS),
        "physical_commands_sent": False,
        "robot_connections_opened": False,
    }
    if not args.apply and not args.verify:
        print(json.dumps(plan, indent=2, sort_keys=True))
        print("PLAN_ONLY=true")
        return 0
    if args.apply and not args.accept_model_licenses:
        parser.error("--apply requires --accept-model-licenses")

    if args.apply:
        for record in DIRECT_ARTIFACTS:
            download_direct(workspace, record)
        download_rednet(workspace)
        download_huggingface(workspace)

    provenance = verify_all(workspace)
    if args.write_provenance is not None:
        write_atomic(
            args.write_provenance.expanduser().resolve(),
            provenance,
        )
    print(json.dumps(provenance, indent=2, sort_keys=True))
    print("CLEANROOM_MODELS_VERIFIED=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
