from __future__ import annotations

import importlib.util
import hashlib
import json
from pathlib import Path
import re
import subprocess

WORKSPACE = Path(__file__).resolve().parents[2]
LOCK_PATH = WORKSPACE / "hub/config/deployments/robot0_cleanroom_sources_v1.json"


def run(*command: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=WORKSPACE,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_robot0_source_lock_is_complete() -> None:
    lock = json.loads(LOCK_PATH.read_text(encoding="utf-8"))
    assert lock["schema_version"] == "focus-robot0-cleanroom-sources-v1"
    assert lock["base_platform"] == {
        "operating_system": "Ubuntu 22.04",
        "architecture": "aarch64",
        "kernel": "5.15.148-tegra",
        "jetpack": "6.2.1+b38",
        "l4t_package": "36.4.7-20250918154033",
        "classification": "observed reference platform",
    }
    for source in lock["sources"].values():
        commit = source.get("commit")
        if commit is not None:
            assert re.fullmatch(r"[0-9a-f]{40}", commit)
    assert len(lock["tinynav_onnx_artifacts"]) == 5
    for artifact in lock["tinynav_onnx_artifacts"]:
        assert artifact["bytes"] > 0
        assert re.fullmatch(r"[0-9a-f]{64}", artifact["sha256"])
    assert len(lock["tensorrt_plans"]) == 4
    assert all(
        record["path"].endswith("_aarch64.plan") for record in lock["tensorrt_plans"]
    )
    assert lock["safety"]["bootstrap_starts_actuation"] is False
    assert lock["safety"]["physical_motion_requires_per_run_authorization"]
    for record in lock["hub_runtime"]["lock_files"]:
        path = WORKSPACE / record["path"]
        assert path.stat().st_size == record["bytes"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]


def test_cleanroom_scripts_have_read_only_default_plans() -> None:
    commands = (
        ("bash", "hub/robot_overlay/bootstrap_robot0_cleanroom.sh"),
        ("bash", "hub/robot_overlay/configure_go2_network.sh"),
        ("bash", "hub/scripts/bootstrap_gpu_hub_cleanroom.sh"),
        ("python3", "hub/tools/fetch_cleanroom_models.py"),
    )
    for command in commands:
        result = run(*command)
        assert result.returncode == 0, result.stderr
        assert "PLAN_ONLY=true" in result.stdout
    assert (
        "No ROS, planner, receiver, camera, or Unitree process is started."
        in run(*commands[0]).stdout
    )
    assert "robot contacted: no" in run(*commands[1]).stdout


def test_cleanroom_shell_entrypoints_parse() -> None:
    scripts = (
        "hub/robot_overlay/bootstrap_robot0_cleanroom.sh",
        "hub/robot_overlay/configure_go2_network.sh",
        "hub/robot_overlay/save_go2_buildmap.sh",
        "hub/robot_overlay/start_go2_buildmap.sh",
        "hub/robot_overlay/stop_wsj_live_command_path.sh",
        "hub/robot_overlay/start_wsj_buildmap_v2.sh",
        "hub/robot_overlay/start_wsj_calibration_observation.sh",
        "hub/scripts/bootstrap_gpu_hub_cleanroom.sh",
        "hub/scripts/install_source_semantic_stack.sh",
        "hub/scripts/realworld_oneclick.sh",
    )
    result = run("bash", "-n", *scripts)
    assert result.returncode == 0, result.stderr


def test_generated_robot_environment_parser_contract(tmp_path: Path) -> None:
    verifier = load_module(
        "verify_robot0_cleanroom",
        WORKSPACE / "hub/robot_overlay/verify_robot0_cleanroom.py",
    )
    env_file = tmp_path / "robot-0.env"
    env_file.write_text(
        "TINYNAV_ROOT=/tmp/robot\\ 0/tinynav\n"
        "FOCUS_HUB_BASE_URL=http://127.0.0.1:18089\n"
        "UNITREE_NET_IF=eth0\n",
        encoding="utf-8",
    )
    assert verifier.parse_assignment_file(env_file) == {
        "TINYNAV_ROOT": "/tmp/robot 0/tinynav",
        "FOCUS_HUB_BASE_URL": "http://127.0.0.1:18089",
        "UNITREE_NET_IF": "eth0",
    }


def test_model_fetch_contract_matches_public_artifact_manifest() -> None:
    fetcher = load_module(
        "fetch_cleanroom_models",
        WORKSPACE / "hub/tools/fetch_cleanroom_models.py",
    )
    manifest = json.loads(
        (WORKSPACE / "manifests/artifacts.json").read_text(encoding="utf-8")
    )
    by_name = {record["name"]: record for record in manifest["artifacts"]}

    expected_standalone = {
        "YOLOv10m checkpoint": fetcher.DIRECT_ARTIFACTS[0],
        "OpenAI CLIP ViT-B/32 checkpoint": fetcher.DIRECT_ARTIFACTS[1],
        "RedNet HM3D semantic map checkpoint": fetcher.REDNET,
    }
    for manifest_name, locked in expected_standalone.items():
        published = by_name[manifest_name]
        assert published["path"] == locked["path"]
        assert published["bytes"] == locked["bytes"]
        assert published["sha256"] == locked["sha256"]

    segformer = by_name["SegFormer-B0 ADE20K real-camera semantic adapter"]
    assert segformer["revision"] == fetcher.SEGFORMER_REVISION
    locked_segformer = [
        {
            "path": record["path"],
            "bytes": record["bytes"],
            "sha256": record["sha256"],
        }
        for record in fetcher.SEGFORMER["files"]
    ]
    assert segformer["files"] == locked_segformer
    glm = by_name["GLM-4V-9B Hugging Face cache"]
    assert glm["revision"] == fetcher.GLM_REVISION
    assert glm["blob_count"] == fetcher.GLM["blob_count"]
    assert glm["model_shards"] == fetcher.GLM["model_shards"]
    assert len(fetcher.GLM_BLOBS) == fetcher.GLM["blob_count"]
    assert (
        sum(size for size, _digest in fetcher.GLM_BLOBS.values())
        == fetcher.GLM["blob_bytes"]
    )
    assert set(fetcher.GLM_SNAPSHOT_BLOBS.values()) == set(fetcher.GLM_BLOBS)
    for size, digest in fetcher.GLM_BLOBS.values():
        assert size > 0
        assert re.fullmatch(r"[0-9a-f]{64}", digest)


def test_cleanroom_entrypoints_have_no_personal_home_paths() -> None:
    paths = (
        "hub/robot_overlay/bootstrap_robot0_cleanroom.sh",
        "hub/robot_overlay/configure_go2_network.sh",
        "hub/robot_overlay/config/go2.env.example",
        "hub/robot_overlay/stop_wsj_live_command_path.sh",
        "hub/scripts/bootstrap_gpu_hub_cleanroom.sh",
        "hub/tools/fetch_cleanroom_models.py",
        "hub/tools/verify_gpu_cleanroom.py",
        "docs/ROBOT0_REPRODUCIBLE_BASELINE.md",
    )
    for relative in paths:
        text = (WORKSPACE / relative).read_text(encoding="utf-8")
        assert "/home/asus" not in text
        assert "/home/nvidia" not in text
