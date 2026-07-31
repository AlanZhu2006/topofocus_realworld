#!/usr/bin/env python3
"""Read-only verifier for the clean-room RTX 4090 Hub environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import tempfile
from typing import Any

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_LOCK = WORKSPACE / "hub/config/deployments/robot0_cleanroom_sources_v1.json"
MOTION_PROCESS_NAMES = (
    "realworld_oneclick",
    "v2_source_episode",
    "v2_wsj_receiver",
    "v2_yunji_receiver",
    "go2_cmd_bridge",
    "focus_guarded_cmd_vel",
)
RUNTIME_PROBE = r"""
import importlib.metadata
import json
import platform

import detectron2
import torch
import torchvision

packages = {}
for distribution in importlib.metadata.distributions():
    name = distribution.metadata.get("Name")
    if name:
        packages[name.lower()] = distribution.version

kernel_result = torch.arange(8, device="cuda").sum().item()
print(json.dumps({
    "python": platform.python_version(),
    "packages": dict(sorted(packages.items())),
    "torch_runtime": {
        "torch": torch.__version__,
        "torchvision": torchvision.__version__,
        "cuda": torch.version.cuda,
        "cuda_available": torch.cuda.is_available(),
        "device": torch.cuda.get_device_name(0),
        "kernel_result": kernel_result,
        "detectron2": detectron2.__version__,
    },
}, sort_keys=True))
"""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checked_file(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"missing required file: {path}")
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def command_record(
    command: list[str],
    failures: list[str],
    label: str,
    *,
    timeout: int = 60,
) -> dict[str, Any]:
    try:
        result = subprocess.run(
            command,
            text=True,
            capture_output=True,
            check=False,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        failures.append(f"{label} failed: {exc}")
        return {"command": command, "error": str(exc)}
    record = {
        "command": command,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }
    if result.returncode != 0:
        failures.append(f"{label} failed with exit {result.returncode}")
    return record


def active_runtime_processes() -> list[dict[str, Any]]:
    own_ancestors: set[int] = set()
    pid = os.getpid()
    while pid > 1:
        own_ancestors.add(pid)
        try:
            pid = int(Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[3])
        except (OSError, ValueError, IndexError):
            break

    matches: list[dict[str, Any]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in own_ancestors:
            continue
        try:
            command = (
                (entry / "cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode(errors="replace")
            )
        except OSError:
            continue
        if any(name in command for name in MOTION_PROCESS_NAMES):
            matches.append({"pid": int(entry.name), "command": command})
    return matches


def os_release() -> dict[str, str]:
    result: dict[str, str] = {}
    for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parsed = shlex.split(value)
        result[key] = parsed[0] if parsed else ""
    return result


def package_version(package: str) -> str | None:
    result = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", package],
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


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
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--env-dir", type=Path, required=True)
    parser.add_argument("--uv", type=Path, required=True)
    parser.add_argument("--model-provenance", type=Path, required=True)
    parser.add_argument("--write-provenance", type=Path)
    args = parser.parse_args()

    workspace = args.workspace.expanduser().resolve()
    lock_path = args.lock.expanduser().resolve()
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    if lock.get("schema_version") != "focus-robot0-cleanroom-sources-v1":
        raise SystemExit(f"unsupported clean-room lock: {lock_path}")
    hub_lock = lock["hub_runtime"]
    env_dir = args.env_dir.expanduser().resolve()
    uv = args.uv.expanduser().resolve()
    model_provenance = args.model_provenance.expanduser().resolve()
    python = env_dir / "bin/python"
    failures: list[str] = []
    observations: dict[str, Any] = {
        "schema_version": "focus-gpu-cleanroom-provenance-v1",
        "classification": "observed read-only host verification",
        "physical_commands_sent": False,
        "robot_connections_opened": False,
        "host": {
            "architecture": platform.machine(),
            "kernel": platform.release(),
            "os_release": os_release(),
        },
        "paths": {
            "workspace": str(workspace),
            "environment": str(env_dir),
        },
        "lock": checked_file(lock_path),
    }

    locked_runtime_files: list[dict[str, Any]] = []
    for expected in hub_lock["lock_files"]:
        try:
            record = checked_file(workspace / expected["path"])
            locked_runtime_files.append(record)
            if record["bytes"] != expected["bytes"]:
                failures.append(f"locked file size mismatch: {expected['path']}")
            if record["sha256"] != expected["sha256"]:
                failures.append(f"locked file SHA-256 mismatch: {expected['path']}")
        except RuntimeError as exc:
            failures.append(str(exc))
    observations["locked_runtime_files"] = locked_runtime_files

    for name, path in (
        ("runtime_pyproject", workspace / "hub/gpu_runtime/pyproject.toml"),
        ("runtime_lock", workspace / "hub/gpu_runtime/uv.lock"),
        ("model_provenance", model_provenance),
        ("python", python),
        ("uv", uv),
    ):
        try:
            observations[name] = checked_file(path)
        except RuntimeError as exc:
            failures.append(str(exc))

    try:
        model_record = json.loads(model_provenance.read_text(encoding="utf-8"))
        observations["model_verification"] = {
            "schema_version": model_record.get("schema_version"),
            "verified": model_record.get("verified"),
            "workspace": model_record.get("workspace"),
        }
        if not model_record.get("verified"):
            failures.append("model provenance is not verified")
        if Path(model_record.get("workspace", "")).resolve() != workspace:
            failures.append("model provenance belongs to another workspace")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        failures.append(f"cannot read model provenance: {exc}")

    git = command_record(
        ["git", "-C", str(workspace), "status", "--porcelain"],
        failures,
        "repository status probe",
    )
    observations["repository_status"] = git
    if git.get("returncode") == 0 and git.get("stdout"):
        failures.append("repository has tracked or untracked changes")
    for label, expression in (("head", "HEAD"), ("tree", "HEAD^{tree}")):
        record = command_record(
            ["git", "-C", str(workspace), "rev-parse", expression],
            failures,
            f"repository {label} probe",
        )
        observations[f"repository_{label}"] = record.get("stdout")

    release = observations["host"]["os_release"]
    if platform.machine() != "x86_64":
        failures.append("reference Hub requires x86_64")
    if release.get("ID") != "ubuntu" or release.get("VERSION_ID") != "22.04":
        failures.append("reference Hub requires Ubuntu 22.04")

    apt_packages = {
        name: package_version(name)
        for name in (
            "build-essential",
            "libc6",
            "ninja-build",
            "nvidia-driver-570",
            "python3.10",
        )
    }
    observations["host"]["package_versions"] = apt_packages

    uv_probe = command_record(
        [str(uv), "--version"],
        failures,
        "uv version probe",
    )
    observations["uv_version_probe"] = uv_probe
    expected_uv = lock["tools"]["uv"]
    if uv_probe.get("stdout") != f"uv {expected_uv}":
        failures.append(f"uv version mismatch: {uv_probe.get('stdout')}")

    smi_probe = command_record(
        [
            "nvidia-smi",
            "--query-gpu=name,driver_version",
            "--format=csv,noheader",
        ],
        failures,
        "NVIDIA GPU probe",
    )
    observations["nvidia_smi"] = smi_probe
    if "RTX 4090" not in str(smi_probe.get("stdout", "")):
        failures.append("reference Hub requires an RTX 4090")

    if python.is_file():
        runtime_probe = command_record(
            [str(python), "-c", RUNTIME_PROBE],
            failures,
            "GPU runtime probe",
            timeout=120,
        )
        observations["runtime_probe"] = runtime_probe
        if runtime_probe.get("returncode") == 0:
            try:
                runtime = json.loads(str(runtime_probe["stdout"]))
                observations["runtime"] = runtime
                expected_packages = {
                    "detectron2": "0.6",
                    "focus-realworld-hub": "0.1.0",
                    "ninja": "1.13.0",
                    "torch": "2.8.0",
                    "torchvision": "0.23.0",
                    "transformers": "4.51.0",
                }
                if runtime.get("python") != hub_lock["python"]:
                    failures.append(f"Hub runtime Python must be {hub_lock['python']}")
                packages = runtime.get("packages", {})
                for name, expected in expected_packages.items():
                    if packages.get(name) != expected:
                        failures.append(
                            f"runtime package mismatch for {name}: "
                            f"{packages.get(name)} != {expected}"
                        )
                torch_runtime = runtime.get("torch_runtime", {})
                if torch_runtime.get("torch") != hub_lock["torch"]:
                    failures.append(f"PyTorch runtime must be {hub_lock['torch']}")
                if torch_runtime.get("torchvision") != hub_lock["torchvision"]:
                    failures.append(
                        "torchvision runtime must be " f"{hub_lock['torchvision']}"
                    )
                if torch_runtime.get("cuda") != hub_lock["cuda_runtime"]:
                    failures.append(
                        "PyTorch CUDA runtime must be " f"{hub_lock['cuda_runtime']}"
                    )
                if not torch_runtime.get("cuda_available"):
                    failures.append("CUDA is unavailable")
                if "RTX 4090" not in str(torch_runtime.get("device", "")):
                    failures.append("PyTorch is not using an RTX 4090")
                if torch_runtime.get("kernel_result") != 28:
                    failures.append("CUDA smoke kernel returned wrong result")
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                failures.append(f"invalid GPU runtime report: {exc}")

    processes = active_runtime_processes()
    observations["active_realworld_processes"] = processes
    if processes:
        failures.append("a real-world runtime or motion process is active")

    observations["failures"] = failures
    observations["verified"] = not failures
    if args.write_provenance is not None:
        write_atomic(
            args.write_provenance.expanduser().resolve(),
            observations,
        )
    print(json.dumps(observations, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
