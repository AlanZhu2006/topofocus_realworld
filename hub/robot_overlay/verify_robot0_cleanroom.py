#!/usr/bin/env python3
"""Read-only Robot 0 clean-room installation verifier."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
from typing import Any


MOTION_PROCESS_NAMES = (
    "go2_cmd_bridge",
    "cmd_vel_control",
    "planning_node.py",
    "v2_wsj_receiver",
    "focus_guarded_cmd_vel",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_value(root: Path, expression: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", expression],
        text=True,
    ).strip()


def active_motion_processes() -> list[dict[str, Any]]:
    own_ancestors: set[int] = set()
    pid = os.getpid()
    while pid > 1:
        own_ancestors.add(pid)
        try:
            pid = int(
                Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[3]
            )
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


def checked_file(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"missing required file: {path}")
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def write_atomic_json(path: Path, payload: dict[str, Any]) -> None:
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
    parser.add_argument("--lock", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--config-root", type=Path, required=True)
    parser.add_argument("--state-root", type=Path, required=True)
    parser.add_argument("--write-provenance", type=Path)
    args = parser.parse_args()

    lock = json.loads(args.lock.read_text(encoding="utf-8"))
    install_root = args.install_root.expanduser().resolve()
    config_root = args.config_root.expanduser().resolve()
    state_root = args.state_root.expanduser().resolve()
    tinynav = install_root / "workspaces/tinynav"
    expected = lock["sources"]["tinynav"]

    failures: list[str] = []
    observations: dict[str, Any] = {
        "schema_version": "focus-robot0-cleanroom-provenance-v1",
        "classification": "observed read-only host verification",
        "physical_commands_sent": False,
        "host": {
            "architecture": platform.machine(),
            "python": platform.python_version(),
            "kernel": platform.release(),
        },
        "paths": {
            "install_root": str(install_root),
            "config_root": str(config_root),
            "state_root": str(state_root),
        },
    }

    try:
        head = git_value(tinynav, "HEAD")
        tree = git_value(tinynav, "HEAD^{tree}")
        observations["tinynav"] = {"head": head, "tree": tree}
        if head != expected["reconstructed_commit"]:
            failures.append(f"TinyNav HEAD mismatch: {head}")
        if tree != expected["reconstructed_tree"]:
            failures.append(f"TinyNav tree mismatch: {tree}")
    except (OSError, subprocess.CalledProcessError) as exc:
        failures.append(f"TinyNav Git verification failed: {exc}")

    files: list[dict[str, Any]] = []
    for record in lock["tinynav_onnx_artifacts"]:
        path = tinynav / record["path"]
        try:
            observed = checked_file(path)
            files.append(observed)
            if observed["bytes"] != record["bytes"]:
                failures.append(f"size mismatch: {path}")
            if observed["sha256"] != record["sha256"]:
                failures.append(f"SHA-256 mismatch: {path}")
        except RuntimeError as exc:
            failures.append(str(exc))
    observations["onnx_artifacts"] = files

    for required in (
        config_root / "robot-0-setup.bash",
        config_root / "robot-0.env",
        tinynav / ".venv/bin/python",
    ):
        try:
            checked_file(required)
        except RuntimeError as exc:
            failures.append(str(exc))

    plans = sorted((tinynav / "tinynav/models").glob("*_aarch64.plan"))
    observations["tensorrt_plans"] = [checked_file(path) for path in plans]
    if len(plans) < 4:
        failures.append(
            f"expected at least four aarch64 TensorRT plans, found {len(plans)}"
        )

    processes = active_motion_processes()
    observations["active_motion_processes"] = processes
    if processes:
        failures.append("a motion or planning process is active")

    observations["failures"] = failures
    if args.write_provenance is not None:
        write_atomic_json(args.write_provenance, observations)
    print(json.dumps(observations, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
