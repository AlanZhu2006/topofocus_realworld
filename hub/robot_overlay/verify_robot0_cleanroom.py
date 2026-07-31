#!/usr/bin/env python3
"""Read-only verifier for a clean-room Robot 0 / Unitree Go2 install.

The verifier never initializes Unitree DDS, starts ROS nodes, or publishes a
command.  Its three levels are cumulative:

* software: locked sources, models, TensorRT plans, config, and safe imports;
* host: software plus the exact reference JetPack/ROS package contract;
* hardware: host plus read-only D435i, USB-power, and host-network checks.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import stat
import subprocess
import tempfile
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOCK = (
    REPOSITORY_ROOT / "hub/config/deployments/robot0_cleanroom_sources_v1.json"
)
MOTION_PROCESS_NAMES = (
    "go2_cmd_bridge",
    "cmd_vel_control",
    "planning_node.py",
    "v2_wsj_receiver",
    "v2_robot0_receiver",
    "focus_guarded_cmd_vel",
)
SOURCE_LAYOUT = {
    "cyclonedds": Path("src/cyclonedds"),
    "gtsam": Path("src/gtsam"),
    "librealsense": Path("src/librealsense"),
    "realsense_ros": Path("workspaces/realsense_ws/src/realsense-ros"),
    "message_filters": Path("workspaces/message_filters_ws/src/message_filters"),
}
RUNTIME_IMPORT_PROBE = r"""
import importlib.metadata
import json
import platform

import cyclonedds
import cv_bridge
import gtsam
import message_filters
import rclpy
import tensorrt
import tinynav
import unitree_sdk2py

print(json.dumps({
    "python": platform.python_version(),
    "cyclonedds": importlib.metadata.version("cyclonedds"),
    "tensorrt": getattr(tensorrt, "__version__", "present"),
    "tinynav": importlib.metadata.version("tinynav"),
    "unitree_sdk2py": importlib.metadata.version("unitree-sdk2py"),
}, sort_keys=True))
"""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
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


def git_value(root: Path, expression: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", expression],
        text=True,
        stderr=subprocess.DEVNULL,
    ).strip()


def git_checkout_record(
    root: Path,
    expected_commit: str,
    failures: list[str],
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "path": str(root),
        "expected_commit": expected_commit,
    }
    try:
        head = git_value(root, "HEAD")
        status = subprocess.check_output(
            ["git", "-C", str(root), "status", "--porcelain"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        failures.append(f"Git verification failed for {root}: {exc}")
        return record
    record.update({"head": head, "clean": not status})
    if head != expected_commit:
        failures.append(
            f"source revision mismatch for {root}: {head} != {expected_commit}"
        )
    if status:
        failures.append(f"source checkout is dirty: {root}")
    return record


def active_motion_processes() -> list[dict[str, Any]]:
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


def parse_assignment_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise RuntimeError(f"invalid assignment at {path}:{line_number}: {line}")
        key, raw_value = line.split("=", 1)
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            raise RuntimeError(f"invalid key at {path}:{line_number}: {key}")
        parsed = shlex.split(raw_value, posix=True)
        if len(parsed) != 1:
            raise RuntimeError(f"invalid value at {path}:{line_number}: {raw_value}")
        values[key] = parsed[0]
    return values


def os_release() -> dict[str, str]:
    values: dict[str, str] = {}
    for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parsed = shlex.split(value)
        values[key] = parsed[0] if parsed else ""
    return values


def dpkg_version(package: str) -> str | None:
    result = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", package],
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def command_record(
    command: list[str],
    failures: list[str],
    label: str,
) -> dict[str, Any]:
    try:
        result = subprocess.run(
            command,
            text=True,
            capture_output=True,
            check=False,
            timeout=30,
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


def usb_devices() -> list[dict[str, Any]]:
    devices: list[dict[str, Any]] = []
    for entry in sorted(Path("/sys/bus/usb/devices").glob("*")):
        vendor_path = entry / "idVendor"
        product_path = entry / "idProduct"
        if not vendor_path.is_file() or not product_path.is_file():
            continue
        record: dict[str, Any] = {
            "sysfs": str(entry),
            "usb_id": (
                f"{vendor_path.read_text().strip()}:"
                f"{product_path.read_text().strip()}"
            ),
        }
        power_path = entry / "power/control"
        if power_path.is_file():
            record["power_control"] = power_path.read_text().strip()
        devices.append(record)
    return devices


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
    default_install = (
        Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
        / "topofocus/robot-0"
    )
    default_config = (
        Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "topofocus"
    )
    default_state = (
        Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
        / "topofocus"
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--install-root", type=Path, default=default_install)
    parser.add_argument("--config-root", type=Path, default=default_config)
    parser.add_argument("--state-root", type=Path, default=default_state)
    parser.add_argument(
        "--level",
        choices=("software", "host", "hardware"),
        default="software",
    )
    parser.add_argument("--write-provenance", type=Path)
    args = parser.parse_args()

    lock_path = args.lock.expanduser().resolve()
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    if lock.get("schema_version") != "focus-robot0-cleanroom-sources-v1":
        raise SystemExit(f"unsupported source lock: {lock_path}")

    install_root = args.install_root.expanduser().resolve()
    config_root = args.config_root.expanduser().resolve()
    state_root = args.state_root.expanduser().resolve()
    tinynav = install_root / "workspaces/tinynav"
    setup_file = config_root / "robot-0-setup.bash"
    env_file = config_root / "robot-0.env"
    tinynav_python = tinynav / ".venv/bin/python"
    failures: list[str] = []
    observations: dict[str, Any] = {
        "schema_version": "focus-robot0-cleanroom-provenance-v2",
        "classification": "observed read-only host verification",
        "verification_level": args.level,
        "physical_commands_sent": False,
        "robot_connections_opened": False,
        "lock": checked_file(lock_path),
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

    expected_tinynav = lock["sources"]["tinynav"]
    try:
        head = git_value(tinynav, "HEAD")
        tree = git_value(tinynav, "HEAD^{tree}")
        status = subprocess.check_output(
            ["git", "-C", str(tinynav), "status", "--porcelain"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
        observations["tinynav"] = {
            "path": str(tinynav),
            "head": head,
            "tree": tree,
            "clean": not status,
        }
        if head != expected_tinynav["reconstructed_commit"]:
            failures.append(f"TinyNav HEAD mismatch: {head}")
        if tree != expected_tinynav["reconstructed_tree"]:
            failures.append(f"TinyNav tree mismatch: {tree}")
        if status:
            failures.append("TinyNav checkout is dirty")
    except (OSError, subprocess.CalledProcessError) as exc:
        failures.append(f"TinyNav Git verification failed: {exc}")

    for relative, key in (
        ("pyproject.toml", "pyproject_sha256"),
        ("uv.lock", "uv_lock_sha256"),
    ):
        path = tinynav / relative
        try:
            record = checked_file(path)
            observations.setdefault("tinynav_lock_files", []).append(record)
            if record["sha256"] != expected_tinynav[key]:
                failures.append(f"TinyNav {relative} SHA-256 mismatch")
        except RuntimeError as exc:
            failures.append(str(exc))

    source_records: dict[str, Any] = {}
    for source_name, relative in SOURCE_LAYOUT.items():
        source = lock["sources"][source_name]
        source_records[source_name] = git_checkout_record(
            install_root / relative,
            source["commit"],
            failures,
        )
    observations["source_checkouts"] = source_records

    onnx_records: list[dict[str, Any]] = []
    for expected in lock["tinynav_onnx_artifacts"]:
        path = tinynav / expected["path"]
        try:
            record = checked_file(path)
            onnx_records.append(record)
            if record["bytes"] != expected["bytes"]:
                failures.append(f"ONNX size mismatch: {path}")
            if record["sha256"] != expected["sha256"]:
                failures.append(f"ONNX SHA-256 mismatch: {path}")
        except RuntimeError as exc:
            failures.append(str(exc))
    observations["onnx_artifacts"] = onnx_records

    plan_records: list[dict[str, Any]] = []
    for expected in lock["tensorrt_plans"]:
        path = tinynav / expected["path"]
        try:
            record = checked_file(path)
            if record["bytes"] <= 0:
                failures.append(f"empty TensorRT plan: {path}")
            record["source_onnx"] = expected["source_onnx"]
            plan_records.append(record)
        except RuntimeError as exc:
            failures.append(str(exc))
    observations["tensorrt_plans"] = plan_records

    required_files = (setup_file, env_file, tinynav_python)
    for required in required_files:
        try:
            checked_file(required)
        except RuntimeError as exc:
            failures.append(str(exc))
    if env_file.is_file():
        mode = stat.S_IMODE(env_file.stat().st_mode)
        observations["environment_file_mode"] = oct(mode)
        if mode & 0o077:
            failures.append(f"environment file is not private: {env_file}")
        try:
            env_values = parse_assignment_file(env_file)
        except RuntimeError as exc:
            failures.append(str(exc))
            env_values = {}
        observations["environment_contract"] = env_values
        expected_values = {
            "TINYNAV_ROOT": str(tinynav),
            "TINYNAV_PATCHED_ROOT": str(tinynav),
            "TINYNAV_PERCEPTION_PATCHED_ROOT": str(tinynav),
            "TINYNAV_PERCEPTION_PATCHED_COMMIT": expected_tinynav[
                "reconstructed_commit"
            ],
            "TINYNAV_PERCEPTION_PATCHED_SHA256": (
                "3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3"
            ),
            "TINYNAV_SETUP": str(setup_file),
            "TINYNAV_PYTHON": str(tinynav_python),
            "FOCUS_ROBOT_STATE_DIR": str(state_root),
            "FOCUS_ROBOT_ID": "robot-0",
            "FOCUS_HUB_BASE_URL": lock["network_defaults"]["hub_endpoint"],
            "UNITREE_NET_IF": lock["network_defaults"]["unitree_interface"],
            "UNITREE_HOST_CIDR": lock["network_defaults"]["host_address"],
            "UNITREE_ROBOT_ADDRESS": lock["network_defaults"]["robot_address"],
        }
        for key, expected in expected_values.items():
            if env_values.get(key) != expected:
                failures.append(
                    f"environment mismatch for {key}: "
                    f"{env_values.get(key)!r} != {expected!r}"
                )
        if any("TOKEN" in key for key in env_values):
            failures.append("tracked/generated environment must not contain tokens")

    if setup_file.is_file() and tinynav_python.is_file():
        runtime = command_record(
            [
                "bash",
                "-c",
                'source "$1"; exec "$2" -c "$3"',
                "verify-robot0-runtime",
                str(setup_file),
                str(tinynav_python),
                RUNTIME_IMPORT_PROBE,
            ],
            failures,
            "safe runtime import probe",
        )
        observations["runtime_import_probe"] = runtime
        if runtime.get("returncode") == 0:
            try:
                runtime_versions = json.loads(str(runtime["stdout"]))
                observations["runtime_versions"] = runtime_versions
                expected_python = lock["tools"]["python"]
                if runtime_versions.get("python") != expected_python:
                    failures.append(
                        "runtime Python mismatch: "
                        f"{runtime_versions.get('python')} != {expected_python}"
                    )
                if runtime_versions.get("cyclonedds") != "0.10.2":
                    failures.append("Python CycloneDDS must be 0.10.2")
                if runtime_versions.get("unitree_sdk2py") != "1.0.1":
                    failures.append("Unitree SDK2 Python must be 1.0.1")
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                failures.append(f"invalid runtime import report: {exc}")

    processes = active_motion_processes()
    observations["active_motion_processes"] = processes
    if processes:
        failures.append("a motion or planning process is active")

    if args.level in {"host", "hardware"}:
        release = os_release()
        observations["host"]["os_release"] = release
        expected_platform = lock["base_platform"]
        if platform.machine() != expected_platform["architecture"]:
            failures.append(
                f"architecture mismatch: {platform.machine()} != "
                f"{expected_platform['architecture']}"
            )
        if release.get("ID") != "ubuntu" or release.get("VERSION_ID") != "22.04":
            failures.append("reference host requires Ubuntu 22.04")
        if platform.release() != expected_platform["kernel"]:
            failures.append(
                f"kernel mismatch: {platform.release()} != "
                f"{expected_platform['kernel']}"
            )

        package_versions: dict[str, str | None] = {}
        for package, expected_version in lock["host_packages"][
            "exact_reference_versions"
        ].items():
            observed_version = dpkg_version(package)
            package_versions[package] = observed_version
            if observed_version != expected_version:
                failures.append(
                    f"package version mismatch for {package}: "
                    f"{observed_version} != {expected_version}"
                )
        observations["host"]["package_versions"] = package_versions

        compatibility = Path("/tinynav")
        observations["host"]["compatibility_link"] = {
            "path": str(compatibility),
            "target": (
                str(compatibility.resolve())
                if compatibility.exists() or compatibility.is_symlink()
                else None
            ),
        }
        if not compatibility.is_symlink() or compatibility.resolve() != tinynav:
            failures.append(f"/tinynav does not resolve to {tinynav}")
        observations["host"]["trtexec"] = command_record(
            ["trtexec", "--version"],
            failures,
            "TensorRT trtexec probe",
        )

    if args.level == "hardware":
        hardware = lock["hardware"]
        devices = usb_devices()
        observations["usb_devices"] = devices
        for field in ("camera_usb_id", "usb3_hub_id"):
            expected_id = hardware[field]
            matching = [item for item in devices if item["usb_id"] == expected_id]
            if not matching:
                failures.append(f"missing USB device {expected_id}")
                continue
            if any(item.get("power_control") != "on" for item in matching):
                failures.append(f"USB power policy is not on for {expected_id}")

        usbfs_path = Path("/sys/module/usbcore/parameters/usbfs_memory_mb")
        try:
            usbfs_memory = int(usbfs_path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError) as exc:
            failures.append(f"cannot read usbfs_memory_mb: {exc}")
            usbfs_memory = None
        observations["usbfs_memory_mb"] = usbfs_memory
        if (
            usbfs_memory is not None
            and usbfs_memory < hardware["usbfs_memory_mb_minimum"]
        ):
            failures.append("usbfs_memory_mb is below the locked minimum")

        realsense_probe = command_record(
            ["rs-enumerate-devices", "-s"],
            failures,
            "RealSense device probe",
        )
        observations["realsense"] = realsense_probe
        if realsense_probe.get("returncode") == 0:
            output = (
                str(realsense_probe.get("stdout", ""))
                + "\n"
                + str(realsense_probe.get("stderr", ""))
            )
            if hardware["camera_firmware"] not in output:
                failures.append("D435i firmware does not match the reference contract")

        interface = observations.get("environment_contract", {}).get("UNITREE_NET_IF")
        host_cidr = lock["network_defaults"]["host_address"]
        robot_address = lock["network_defaults"]["robot_address"]
        if interface:
            address_probe = command_record(
                ["ip", "-o", "-4", "addr", "show", "dev", str(interface)],
                failures,
                "Unitree host-interface address probe",
            )
            observations["unitree_network_address"] = address_probe
            if address_probe.get("returncode") == 0 and host_cidr not in str(
                address_probe.get("stdout", "")
            ):
                failures.append(
                    f"{interface} does not carry reference address {host_cidr}"
                )
            route_probe = command_record(
                ["ip", "route", "get", robot_address],
                failures,
                "Unitree host-interface route probe",
            )
            observations["unitree_network_route"] = route_probe
            if route_probe.get("returncode") == 0 and f"dev {interface}" not in str(
                route_probe.get("stdout", "")
            ):
                failures.append(f"route to {robot_address} does not use {interface}")

    observations["failures"] = failures
    observations["verified"] = not failures
    if args.write_provenance is not None:
        write_atomic_json(
            args.write_provenance.expanduser().resolve(),
            observations,
        )
    print(json.dumps(observations, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
