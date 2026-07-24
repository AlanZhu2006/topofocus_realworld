#!/usr/bin/env python3
"""Prove that a robot tracking process predates a validated debug run.

This read-only probe distinguishes a chassis-only power cycle from a restart
of the process that owns the robot-local odometry origin.  It reads Linux
process/tmux/systemd metadata only; it never opens a robot control interface.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


SCHEMA_VERSION = "focus-tracking-epoch-probe-v1"


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def read_boot_time_s() -> int:
    for line in Path("/proc/stat").read_text(encoding="utf-8").splitlines():
        key, *values = line.split()
        if key == "btime" and len(values) == 1:
            return int(values[0])
    raise ValueError("/proc/stat does not contain btime")


def read_boot_id() -> str:
    value = Path("/proc/sys/kernel/random/boot_id").read_text(
        encoding="utf-8"
    ).strip()
    if not value:
        raise ValueError("host boot ID is empty")
    return value


def process_identity(pid: int) -> dict[str, object]:
    if pid <= 0:
        raise ValueError("tracking process PID is not positive")
    proc = Path("/proc") / str(pid)
    raw_stat = (proc / "stat").read_text(encoding="utf-8")
    close = raw_stat.rfind(")")
    if close < 0:
        raise ValueError(f"invalid /proc/{pid}/stat")
    # Fields following ')' begin at Linux proc field 3 (state). starttime is
    # field 22, hence index 19 in this suffix.
    suffix = raw_stat[close + 2 :].split()
    if len(suffix) <= 19:
        raise ValueError(f"short /proc/{pid}/stat")
    start_ticks = int(suffix[19])
    clock_ticks = int(os.sysconf("SC_CLK_TCK"))
    boot_time_s = read_boot_time_s()
    start_time_ns = (
        boot_time_s * 1_000_000_000
        + start_ticks * 1_000_000_000 // clock_ticks
    )
    command = (proc / "cmdline").read_bytes()
    return {
        "pid": pid,
        "process_start_ticks": start_ticks,
        "clock_ticks_per_s": clock_ticks,
        "process_start_time_ns": start_time_ns,
        "executable": os.readlink(proc / "exe"),
        "command_line_sha256": sha256_bytes(command),
    }


def wsj_tracking_identity(session: str, window: str) -> dict[str, object]:
    locator = f"{session}:{window}"
    output = subprocess.check_output(
        [
            "tmux",
            "display-message",
            "-p",
            "-t",
            locator,
            "#{pane_pid}\t#{pane_dead}\t#{pane_start_command}",
        ],
        text=True,
    ).rstrip("\n")
    fields = output.split("\t", 2)
    if len(fields) != 3:
        raise ValueError(f"unexpected tmux tracking identity: {output!r}")
    pid_text, pane_dead, start_command = fields
    if pane_dead != "0":
        raise ValueError(f"WSJ tracking pane is dead: {locator}")
    identity = process_identity(int(pid_text))
    identity.update(
        {
            "kind": "tmux_tracking_process",
            "locator": locator,
            "start_command_sha256": sha256_bytes(start_command.encode()),
        }
    )
    return identity


def yunji_tracking_identity(service: str) -> dict[str, object]:
    active = subprocess.check_output(
        ["systemctl", "is-active", service], text=True
    ).strip()
    if active != "active":
        raise ValueError(f"Yunji tracking service is not active: {service}")
    output = subprocess.check_output(
        [
            "systemctl",
            "show",
            service,
            "--property=MainPID",
            "--property=FragmentPath",
            "--property=ExecMainStartTimestampMonotonic",
        ],
        text=True,
    )
    values = {}
    for line in output.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key] = value
    identity = process_identity(int(values.get("MainPID", "0")))
    identity.update(
        {
            "kind": "systemd_tracking_process",
            "locator": service,
            "unit_fragment_path": values.get("FragmentPath", ""),
            "exec_start_monotonic": values.get(
                "ExecMainStartTimestampMonotonic", ""
            ),
        }
    )
    return identity


def atomic_write(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--robot-id", choices=("robot-0", "robot-1"), required=True)
    result.add_argument("--debug-passed-at-ns", type=int, required=True)
    result.add_argument(
        "--wsj-session", default="tinynav_semantic_nav_auto"
    )
    result.add_argument("--wsj-tracking-window", default="perception")
    result.add_argument(
        "--yunji-service", default="focus-yunji-odin1-driver.service"
    )
    result.add_argument("--output", type=Path, required=True)
    return result


def main() -> int:
    args = parser().parse_args()
    probed_at_ns = time.time_ns()
    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "robot_id": args.robot_id,
        "probed_at_ns": probed_at_ns,
        "debug_passed_at_ns": args.debug_passed_at_ns,
        "classification": (
            "observed_linux_tracking_process_epoch_compared_with_"
            "validated_debug_timestamp"
        ),
        "host_boot_id": read_boot_id(),
        "sources": [
            "/proc/stat",
            "/proc/sys/kernel/random/boot_id",
            "/proc/<pid>/stat",
            "/proc/<pid>/cmdline",
            (
                "tmux pane metadata"
                if args.robot_id == "robot-0"
                else "systemd unit metadata"
            ),
        ],
        "safety": {
            "read_only_host_metadata": True,
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
        },
    }
    try:
        if args.debug_passed_at_ns <= 0:
            raise ValueError("session has no validated debug timestamp")
        tracking = (
            wsj_tracking_identity(args.wsj_session, args.wsj_tracking_window)
            if args.robot_id == "robot-0"
            else yunji_tracking_identity(args.yunji_service)
        )
        start_ns = int(tracking["process_start_time_ns"])
        passed = start_ns <= args.debug_passed_at_ns
        payload.update(
            {
                "passed": passed,
                "tracking_epoch": tracking,
                "continuity": {
                    "passed": passed,
                    "criterion": (
                        "same live tracking process started no later than "
                        "the session's strict debug validation"
                    ),
                    "process_started_after_debug_by_s": max(
                        0.0,
                        (start_ns - args.debug_passed_at_ns) / 1e9,
                    ),
                },
            }
        )
        if not passed:
            payload["error"] = (
                "tracking process restarted after strict debug; the old "
                "shared-frame transform cannot be reused directly"
            )
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        payload.update({"passed": False, "error": str(exc)})
    atomic_write(args.output, payload)
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(
        json.dumps(
            {
                "output": str(args.output),
                "size_bytes": args.output.stat().st_size,
                "sha256": digest,
                "passed": payload["passed"],
                "robot_id": args.robot_id,
                "robot_commands_issued": False,
            },
            sort_keys=True,
        )
    )
    return 0 if payload["passed"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
