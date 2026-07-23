#!/usr/bin/env python3
"""Continuous, source-derived two-robot VLM scene runner (shadow only).

Each round waits for a new, fresh, synchronized semantic-map keyframe from
every robot and invokes ``live_vlm_shadow.py`` with one persistent episode
state.  That state preserves the HPC loop's shared directional history,
previous per-robot positions, 0/24/49/... decision clock, frontier/history
branch and ``Find_Goal`` semantic override.

This runner has no GOAL mode and cannot report navigation success.  When the
source algorithm finds its target semantic channel, this runner pauses at the
handoff because the source still requires robot-local planner STOP and its
HM3D success metric also uses GT target evidence. Only a future robot-local
planner/HIL result plus independent target validation may establish physical
navigation success.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import httpx


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES  # noqa: E402
from focus_hub.map_snapshot import load_map_snapshot  # noqa: E402
from focus_hub.models import ObservationMetadata  # noqa: E402
from focus_hub.shadow_coordination import sha256_file  # noqa: E402
from focus_hub.source_episode import (  # noqa: E402
    SOURCE_MAX_EPISODE_STEPS,
    SOURCE_HM3D_OBJECTNAV_GOALS,
    SourceEpisodeState,
    source_decision_round_limit,
    source_decision_step,
)


SCENE_MANIFEST_SCHEMA_VERSION = "focus-live-vlm-shadow-scene-v1"


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    name: str
    snapshot_dir: Path


@dataclass(frozen=True)
class ReadyInput:
    robot_id: str
    name: str
    sequence: int
    capture_time_ns: int
    transform_version: str
    map_sha256: str


class SceneSafetyAbort(RuntimeError):
    """A fail-closed map/session violation, distinct from ordinary waiting."""


def parse_robot_spec(value: str) -> RobotSpec:
    parts = value.split(":", 2)
    if len(parts) != 3 or not all(parts):
        raise argparse.ArgumentTypeError(
            f"expected ROBOT_ID:NAME:SNAPSHOT_DIR, got {value!r}"
        )
    return RobotSpec(parts[0], parts[1], Path(parts[2]).expanduser().resolve())


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def read_json(path: Path) -> dict[str, object] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    return value if isinstance(value, dict) else None


def check_hub_shadow_policy(hub_url: str, robot_ids: tuple[str, ...]) -> None:
    response = httpx.get(f"{hub_url}/healthz", timeout=5.0)
    response.raise_for_status()
    health = response.json()
    enabled = health.get("goal_output_enabled", {})
    if not isinstance(enabled, dict) or any(
        enabled.get(robot_id) is not False for robot_id in robot_ids
    ):
        raise SceneSafetyAbort(
            "refusing scene runner while Hub GOAL output is not disabled for every robot"
        )


def check_glm(glm_url: str) -> None:
    response = httpx.get(f"{glm_url}/models", timeout=10.0)
    response.raise_for_status()


def inspect_ready_input(
    spec: RobotSpec,
    *,
    spool: Path,
    calibration_id: str,
    previous_sequence: int | None,
) -> ReadyInput | None:
    status = read_json(spec.snapshot_dir / "live_status.json")
    summary = read_json(spec.snapshot_dir / "map_summary.json")
    if status is None or summary is None:
        return None
    block_reason = status.get("mapping_blocked_reason")
    if block_reason is not None:
        raise SceneSafetyAbort(f"{spec.name} map is blocked: {block_reason}")
    if status.get("shared_frame_calibration_id") != calibration_id:
        raise SceneSafetyAbort(f"{spec.name} live status calibration mismatch")
    if summary.get("shared_frame_calibration_id") != calibration_id:
        raise SceneSafetyAbort(f"{spec.name} map summary calibration mismatch")
    if status.get("transform_version") != summary.get("transform_version"):
        raise SceneSafetyAbort(f"{spec.name} status/summary transform mismatch")

    semantic_mapping = summary.get("semantic_mapping")
    if not isinstance(semantic_mapping, dict):
        raise SceneSafetyAbort(f"{spec.name} map summary has no semantic mapping")
    yolo = semantic_mapping.get("yolo_reinforcement")
    if not isinstance(yolo, dict) or yolo.get("enabled") is not True:
        raise SceneSafetyAbort(f"{spec.name} YOLO reinforcement is not enabled")
    try:
        sequence = int(yolo.get("last_sequence", -1))
    except (TypeError, ValueError) as exc:
        raise SceneSafetyAbort(f"{spec.name} has an invalid YOLO sequence") from exc
    if sequence < 0 or (previous_sequence is not None and sequence <= previous_sequence):
        return None

    metadata_path = spool / spec.robot_id / f"{sequence:020d}" / "metadata.json"
    try:
        metadata = ObservationMetadata.model_validate_json(
            metadata_path.read_text(encoding="utf-8")
        )
    except (FileNotFoundError, OSError, ValueError):
        return None
    if metadata.robot_id != spec.robot_id or metadata.sequence != sequence:
        raise SceneSafetyAbort(f"{spec.name} source metadata identity mismatch")
    transform_version = str(status.get("transform_version", ""))
    if metadata.pose.transform_version != transform_version:
        raise SceneSafetyAbort(f"{spec.name} source/status transform mismatch")

    snapshot = load_map_snapshot(spec.snapshot_dir / "central_map.npz")
    if snapshot is None:
        return None
    if snapshot.shared_frame_calibration_id != calibration_id:
        raise SceneSafetyAbort(f"{spec.name} map snapshot calibration mismatch")
    if snapshot.transform_version != transform_version:
        raise SceneSafetyAbort(f"{spec.name} snapshot/status transform mismatch")
    return ReadyInput(
        robot_id=spec.robot_id,
        name=spec.name,
        sequence=sequence,
        capture_time_ns=metadata.capture_time_ns,
        transform_version=transform_version,
        map_sha256=sha256_file(spec.snapshot_dir / "central_map.npz"),
    )


def input_timing(
    inputs: list[ReadyInput],
    *,
    now_ns: int,
    max_input_age_s: float,
    max_sync_skew_s: float,
) -> dict[str, object]:
    ages = [(now_ns - item.capture_time_ns) / 1e9 for item in inputs]
    skew = (max(item.capture_time_ns for item in inputs) - min(
        item.capture_time_ns for item in inputs
    )) / 1e9
    return {
        "input_ages_s": ages,
        "oldest_input_age_s": max(ages),
        "cross_robot_capture_skew_s": skew,
        "fresh": min(ages) >= -1.0 and max(ages) <= max_input_age_s,
        "synchronized": skew <= max_sync_skew_s,
    }


def wait_for_round_inputs(
    specs: list[RobotSpec],
    *,
    spool: Path,
    calibration_id: str,
    previous_sequences: dict[str, int],
    max_input_age_s: float,
    max_sync_skew_s: float,
    poll_s: float,
    max_idle_s: float,
    stop_requested,
) -> tuple[list[ReadyInput], dict[str, object]]:
    deadline = time.monotonic() + max_idle_s
    last_detail = "waiting for map files"
    while time.monotonic() < deadline:
        if stop_requested():
            raise InterruptedError("operator stop requested")
        inputs: list[ReadyInput] = []
        for spec in specs:
            ready = inspect_ready_input(
                spec,
                spool=spool,
                calibration_id=calibration_id,
                previous_sequence=previous_sequences.get(spec.robot_id),
            )
            if ready is None:
                last_detail = f"waiting for a new accepted keyframe from {spec.name}"
                break
            inputs.append(ready)
        if len(inputs) == len(specs):
            timing = input_timing(
                inputs,
                now_ns=time.time_ns(),
                max_input_age_s=max_input_age_s,
                max_sync_skew_s=max_sync_skew_s,
            )
            if timing["fresh"] and timing["synchronized"]:
                return inputs, timing
            last_detail = (
                f"waiting for fresh/synchronized inputs: "
                f"oldest_age={float(timing['oldest_input_age_s']):.3f}s, "
                f"skew={float(timing['cross_robot_capture_skew_s']):.3f}s"
            )
        time.sleep(poll_s)
    raise TimeoutError(f"no valid scene round within {max_idle_s:.1f}s: {last_detail}")


def provenance_record(path: Path, status: str) -> dict[str, object]:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "status": status,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", action="append", type=parse_robot_spec, required=True)
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument(
        "--goal-category",
        choices=SOURCE_HM3D_OBJECTNAV_GOALS,
        default="chair",
    )
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--hub-url", default="http://127.0.0.1:8088")
    parser.add_argument("--glm-url", default="http://127.0.0.1:31511/v1")
    parser.add_argument("--admin-token-file", type=Path, default=WORKSPACE / "hub/runtime/admin_token")
    parser.add_argument("--registry-state", type=Path, default=WORKSPACE / "hub/runtime/state/registry_state.json")
    parser.add_argument("--poll-s", type=float, default=1.0)
    parser.add_argument("--max-idle-s", type=float, default=300.0)
    parser.add_argument("--max-input-age-s", type=float, default=30.0)
    parser.add_argument("--max-sync-skew-s", type=float, default=5.0)
    parser.add_argument(
        "--max-rounds",
        type=int,
        default=source_decision_round_limit(),
        help="bounded shadow rounds; source 500-step episode contains 21 decisions",
    )
    parser.add_argument("--publish-hold", action="store_true")
    parser.add_argument("--write-foxglove-targets", action="store_true")
    args = parser.parse_args()

    specs: list[RobotSpec] = args.robot
    robot_ids = tuple(spec.robot_id for spec in specs)
    if len(specs) < 2 or len(set(robot_ids)) != len(specs):
        parser.error("at least two unique robots are required")
    if not args.scene_id or args.scene_id == "UNSET":
        parser.error("a unique --scene-id is required")
    if any(value <= 0.0 for value in (
        args.poll_s,
        args.max_idle_s,
        args.max_input_age_s,
    )) or args.max_sync_skew_s < 0.0:
        parser.error("poll/idle/freshness values must be valid and positive")
    source_round_limit = source_decision_round_limit()
    if args.max_rounds <= 0 or args.max_rounds > source_round_limit:
        parser.error(f"--max-rounds must be in [1, {source_round_limit}]")

    output = args.output.expanduser().resolve()
    if output.exists():
        print(f"refusing to overwrite existing scene output: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)
    spool = args.spool.expanduser().resolve()
    state_path = output / "scene_state.json"
    manifest_path = output / "scene_manifest.json"
    events_path = output / "scene_events.jsonl"
    scene_run_id = f"scene-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
    state = SourceEpisodeState(
        scene_id=args.scene_id,
        goal_category=args.goal_category,
        shared_frame_calibration_id=args.calibration_id,
        robot_ids=robot_ids,
        source_find_goal={robot_id: False for robot_id in robot_ids},
    )
    atomic_write_json(state_path, state.to_dict())

    source_main = WORKSPACE / "source/Focus_realworld/main.py"
    source_agent = WORKSPACE / "source/Focus_realworld/agents/vlm_agents.py"
    source_arguments = WORKSPACE / "source/Focus_realworld/arguments.py"
    source_constants = WORKSPACE / "source/Focus_realworld/constants.py"
    source_task = WORKSPACE / "source/Focus_realworld/tasks/multi_objectnav_hm3d.yaml"
    source_task_core = (
        WORKSPACE / "dependencies/habitat-lab/habitat/core/embodied_task.py"
    )
    source_navigation_task = (
        WORKSPACE / "dependencies/habitat-lab/habitat/tasks/nav/nav.py"
    )
    round_tool = WORKSPACE / "hub/tools/live_vlm_shadow.py"
    manifest: dict[str, object] = {
        "schema_version": SCENE_MANIFEST_SCHEMA_VERSION,
        "scene_run_id": scene_run_id,
        "scene_id": args.scene_id,
        "status": "running_shadow_scene",
        "outcome": "not_navigation_success",
        "goal_category": args.goal_category,
        "shared_frame_calibration_id": args.calibration_id,
        "robot_ids": list(robot_ids),
        "started_at_ns": time.time_ns(),
        "source_contract": {
            "max_episode_steps": SOURCE_MAX_EPISODE_STEPS,
            "decision_steps": [
                source_decision_step(index) for index in range(source_round_limit)
            ],
            "history_scope": "shared_across_agents",
            "find_goal": "any positive target semantic; largest connected component",
            "success": (
                "source episode ends on robot-local STOP; HM3D Total_SR also "
                "requires GT target evidence; both unavailable in shadow"
            ),
        },
        "safety": {
            "robot_commands_sent": False,
            "goal_publication_code_path_present": False,
            "hub_decision_mode_if_published": "HOLD",
            "allow_goal_changed": False,
            "random_physical_goal_suppressed": True,
        },
        "provenance": [
            provenance_record(source_main, "immutable authoritative source"),
            provenance_record(source_agent, "immutable authoritative source"),
            provenance_record(source_arguments, "immutable authoritative source"),
            provenance_record(source_constants, "immutable authoritative source"),
            provenance_record(source_task, "immutable authoritative source"),
            provenance_record(
                source_task_core,
                "immutable authoritative source dependency",
            ),
            provenance_record(
                source_navigation_task,
                "immutable authoritative source dependency",
            ),
            provenance_record(round_tool, "source-derived deployment adapter"),
            provenance_record(Path(__file__), "source-derived continuous shadow runner"),
        ],
        "rounds": [],
    }
    atomic_write_json(manifest_path, manifest)

    stop = False

    def request_stop(_signal, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def emit(event: dict[str, object]) -> None:
        with events_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"timestamp_ns": time.time_ns(), **event}, sort_keys=True) + "\n")

    exit_code = 0
    try:
        check_hub_shadow_policy(args.hub_url, robot_ids)
        check_glm(args.glm_url)
        emit({"event": "preflight_passed"})
        for _ in range(args.max_rounds):
            state = SourceEpisodeState.from_dict(
                json.loads(state_path.read_text(encoding="utf-8"))
            )
            inputs, timing = wait_for_round_inputs(
                specs,
                spool=spool,
                calibration_id=args.calibration_id,
                previous_sequences=state.last_source_sequences,
                max_input_age_s=args.max_input_age_s,
                max_sync_skew_s=args.max_sync_skew_s,
                poll_s=args.poll_s,
                max_idle_s=args.max_idle_s,
                stop_requested=lambda: stop,
            )
            check_hub_shadow_policy(args.hub_url, robot_ids)
            round_index = state.round_index
            source_step = state.source_step
            round_dir = output / f"round_{round_index:02d}_step_{source_step:03d}"
            command = [
                str(WORKSPACE / "hub/.venv/bin/python"),
                str(round_tool),
            ]
            for spec in specs:
                command.extend([
                    "--robot",
                    f"{spec.robot_id}:{spec.name}:{spec.snapshot_dir}",
                ])
            for item in inputs:
                command.extend([
                    "--expected-source-sequence",
                    f"{item.robot_id}:{item.sequence}",
                    "--expected-map-sha256",
                    f"{item.robot_id}:{item.map_sha256}",
                ])
            command.extend([
                "--spool", str(spool),
                "--output", str(round_dir),
                "--glm-url", args.glm_url,
                "--goal-category", args.goal_category,
                "--expected-shared-frame-calibration-id", args.calibration_id,
                "--source-step", str(source_step),
                "--scene-state-file", str(state_path),
                "--hub-url", args.hub_url,
                "--admin-token-file", str(args.admin_token_file.expanduser().resolve()),
                "--registry-state", str(args.registry_state.expanduser().resolve()),
                "--max-input-age-s", str(args.max_input_age_s),
                "--max-sync-skew-s", str(args.max_sync_skew_s),
            ])
            for category in HM3D_CATEGORY_NAMES:
                command.extend(["--trusted-category", category])
            if args.publish_hold:
                command.append("--publish-hold")
            if args.write_foxglove_targets:
                command.append("--write-foxglove-targets")
            emit({
                "event": "round_started",
                "round_index": round_index,
                "source_step": source_step,
                "inputs": [item.__dict__ for item in inputs],
                "timing": timing,
            })
            environment = os.environ.copy()
            environment["PYTHONPATH"] = str(WORKSPACE / "hub/src")
            completed = subprocess.run(
                command,
                cwd=WORKSPACE,
                env=environment,
                capture_output=True,
                text=True,
                check=False,
            )
            stdout_path = output / f"round_{round_index:02d}_stdout.log"
            stderr_path = output / f"round_{round_index:02d}_stderr.log"
            stdout_path.write_text(completed.stdout, encoding="utf-8")
            stderr_path.write_text(completed.stderr, encoding="utf-8")
            if completed.returncode != 0:
                raise RuntimeError(
                    f"round {round_index} failed with exit {completed.returncode}; "
                    f"see {stderr_path}"
                )
            round_manifest_path = round_dir / "shadow_manifest.json"
            round_manifest = json.loads(round_manifest_path.read_text(encoding="utf-8"))
            if round_manifest.get("status") != "complete_shadow_only":
                raise RuntimeError(f"round {round_index} did not complete shadow-only")
            safety = round_manifest.get("safety", {})
            if not isinstance(safety, dict) or safety.get("robot_commands_sent") is not False:
                raise RuntimeError(f"round {round_index} lost its no-command invariant")
            updated_state = SourceEpisodeState.from_dict(
                json.loads(state_path.read_text(encoding="utf-8"))
            )
            if updated_state.round_index != round_index + 1:
                raise RuntimeError("round did not advance persistent episode state exactly once")
            round_record = {
                "round_index": round_index,
                "source_step": source_step,
                "input_sequences": {
                    item.robot_id: item.sequence for item in inputs
                },
                "input_timing": timing,
                "manifest_path": str(round_manifest_path.resolve()),
                "manifest_size_bytes": round_manifest_path.stat().st_size,
                "manifest_sha256": sha256_file(round_manifest_path),
                "status": round_manifest["status"],
                "source_episode_round_status": round_manifest.get(
                    "source_episode_round_status"
                ),
                "final_shadow_selections": round_manifest.get(
                    "final_shadow_selections", {}
                ),
            }
            rounds = manifest["rounds"]
            if not isinstance(rounds, list):
                raise RuntimeError("scene manifest rounds became malformed")
            rounds.append(round_record)
            emit({"event": "round_completed", **round_record})

            if any(updated_state.source_find_goal.values()):
                manifest["status"] = (
                    "paused_shadow_target_found_awaiting_robot_local_planner_stop"
                )
                manifest["terminal_reason"] = (
                    "HPC Find_Goal semantic override fired; source still requires "
                    "local planner STOP and independent target validation"
                )
                break
            if source_step >= SOURCE_MAX_EPISODE_STEPS - 1:
                manifest["status"] = "complete_shadow_max_steps_without_target"
                manifest["terminal_reason"] = "source 500-step episode budget exhausted"
                break
        else:
            manifest["status"] = "complete_requested_shadow_rounds"
            manifest["terminal_reason"] = (
                "operator-requested round bound reached before a source terminal condition"
            )
    except InterruptedError as exc:
        manifest["status"] = "stopped_by_operator"
        manifest["terminal_reason"] = str(exc)
    except (SceneSafetyAbort, TimeoutError, RuntimeError, httpx.HTTPError) as exc:
        manifest["status"] = "aborted_fail_closed"
        manifest["terminal_reason"] = f"{type(exc).__name__}: {exc}"
        exit_code = 1
    except Exception as exc:  # noqa: BLE001 - every unexpected fault fails closed
        manifest["status"] = "aborted_internal_fail_closed"
        manifest["terminal_reason"] = f"{type(exc).__name__}: {exc}"
        exit_code = 1
    finally:
        manifest["completed_at_ns"] = time.time_ns()
        manifest["outcome"] = "not_navigation_success"
        manifest["final_state_path"] = str(state_path.resolve())
        if state_path.is_file():
            manifest["final_state_sha256"] = sha256_file(state_path)
        atomic_write_json(manifest_path, manifest)
        emit({
            "event": "scene_terminal",
            "status": manifest["status"],
            "terminal_reason": manifest.get("terminal_reason", ""),
        })

    print(json.dumps({
        "scene_run_id": scene_run_id,
        "status": manifest["status"],
        "outcome": manifest["outcome"],
        "rounds": len(manifest["rounds"]),
        "manifest": str(manifest_path),
    }, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
