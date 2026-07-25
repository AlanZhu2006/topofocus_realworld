#!/usr/bin/env python3
"""Run a continuous source-derived supervised two-robot physical episode.

The runner composes the already validated one-round pieces without moving
velocity authority into the Hub:

* freeze a fresh synchronized RGB-D/map pair;
* run the immutable-source-derived VLM cascade with one persistent
  ``SourceEpisodeState``;
* publish one atomic, expiring v2 high-level goal batch;
* renew that batch while robot-local TinyNav/WATER planners remain active;
* after the source-equivalent 24/25 feedback ticks, HOLD both robots, freeze
  the next round and repeat; and
* once a semantic-region leg reports ARRIVED, HOLD the pair and seal a fresh
  terminal RGB-D/map evidence bundle.

``Find_Goal`` and robot-local ARRIVED remain distinct from official SR/SPL.
The automatic evidence bundle is observed evidence for later independent
annotation; this process never claims that its own model output verifies the
target or that the stop lies in a pre-surveyed goal region.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid

import httpx


WORKSPACE = Path(__file__).resolve().parents[2]
HUB_DIR = WORKSPACE / "hub"
TOOLS_DIR = HUB_DIR / "tools"
sys.path.insert(0, str(HUB_DIR / "src"))
sys.path.insert(0, str(TOOLS_DIR))

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES  # noqa: E402
from focus_hub.map_snapshot import load_map_snapshot  # noqa: E402
from focus_hub.models import ObservationMetadata  # noqa: E402
from focus_hub.realworld_session import (  # noqa: E402
    RealworldSession,
    load_session_file,
    resolve_workspace_path,
    session_contract_sha256,
    validate_session,
)
from focus_hub.source_episode import (  # noqa: E402
    SOURCE_HM3D_OBJECTNAV_GOALS,
    SOURCE_MAX_EPISODE_STEPS,
    SOURCE_NUM_LOCAL_STEPS,
    SourceEpisodeState,
    source_decision_round_limit,
)
from focus_hub.transport_v2 import DecisionBatchV2  # noqa: E402
from focus_hub.v2_episode_control import (  # noqa: E402
    next_coordination_batch,
    recoverable_frontier_failure,
)
from focus_hub.v2_frontier_clearance import (  # noqa: E402
    apply_frontier_clearance_guard,
)
from focus_hub.v2_route_conflict import apply_route_conflict_guard  # noqa: E402
from focus_hub.v2_scene_batch import build_batch_from_shadow_manifest  # noqa: E402
from freeze_realworld_inputs import freeze, stable_copy_map  # noqa: E402
from manage_realworld_session import resolve_session_argument  # noqa: E402


LIVE_CONFIRMATION = "OPERATOR_PRESENT_AND_ROBOTS_CLEAR"
RUN_SCHEMA_VERSION = "focus-v2-supervised-episode-run-v1"
SCENE_SCHEMA_VERSION = "focus-v2-source-live-scene-v1"
VLM_TARGET_EVENT_SCHEMA_VERSION = "focus-vlm-target-event-v1"
VLM_TARGET_EVENT_FILENAME = "vlm_target_event.json"
ACTIVE_FEEDBACK = {"RECEIVED", "ACCEPTED", "NAVIGATING"}
FAILURE_FEEDBACK = {
    "REJECTED",
    "OPERATOR_INTERVENTION",
    "LOCAL_ESTOP",
    "STOPPED",
    "HOLDING",
}
HOLD_FEEDBACK = {"HOLDING"}
SAFE_REASON = re.compile(r"[^a-zA-Z0-9_.-]+")


@dataclass(frozen=True)
class RoundInspection:
    failures: dict[str, dict[str, object]]
    semantic_arrivals: dict[str, dict[str, object]]
    frontier_arrivals: dict[str, dict[str, object]]
    current_feedback_ready: bool
    newest_server_time_ns: int


@dataclass(frozen=True)
class RoundResult:
    status: str
    reason: str
    final_states: dict[str, dict[str, object]]
    semantic_arrivals: dict[str, dict[str, object]]
    latest_events: dict[str, dict[str, object]]
    feedback_counts: dict[str, int]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def append_jsonl(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def vlm_target_event_payload(
    decision,
    *,
    publication_reason: str,
    published_at_ns: int,
) -> dict[str, object]:
    """Build the compact event consumed by the Foxglove Log panel.

    Frontier targets carry an authoritative shared-frame point. A semantic
    target is a region, not a point, so its logged coordinate is explicitly
    the display-only centroid; the robot-local adapter still resolves and
    freezes one reachable approach point for the leg.
    """

    target = decision.target
    target_summary: dict[str, object] | None = None
    if target is not None and target.kind == "FRONTIER_POINT":
        target_summary = {
            "kind": target.kind,
            "target_id": target.frontier_id,
            "frame_id": target.pose.frame_id,
            "shared_xy_m": [target.pose.x, target.pose.y],
            "yaw_rad": target.pose.yaw_rad,
            "coordinate_authority": "authoritative_frontier_goal_pose",
        }
    elif target is not None and target.kind == "SEMANTIC_REGION":
        target_summary = {
            "kind": target.kind,
            "target_id": target.category,
            "frame_id": target.display_centroid.frame_id,
            "shared_xy_m": [
                target.display_centroid.x,
                target.display_centroid.y,
            ],
            "coordinate_authority": (
                "display_only_semantic_region_centroid"
            ),
            "component_size_cells": target.region.component_size_cells,
            "region_origin_xy_m": list(target.region.origin_xy_m),
            "region_resolution_m": target.region.resolution_m,
            "region_height": target.region.height,
            "region_width": target.region.width,
        }
    active = decision.robot_id in decision.coordination.active_robot_ids
    return {
        "schema_version": VLM_TARGET_EVENT_SCHEMA_VERSION,
        "event_id": (
            f"{decision.decision_id}:{publication_reason}"
        ),
        "published_at_ns": published_at_ns,
        "status": "hub_accepted_high_level_decision",
        "classification": (
            "observed_hub_accepted_source_derived_vlm_decision"
        ),
        "robot_id": decision.robot_id,
        "scene_id": decision.scene_id,
        "episode_id": decision.episode_id,
        "round_index": decision.round_index,
        "source_step": decision.source_step,
        "decision_batch_id": decision.decision_batch_id,
        "decision_id": decision.decision_id,
        "leg_id": decision.leg_id,
        "lease_sequence": decision.lease_sequence,
        "execution_epoch": decision.coordination.execution_epoch,
        "active": active,
        "mode": decision.mode.value,
        "goal_category": decision.goal_category,
        "publication_reason": publication_reason,
        "decision_reason": decision.reason,
        "target": target_summary,
    }


def write_foxglove_vlm_target_events(
    session: RealworldSession,
    batch: DecisionBatchV2,
    *,
    publication_reason: str,
    published_at_ns: int,
) -> dict[str, str]:
    """Atomically expose the actual post-guard batch to the dashboard relay."""

    decisions = {decision.robot_id: decision for decision in batch.decisions}
    written: dict[str, str] = {}
    for robot in session.robots:
        decision = decisions.get(robot.robot_id)
        if decision is None:
            raise ValueError(
                f"published batch lacks session robot {robot.robot_id}"
            )
        map_dir = resolve_workspace_path(WORKSPACE, robot.map_dir)
        path = map_dir / VLM_TARGET_EVENT_FILENAME
        atomic_write_json(
            path,
            vlm_target_event_payload(
                decision,
                publication_reason=publication_reason,
                published_at_ns=published_at_ns,
            ),
        )
        written[robot.robot_id] = str(path)
    return written


def artifact_record(
    path: Path,
    *,
    classification: str,
    source_path: Path | None = None,
) -> dict[str, object]:
    resolved = path.expanduser().resolve()
    return {
        "path": str(resolved),
        "source_path": (
            None if source_path is None else str(source_path.expanduser().resolve())
        ),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
        "classification": classification,
    }


def source_round_step_quota(shadow_manifest: dict[str, object]) -> int:
    """Return the exact delta in the source logical decision clock.

    Round zero advances 0 -> 24, then each subsequent round advances by 25.
    Real hardware has no Habitat action counter, so the live adapter records
    one fresh robot navigation feedback event as one *source-derived physical
    clock tick*.  It never labels that tick an observed Habitat action.
    """

    source = shadow_manifest.get("source_episode")
    if not isinstance(source, dict) or source.get("enabled") is not True:
        raise ValueError("live source episode requires an enabled persistent scene state")
    current = source.get("logical_l_step")
    following = source.get("next_logical_l_step")
    if (
        isinstance(current, bool)
        or not isinstance(current, int)
        or isinstance(following, bool)
        or not isinstance(following, int)
    ):
        raise ValueError("source episode manifest lacks its logical step transition")
    delta = following - current
    if delta not in {SOURCE_NUM_LOCAL_STEPS - 1, SOURCE_NUM_LOCAL_STEPS}:
        raise ValueError(f"invalid source logical step delta: {current}->{following}")
    return delta


def inspect_round_states(
    states: dict[str, dict[str, object]],
    current: DecisionBatchV2,
    active_robot_ids: set[str],
) -> RoundInspection:
    decisions = {item.robot_id: item for item in current.decisions}
    failures: dict[str, dict[str, object]] = {}
    semantic_arrivals: dict[str, dict[str, object]] = {}
    frontier_arrivals: dict[str, dict[str, object]] = {}
    current_feedback_ready = True
    newest_server_time_ns = max(
        (int(item.get("server_time_ns", 0)) for item in states.values()),
        default=0,
    )
    for robot_id in active_robot_ids:
        state = states.get(robot_id, {})
        event = state.get("latest_event")
        decision = decisions[robot_id]
        if not isinstance(event, dict) or event.get("leg_id") != decision.leg_id:
            current_feedback_ready = False
            continue
        status = str(event.get("status", ""))
        if status in FAILURE_FEEDBACK:
            failures[robot_id] = dict(event)
        elif status == "ARRIVED":
            target = decision.target
            if target is not None and target.kind == "SEMANTIC_REGION":
                semantic_arrivals[robot_id] = dict(event)
            else:
                frontier_arrivals[robot_id] = dict(event)
        elif status not in ACTIVE_FEEDBACK:
            current_feedback_ready = False
        if event.get("decision_id") != decision.decision_id:
            current_feedback_ready = False
        received_at_ns = int(state.get("latest_event_received_at_ns", 0))
        if newest_server_time_ns - received_at_ns > 2_000_000_000:
            current_feedback_ready = False
    return RoundInspection(
        failures=failures,
        semantic_arrivals=semantic_arrivals,
        frontier_arrivals=frontier_arrivals,
        current_feedback_ready=current_feedback_ready,
        newest_server_time_ns=newest_server_time_ns,
    )


def update_event_records(
    latest: dict[str, dict[str, object]],
    states: dict[str, dict[str, object]],
) -> None:
    for robot_id, state in states.items():
        event = state.get("latest_event")
        if isinstance(event, dict):
            latest[robot_id] = dict(event)


def evaluation_seed_from_events(
    latest: dict[str, dict[str, object]],
    semantic_arrivals: dict[str, dict[str, object]],
) -> dict[str, object]:
    seeds: dict[str, object] = {}
    for robot_id in ("robot-0", "robot-1"):
        event = semantic_arrivals.get(robot_id, latest.get(robot_id))
        if not isinstance(event, dict):
            seeds[robot_id] = {"status": "missing_navigation_event"}
            continue
        status = str(event.get("status", ""))
        seeds[robot_id] = {
            "status": "source_derived_from_robot_navigation_event",
            "latest_navigation_status": status,
            "episode_start_local_pose": event.get("episode_start_local_pose"),
            "stop_local_pose": event.get("local_pose"),
            "actual_path_length_m": event.get("path_length_m_from_episode_start"),
            "local_planner_stopped": (
                status == "ARRIVED"
                and event.get("velocity_zero_confirmed") is True
            ),
            "terminal_observation_sequence": event.get(
                "terminal_observation_sequence"
            ),
            "arrival_target_kind": (
                "SEMANTIC_REGION"
                if robot_id in semantic_arrivals
                else "not_semantic_arrival"
            ),
        }
    return seeds


def verified_spool_observation(
    spool: Path,
    robot_id: str,
    sequence: int,
) -> tuple[ObservationMetadata, Path, Path, Path]:
    source = spool / robot_id / f"{sequence:020d}"
    metadata_path = source / "metadata.json"
    metadata = ObservationMetadata.model_validate_json(
        metadata_path.read_text(encoding="utf-8")
    )
    if metadata.robot_id != robot_id or metadata.sequence != sequence:
        raise ValueError(f"{robot_id} terminal observation identity mismatch")
    rgb_name = "rgb.jpg" if metadata.rgb_encoding == "jpeg" else "rgb.png"
    rgb_path = source / rgb_name
    depth_path = source / "depth.png"
    for path, size, digest in (
        (rgb_path, metadata.rgb_size_bytes, metadata.rgb_sha256),
        (depth_path, metadata.depth_size_bytes, metadata.depth_sha256),
    ):
        if not path.is_file() or path.stat().st_size != size:
            raise ValueError(f"terminal payload size mismatch: {path}")
        if sha256_file(path) != digest:
            raise ValueError(f"terminal payload hash mismatch: {path}")
    return metadata, metadata_path, rgb_path, depth_path


def atomic_copy_verified(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    shutil.copy2(source, temporary)
    if (
        temporary.stat().st_size != source.stat().st_size
        or sha256_file(temporary) != sha256_file(source)
    ):
        temporary.unlink(missing_ok=True)
        raise RuntimeError(f"copied evidence differs from source: {source}")
    os.replace(temporary, destination)


class EpisodeClient:
    def __init__(self, base_url: str, admin_token: str) -> None:
        self.client = httpx.Client(
            base_url=base_url.rstrip("/"),
            headers={"X-Admin-Token": admin_token},
            timeout=5.0,
        )

    def close(self) -> None:
        self.client.close()

    def publish(self, batch: DecisionBatchV2) -> dict[str, object]:
        response = self.client.post(
            "/v2/admin/decision-batches",
            json=batch.model_dump(mode="json"),
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise RuntimeError("Hub returned malformed decision acknowledgement")
        return payload

    def state(self, robot_id: str) -> dict[str, object]:
        response = self.client.get(
            f"/v2/admin/robots/{robot_id}/navigation-state"
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise RuntimeError("Hub returned malformed navigation state")
        return payload

    def readiness(self, robot_id: str) -> dict[str, object]:
        response = self.client.get(
            f"/v2/admin/robots/{robot_id}/runtime-readiness"
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise RuntimeError("Hub returned malformed runtime readiness")
        return payload


def wait_for_hold_ack(
    client: EpisodeClient,
    hold_batch: DecisionBatchV2,
    *,
    timeout_s: float,
    poll_s: float,
) -> dict[str, dict[str, object]]:
    expected = {item.robot_id: item.decision_id for item in hold_batch.decisions}
    deadline = time.monotonic() + timeout_s
    last_states: dict[str, dict[str, object]] = {}
    while time.monotonic() < deadline:
        last_states = {
            robot_id: client.state(robot_id) for robot_id in sorted(expected)
        }
        complete = True
        for robot_id, decision_id in expected.items():
            event = last_states[robot_id].get("latest_event")
            if (
                not isinstance(event, dict)
                or event.get("decision_id") != decision_id
                or event.get("status") not in HOLD_FEEDBACK
                or event.get("velocity_zero_confirmed") is not True
            ):
                complete = False
                break
        if complete:
            return last_states
        time.sleep(poll_s)
    raise TimeoutError(
        "timed out waiting for both robot-local HOLD/zero acknowledgements"
    )


def wait_and_seal_terminal_evidence(
    *,
    client: EpisodeClient,
    session: RealworldSession,
    spool: Path,
    semantic_arrivals: dict[str, dict[str, object]],
    arrival_received_at_ns: dict[str, int],
    decision_inputs: dict[str, int],
    output: Path,
    timeout_s: float,
    poll_s: float,
) -> dict[str, object]:
    """Preserve post-arrival observations without self-verifying semantics."""

    robots = {item.robot_id: item for item in session.robots}
    deadline = time.monotonic() + timeout_s
    selected: dict[str, tuple[int, dict[str, object], str]] = {}
    latest_seen: dict[str, dict[str, object]] = {}
    while time.monotonic() < deadline:
        for robot_id in robots:
            readiness = client.readiness(robot_id)
            latest_seen[robot_id] = readiness
            sequence = int(readiness.get("last_observation_sequence", -1))
            received_at_ns = int(
                readiness.get("last_observation_received_at_ns", 0)
            )
            if sequence < decision_inputs.get(robot_id, 0):
                continue
            if robot_id in semantic_arrivals:
                if (
                    sequence <= decision_inputs.get(robot_id, -1)
                    or received_at_ns
                    < arrival_received_at_ns.get(robot_id, 0)
                ):
                    continue
                status = "observed_post_arrival_hub_observation"
            else:
                status = "observed_scene_terminal_hub_observation"
            try:
                metadata, _, _, _ = verified_spool_observation(
                    spool, robot_id, sequence
                )
            except (FileNotFoundError, OSError, ValueError):
                continue
            if robot_id in semantic_arrivals and metadata.capture_time_ns < int(
                semantic_arrivals[robot_id].get("observed_at_ns", 0)
            ):
                continue
            selected[robot_id] = (sequence, readiness, status)
        if all(robot_id in selected for robot_id in robots):
            break
        time.sleep(poll_s)

    # Preserve the newest verifiable frame even when a sender did not produce
    # a post-arrival frame in time. The manifest keeps this fallback explicit
    # and official success remains false.
    for robot_id in robots:
        if robot_id in selected:
            continue
        readiness = latest_seen.get(robot_id) or client.readiness(robot_id)
        sequence = int(readiness.get("last_observation_sequence", -1))
        if sequence < 0:
            raise RuntimeError(f"{robot_id} has no terminal observation")
        verified_spool_observation(spool, robot_id, sequence)
        selected[robot_id] = (
            sequence,
            readiness,
            "fallback_latest_observation_no_post_arrival_proof",
        )

    records: dict[str, object] = {}
    for robot_id, robot in sorted(robots.items()):
        sequence, readiness, timing_status = selected[robot_id]
        metadata, metadata_path, rgb_path, depth_path = verified_spool_observation(
            spool, robot_id, sequence
        )
        robot_output = output / robot_id
        rgb_destination = robot_output / rgb_path.name
        depth_destination = robot_output / "depth.png"
        metadata_destination = robot_output / "metadata.json"
        atomic_copy_verified(rgb_path, rgb_destination)
        atomic_copy_verified(depth_path, depth_destination)
        atomic_copy_verified(metadata_path, metadata_destination)

        map_destination = robot_output / "map"
        map_source = resolve_workspace_path(WORKSPACE, robot.map_dir)
        map_copy_error: str | None = None
        for _ in range(5):
            try:
                stable_copy_map(map_source, map_destination)
                break
            except RuntimeError as exc:
                map_copy_error = str(exc)
                if map_destination.exists():
                    shutil.rmtree(map_destination)
                time.sleep(0.2)
        else:
            raise RuntimeError(
                f"could not freeze terminal map for {robot_id}: {map_copy_error}"
            )
        map_artifacts = [
            artifact_record(
                path,
                source_path=map_source / path.name,
                classification="source_derived_terminal_live_map_snapshot",
            )
            for path in sorted(map_destination.iterdir())
            if path.is_file()
        ]
        arrival = semantic_arrivals.get(robot_id)
        capture_delta_s = (
            None
            if arrival is None
            else (
                metadata.capture_time_ns
                - int(arrival.get("observed_at_ns", 0))
            )
            / 1e9
        )
        records[robot_id] = {
            "semantic_arrival": arrival,
            "observation_sequence": sequence,
            "observation_timing_status": timing_status,
            "capture_minus_arrival_s": capture_delta_s,
            "hub_observation_received_at_ns": int(
                readiness.get("last_observation_received_at_ns", 0)
            ),
            "rgb": artifact_record(
                rgb_destination,
                source_path=rgb_path,
                classification="observed_terminal_rgb_candidate",
            ),
            "depth": artifact_record(
                depth_destination,
                source_path=depth_path,
                classification="observed_terminal_aligned_depth",
            ),
            "metadata": artifact_record(
                metadata_destination,
                source_path=metadata_path,
                classification="observed_terminal_observation_metadata",
            ),
            "map_artifacts": map_artifacts,
            "independent_target_verified": False,
            "surveyed_goal_region_verified": False,
        }

    manifest = {
        "schema_version": "focus-terminal-evidence-bundle-v1",
        "created_at_ns": time.time_ns(),
        "status": (
            "complete_post_arrival_candidate"
            if all(
                record["observation_timing_status"]
                == "observed_post_arrival_hub_observation"
                for robot_id, record in records.items()
                if robot_id in semantic_arrivals
            )
            else "complete_with_explicit_terminal_timing_fallback"
        ),
        "robots": records,
        "official_success_verified": False,
        "remaining_verification": [
            "independent target annotation from the preserved terminal RGB",
            "final pose membership in the pre-surveyed valid goal region",
            "surveyed shortest-path evidence for SPL",
        ],
        "classification": (
            "observed/source-derived evidence bundle; model output is not "
            "independent target truth"
        ),
    }
    manifest_path = output / "terminal_evidence.json"
    atomic_write_json(manifest_path, manifest)
    return {
        "manifest": artifact_record(
            manifest_path,
            classification="source_derived_terminal_evidence_index",
        ),
        "status": manifest["status"],
        "robots": records,
    }


def freeze_next_round(
    *,
    session_path: Path,
    session: RealworldSession,
    output: Path,
    minimum_sequences: dict[str, int],
    max_input_age_s: float,
    max_sync_skew_s: float,
    timeout_s: float,
    poll_s: float,
    rejection_log: Path,
) -> dict[str, object]:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        try:
            return freeze(
                WORKSPACE,
                session_path,
                session,
                output,
                max_input_age_s=max_input_age_s,
                max_sync_skew_s=max_sync_skew_s,
                minimum_source_sequences=minimum_sequences,
            )
        except (
            FileNotFoundError,
            RuntimeError,
            ValueError,
            json.JSONDecodeError,
            OSError,
        ) as exc:
            last_error = f"{type(exc).__name__}: {exc}"
            append_jsonl(
                rejection_log,
                {"t_ns": time.time_ns(), "error": last_error},
            )
            # A latched map block is immutable for this map process/session.
            # Retrying it for the entire synchronization window cannot recover
            # and used to waste 45 seconds before the same controller HOLD.
            if "frozen map blocked:" in str(exc):
                raise RuntimeError(
                    f"non-recoverable round input: {exc}"
                ) from exc
            time.sleep(poll_s)
    raise TimeoutError(
        f"no fresh synchronized round input within {timeout_s:.1f}s: {last_error}"
    )


def run_shadow_round(
    *,
    session: RealworldSession,
    accepted: dict[str, object],
    accepted_dir: Path,
    state_path: Path,
    output: Path,
    goal_category: str,
    glm_url: str,
    hub_url: str,
    admin_token_file: Path,
    registry_state: Path,
    max_input_age_s: float,
    max_sync_skew_s: float,
) -> dict[str, object]:
    rows_raw = accepted.get("robots")
    if not isinstance(rows_raw, list):
        raise ValueError("frozen input manifest has no robot rows")
    rows = {
        str(item["robot_id"]): item
        for item in rows_raw
        if isinstance(item, dict) and "robot_id" in item
    }
    state = SourceEpisodeState.from_dict(
        json.loads(state_path.read_text(encoding="utf-8"))
    )
    command = [
        sys.executable,
        "-u",
        str(TOOLS_DIR / "live_vlm_shadow.py"),
    ]
    for robot in sorted(session.robots, key=lambda item: item.robot_id):
        row = rows.get(robot.robot_id)
        if not isinstance(row, dict):
            raise ValueError(f"frozen input lacks {robot.robot_id}")
        command.extend(
            [
                "--robot",
                f"{robot.robot_id}:{robot.name}:{accepted_dir / robot.name}",
                "--expected-source-sequence",
                f"{robot.robot_id}:{int(row['source_sequence'])}",
                "--expected-map-sha256",
                f"{robot.robot_id}:{row['map_sha256']}",
            ]
        )
    command.extend(
        [
            "--spool",
            str(resolve_workspace_path(WORKSPACE, session.runtime.spool_dir)),
            "--output",
            str(output),
            "--glm-url",
            glm_url,
            "--goal-category",
            goal_category,
            "--expected-shared-frame-calibration-id",
            session.calibration.calibration_id,
            "--realworld-session-id",
            session.session_id,
            "--realworld-session-contract-sha256",
            session_contract_sha256(session),
            "--source-step",
            str(state.source_step),
            "--scene-state-file",
            str(state_path),
            "--hub-url",
            hub_url,
            "--admin-token-file",
            str(admin_token_file),
            "--registry-state",
            str(registry_state),
            "--max-input-age-s",
            str(max_input_age_s),
            "--max-sync-skew-s",
            str(max_sync_skew_s),
            "--publish-hold",
            "--write-foxglove-targets",
        ]
    )
    for category in HM3D_CATEGORY_NAMES:
        command.extend(["--trusted-category", category])
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(HUB_DIR / "src")
    completed = subprocess.run(
        command,
        cwd=WORKSPACE,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    (output.parent / "vlm_stdout.log").write_text(
        completed.stdout, encoding="utf-8"
    )
    (output.parent / "vlm_stderr.log").write_text(
        completed.stderr, encoding="utf-8"
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"VLM round failed with exit {completed.returncode}; "
            f"see {output.parent / 'vlm_stderr.log'}"
        )
    manifest_path = output / "shadow_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "complete_shadow_only":
        raise RuntimeError("VLM round did not complete shadow-only")
    source = manifest.get("source_episode")
    if not isinstance(source, dict) or source.get("enabled") is not True:
        raise RuntimeError("VLM round silently fell back to one-shot mode")
    updated = SourceEpisodeState.from_dict(
        json.loads(state_path.read_text(encoding="utf-8"))
    )
    if updated.round_index != state.round_index + 1:
        raise RuntimeError("VLM round did not advance source state exactly once")
    return manifest


def prepare_output(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if resolved.exists():
        if any(resolved.iterdir()):
            raise FileExistsError(f"refusing non-empty episode output: {resolved}")
    else:
        resolved.mkdir(parents=True)
    return resolved


def frozen_shared_robot_positions(
    accepted: dict[str, object],
) -> tuple[
    dict[str, tuple[float, float]],
    dict[str, dict[str, object]],
    dict[str, str],
]:
    """Read source-derived base poses from the already frozen map snapshots."""

    positions: dict[str, tuple[float, float]] = {}
    provenance: dict[str, dict[str, object]] = {}
    errors: dict[str, str] = {}
    rows = accepted.get("robots")
    if not isinstance(rows, list):
        return positions, provenance, {"accepted_inputs": "robots is malformed"}
    for row in rows:
        if not isinstance(row, dict):
            continue
        robot_id = str(row.get("robot_id", ""))
        try:
            status_path = Path(str(row["map_dir"])) / "live_status.json"
            status = json.loads(status_path.read_text(encoding="utf-8"))
            if status.get("frame_id") != "shared_world":
                raise ValueError("live status is not in shared_world")
            xy = status.get("last_robot_xy_m")
            if (
                not isinstance(xy, list)
                or len(xy) != 2
                or isinstance(xy[0], bool)
                or isinstance(xy[1], bool)
                or not isinstance(xy[0], (int, float))
                or not isinstance(xy[1], (int, float))
            ):
                raise ValueError("last_robot_xy_m is missing or malformed")
            point = (float(xy[0]), float(xy[1]))
            if not all(math.isfinite(value) for value in point):
                raise ValueError("last_robot_xy_m is not finite")
            positions[robot_id] = point
            provenance[robot_id] = artifact_record(
                status_path,
                classification=(
                    "source_derived_frozen_shared_frame_robot_pose"
                ),
            )
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            errors[robot_id or "unknown"] = str(exc)[:300]
    return positions, provenance, errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-file", default="current")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument(
        "--goal-category",
        choices=SOURCE_HM3D_OBJECTNAV_GOALS,
        default="chair",
    )
    parser.add_argument("--hub-url", default="http://127.0.0.1:8188")
    parser.add_argument("--glm-url", default="http://127.0.0.1:31511/v1")
    parser.add_argument("--admin-token-file", type=Path, required=True)
    parser.add_argument(
        "--registry-state",
        type=Path,
        default=HUB_DIR / "runtime/state/registry_state.json",
    )
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--robot-0-min-sequence", type=int, default=0)
    parser.add_argument("--robot-1-min-sequence", type=int, default=0)
    parser.add_argument("--lease-s", type=float, default=8.0)
    parser.add_argument("--renew-before-s", type=float, default=3.0)
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--max-runtime-s", type=float, default=600.0)
    parser.add_argument("--round-input-timeout-s", type=float, default=300.0)
    parser.add_argument("--hold-ack-timeout-s", type=float, default=8.0)
    parser.add_argument("--terminal-evidence-timeout-s", type=float, default=10.0)
    parser.add_argument("--max-input-age-s", type=float, default=60.0)
    parser.add_argument("--max-sync-skew-s", type=float, default=5.0)
    parser.add_argument(
        "--route-conflict-min-separation-m",
        type=float,
        default=0.9,
        help=(
            "serialize concurrent physical goals when conservative shared-frame "
            "route corridors come closer than this distance"
        ),
    )
    parser.add_argument(
        "--robot-0-frontier-clearance-m",
        type=float,
        default=0.35,
        help="WSJ footprint clearance required inside a frontier arrival disk",
    )
    parser.add_argument(
        "--robot-1-frontier-clearance-m",
        type=float,
        default=0.34,
        help="Yunji footprint clearance required inside a frontier arrival disk",
    )
    parser.add_argument(
        "--max-rounds",
        type=int,
        default=source_decision_round_limit(),
    )
    parser.add_argument("--enable-live-goal-publication", action="store_true")
    parser.add_argument("--operator-confirmation", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.enable_live_goal_publication:
        print(
            "continuous source episode requires explicit "
            "--enable-live-goal-publication",
            file=sys.stderr,
        )
        return 2
    if args.operator_confirmation != LIVE_CONFIRMATION:
        print(
            f"live publication requires --operator-confirmation {LIVE_CONFIRMATION}",
            file=sys.stderr,
        )
        return 2
    if not 1.0 <= args.lease_s <= 10.0:
        raise ValueError("--lease-s must be between 1 and 10 seconds")
    if not 0.5 <= args.renew_before_s < args.lease_s:
        raise ValueError("--renew-before-s must be shorter than the lease")
    if any(
        value <= 0.0
        for value in (
            args.poll_s,
            args.max_runtime_s,
            args.round_input_timeout_s,
            args.hold_ack_timeout_s,
            args.terminal_evidence_timeout_s,
            args.max_input_age_s,
        )
    ) or args.max_sync_skew_s < 0.0:
        raise ValueError("runtime/freshness timeouts must be valid and positive")
    if not 1 <= args.max_rounds <= source_decision_round_limit():
        raise ValueError("max rounds exceeds the immutable source episode")
    if args.robot_0_min_sequence < 0 or args.robot_1_min_sequence < 0:
        raise ValueError("minimum source sequences must be non-negative")
    if not 0.5 <= args.route_conflict_min_separation_m <= 5.0:
        raise ValueError(
            "--route-conflict-min-separation-m must be between 0.5 and 5.0"
        )
    if not all(
        math.isfinite(value) and 0.15 <= value <= 0.75
        for value in (
            args.robot_0_frontier_clearance_m,
            args.robot_1_frontier_clearance_m,
        )
    ):
        raise ValueError(
            "frontier clearance must be finite and within [0.15, 0.75] m"
        )

    output = prepare_output(args.output)
    session_path = resolve_session_argument(args.session_file)
    session = load_session_file(session_path)
    validate_session(
        WORKSPACE,
        session,
        require_maps=True,
        require_current_code=True,
    )
    if args.goal_category != session.runtime.map_goal_category:
        raise ValueError(
            "live episode goal differs from the session map semantic target"
        )
    admin_token_file = args.admin_token_file.expanduser().resolve()
    admin_token = admin_token_file.read_text(encoding="utf-8").strip()
    if not admin_token:
        raise ValueError("admin token is empty")
    registry_state = args.registry_state.expanduser().resolve()
    robot_config = args.robot_config.expanduser().resolve()
    spool = resolve_workspace_path(WORKSPACE, session.runtime.spool_dir)
    robots = tuple(sorted(session.robots, key=lambda item: item.robot_id))
    robot_ids = tuple(item.robot_id for item in robots)

    state_path = output / "scene_state.json"
    state = SourceEpisodeState(
        scene_id=args.scene_id,
        goal_category=args.goal_category,
        shared_frame_calibration_id=session.calibration.calibration_id,
        robot_ids=robot_ids,
        source_find_goal={robot_id: False for robot_id in robot_ids},
    )
    atomic_write_json(state_path, state.to_dict())
    events_path = output / "controller_events.jsonl"
    scene_manifest_path = output / "scene_manifest.json"
    scene_manifest: dict[str, object] = {
        "schema_version": SCENE_SCHEMA_VERSION,
        "scene_id": args.scene_id,
        "episode_id": args.episode_id,
        "goal_category": args.goal_category,
        "session_id": session.session_id,
        "session_contract_sha256": session_contract_sha256(session),
        "started_at_ns": time.time_ns(),
        "status": "running",
        "source_contract": {
            "decision_clock": "0,24,49,...,499",
            "physical_clock_adapter": (
                "one fresh robot-local navigation feedback event per "
                "source-derived logical tick"
            ),
            "max_episode_steps": SOURCE_MAX_EPISODE_STEPS,
            "max_rounds": args.max_rounds,
            "target_override": (
                "largest connected positive goal-category semantic component"
            ),
        },
        "safety": {
            "hub_publishes_high_level_targets_only": True,
            "robot_retains_stop_and_rejection_authority": True,
            "lease_s": args.lease_s,
            "operator_confirmation": LIVE_CONFIRMATION,
            "route_conflict_guard": {
                "minimum_separation_m": (
                    args.route_conflict_min_separation_m
                ),
                "policy": (
                    "preserve source-derived VLM allocations; serialize "
                    "physical execution when straight shared-frame route "
                    "corridors overlap or a shared pose is unavailable"
                ),
            },
            "frontier_clearance_guard": {
                "robot_clearance_m": {
                    "robot-0": args.robot_0_frontier_clearance_m,
                    "robot-1": args.robot_1_frontier_clearance_m,
                },
                "policy": (
                    "preserve source/VLM selection in the candidate artifact; "
                    "withhold physical authority when no known-free "
                    "footprint-clear cell exists inside its arrival disk"
                ),
            },
        },
        "provenance": [
            artifact_record(
                Path(__file__),
                classification="source_derived_live_episode_orchestrator",
            ),
            artifact_record(
                TOOLS_DIR / "live_vlm_shadow.py",
                classification="source_derived_vlm_round_adapter",
            ),
            artifact_record(
                TOOLS_DIR / "freeze_realworld_inputs.py",
                classification="observed_input_freeze_adapter",
            ),
            artifact_record(
                WORKSPACE / "source/Focus_realworld/main.py",
                classification="immutable_authoritative_source",
            ),
            artifact_record(
                WORKSPACE / "source/Focus_realworld/agents/vlm_agents.py",
                classification="immutable_authoritative_source",
            ),
        ],
        "rounds": [],
    }
    atomic_write_json(scene_manifest_path, scene_manifest)

    client = EpisodeClient(args.hub_url, admin_token)
    current: DecisionBatchV2 | None = None
    epoch = -1
    publish_count = 0
    latest_events: dict[str, dict[str, object]] = {}
    semantic_arrivals_all: dict[str, dict[str, object]] = {}
    final_states: dict[str, dict[str, object]] = {}
    outcome = "aborted_before_publish"
    terminal_bundle: dict[str, object] | None = None
    overall_deadline = time.monotonic() + args.max_runtime_s

    def emit(event: str, **fields: object) -> None:
        append_jsonl(
            events_path,
            {"t_ns": time.time_ns(), "event": event, **fields},
        )

    def publish(
        batch: DecisionBatchV2,
        reason: str,
        *,
        expose_new_vlm_target: bool = False,
    ) -> None:
        nonlocal current, epoch, publish_count
        response = client.publish(batch)
        published_at_ns = time.time_ns()
        current = batch
        epoch = batch.decisions[0].coordination.execution_epoch
        publish_count += 1
        safe_reason = SAFE_REASON.sub("_", reason).strip("_") or "batch"
        atomic_write_json(
            output / f"batch_{publish_count:03d}_{safe_reason}.json",
            batch.model_dump(mode="json"),
        )
        foxglove_events = {}
        if expose_new_vlm_target:
            foxglove_events = write_foxglove_vlm_target_events(
                session,
                batch,
                publication_reason=reason,
                published_at_ns=published_at_ns,
            )
        emit(
            "batch_published",
            reason=reason,
            execution_epoch=epoch,
            decision_batch_id=batch.decisions[0].decision_batch_id,
            active_robot_ids=list(
                batch.decisions[0].coordination.active_robot_ids
            ),
            decision_ids=[item.decision_id for item in batch.decisions],
            foxglove_vlm_target_events=foxglove_events,
            response=response,
        )

    def transition(active: set[str], reason: str) -> DecisionBatchV2:
        nonlocal epoch
        if current is None:
            raise RuntimeError("cannot transition before the first batch")
        next_epoch = epoch + 1
        issued = time.time_ns()
        ordered = tuple(
            item.robot_id
            for item in current.decisions
            if item.robot_id in active
        )
        batch = next_coordination_batch(
            current,
            active_robot_ids=ordered,
            execution_epoch=next_epoch,
            issued_at_ns=issued,
            expires_at_ns=issued + int(args.lease_s * 1e9),
            identity_token=uuid.uuid4().hex[:8],
        )
        publish(batch, reason)
        return batch

    def hold_and_confirm(reason: str) -> dict[str, dict[str, object]]:
        hold = transition(set(), reason)
        states = wait_for_hold_ack(
            client,
            hold,
            timeout_s=args.hold_ack_timeout_s,
            poll_s=args.poll_s,
        )
        update_event_records(latest_events, states)
        emit("hold_acknowledged", reason=reason)
        return states

    def readiness_for(active: set[str]) -> dict[str, dict[str, object]]:
        reports = {
            robot_id: client.readiness(robot_id)
            for robot_id in sorted(active)
        }
        blocked = {
            robot_id: report
            for robot_id, report in reports.items()
            if report.get("ready_for_goal") is not True
        }
        if blocked:
            raise RuntimeError(
                "robot-local runtime readiness blocked GOAL: "
                + json.dumps(blocked, sort_keys=True)
            )
        return reports

    def monitor_round(
        *,
        target_found: bool,
        step_quota: int,
    ) -> RoundResult:
        nonlocal current, final_states
        if current is None:
            raise RuntimeError("round monitor has no current batch")
        active = set(current.decisions[0].coordination.active_robot_ids)
        semantic_robot_ids = {
            item.robot_id
            for item in current.decisions
            if item.target is not None and item.target.kind == "SEMANTIC_REGION"
        }
        if target_found and not semantic_robot_ids:
            return RoundResult(
                "failure",
                "source Find_Goal persisted without a semantic-region decision",
                {},
                {},
                {},
                {robot_id: 0 for robot_id in active},
            )
        seen_event_ids: dict[str, set[str]] = {
            robot_id: set() for robot_id in active
        }
        feedback_counts = {robot_id: 0 for robot_id in active}
        round_latest: dict[str, dict[str, object]] = {}
        while active:
            if time.monotonic() >= overall_deadline:
                final_states = hold_and_confirm("episode_runtime_timeout_hold")
                return RoundResult(
                    "failure",
                    "episode runtime timeout",
                    final_states,
                    {},
                    round_latest,
                    feedback_counts,
                )
            states = {
                item.robot_id: client.state(item.robot_id)
                for item in current.decisions
            }
            final_states = states
            update_event_records(latest_events, states)
            update_event_records(round_latest, states)
            decisions = {item.robot_id: item for item in current.decisions}
            for robot_id in active:
                event = states[robot_id].get("latest_event")
                if not isinstance(event, dict):
                    continue
                decision = decisions[robot_id]
                event_id = str(event.get("event_id", ""))
                status = str(event.get("status", ""))
                if (
                    event_id
                    and event_id not in seen_event_ids[robot_id]
                    and event.get("leg_id") == decision.leg_id
                    and status in ACTIVE_FEEDBACK
                ):
                    seen_event_ids[robot_id].add(event_id)
                    feedback_counts[robot_id] += 1
            inspection = inspect_round_states(states, current, active)
            if inspection.failures:
                recoverable = {
                    robot_id: event
                    for robot_id, event in inspection.failures.items()
                    if recoverable_frontier_failure(
                        decisions[robot_id], event
                    )
                }
                terminal = {
                    robot_id: event
                    for robot_id, event in inspection.failures.items()
                    if robot_id not in recoverable
                }
                if terminal:
                    failed_robot, event = next(iter(terminal.items()))
                    final_states = hold_and_confirm(
                        f"{failed_robot}_"
                        f"{str(event.get('status', '')).lower()}_hold"
                    )
                    return RoundResult(
                        "failure",
                        f"{failed_robot} {event.get('status')}: "
                        f"{event.get('reason_code')}",
                        final_states,
                        {},
                        round_latest,
                        feedback_counts,
                    )
                failed_frontiers = set(recoverable)
                active.difference_update(failed_frontiers)
                emit(
                    "frontier_failures_isolated",
                    failed_robot_ids=sorted(failed_frontiers),
                    failures={
                        robot_id: {
                            "status": event.get("status"),
                            "reason_code": event.get("reason_code"),
                            "decision_id": event.get("decision_id"),
                            "leg_id": event.get("leg_id"),
                        }
                        for robot_id, event in sorted(recoverable.items())
                    },
                    remaining_active_robot_ids=sorted(active),
                )
                if active:
                    transition(active, "frontier_failure_isolation")
                    continue
                final_states = hold_and_confirm(
                    "all_frontier_failures_replan_hold"
                )
                return RoundResult(
                    "replan",
                    "all active frontier legs were locally blocked; "
                    "continue with a fresh source round",
                    final_states,
                    {},
                    round_latest,
                    feedback_counts,
                )
            if inspection.semantic_arrivals:
                arrivals_with_receipts = {}
                for robot_id, event in inspection.semantic_arrivals.items():
                    row = dict(event)
                    row["_hub_received_at_ns"] = int(
                        states[robot_id].get("latest_event_received_at_ns", 0)
                    )
                    arrivals_with_receipts[robot_id] = row
                final_states = hold_and_confirm(
                    "semantic_arrival_episode_complete_hold"
                )
                return RoundResult(
                    "semantic_arrival",
                    "robot-local planner ARRIVED at a semantic-region leg",
                    final_states,
                    arrivals_with_receipts,
                    round_latest,
                    feedback_counts,
                )
            if inspection.frontier_arrivals:
                active.difference_update(inspection.frontier_arrivals)
                if not active:
                    final_states = hold_and_confirm(
                        "frontier_round_complete_hold"
                    )
                    return RoundResult(
                        "replan",
                        "all active frontier legs arrived",
                        final_states,
                        {},
                        round_latest,
                        feedback_counts,
                    )
                transition(active, "frontier_arrival_transition")
                continue
            if (
                not target_found
                and active
                and all(
                    feedback_counts.get(robot_id, 0) >= step_quota
                    for robot_id in active
                )
            ):
                final_states = hold_and_confirm("source_step_boundary_hold")
                return RoundResult(
                    "replan",
                    f"source-derived {step_quota}-tick round completed",
                    final_states,
                    {},
                    round_latest,
                    feedback_counts,
                )

            current_by_robot = {
                item.robot_id: item for item in current.decisions
            }
            expires_at_ns = min(
                current_by_robot[robot_id].expires_at_ns
                for robot_id in active
            )
            if (
                expires_at_ns - inspection.newest_server_time_ns
                <= int(args.renew_before_s * 1e9)
            ):
                if not inspection.current_feedback_ready:
                    final_states = hold_and_confirm("feedback_missing_hold")
                    return RoundResult(
                        "failure",
                        "current lease feedback was missing or stale",
                        final_states,
                        {},
                        round_latest,
                        feedback_counts,
                    )
                transition(active, "lease_renewal")
                continue
            time.sleep(args.poll_s)
        raise RuntimeError("round active set became empty without a terminal result")

    minimum_sequences = {
        "robot-0": args.robot_0_min_sequence,
        "robot-1": args.robot_1_min_sequence,
    }
    exit_code = 4
    try:
        emit("scene_started", session_id=session.session_id)
        for requested_round in range(args.max_rounds):
            if time.monotonic() >= overall_deadline:
                if current is not None:
                    final_states = hold_and_confirm(
                        "episode_runtime_timeout_before_round_hold"
                    )
                outcome = "failed_runtime_timeout_holding"
                break
            state_before = SourceEpisodeState.from_dict(
                json.loads(state_path.read_text(encoding="utf-8"))
            )
            if state_before.round_index != requested_round:
                raise RuntimeError("persistent source round index drifted")
            round_dir = output / (
                f"round_{state_before.round_index:02d}_"
                f"step_{state_before.source_step:03d}"
            )
            round_dir.mkdir()
            accepted_dir = round_dir / "accepted"
            accepted = freeze_next_round(
                session_path=session_path,
                session=session,
                output=accepted_dir,
                minimum_sequences=minimum_sequences,
                max_input_age_s=args.max_input_age_s,
                max_sync_skew_s=args.max_sync_skew_s,
                timeout_s=min(
                    args.round_input_timeout_s,
                    max(0.1, overall_deadline - time.monotonic()),
                ),
                poll_s=max(args.poll_s, 0.5),
                rejection_log=round_dir / "freeze_rejections.jsonl",
            )
            atomic_write_json(round_dir / "freeze_result.json", accepted)
            rows = {
                str(row["robot_id"]): row
                for row in accepted["robots"]
                if isinstance(row, dict)
            }
            minimum_sequences = {
                robot_id: int(row["source_sequence"]) + 1
                for robot_id, row in rows.items()
            }
            emit(
                "round_inputs_frozen",
                round_index=state_before.round_index,
                source_step=state_before.source_step,
                sequences={
                    robot_id: int(row["source_sequence"])
                    for robot_id, row in rows.items()
                },
            )
            shadow_dir = round_dir / "shadow"
            shadow_manifest = run_shadow_round(
                session=session,
                accepted=accepted,
                accepted_dir=accepted_dir,
                state_path=state_path,
                output=shadow_dir,
                goal_category=args.goal_category,
                glm_url=args.glm_url,
                hub_url=args.hub_url,
                admin_token_file=admin_token_file,
                registry_state=registry_state,
                max_input_age_s=args.max_input_age_s,
                max_sync_skew_s=args.max_sync_skew_s,
            )
            shadow_path = shadow_dir / "shadow_manifest.json"
            target_found = (
                shadow_manifest.get("source_episode_round_status")
                == "target_found_awaiting_robot_local_planner_stop"
            )
            step_quota = source_round_step_quota(shadow_manifest)
            next_epoch = epoch + 1
            built = build_batch_from_shadow_manifest(
                shadow_path,
                registry_state,
                scene_id=args.scene_id,
                episode_id=args.episode_id,
                execution_epoch=next_epoch,
                now_ns=time.time_ns(),
                robot_config_path=robot_config,
                lease_duration_ns=int(args.lease_s * 1e9),
            )
            atomic_write_json(
                round_dir / "controller_preflight.json", built.report
            )
            atomic_write_json(
                round_dir / "vlm_candidate_batch.json",
                built.batch.model_dump(mode="json"),
            )
            fused_snapshot = load_map_snapshot(
                shadow_dir / "fused_decision_map.npz"
            )
            if fused_snapshot is None:
                raise RuntimeError("shadow round lacks fused decision map")
            shared_positions, pose_provenance, pose_errors = (
                frozen_shared_robot_positions(accepted)
            )
            clearance_guarded_batch, frontier_clearance_guard = (
                apply_frontier_clearance_guard(
                    built.batch,
                    fused_snapshot,
                    clearance_by_robot_m={
                        "robot-0": args.robot_0_frontier_clearance_m,
                        "robot-1": args.robot_1_frontier_clearance_m,
                    },
                    fallback_frontiers=shadow_manifest.get(
                        "remaining_frontiers", []
                    ),
                    robot_xy_by_robot=shared_positions,
                )
            )
            atomic_write_json(
                round_dir / "frontier_clearance_guard.json",
                frontier_clearance_guard,
            )
            emit(
                "frontier_clearance_guard_evaluated",
                status=frontier_clearance_guard["status"],
                original_active_robot_ids=frontier_clearance_guard[
                    "original_active_robot_ids"
                ],
                effective_active_robot_ids=frontier_clearance_guard[
                    "effective_active_robot_ids"
                ],
                blocked_robot_ids=frontier_clearance_guard[
                    "blocked_robot_ids"
                ],
            )
            guarded_batch, route_guard = apply_route_conflict_guard(
                clearance_guarded_batch,
                shared_start_xy=shared_positions,
                minimum_separation_m=args.route_conflict_min_separation_m,
                priority_index=requested_round,
            )
            route_guard["pose_provenance"] = pose_provenance
            route_guard["pose_errors"] = pose_errors
            atomic_write_json(
                round_dir / "route_conflict_guard.json", route_guard
            )
            atomic_write_json(
                round_dir / "initial_batch.json",
                guarded_batch.model_dump(mode="json"),
            )
            emit(
                "route_conflict_guard_evaluated",
                status=route_guard["status"],
                minimum_predicted_separation_m=route_guard[
                    "minimum_predicted_separation_m"
                ],
                minimum_required_separation_m=route_guard[
                    "minimum_required_separation_m"
                ],
                original_active_robot_ids=route_guard[
                    "original_active_robot_ids"
                ],
                effective_active_robot_ids=route_guard[
                    "effective_active_robot_ids"
                ],
            )
            if built.report.get("preflight_ready") is not True:
                raise RuntimeError(
                    "round command preflight failed: "
                    + json.dumps(built.report.get("blockers"), sort_keys=True)
                )
            active = set(
                guarded_batch.decisions[0].coordination.active_robot_ids
            )
            if not active:
                publish(
                    guarded_batch,
                    f"round_{requested_round}_no_allocation_hold",
                    expose_new_vlm_target=True,
                )
                final_states = wait_for_hold_ack(
                    client,
                    guarded_batch,
                    timeout_s=args.hold_ack_timeout_s,
                    poll_s=args.poll_s,
                )
                outcome = "failed_no_safe_goal_allocation_holding"
                break
            readiness = readiness_for(active)
            atomic_write_json(round_dir / "runtime_readiness.json", readiness)
            publish(
                guarded_batch,
                f"round_{requested_round}_goal",
                expose_new_vlm_target=True,
            )
            round_result = monitor_round(
                target_found=target_found,
                step_quota=step_quota,
            )
            round_record = {
                "round_index": state_before.round_index,
                "source_step": state_before.source_step,
                "source_step_quota": step_quota,
                "target_found": target_found,
                "input_sequences": {
                    robot_id: int(row["source_sequence"])
                    for robot_id, row in rows.items()
                },
                "accepted_inputs": artifact_record(
                    accepted_dir / "accepted_inputs.json",
                    classification="observed_strict_frozen_input_manifest",
                ),
                "shadow_manifest": artifact_record(
                    shadow_path,
                    classification="source_derived_frozen_vlm_round",
                ),
                "controller_preflight": artifact_record(
                    round_dir / "controller_preflight.json",
                    classification="source_derived_v2_batch_preflight",
                ),
                "vlm_candidate_batch": artifact_record(
                    round_dir / "vlm_candidate_batch.json",
                    classification=(
                        "source_derived_unmodified_vlm_candidate_batch"
                    ),
                ),
                "frontier_clearance_guard": artifact_record(
                    round_dir / "frontier_clearance_guard.json",
                    classification=(
                        "source_derived_realworld_execution_guard"
                    ),
                ),
                "route_conflict_guard": artifact_record(
                    round_dir / "route_conflict_guard.json",
                    classification=(
                        "source_derived_realworld_execution_guard"
                    ),
                ),
                "feedback_counts": round_result.feedback_counts,
                "execution_status": round_result.status,
                "execution_reason": round_result.reason,
            }
            rounds = scene_manifest["rounds"]
            if not isinstance(rounds, list):
                raise RuntimeError("scene manifest rounds became malformed")
            rounds.append(round_record)
            atomic_write_json(scene_manifest_path, scene_manifest)
            emit("round_completed", **round_record)

            if round_result.status == "semantic_arrival":
                for robot_id, event in round_result.semantic_arrivals.items():
                    clean = dict(event)
                    clean.pop("_hub_received_at_ns", None)
                    semantic_arrivals_all[robot_id] = clean
                arrival_received = {
                    robot_id: int(event["_hub_received_at_ns"])
                    for robot_id, event in round_result.semantic_arrivals.items()
                }
                decision_inputs = {
                    robot_id: int(row["source_sequence"])
                    for robot_id, row in rows.items()
                }
                terminal_bundle = wait_and_seal_terminal_evidence(
                    client=client,
                    session=session,
                    spool=spool,
                    semantic_arrivals=semantic_arrivals_all,
                    arrival_received_at_ns=arrival_received,
                    decision_inputs=decision_inputs,
                    output=output / "terminal",
                    timeout_s=args.terminal_evidence_timeout_s,
                    poll_s=args.poll_s,
                )
                outcome = (
                    "completed_semantic_arrival_terminal_evidence_"
                    "captured_unverified"
                )
                exit_code = 0
                break
            if round_result.status == "failure":
                outcome = "failed_robot_or_controller_holding"
                break
            if requested_round == args.max_rounds - 1:
                outcome = "failed_source_max_steps_without_target_holding"
                break
        else:
            outcome = "failed_source_round_limit_without_target_holding"
    except KeyboardInterrupt:
        outcome = "operator_interrupted"
        if current is not None:
            try:
                final_states = hold_and_confirm("operator_interrupt_hold")
                outcome = "operator_interrupted_holding"
            except Exception as hold_exc:  # noqa: BLE001
                emit("hold_publish_failed", error=str(hold_exc)[:500])
    except Exception as exc:  # noqa: BLE001 - every controller fault fails closed
        outcome = f"controller_error_{type(exc).__name__}"
        emit("controller_error", error=str(exc)[:1000])
        if current is not None:
            try:
                final_states = hold_and_confirm("controller_error_hold")
                outcome += "_holding"
            except Exception as hold_exc:  # noqa: BLE001
                emit("hold_publish_failed", error=str(hold_exc)[:500])
    finally:
        client.close()

    evaluation_seed = evaluation_seed_from_events(
        latest_events, semantic_arrivals_all
    )
    scene_manifest["status"] = outcome
    scene_manifest["completed_at_ns"] = time.time_ns()
    scene_manifest["published_batches"] = publish_count
    scene_manifest["published_batch_artifacts"] = [
        artifact_record(
            path,
            classification="source_derived_published_v2_batch",
        )
        for path in sorted(output.glob("batch_*.json"))
    ]
    controller_event_artifact = artifact_record(
        events_path,
        classification="source_derived_controller_event_log",
    )
    scene_manifest["controller_event_log"] = controller_event_artifact
    scene_manifest["final_state"] = artifact_record(
        state_path,
        classification="source_derived_persistent_episode_state",
    )
    if terminal_bundle is not None:
        scene_manifest["terminal_evidence"] = terminal_bundle
    atomic_write_json(scene_manifest_path, scene_manifest)
    report = {
        "schema_version": RUN_SCHEMA_VERSION,
        "scene_id": args.scene_id,
        "episode_id": args.episode_id,
        "target_category": args.goal_category,
        "outcome": outcome,
        "high_level_batches_published": publish_count,
        "source_rounds_completed": len(scene_manifest["rounds"]),
        "live_goal_publication_enabled": True,
        "operator_confirmation": LIVE_CONFIRMATION,
        "robot_velocity_commands_sent_by_hub": False,
        "official_success_verified": False,
        "automatic_terminal_candidate_complete": (
            outcome
            == "completed_semantic_arrival_terminal_evidence_captured_unverified"
        ),
        "final_navigation_states": final_states,
        "evaluation_seed": evaluation_seed,
        "terminal_evidence": terminal_bundle,
        "evaluation_status": (
            "semantic ARRIVED and terminal evidence are automatic; independent "
            "target annotation, surveyed goal-region membership and shortest "
            "paths remain required before official SR/SPL"
        ),
        "scene_manifest": artifact_record(
            scene_manifest_path,
            classification="source_derived_continuous_scene_manifest",
        ),
        "controller_event_log": controller_event_artifact,
    }
    atomic_write_json(output / "episode_report.json", report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
