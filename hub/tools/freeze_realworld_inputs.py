#!/usr/bin/env python3
"""Freeze one strict, synchronized, command-capable two-robot input pair.

The live map daemons update their files atomically but independently.  This
tool copies each map only while its source files remain unchanged, validates
the persistent session contract and the exact YOLO source observation, then
renames the completed directory into place.  It reads local Hub artifacts
only and has no robot, ROS, WATER, TinyNav, or command interface.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[2]
HUB_DIR = WORKSPACE / "hub"
sys.path.insert(0, str(HUB_DIR / "src"))
sys.path.insert(0, str(HUB_DIR / "tools"))

from focus_hub.map_snapshot import (  # noqa: E402
    MapSnapshot,
    load_map_snapshot,
    validate_fusion_contract,
)
from focus_hub.models import (  # noqa: E402
    LocalizationState,
    ObservationMetadata,
    SafetyState,
)
from focus_hub.realworld_session import (  # noqa: E402
    RealworldSession,
    expected_map_session_contract,
    load_session_file,
    resolve_workspace_path,
    session_contract_sha256,
    validate_session,
)
from manage_realworld_session import resolve_session_argument  # noqa: E402


REQUIRED_MAP_FILES = (
    "central_map.npz",
    "map_summary.json",
    "live_status.json",
    "map_session_contract.json",
)

WSJ_MAPPING_HEALTH_DETAIL = (
    "slam_optimizer_imu_valid;covariance_unavailable"
)
WSJ_RECOVERED_MAPPING_HEALTH = re.compile(
    r"^slam_optimizer_imu_valid_after_overwrite_recovery:"
    r"[1-9][0-9]*;covariance_unavailable$"
)


def mapping_health_classification(
    robot_id: str,
    metadata: ObservationMetadata,
) -> str:
    """Require pose health appropriate to freezing perception inputs.

    Command readiness and map-input readiness are deliberately separate.
    TinyNav currently publishes an all-zero odometry covariance, so the WSJ
    sender correctly refuses to claim command-ready TRACKING even when its
    independently checked optimizer/IMU telemetry passes.  The armed WSJ
    receiver remains the sole authority that may later post READY command
    health; accepting this exact fail-closed sender state here cannot enable
    motion.
    """

    health = metadata.health
    if health.ready_for_goal():
        return "command_ready"
    wsj_mapping_health = (
        robot_id == "robot-0"
        and health.safety_state == SafetyState.UNKNOWN
        and health.localization_state == LocalizationState.DEGRADED
        and not health.estop_engaged
        and not health.collision_avoidance_ready
        and not health.motor_controller_ready
    )
    if wsj_mapping_health and health.detail == WSJ_MAPPING_HEALTH_DETAIL:
        return "tinynav_optimizer_imu_valid_covariance_unavailable"
    if wsj_mapping_health and WSJ_RECOVERED_MAPPING_HEALTH.fullmatch(
        health.detail
    ):
        return (
            "tinynav_optimizer_imu_valid_after_stable_"
            "overwrite_recovery"
        )
    raise ValueError(
        f"{robot_id} source health is not ready for strict mapping: "
        f"{health.detail or health.localization_state.value}"
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact(
    path: Path,
    *,
    classification: str,
    recorded_path: Path | None = None,
) -> dict[str, object]:
    return {
        "path": str((recorded_path or path).resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "classification": classification,
    }


def stable_copy_map(source: Path, destination: Path) -> None:
    """Copy one snapshot while proving none of its source files changed."""
    destination.mkdir(parents=True)
    sources = [source / name for name in REQUIRED_MAP_FILES]
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError(path)
    before = {}
    for path in sources:
        stat = path.stat()
        before[path.name] = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
    before_json = {
        path.name: path.read_bytes()
        for path in sources
        if path.suffix == ".json"
    }
    for path in sources:
        shutil.copy2(path, destination / path.name)
    after = {}
    for path in sources:
        stat = path.stat()
        after[path.name] = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
    after_json = {
        path.name: path.read_bytes()
        for path in sources
        if path.suffix == ".json"
    }
    if before != after or before_json != after_json:
        raise RuntimeError(f"map changed while freezing: {source}")
    for name, expected in before_json.items():
        if (destination / name).read_bytes() != expected:
            raise RuntimeError(f"frozen map copy differs: {destination / name}")


def validate_frozen_robot(
    session: RealworldSession,
    robot_id: str,
    frozen_dir: Path,
    final_dir: Path,
    spool: Path,
    *,
    now_ns: int,
    max_input_age_s: float,
    minimum_source_sequence: int,
) -> tuple[dict[str, object], MapSnapshot, ObservationMetadata]:
    robots = {robot.robot_id: robot for robot in session.robots}
    robot = robots[robot_id]
    summary = json.loads(
        (frozen_dir / "map_summary.json").read_text(encoding="utf-8")
    )
    status = json.loads(
        (frozen_dir / "live_status.json").read_text(encoding="utf-8")
    )
    map_contract = json.loads(
        (frozen_dir / "map_session_contract.json").read_text(
            encoding="utf-8"
        )
    )
    snapshot = load_map_snapshot(frozen_dir / "central_map.npz")
    if snapshot is None:
        raise RuntimeError(f"frozen map disappeared: {frozen_dir}")
    if map_contract != expected_map_session_contract(session, robot):
        raise ValueError(f"{robot_id} frozen map session contract mismatch")
    for payload, label in ((summary, "summary"), (status, "status")):
        if payload.get("robot_id") != robot_id:
            raise ValueError(f"{robot_id} frozen {label} identity mismatch")
        if payload.get("frame_id") != "shared_world":
            raise ValueError(f"{robot_id} frozen {label} frame mismatch")
        if payload.get("transform_version") != robot.transform_version:
            raise ValueError(f"{robot_id} frozen {label} transform mismatch")
        if (
            payload.get("shared_frame_calibration_id")
            != session.calibration.calibration_id
        ):
            raise ValueError(f"{robot_id} frozen {label} calibration mismatch")
        if payload.get("mapping_blocked_reason") is not None:
            raise ValueError(
                f"{robot_id} frozen map blocked: "
                f"{payload.get('mapping_blocked_reason')}"
            )
    if snapshot.frame_id != "shared_world":
        raise ValueError(f"{robot_id} map snapshot frame mismatch")
    if snapshot.transform_version != robot.transform_version:
        raise ValueError(f"{robot_id} map snapshot transform mismatch")
    if (
        snapshot.shared_frame_calibration_id
        != session.calibration.calibration_id
    ):
        raise ValueError(f"{robot_id} map snapshot calibration mismatch")
    if not snapshot.snapshot_id or summary.get("snapshot_id") != snapshot.snapshot_id:
        raise ValueError(
            f"{robot_id} map/summary snapshot generation is absent or mismatched"
        )

    semantic = summary.get("semantic_mapping")
    yolo = (
        semantic.get("yolo_reinforcement")
        if isinstance(semantic, dict)
        else None
    )
    if not isinstance(yolo, dict) or yolo.get("enabled") is not True:
        raise ValueError(f"{robot_id} has no enabled YOLO evidence")
    sequence = int(yolo.get("last_sequence", -1))
    if sequence <= robot.map_start_after_sequence:
        raise ValueError(f"{robot_id} source sequence predates this session")
    if sequence < minimum_source_sequence:
        raise ValueError(
            f"{robot_id} source sequence {sequence} predates the current Hub "
            f"epoch minimum {minimum_source_sequence}"
        )
    source_dir = spool / robot_id / f"{sequence:020d}"
    metadata_path = source_dir / "metadata.json"
    metadata = ObservationMetadata.model_validate_json(
        metadata_path.read_text(encoding="utf-8")
    )
    if metadata.robot_id != robot_id or metadata.sequence != sequence:
        raise ValueError(f"{robot_id} source observation identity mismatch")
    if metadata.pose.transform_version != robot.transform_version:
        raise ValueError(f"{robot_id} source observation transform mismatch")
    if metadata.mapping_only or metadata.base_T_camera is None:
        raise ValueError(f"{robot_id} source observation is not command-capable")
    mapping_health = mapping_health_classification(robot_id, metadata)
    age_s = (now_ns - metadata.capture_time_ns) / 1e9
    if age_s < -0.25 or age_s > max_input_age_s:
        raise ValueError(
            f"{robot_id} source age {age_s:.3f}s is outside the strict window"
        )

    rgb_name = "rgb.jpg" if metadata.rgb_encoding == "jpeg" else "rgb.png"
    rgb_path = source_dir / rgb_name
    depth_path = source_dir / "depth.png"
    for path, expected_size, expected_hash in (
        (rgb_path, metadata.rgb_size_bytes, metadata.rgb_sha256),
        (depth_path, metadata.depth_size_bytes, metadata.depth_sha256),
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
        if path.stat().st_size != expected_size:
            raise ValueError(f"source payload size mismatch: {path}")
        if sha256_file(path) != expected_hash:
            raise ValueError(f"source payload hash mismatch: {path}")

    record = {
        "robot_id": robot_id,
        "name": robot.name,
        "map_dir": str(final_dir.resolve()),
        "map_sha256": sha256_file(frozen_dir / "central_map.npz"),
        "source_sequence": sequence,
        "source_capture_time_ns": metadata.capture_time_ns,
        "source_age_s": age_s,
        "source_mapping_health": {
            "classification": mapping_health,
            "command_ready": metadata.health.ready_for_goal(),
            "localization_state": metadata.health.localization_state.value,
            "detail": metadata.health.detail,
        },
        "source_metadata": artifact(
            metadata_path,
            classification="observed_append_only_hub_spool_metadata",
        ),
        "source_rgb": artifact(
            rgb_path,
            classification="observed_append_only_hub_spool_rgb",
        ),
        "source_depth": artifact(
            depth_path,
            classification="observed_append_only_hub_spool_depth",
        ),
        "frozen_map_artifacts": [
            artifact(
                frozen_dir / name,
                classification="source_derived_frozen_session_map_input",
                recorded_path=final_dir / name,
            )
            for name in REQUIRED_MAP_FILES
        ],
    }
    return record, snapshot, metadata


def freeze(
    workspace: Path,
    session_path: Path,
    session: RealworldSession,
    output: Path,
    *,
    max_input_age_s: float,
    max_sync_skew_s: float,
    minimum_source_sequences: dict[str, int] | None = None,
    now_ns: int | None = None,
) -> dict[str, object]:
    if output.exists():
        raise FileExistsError(f"refusing to replace frozen input: {output}")
    validate_session(
        workspace,
        session,
        require_maps=True,
        require_current_code=True,
    )
    runtime = session.runtime
    spool = resolve_workspace_path(workspace, runtime.spool_dir)
    timestamp_ns = time.time_ns() if now_ns is None else now_ns
    minima = minimum_source_sequences or {}
    temporary = output.with_name(f".{output.name}.{os.getpid()}.tmp")
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir(parents=True)
    try:
        records: list[dict[str, object]] = []
        snapshots: list[MapSnapshot] = []
        metadata_rows: list[ObservationMetadata] = []
        for robot in sorted(session.robots, key=lambda item: item.robot_id):
            source = resolve_workspace_path(workspace, robot.map_dir)
            destination = temporary / robot.name
            stable_copy_map(source, destination)
            record, snapshot, metadata = validate_frozen_robot(
                session,
                robot.robot_id,
                destination,
                output / robot.name,
                spool,
                now_ns=timestamp_ns,
                max_input_age_s=max_input_age_s,
                minimum_source_sequence=minima.get(robot.robot_id, 0),
            )
            records.append(record)
            snapshots.append(snapshot)
            metadata_rows.append(metadata)
        validate_fusion_contract(snapshots)
        skew_s = (
            max(item.capture_time_ns for item in metadata_rows)
            - min(item.capture_time_ns for item in metadata_rows)
        ) / 1e9
        if skew_s > max_sync_skew_s:
            raise ValueError(
                f"cross-robot source skew {skew_s:.3f}s exceeds "
                f"{max_sync_skew_s:.3f}s"
            )
        payload = {
            "schema_version": "focus-realworld-frozen-input-v1",
            "accepted_at_ns": timestamp_ns,
            "session_file": str(session_path.resolve()),
            "session_id": session.session_id,
            "session_contract_sha256": session_contract_sha256(session),
            "max_input_age_s": max_input_age_s,
            "max_sync_skew_s": max_sync_skew_s,
            "observed_sync_skew_s": skew_s,
            "robots": records,
            "safety": {
                "local_artifacts_read_only": True,
                "robot_interfaces_used": False,
                "robot_commands_issued": False,
                "strict_mapping_health": True,
                "command_health_deferred_to_live_receiver": True,
            },
        }
        manifest_path = temporary / "accepted_inputs.json"
        manifest_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, output)
        return payload
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-file", default="current")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-input-age-s", type=float, default=60.0)
    parser.add_argument("--max-sync-skew-s", type=float, default=5.0)
    parser.add_argument("--robot-0-min-sequence", type=int, default=0)
    parser.add_argument("--robot-1-min-sequence", type=int, default=0)
    args = parser.parse_args()
    if args.max_input_age_s <= 0 or args.max_sync_skew_s < 0:
        parser.error("input age must be positive and skew must be non-negative")
    if args.robot_0_min_sequence < 0 or args.robot_1_min_sequence < 0:
        parser.error("minimum source sequences must be non-negative")
    try:
        session_path = resolve_session_argument(args.session_file)
        session = load_session_file(session_path)
        payload = freeze(
            WORKSPACE,
            session_path,
            session,
            args.output.expanduser().resolve(),
            max_input_age_s=args.max_input_age_s,
            max_sync_skew_s=args.max_sync_skew_s,
            minimum_source_sequences={
                "robot-0": args.robot_0_min_sequence,
                "robot-1": args.robot_1_min_sequence,
            },
        )
    except (
        FileExistsError,
        FileNotFoundError,
        RuntimeError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        print(f"input freeze rejected: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
