"""Convert one frozen source-faithful VLM shadow round into a v2 batch.

This module performs no network I/O.  It verifies every frozen artifact,
reconstructs the exact accepted observation identities, and emits a strict
two-robot high-level decision candidate plus an explicit readiness report.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import cv2
import numpy as np

from .map_snapshot import MapSnapshot, load_map_snapshot
from .models import ObservationMetadata
from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2


@dataclass(frozen=True)
class SceneBatchBuild:
    batch: DecisionBatchV2
    report: dict[str, object]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON object required: {path}")
    return payload


def _portable_path(raw_path: object, manifest_dir: Path) -> Path:
    declared = Path(str(raw_path)).expanduser()
    if declared.is_file():
        return declared.resolve()
    parts = declared.parts
    if manifest_dir.name in parts:
        index = parts.index(manifest_dir.name)
        relocated = manifest_dir.joinpath(*parts[index + 1 :])
        if relocated.is_file():
            return relocated.resolve()
    for candidate in (
        manifest_dir / declared.name,
        manifest_dir / "source_goal_masks" / declared.name,
    ):
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"declared artifact is unavailable: {declared}")


def _verify_artifact(
    record: dict[str, Any], manifest_dir: Path
) -> tuple[Path, dict[str, object]]:
    path = _portable_path(record.get("preserved_path"), manifest_dir)
    actual_size = path.stat().st_size
    actual_sha = sha256_file(path)
    if actual_size != int(record.get("size_bytes", -1)):
        raise ValueError(f"artifact size mismatch: {path}")
    if actual_sha != str(record.get("sha256", "")):
        raise ValueError(f"artifact SHA-256 mismatch: {path}")
    return path, {
        "source_path": record.get("source_path"),
        "preserved_path": str(path),
        "size_bytes": actual_size,
        "sha256": actual_sha,
        "classification": record.get("status", "unverified classification"),
        "verification": "locally observed bytes match manifest",
    }


def _bounded_id(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.:-]+", "-", value).strip("-")
    if not cleaned:
        raise ValueError("decision identity became empty")
    if len(cleaned) <= 128:
        return cleaned
    suffix = hashlib.sha256(cleaned.encode("utf-8")).hexdigest()[:16]
    return f"{cleaned[:111]}-{suffix}"


def _registry_entries(registry_state: dict[str, Any]) -> dict[str, dict[str, Any]]:
    entries = registry_state.get("robots")
    if not isinstance(entries, dict):
        raise ValueError("registry state has no robots object")
    return entries


def _policy_entries(robot_config: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if robot_config is None:
        return {}
    entries = robot_config.get("robots")
    if not isinstance(entries, dict):
        raise ValueError("robot config has no robots object")
    return entries


def _input_paths(
    records: list[tuple[dict[str, Any], Path, dict[str, object]]],
    *,
    name: str,
    sequence: int,
) -> tuple[Path, Path, Path, Path, list[dict[str, object]]]:
    matches = [item for item in records if item[1].parent.name == name]
    by_name = {item[1].name: item for item in matches}
    metadata_name = f"source_{sequence}_metadata.json"
    depth_name = f"source_{sequence}_depth.png"
    rgb_candidates = (f"source_{sequence}.jpg", f"source_{sequence}.png")
    if "central_map.npz" not in by_name or metadata_name not in by_name:
        raise ValueError(f"{name} frozen map/metadata artifacts are missing")
    if depth_name not in by_name:
        raise ValueError(f"{name} frozen depth artifact is missing")
    rgb_item = next((by_name[value] for value in rgb_candidates if value in by_name), None)
    if rgb_item is None:
        raise ValueError(f"{name} frozen RGB artifact is missing")
    selected = [
        by_name["central_map.npz"],
        by_name[metadata_name],
        by_name[depth_name],
        rgb_item,
    ]
    return (
        selected[0][1],
        selected[1][1],
        selected[3][1],
        selected[2][1],
        [item[2] for item in selected],
    )


def _observation_identity(
    metadata_path: Path, rgb_path: Path, depth_path: Path
) -> tuple[ObservationMetadata, dict[str, object]]:
    metadata = ObservationMetadata.model_validate(_load_json(metadata_path))
    rgb_sha = sha256_file(rgb_path)
    depth_sha = sha256_file(depth_path)
    if rgb_path.stat().st_size != metadata.rgb_size_bytes:
        raise ValueError(f"RGB byte count differs from metadata: {rgb_path}")
    if depth_path.stat().st_size != metadata.depth_size_bytes:
        raise ValueError(f"depth byte count differs from metadata: {depth_path}")
    if rgb_sha != metadata.rgb_sha256:
        raise ValueError(f"RGB hash differs from metadata: {rgb_path}")
    if depth_sha != metadata.depth_sha256:
        raise ValueError(f"depth hash differs from metadata: {depth_path}")
    payload_sha = hashlib.sha256(
        metadata.model_dump_json().encode("utf-8")
        + rgb_sha.encode("ascii")
        + depth_sha.encode("ascii")
    ).hexdigest()
    return metadata, {
        "sequence": metadata.sequence,
        "capture_time_ns": metadata.capture_time_ns,
        "payload_sha256": payload_sha,
    }


def _validate_snapshot(
    path: Path, *, expected_sha: str, expected_transform: str
) -> MapSnapshot:
    if sha256_file(path) != expected_sha:
        raise ValueError(f"robot map hash differs from VLM result: {path}")
    snapshot = load_map_snapshot(path)
    if snapshot is None:
        raise RuntimeError(f"map snapshot disappeared: {path}")
    if snapshot.map_format_version != "focus-hub-central-map-v3":
        raise ValueError(f"unsupported map format: {snapshot.map_format_version}")
    if snapshot.frame_id != "shared_world":
        raise ValueError(f"map frame is not shared_world: {path}")
    if not math.isclose(snapshot.resolution_m, 0.05, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError(f"map resolution is not 0.05 m: {path}")
    if snapshot.transform_version != expected_transform:
        raise ValueError(f"map transform differs from VLM result: {path}")
    if not snapshot.shared_frame_calibration_id:
        raise ValueError(f"map lacks shared calibration identity: {path}")
    return snapshot


def _semantic_target(
    selection: dict[str, Any],
    *,
    robot_id: str,
    manifest_dir: Path,
    snapshot: MapSnapshot,
) -> tuple[dict[str, object], dict[str, object]]:
    mask_path = _portable_path(selection.get("mask_path"), manifest_dir)
    png = mask_path.read_bytes()
    if len(png) != int(selection.get("mask_size_bytes", -1)):
        raise ValueError(f"semantic mask size mismatch: {mask_path}")
    png_sha = hashlib.sha256(png).hexdigest()
    if png_sha != str(selection.get("mask_sha256", "")):
        raise ValueError(f"semantic mask hash mismatch: {mask_path}")
    mask = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if mask is None or mask.ndim != 2 or mask.dtype != np.uint8:
        raise ValueError(f"semantic mask is not grayscale uint8: {mask_path}")
    if mask.shape != snapshot.grid.shape[1:]:
        raise ValueError("semantic mask shape differs from its source map")
    if not set(int(value) for value in np.unique(mask)).issubset({0, 255}):
        raise ValueError("semantic mask contains non-binary pixels")
    component_size = int(np.count_nonzero(mask == 255))
    if component_size != int(selection.get("size_cells", -1)):
        raise ValueError("semantic mask cell count differs from VLM record")
    target = {
        "kind": "SEMANTIC_REGION",
        "category": str(selection["category"]),
        "source_robot_id": robot_id,
        "evidence_status": "model_inference_map_projected_unverified",
        "source_goal_dilation_cells": 10,
        "region": {
            "frame_id": "shared_world",
            "origin_xy_m": list(snapshot.origin_xy_m),
            "resolution_m": snapshot.resolution_m,
            "height": int(mask.shape[0]),
            "width": int(mask.shape[1]),
            "row_axis": "+y",
            "column_axis": "+x",
            "encoding": "png_u8_0_255_base64",
            "component_size_cells": component_size,
            "payload_size_bytes": len(png),
            "payload_sha256": png_sha,
            "payload_base64": base64.b64encode(png).decode("ascii"),
        },
        "display_centroid": {
            "frame_id": "shared_world",
            "x": float(selection["x_m"]),
            "y": float(selection["y_m"]),
            "authority": "display_only",
        },
    }
    artifact = {
        "source_path": str(selection.get("mask_path")),
        "preserved_path": str(mask_path),
        "size_bytes": len(png),
        "sha256": png_sha,
        "classification": str(
            selection.get(
                "evidence_status", "model inference map projected unverified"
            )
        ),
        "verification": "locally observed bytes, shape, binary values and cell count",
    }
    return target, artifact


def _frontier_target(
    selection: dict[str, Any], robot_result: dict[str, Any]
) -> dict[str, object]:
    robot_xy = robot_result.get("robot_xy_m")
    if not isinstance(robot_xy, list) or len(robot_xy) != 2:
        raise ValueError("VLM robot result has no shared-world robot position")
    x_m = float(selection["x_m"])
    y_m = float(selection["y_m"])
    yaw = math.atan2(y_m - float(robot_xy[1]), x_m - float(robot_xy[0]))
    return {
        "kind": "FRONTIER_POINT",
        "frontier_id": str(selection["target_id"]),
        "source_goal_dilation_cells": 10,
        "pose": {
            "frame_id": "shared_world",
            "x": x_m,
            "y": y_m,
            "z": 0.0,
            "yaw_rad": yaw,
        },
    }


def build_batch_from_shadow_manifest(
    manifest_path: Path | str,
    registry_state_path: Path | str,
    *,
    scene_id: str,
    episode_id: str,
    execution_epoch: int,
    now_ns: int,
    robot_config_path: Path | str | None = None,
    lease_duration_ns: int = 8_000_000_000,
) -> SceneBatchBuild:
    """Build and preflight a two-robot v2 batch without publishing it."""

    manifest_path = Path(manifest_path).expanduser().resolve()
    registry_state_path = Path(registry_state_path).expanduser().resolve()
    manifest = _load_json(manifest_path)
    registry_state = _load_json(registry_state_path)
    robot_config = (
        None
        if robot_config_path is None
        else _load_json(Path(robot_config_path).expanduser().resolve())
    )
    if manifest.get("status") != "complete_shadow_only":
        raise ValueError("only a complete_shadow_only manifest can become a v2 batch")
    safety = manifest.get("safety")
    if not isinstance(safety, dict) or safety.get("robot_commands_sent") is not False:
        raise ValueError("shadow manifest has no explicit no-motion safety record")
    robot_results_raw = manifest.get("robots")
    if not isinstance(robot_results_raw, list) or len(robot_results_raw) != 2:
        raise ValueError("shadow manifest must contain exactly two robot results")
    robot_results = {
        str(record["robot_id"]): record for record in robot_results_raw
    }
    if len(robot_results) != 2:
        raise ValueError("shadow manifest robot IDs are not unique")
    robot_ids = tuple(record["robot_id"] for record in robot_results_raw)
    registry_entries = _registry_entries(registry_state)
    if set(robot_ids) != set(registry_entries):
        raise ValueError("manifest and registry robot sets differ")
    policies = _policy_entries(robot_config)

    input_records_raw = manifest.get("input_artifacts")
    if not isinstance(input_records_raw, list):
        raise ValueError("shadow manifest has no input artifact list")
    verified_records: list[
        tuple[dict[str, Any], Path, dict[str, object]]
    ] = []
    for raw in input_records_raw:
        if not isinstance(raw, dict):
            raise ValueError("input artifact record is malformed")
        path, verified = _verify_artifact(raw, manifest_path.parent)
        verified_records.append((raw, path, verified))

    blockers: list[dict[str, object]] = []
    unverified: list[dict[str, object]] = []
    observations: dict[str, dict[str, object]] = {}
    metadata_by_robot: dict[str, ObservationMetadata] = {}
    snapshots: dict[str, MapSnapshot] = {}
    map_paths: dict[str, Path] = {}
    verified_artifacts: list[dict[str, object]] = []

    for robot_id in robot_ids:
        result = robot_results[robot_id]
        name = str(result["name"])
        sequence = int(result["source_sequence"])
        map_path, metadata_path, rgb_path, depth_path, selected_artifacts = _input_paths(
            verified_records,
            name=name,
            sequence=sequence,
        )
        verified_artifacts.extend(selected_artifacts)
        metadata, identity = _observation_identity(metadata_path, rgb_path, depth_path)
        if metadata.robot_id != robot_id or metadata.sequence != sequence:
            raise ValueError(f"{robot_id} frozen metadata identity mismatch")
        expected_transform = str(result["map_transform_version"])
        snapshot = _validate_snapshot(
            map_path,
            expected_sha=str(result["map_snapshot_sha256"]),
            expected_transform=expected_transform,
        )
        if snapshot.shared_frame_calibration_id != manifest.get(
            "shared_frame_calibration_id"
        ):
            raise ValueError(f"{robot_id} map calibration differs from manifest")
        observations[robot_id] = identity
        metadata_by_robot[robot_id] = metadata
        snapshots[robot_id] = snapshot
        map_paths[robot_id] = map_path

        if metadata.mapping_only:
            blockers.append({
                "code": "INPUT_MAPPING_ONLY",
                "robot_id": robot_id,
                "detail": "frozen v1 observation cannot authorize GOAL",
            })
        if metadata.base_T_camera is None:
            blockers.append({
                "code": "BASE_T_CAMERA_ABSENT",
                "robot_id": robot_id,
                "detail": "measured base-to-camera extrinsic is absent",
            })
        if metadata.health.estop_engaged:
            blockers.append({
                "code": "FROZEN_ESTOP_ENGAGED",
                "robot_id": robot_id,
                "detail": metadata.health.detail,
            })
        elif metadata.health.localization_state.value == "LOST":
            blockers.append({
                "code": "FROZEN_LOCALIZATION_LOST",
                "robot_id": robot_id,
                "detail": metadata.health.detail,
            })
        elif not metadata.health.ready_for_goal():
            unverified.append({
                "code": "RUNTIME_HEALTH_RECHECK_REQUIRED",
                "robot_id": robot_id,
                "detail": (
                    "frozen perception health was not command-ready; live "
                    "publication must prove a fresh robot-receiver heartbeat"
                ),
            })
        blocked_reason = result.get("input_mapping_blocked_reason")
        if blocked_reason:
            blockers.append({
                "code": "MAPPING_BLOCKED",
                "robot_id": robot_id,
                "detail": str(blocked_reason),
            })

        persisted = registry_entries[robot_id]
        persisted_sequence = int(persisted.get("last_sequence", -1))
        if persisted_sequence < sequence:
            blockers.append({
                "code": "REGISTRY_SEQUENCE_BEHIND",
                "robot_id": robot_id,
                "detail": f"persisted={persisted_sequence}, required={sequence}",
            })
        elif persisted_sequence == sequence:
            if persisted.get("last_payload_digest") != identity["payload_sha256"]:
                blockers.append({
                    "code": "REGISTRY_PAYLOAD_MISMATCH",
                    "robot_id": robot_id,
                    "detail": "same sequence has a different accepted digest",
                })
        else:
            unverified.append({
                "code": "IN_MEMORY_HISTORY_REQUIRED",
                "robot_id": robot_id,
                "detail": (
                    f"persisted latest={persisted_sequence}; Hub API must confirm "
                    f"historical sequence={sequence} remains in memory"
                ),
            })

        policy = policies.get(robot_id)
        if policy is not None:
            if not bool(policy.get("allow_goal", False)):
                blockers.append({
                    "code": "GOAL_POLICY_DISABLED",
                    "robot_id": robot_id,
                    "detail": "robot config allow_goal is false",
                })
            configured_transform = str(policy.get("transform_version", ""))
            if configured_transform != expected_transform:
                blockers.append({
                    "code": "POLICY_TRANSFORM_MISMATCH",
                    "robot_id": robot_id,
                    "detail": (
                        f"configured={configured_transform!r}, "
                        f"required={expected_transform!r}"
                    ),
                })

    capture_times = [int(observations[robot_id]["capture_time_ns"]) for robot_id in robot_ids]
    oldest_age_ns = now_ns - min(capture_times)
    capture_skew_ns = max(capture_times) - min(capture_times)
    # Must exceed the observed 29-30 s real GLM cascade plus the time needed
    # to obtain a synchronized dual-robot input pair.  This is provenance
    # freshness only; physical authority still expires every 8 seconds.
    if oldest_age_ns > 60_000_000_000:
        blockers.append({
            "code": "INPUT_STALE",
            "robot_id": None,
            "detail": f"oldest frozen input age is {oldest_age_ns / 1e9:.3f}s",
        })
    if capture_skew_ns > 5_000_000_000:
        blockers.append({
            "code": "INPUT_SKEW",
            "robot_id": None,
            "detail": f"cross-robot capture skew is {capture_skew_ns / 1e9:.3f}s",
        })

    selections_raw = manifest.get("final_shadow_selections", {})
    if not isinstance(selections_raw, dict):
        raise ValueError("final_shadow_selections is malformed")
    active_robot_ids = tuple(
        robot_id for robot_id in robot_ids if robot_id in selections_raw
    )
    source_episode = manifest.get("source_episode", {})
    if not isinstance(source_episode, dict):
        source_episode = {}
    source_step = int(source_episode.get("logical_l_step", 0))
    next_round = int(source_episode.get("next_round_index", 1))
    round_index = max(0, next_round - 1)
    run_id = str(manifest.get("run_id", manifest_path.parent.name))
    batch_id = _bounded_id(f"{run_id}-epoch-{execution_epoch}")

    fused_artifact = manifest.get("decision_map_artifact")
    fused_artifact_sha: str | None = None
    if isinstance(fused_artifact, dict):
        fused_path, fused_verified = _verify_artifact(
            fused_artifact, manifest_path.parent
        )
        fused_artifact_sha = sha256_file(fused_path)
        verified_artifacts.append(fused_verified)

    decisions: list[HighLevelDecisionV2] = []
    for robot_id in robot_ids:
        result = robot_results[robot_id]
        selection_raw = selections_raw.get(robot_id)
        selection = selection_raw if isinstance(selection_raw, dict) else None
        mode = "GOAL" if selection is not None else "HOLD"
        target: dict[str, object] | None = None
        map_snapshot_sha = sha256_file(map_paths[robot_id])
        if selection is not None:
            kind = str(selection.get("kind", ""))
            if kind == "semantic_goal":
                target, semantic_artifact = _semantic_target(
                    selection,
                    robot_id=robot_id,
                    manifest_dir=manifest_path.parent,
                    snapshot=snapshots[robot_id],
                )
                verified_artifacts.append(semantic_artifact)
            elif kind in {"frontier", "history"}:
                if fused_artifact_sha is None:
                    raise ValueError(
                        "frontier/history selection lacks fused decision-map provenance; "
                        "rerun live_vlm_shadow with the current implementation"
                    )
                target = _frontier_target(selection, result)
                map_snapshot_sha = fused_artifact_sha
            else:
                raise ValueError(f"unsupported VLM selection kind: {kind!r}")
        map_version = int(registry_entries[robot_id].get("map_version", -1))
        decision = HighLevelDecisionV2.model_validate({
            "robot_id": robot_id,
            "scene_id": scene_id,
            "episode_id": episode_id,
            "round_index": round_index,
            "source_step": source_step,
            "decision_batch_id": batch_id,
            "leg_id": _bounded_id(f"{batch_id}-{robot_id}-leg-0"),
            "decision_id": _bounded_id(f"{batch_id}-{robot_id}-lease-0"),
            "lease_sequence": 0,
            "mode": mode,
            "coordination": {
                "execution_epoch": execution_epoch,
                "active_robot_ids": list(active_robot_ids),
            },
            "goal_category": str(manifest["goal_category"]),
            "input_observations": observations,
            "map_provenance": {
                "map_version": map_version,
                "map_snapshot_sha256": map_snapshot_sha,
                "map_format_version": "focus-hub-central-map-v3",
                "frame_id": "shared_world",
                "resolution_m": snapshots[robot_id].resolution_m,
                "transform_version": str(result["map_transform_version"]),
                "shared_frame_calibration_id": str(
                    snapshots[robot_id].shared_frame_calibration_id
                ),
            },
            "issued_at_ns": now_ns,
            "expires_at_ns": now_ns + lease_duration_ns,
            "target": target,
            "reason": (
                f"source-faithful VLM {selection.get('kind') if selection else 'no-selection'} "
                f"from frozen manifest {run_id}"
            ),
        })
        decisions.append(decision)
    batch = DecisionBatchV2(decisions=tuple(decisions))

    manifest_artifact = {
        "source_path": str(manifest_path),
        "preserved_path": str(manifest_path),
        "size_bytes": manifest_path.stat().st_size,
        "sha256": sha256_file(manifest_path),
        "classification": "observed frozen VLM shadow manifest",
        "verification": "locally observed",
    }
    report: dict[str, object] = {
        "schema_version": "focus-v2-scene-batch-build-v1",
        "status": "candidate_built_no_network_no_motion",
        "manifest": manifest_artifact,
        "decision_batch_id": batch_id,
        "active_robot_ids": list(active_robot_ids),
        "robot_commands_sent": False,
        "network_used": False,
        "preflight_ready": not blockers,
        "blockers": blockers,
        "unverified_runtime_checks": unverified,
        "input_timing": {
            "oldest_age_s": oldest_age_ns / 1e9,
            "cross_robot_capture_skew_s": capture_skew_ns / 1e9,
        },
        "verified_artifacts": verified_artifacts,
        "classification": (
            "source-derived candidate from observed frozen artifacts; "
            "physical execution unverified"
        ),
    }
    return SceneBatchBuild(batch=batch, report=report)
