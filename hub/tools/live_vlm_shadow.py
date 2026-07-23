#!/usr/bin/env python3
"""One-shot live two-robot VLM shadow scheduler.

The tool freezes current per-robot map/source artifacts, validates their
shared-frame contract, fuses them, filters untrusted semantic classes only in
the decision copy, and runs the real Perception -> Judgment -> Decision VLM
cascade sequentially.  A chosen frontier is removed before the next robot is
scheduled.

This file contains no GOAL publication path.  With ``--publish-hold`` it sends
only versioned, expiring HOLD decisions whose reason records the would-be
target.  With ``--write-foxglove-targets`` it writes display-only target files
for the existing relay.  Neither option can actuate a robot.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
import shutil
import sys
import time
import uuid

import cv2
import httpx
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.central_mapping import HM3D_CATEGORY_NAMES  # noqa: E402
from focus_hub.directional_memory import DirectionalMemory  # noqa: E402
from focus_hub.frontiers import (  # noqa: E402
    Frontier,
    extract_frontiers,
    render_semantic_decision_map,
)
from focus_hub.fusion import align_and_fuse_grids  # noqa: E402
from focus_hub.map_snapshot import (  # noqa: E402
    MapSnapshot,
    load_map_snapshot,
    validate_fusion_contract,
)
from focus_hub.models import Decision, ObservationMetadata  # noqa: E402
from focus_hub.shadow_coordination import (  # noqa: E402
    SHADOW_SCHEMA_VERSION,
    build_shadow_target_payload,
    collapse_detection_records,
    filter_semantic_categories,
    heading_deg_from_camera_pose,
    sha256_file,
    validate_shadow_input_timing,
    world_to_cell,
)
from focus_hub.source_episode import (  # noqa: E402
    SOURCE_EARLY_FRONTIER_STEP,
    SOURCE_HM3D_OBJECTNAV_GOALS,
    SourceEpisodeState,
    extract_source_goal_component,
)
from focus_hub.vlm_decision import run_decision_cascade  # noqa: E402
from focus_hub.vlm_prompts import (  # noqa: E402
    extract_scene_objects,
    format_scene_objects_for_prompt,
)


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    name: str
    snapshot_dir: Path


@dataclass(frozen=True)
class RobotContext:
    spec: RobotSpec
    snapshot: MapSnapshot
    map_sha256: str
    map_summary: dict[str, object]
    live_status: dict[str, object]
    metadata: ObservationMetadata
    source_sequence: int
    rgb_bgr: np.ndarray
    detections: dict[str, float]
    T_shared_camera: np.ndarray
    artifacts: list[dict[str, object]]


def parse_robot_spec(value: str) -> RobotSpec:
    parts = value.split(":", 2)
    if len(parts) != 3 or not all(parts):
        raise argparse.ArgumentTypeError(
            f"expected ROBOT_ID:NAME:SNAPSHOT_DIR, got {value!r}"
        )
    return RobotSpec(parts[0], parts[1], Path(parts[2]).expanduser().resolve())


def parse_expected_value(value: str) -> tuple[str, str]:
    robot_id, separator, expected = value.partition(":")
    if not separator or not robot_id or not expected:
        raise argparse.ArgumentTypeError(
            f"expected ROBOT_ID:VALUE, got {value!r}"
        )
    return robot_id, expected


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def preserved_copy(
    source: Path,
    destination: Path,
    *,
    status: str,
) -> dict[str, object]:
    if not source.is_file():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return {
        "source_path": str(source.resolve()),
        "preserved_path": str(destination.resolve()),
        "size_bytes": destination.stat().st_size,
        "sha256": sha256_file(destination),
        "status": status,
    }


def load_context(
    spec: RobotSpec,
    spool: Path,
    output: Path,
    *,
    allow_blocked_shadow_input: bool,
) -> RobotContext:
    input_dir = output / "inputs" / spec.name
    artifacts: list[dict[str, object]] = []
    map_copy = input_dir / "central_map.npz"
    summary_copy = input_dir / "map_summary.json"
    status_copy = input_dir / "live_status.json"
    artifacts.append(preserved_copy(
        spec.snapshot_dir / "central_map.npz",
        map_copy,
        status="model/source-derived frozen live map input",
    ))
    artifacts.append(preserved_copy(
        spec.snapshot_dir / "map_summary.json",
        summary_copy,
        status="observed frozen live map summary",
    ))
    artifacts.append(preserved_copy(
        spec.snapshot_dir / "live_status.json",
        status_copy,
        status="observed frozen live runtime status",
    ))

    snapshot = load_map_snapshot(map_copy)
    if snapshot is None:
        raise RuntimeError(f"map snapshot disappeared while copying: {map_copy}")
    map_summary = json.loads(summary_copy.read_text(encoding="utf-8"))
    live_status = json.loads(status_copy.read_text(encoding="utf-8"))
    if (
        live_status.get("mapping_blocked_reason") is not None
        and not allow_blocked_shadow_input
    ):
        raise RuntimeError(
            f"{spec.name} map is blocked: {live_status['mapping_blocked_reason']}"
        )
    if live_status.get("transform_version") != snapshot.transform_version:
        raise RuntimeError(f"{spec.name} status/snapshot transform mismatch")

    semantic_mapping = map_summary.get("semantic_mapping", {})
    if not isinstance(semantic_mapping, dict):
        raise RuntimeError(f"{spec.name} map summary lacks semantic mapping status")
    yolo_status = semantic_mapping.get("yolo_reinforcement", {})
    if not isinstance(yolo_status, dict) or not yolo_status.get("enabled"):
        raise RuntimeError(f"{spec.name} map has no enabled YOLO evidence")
    source_sequence = int(yolo_status.get("last_sequence", -1))
    if source_sequence < 0:
        raise RuntimeError(f"{spec.name} map has no YOLO source sequence")

    source_dir = spool / spec.robot_id / f"{source_sequence:020d}"
    metadata_copy = input_dir / f"source_{source_sequence}_metadata.json"
    depth_copy = input_dir / f"source_{source_sequence}_depth.png"
    artifacts.append(preserved_copy(
        source_dir / "metadata.json",
        metadata_copy,
        status="observed spooled VLM/source-pose input",
    ))
    artifacts.append(preserved_copy(
        source_dir / "depth.png",
        depth_copy,
        status="observed spooled aligned-depth provenance",
    ))
    rgb_candidates = [source_dir / "rgb.jpg", source_dir / "rgb.png"]
    rgb_source = next((candidate for candidate in rgb_candidates if candidate.is_file()), None)
    if rgb_source is None:
        raise FileNotFoundError(f"no RGB source in {source_dir}")
    rgb_copy = input_dir / f"source_{source_sequence}{rgb_source.suffix}"
    artifacts.append(preserved_copy(
        rgb_source,
        rgb_copy,
        status="observed spooled VLM RGB input",
    ))

    metadata = ObservationMetadata.model_validate_json(
        metadata_copy.read_text(encoding="utf-8")
    )
    if metadata.robot_id != spec.robot_id or metadata.sequence != source_sequence:
        raise RuntimeError(f"{spec.name} source metadata identity mismatch")
    if metadata.pose.transform_version != snapshot.transform_version:
        raise RuntimeError(f"{spec.name} source/snapshot transform mismatch")
    rgb_bgr = cv2.imread(str(rgb_copy), cv2.IMREAD_COLOR)
    if rgb_bgr is None:
        raise RuntimeError(f"failed to decode {rgb_copy}")
    T_shared_camera = np.asarray(
        metadata.pose.shared_T_camera.matrix, dtype=np.float64
    ).reshape(4, 4)
    detections_raw = yolo_status.get("last_detections", [])
    if not isinstance(detections_raw, list):
        raise RuntimeError(f"{spec.name} persisted YOLO detections are malformed")
    detections = collapse_detection_records(detections_raw)

    return RobotContext(
        spec=spec,
        snapshot=snapshot,
        map_sha256=sha256_file(map_copy),
        map_summary=map_summary,
        live_status=live_status,
        metadata=metadata,
        source_sequence=source_sequence,
        rgb_bgr=rgb_bgr,
        detections=detections,
        T_shared_camera=T_shared_camera,
        artifacts=artifacts,
    )


def frontier_record(frontier: Frontier) -> dict[str, object]:
    return {
        "frontier_id": frontier.frontier_id,
        "row": frontier.row,
        "col": frontier.col,
        "x_m": frontier.x_m,
        "y_m": frontier.y_m,
        "size_cells": frontier.size_cells,
    }


def registry_map_versions(path: Path, robot_ids: list[str]) -> dict[str, int]:
    state = json.loads(path.read_text(encoding="utf-8"))
    robots = state.get("robots", {})
    if not isinstance(robots, dict):
        raise ValueError("registry state has no robots object")
    versions: dict[str, int] = {}
    for robot_id in robot_ids:
        robot = robots.get(robot_id)
        if not isinstance(robot, dict):
            raise ValueError(f"registry state has no {robot_id}")
        versions[robot_id] = int(robot["map_version"])
    return versions


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        action="append",
        type=parse_robot_spec,
        required=True,
        help="repeat ROBOT_ID:NAME:SNAPSHOT_DIR in allocation order",
    )
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--expected-source-sequence",
        action="append",
        type=parse_expected_value,
        default=[],
        metavar="ROBOT_ID:SEQUENCE",
        help="lock a continuous-scene round to its accepted source keyframe",
    )
    parser.add_argument(
        "--expected-map-sha256",
        action="append",
        type=parse_expected_value,
        default=[],
        metavar="ROBOT_ID:SHA256",
        help="lock a continuous-scene round to its accepted map snapshot",
    )
    parser.add_argument("--glm-url", default="http://127.0.0.1:31511/v1")
    parser.add_argument(
        "--goal-category",
        choices=SOURCE_HM3D_OBJECTNAV_GOALS,
        default="chair",
    )
    parser.add_argument("--trusted-category", action="append", default=None)
    parser.add_argument(
        "--expected-shared-frame-calibration-id",
        default=None,
        help="reject maps from any other calibration/session ID",
    )
    parser.add_argument("--vlm-timeout-s", type=float, default=300.0)
    parser.add_argument(
        "--early-episode-steps",
        type=int,
        default=SOURCE_EARLY_FRONTIER_STEP,
        help="locked to the executable HPC main.py threshold (125)",
    )
    parser.add_argument(
        "--source-step",
        type=int,
        default=None,
        help=(
            "HPC logical l_step for this decision; with --scene-state-file it "
            "must equal that state's exact 0,24,49,... source-derived clock"
        ),
    )
    parser.add_argument(
        "--scene-state-file",
        type=Path,
        default=None,
        help=(
            "persist shared HPC episode history/previous positions across "
            "rounds; the file must already contain a validated scene state"
        ),
    )
    parser.add_argument("--hub-url", default="http://127.0.0.1:8088")
    parser.add_argument(
        "--admin-token-file", type=Path, default=WORKSPACE / "hub/runtime/admin_token"
    )
    parser.add_argument(
        "--registry-state",
        type=Path,
        default=WORKSPACE / "hub/runtime/state/registry_state.json",
    )
    parser.add_argument("--publish-hold", action="store_true")
    parser.add_argument("--write-foxglove-targets", action="store_true")
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="freeze and validate inputs/frontiers without calling GLM or publishing",
    )
    parser.add_argument(
        "--allow-blocked-shadow-input",
        action="store_true",
        help=(
            "permit a locked map only for this non-command shadow analysis; "
            "the block reason is preserved and GOAL publication remains impossible"
        ),
    )
    parser.add_argument("--max-input-age-s", type=float, default=30.0)
    parser.add_argument("--max-sync-skew-s", type=float, default=5.0)
    parser.add_argument(
        "--allow-stale-shadow-input",
        action="store_true",
        help=(
            "permit stale/asynchronous inputs only for non-command forensic "
            "shadow analysis; exact ages and violations remain in the manifest"
        ),
    )
    parser.add_argument("--hold-expiry-s", type=float, default=30.0)
    parser.add_argument("--display-expiry-s", type=float, default=600.0)
    args = parser.parse_args()

    specs: list[RobotSpec] = args.robot
    if len(specs) < 2:
        parser.error("shadow fusion requires at least two --robot inputs")
    if len({spec.robot_id for spec in specs}) != len(specs):
        parser.error("robot IDs must be unique")
    expected_sequences = dict(args.expected_source_sequence)
    expected_map_hashes = dict(args.expected_map_sha256)
    if len(expected_sequences) != len(args.expected_source_sequence):
        parser.error("duplicate --expected-source-sequence robot ID")
    if len(expected_map_hashes) != len(args.expected_map_sha256):
        parser.error("duplicate --expected-map-sha256 robot ID")
    known_robot_ids = {spec.robot_id for spec in specs}
    if not set(expected_sequences).issubset(known_robot_ids):
        parser.error("expected source sequence contains an unknown robot ID")
    if not set(expected_map_hashes).issubset(known_robot_ids):
        parser.error("expected map hash contains an unknown robot ID")
    if expected_sequences or expected_map_hashes:
        if set(expected_sequences) != known_robot_ids:
            parser.error("expected source sequences must cover every robot")
        if set(expected_map_hashes) != known_robot_ids:
            parser.error("expected map hashes must cover every robot")
    if any(not value.isdigit() for value in expected_sequences.values()):
        parser.error("expected source sequences must be non-negative integers")
    if any(
        len(value) != 64
        or any(character not in "0123456789abcdef" for character in value.lower())
        for value in expected_map_hashes.values()
    ):
        parser.error("expected map hashes must be 64 hexadecimal characters")
    if args.preflight_only and (args.publish_hold or args.write_foxglove_targets):
        parser.error(
            "--preflight-only cannot publish HOLD or write Foxglove targets"
        )
    if args.preflight_only and args.scene_state_file is not None:
        parser.error("--preflight-only cannot mutate a persistent scene state")
    if (
        args.vlm_timeout_s <= 0.0
        or args.hold_expiry_s <= 0.0
        or args.display_expiry_s <= 0.0
        or args.max_input_age_s <= 0.0
        or args.max_sync_skew_s < 0.0
    ):
        parser.error("timeouts and expiries must be positive")
    trusted_categories = tuple(args.trusted_category or [args.goal_category])
    if args.early_episode_steps != SOURCE_EARLY_FRONTIER_STEP:
        parser.error(
            "--early-episode-steps is locked to the HPC source value "
            f"{SOURCE_EARLY_FRONTIER_STEP}"
        )
    source_step = 0 if args.source_step is None else args.source_step
    if source_step < 0:
        parser.error("--source-step must be non-negative")

    output = args.output.expanduser().resolve()
    if output.exists():
        print(f"refusing to overwrite existing output: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)
    spool = args.spool.expanduser().resolve()
    manifest_path = output / "shadow_manifest.json"
    scene_state_path = (
        None
        if args.scene_state_file is None
        else args.scene_state_file.expanduser().resolve()
    )
    scene_state: SourceEpisodeState | None = None
    scene_state_before_artifact: dict[str, object] | None = None
    if scene_state_path is not None:
        if not scene_state_path.is_file():
            raise FileNotFoundError(f"scene state does not exist: {scene_state_path}")
        frozen_state_path = output / "input_scene_state.json"
        scene_state_before_artifact = preserved_copy(
            scene_state_path,
            frozen_state_path,
            status="source-derived persistent HPC episode state before this round",
        )
        scene_state = SourceEpisodeState.from_dict(
            json.loads(frozen_state_path.read_text(encoding="utf-8"))
        )
        if source_step != scene_state.source_step:
            raise RuntimeError(
                "source step does not match persistent scene clock: "
                f"requested={source_step}, expected={scene_state.source_step}"
            )
        if tuple(spec.robot_id for spec in specs) != scene_state.robot_ids:
            raise RuntimeError("scene state robot order/identity mismatch")
        if args.goal_category != scene_state.goal_category:
            raise RuntimeError("scene state goal category mismatch")
    started_at_ns = time.time_ns()
    run_id = f"shadow-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
    manifest: dict[str, object] = {
        "schema_version": SHADOW_SCHEMA_VERSION,
        "run_id": run_id,
        "status": "running_shadow_only",
        "safety": {
            "robot_commands_sent": False,
            "goal_publication_code_path_present": False,
            "hub_decision_mode_if_published": "HOLD",
            "allow_goal_changed": False,
        },
        "started_at_ns": started_at_ns,
        "goal_category": args.goal_category,
        "source_episode": {
            "enabled": scene_state is not None,
            "logical_l_step": source_step,
            "decision_cadence_source": (
                "source/Focus_realworld/main.py: l_step==0 or l_step%25==24"
            ),
            "target_override_source": (
                "source/Focus_realworld/agents/vlm_agents.py: Find_Goal + largest connected target mask"
            ),
            "scene_state_before": scene_state_before_artifact,
            "clock_status": (
                "source-derived shadow clock; not observed physical action count"
                if scene_state is not None
                else "one-shot compatibility value"
            ),
        },
        "trusted_semantic_categories": list(trusted_categories),
        "allow_blocked_shadow_input": args.allow_blocked_shadow_input,
        "allow_stale_shadow_input": args.allow_stale_shadow_input,
        "robots": [],
    }
    atomic_write_json(manifest_path, manifest)

    contexts = [
        load_context(
            spec,
            spool,
            output,
            allow_blocked_shadow_input=args.allow_blocked_shadow_input,
        )
        for spec in specs
    ]
    for context in contexts:
        expected_sequence = expected_sequences.get(context.spec.robot_id)
        if (
            expected_sequence is not None
            and context.source_sequence != int(expected_sequence)
        ):
            raise RuntimeError(
                f"{context.spec.name} source sequence changed after scene acceptance: "
                f"expected={expected_sequence}, frozen={context.source_sequence}"
            )
        expected_map_hash = expected_map_hashes.get(context.spec.robot_id)
        if (
            expected_map_hash is not None
            and context.map_sha256 != expected_map_hash
        ):
            raise RuntimeError(
                f"{context.spec.name} map changed after scene acceptance: "
                f"expected={expected_map_hash}, frozen={context.map_sha256}"
            )
    timing = validate_shadow_input_timing(
        [context.metadata.capture_time_ns for context in contexts],
        now_ns=time.time_ns(),
        max_input_age_s=args.max_input_age_s,
        max_sync_skew_s=args.max_sync_skew_s,
        allow_stale_forensic_input=args.allow_stale_shadow_input,
    )
    snapshots = [context.snapshot for context in contexts]
    frame_id, resolution_m, calibration_id = validate_fusion_contract(snapshots)
    if (
        args.expected_shared_frame_calibration_id is not None
        and calibration_id != args.expected_shared_frame_calibration_id
    ):
        raise RuntimeError(
            "shared calibration mismatch: "
            f"expected {args.expected_shared_frame_calibration_id!r}, "
            f"got {calibration_id!r}"
        )
    fused_grid, fused_origin = align_and_fuse_grids(
        [snapshot.grid for snapshot in snapshots],
        [snapshot.origin_xy_m for snapshot in snapshots],
        resolution_m,
    )
    if scene_state is not None:
        if set(trusted_categories) != set(HM3D_CATEGORY_NAMES):
            raise RuntimeError(
                "persistent HPC scene requires all 15 source semantic categories"
            )
        scene_state.validate_contract(
            goal_category=args.goal_category,
            calibration_id=calibration_id,
            robot_ids=tuple(context.spec.robot_id for context in contexts),
            fused_origin_xy_m=fused_origin,
            resolution_m=resolution_m,
            fused_shape_hw=(int(fused_grid.shape[1]), int(fused_grid.shape[2])),
        )
    decision_grid, hidden_semantic_counts = filter_semantic_categories(
        fused_grid,
        HM3D_CATEGORY_NAMES,
        trusted_categories,
    )
    decision_map_path = output / "fused_decision_map.npz"
    decision_map_temporary = output / ".fused_decision_map.tmp.npz"
    np.savez_compressed(
        decision_map_temporary,
        grid=decision_grid,
        origin_xy_m=np.asarray(fused_origin, dtype=np.float64),
        resolution_m=np.asarray(resolution_m),
        frame_id=np.asarray(frame_id),
        transform_version=np.asarray("multi-robot-source-derived"),
        shared_frame_calibration_id=np.asarray(calibration_id),
        map_format_version=np.asarray("focus-hub-central-map-v3"),
    )
    os.replace(decision_map_temporary, decision_map_path)
    decision_map_artifact = {
        "source_paths": [
            str((output / "inputs" / context.spec.name / "central_map.npz").resolve())
            for context in contexts
        ],
        "preserved_path": str(decision_map_path.resolve()),
        "size_bytes": decision_map_path.stat().st_size,
        "sha256": sha256_file(decision_map_path),
        "status": (
            "source-derived frozen fused VLM decision map; trusted-category "
            "filter recorded in manifest"
        ),
    }
    frontiers = extract_frontiers(decision_grid, fused_origin, resolution_m)
    semantic_goals = {
        context.spec.robot_id: extract_source_goal_component(
            context.snapshot,
            args.goal_category,
        )
        for context in contexts
    }
    if scene_state is None and len(frontiers) < len(contexts):
        raise RuntimeError(
            f"need at least {len(contexts)} frontiers for distinct allocation, "
            f"found {len(frontiers)}"
        )
    if scene_state is not None and not frontiers and not any(semantic_goals.values()):
        # Upstream falls back to a random map point when no frontier exists.
        # A random physical target is not safe to synthesize in a deployment
        # adapter, so preserve the algorithmic condition but fail closed to
        # HOLD and record the explicit deviation below.
        manifest["source_no_frontier_random_mode"] = {
            "observed": True,
            "hpc_behavior": "random map point",
            "real_robot_adapter": "HOLD; random physical goal suppressed",
        }
    scene_objects = format_scene_objects_for_prompt(extract_scene_objects(
        decision_grid[2 : 2 + len(HM3D_CATEGORY_NAMES)],
        HM3D_CATEGORY_NAMES,
    ))
    semantic_goal_records: dict[str, dict[str, object] | None] = {}
    for context in contexts:
        component = semantic_goals[context.spec.robot_id]
        if component is None:
            semantic_goal_records[context.spec.robot_id] = None
            continue
        mask_dir = output / "source_goal_masks"
        mask_dir.mkdir(parents=True, exist_ok=True)
        mask_path = mask_dir / f"{context.spec.name}_{args.goal_category}.png"
        if not cv2.imwrite(str(mask_path), component.mask.astype(np.uint8) * 255):
            raise RuntimeError(f"failed to write {mask_path}")
        record = component.to_record()
        record["mask_path"] = str(mask_path.resolve())
        record["mask_size_bytes"] = mask_path.stat().st_size
        record["mask_sha256"] = sha256_file(mask_path)
        semantic_goal_records[context.spec.robot_id] = record
    manifest.update({
        "frame_id": frame_id,
        "shared_frame_calibration_id": calibration_id,
        "resolution_m": resolution_m,
        "fused_origin_xy_m": list(fused_origin),
        "fused_shape": list(decision_grid.shape),
        "hidden_untrusted_semantic_cells": hidden_semantic_counts,
        "decision_map_artifact": decision_map_artifact,
        "frontiers": [frontier_record(frontier) for frontier in frontiers],
        "scene_objects_for_vlm": scene_objects,
        "source_semantic_goals": semantic_goal_records,
        "input_timing": timing,
        "input_artifacts": [
            artifact for context in contexts for artifact in context.artifacts
        ],
    })
    atomic_write_json(manifest_path, manifest)

    if args.preflight_only:
        completed_at_ns = time.time_ns()
        manifest.update({
            "status": "complete_preflight_only",
            "completed_at_ns": completed_at_ns,
            "elapsed_s": (completed_at_ns - started_at_ns) / 1e9,
            "safety": {
                "robot_commands_sent": False,
                "goal_publication_code_path_present": False,
                "hub_decisions_published": False,
                "allow_goal_changed": False,
            },
        })
        atomic_write_json(manifest_path, manifest)
        print(json.dumps({
            "run_id": run_id,
            "status": manifest["status"],
            "frontier_count": len(frontiers),
            "hidden_untrusted_semantic_cells": hidden_semantic_counts,
            "manifest": str(manifest_path),
        }, indent=2, sort_keys=True))
        return 0

    remaining = list(frontiers)
    robot_results: list[dict[str, object]] = []
    allocations: dict[str, Frontier] = {}
    selections: dict[str, dict[str, object]] = {}
    shared_memory = None if scene_state is None else scene_state.memory
    remaining_history_indices: list[int] | None = None
    remaining_history_scores: dict[int, float] | None = None
    for index, context in enumerate(contexts, start=1):
        # Upstream assigns both robots its sole frontier when only one exists.
        # Preserve that behavior in a persistent source episode; the legacy
        # one-shot path retains its stricter distinct-frontier requirement.
        frontier_reused = bool(
            scene_state is not None
            and len(frontiers) == 1
            and not remaining
        )
        decision_frontiers = list(frontiers if frontier_reused else remaining)
        candidate_frontiers = list(decision_frontiers)
        robot_xy = (
            float(context.T_shared_camera[0, 3]),
            float(context.T_shared_camera[1, 3]),
        )
        robot_rc = world_to_cell(
            robot_xy,
            fused_origin,
            resolution_m,
            decision_grid.shape[1:],
        )
        heading_deg = heading_deg_from_camera_pose(context.T_shared_camera)
        memory = shared_memory if shared_memory is not None else DirectionalMemory()
        pre_goal_point = (
            None
            if scene_state is None
            else scene_state.previous_positions_rc.get(context.spec.robot_id)
        )
        judgment_map = render_semantic_decision_map(
            decision_grid,
            HM3D_CATEGORY_NAMES,
            decision_frontiers,
            robot_rc,
            heading_deg,
            history_nodes=memory.history_nodes,
            pre_goal_rc=pre_goal_point,
        )
        decision_map = render_semantic_decision_map(
            decision_grid,
            HM3D_CATEGORY_NAMES,
            decision_frontiers,
            robot_rc,
            heading_deg,
            pre_goal_rc=pre_goal_point,
        )
        judgment_path = output / f"{context.spec.name}_judgment_map.jpg"
        decision_path = output / f"{context.spec.name}_decision_map.jpg"
        if not cv2.imwrite(str(judgment_path), judgment_map):
            raise RuntimeError(f"failed to write {judgment_path}")
        if not cv2.imwrite(str(decision_path), decision_map):
            raise RuntimeError(f"failed to write {decision_path}")

        call_started = time.perf_counter()
        cascade = run_decision_cascade(
            rgb_bgr=context.rgb_bgr,
            judgment_map_bgr=judgment_map,
            decision_map_bgr=decision_map,
            frontiers=decision_frontiers,
            target=args.goal_category,
            detections=context.detections,
            scene_objects=scene_objects,
            cur_location_rc=robot_rc,
            heading_deg=heading_deg,
            pre_goal_point=pre_goal_point,
            step=source_step,
            early_episode_step_threshold=args.early_episode_steps,
            memory=memory,
            base_url=args.glm_url,
            timeout_s=args.vlm_timeout_s,
            history_candidate_indices=remaining_history_indices,
            history_candidate_scores=remaining_history_scores,
        )
        elapsed_s = time.perf_counter() - call_started
        chosen = cascade.frontier_choice.frontier if cascade.frontier_choice else None
        exploration_selection: dict[str, object] | None = None
        if chosen is not None:
            if all(item.frontier_id != chosen.frontier_id for item in decision_frontiers):
                raise RuntimeError("VLM returned a frontier outside its candidate set")
            allocations[context.spec.robot_id] = chosen
            if not frontier_reused:
                remaining = [
                    item for item in remaining if item.frontier_id != chosen.frontier_id
                ]
            exploration_selection = {
                "kind": "frontier",
                "target_id": chosen.frontier_id,
                **frontier_record(chosen),
                "source_behavior": (
                    "sole frontier reused across robots"
                    if frontier_reused
                    else "sequential frontier removed before next robot"
                ),
            }
        if scene_state is not None and remaining_history_indices is None:
            # This is the source's shared ``history_nodes_copy`` snapshot,
            # created during agent 0's pass and consumed sequentially.
            remaining_history_indices = list(range(len(memory.history_nodes)))
            remaining_history_scores = {
                history_index: memory.history_score[history_index]
                for history_index in remaining_history_indices
            }
        if cascade.history_choice_index is not None:
            history_index = cascade.history_choice_index
            history_row, history_col = memory.history_nodes[history_index]
            exploration_selection = {
                "kind": "history",
                "target_id": f"history-{history_index}",
                "history_index": history_index,
                "row": history_row,
                "col": history_col,
                "x_m": fused_origin[0] + (history_col + 0.5) * resolution_m,
                "y_m": fused_origin[1] + (history_row + 0.5) * resolution_m,
                "history_score": (
                    memory.history_score[history_index]
                    if remaining_history_scores is None
                    else remaining_history_scores[history_index]
                ),
                "source_behavior": "first argmax of shared history_score_copy",
            }
            if (
                remaining_history_indices is not None
                and len(remaining_history_indices) > 1
            ):
                # Source reuses the sole copied history node, but deletes one
                # selected entry when multiple copied candidates exist.
                remaining_history_indices = [
                    item for item in remaining_history_indices if item != history_index
                ]

        semantic_selection = semantic_goal_records[context.spec.robot_id]
        final_selection = (
            semantic_selection
            if semantic_selection is not None
            else exploration_selection
        )
        if final_selection is not None:
            selections[context.spec.robot_id] = final_selection
        if scene_state is not None:
            scene_state.previous_positions_rc[context.spec.robot_id] = robot_rc
            scene_state.last_source_sequences[context.spec.robot_id] = (
                context.source_sequence
            )
            scene_state.source_find_goal[context.spec.robot_id] = bool(
                scene_state.source_find_goal.get(context.spec.robot_id, False)
                or semantic_selection is not None
            )
        result = {
            "robot_id": context.spec.robot_id,
            "name": context.spec.name,
            "allocation_order": index,
            "source_sequence": context.source_sequence,
            "source_capture_time_ns": context.metadata.capture_time_ns,
            "robot_xy_m": list(robot_xy),
            "robot_rc": list(robot_rc),
            "heading_deg_camera_forward_approximation": heading_deg,
            "detections": context.detections,
            "candidate_frontiers": [
                frontier_record(frontier)
                for frontier in candidate_frontiers
            ],
            "perception_pr": (
                None if cascade.perception_pr is None else list(cascade.perception_pr)
            ),
            "judgment_pr": (
                None if cascade.judgment_pr is None else list(cascade.judgment_pr)
            ),
            "gate_passed": cascade.gate_passed,
            "gate_reason": cascade.gate_reason,
            "updated_history_index": cascade.history_index,
            "selected_history_index": cascade.history_choice_index,
            "errors": list(cascade.errors),
            "vlm_elapsed_s": elapsed_s,
            "allocated_frontier": None if chosen is None else frontier_record(chosen),
            "exploration_selection_before_target_override": exploration_selection,
            "source_find_goal": semantic_selection is not None,
            "semantic_goal_override": semantic_selection,
            "final_shadow_selection": final_selection,
            "frontier_reused": frontier_reused,
            "choice_probabilities": (
                {} if cascade.frontier_choice is None
                else cascade.frontier_choice.probabilities
            ),
            "choice_raw_content": (
                "" if cascade.frontier_choice is None
                else cascade.frontier_choice.raw_content
            ),
            "map_transform_version": context.snapshot.transform_version,
            "map_snapshot_sha256": context.map_sha256,
            "input_mapping_blocked_reason": context.live_status.get(
                "mapping_blocked_reason"
            ),
        }
        robot_results.append(result)
        manifest["robots"] = robot_results
        manifest["status"] = f"shadow_vlm_completed_{index}_of_{len(contexts)}"
        atomic_write_json(manifest_path, manifest)

    completed_at_ns = time.time_ns()
    map_versions = registry_map_versions(
        args.registry_state.expanduser().resolve(),
        [context.spec.robot_id for context in contexts],
    )
    publish_results: dict[str, object] = {}
    if args.publish_hold:
        admin_token = args.admin_token_file.expanduser().read_text().strip()
        if not admin_token:
            raise RuntimeError("admin token is empty")
        for context in contexts:
            selection = selections.get(context.spec.robot_id)
            if selection is None:
                reason = (
                    f"SHADOW ONLY no motion; source episode made no safe allocation for "
                    f"{args.goal_category}"
                )
                frontier_id = None
            else:
                target_id = str(selection["target_id"])
                kind = str(selection["kind"])
                reason = (
                    f"SHADOW ONLY no motion; source {kind} {target_id} at "
                    f"shared_world ({float(selection['x_m']):.3f},"
                    f"{float(selection['y_m']):.3f}) for "
                    f"{args.goal_category}"
                )
                frontier_id = target_id
            now_ns = time.time_ns()
            decision = Decision(
                robot_id=context.spec.robot_id,
                decision_id=f"{run_id}-{context.spec.robot_id}",
                mode="HOLD",
                map_version=map_versions[context.spec.robot_id],
                transform_version=context.snapshot.transform_version,
                issued_at_ns=now_ns,
                expires_at_ns=now_ns + int(args.hold_expiry_s * 1e9),
                frontier_id=frontier_id,
                reason=reason,
            )
            response = httpx.post(
                f"{args.hub_url}/v1/admin/decisions",
                json=json.loads(decision.model_dump_json()),
                headers={"X-Admin-Token": admin_token},
                timeout=10.0,
            )
            publish_results[context.spec.robot_id] = {
                "mode": "HOLD",
                "decision_id": decision.decision_id,
                "status_code": response.status_code,
                "response": response.text[:500],
            }
            response.raise_for_status()

    target_files: dict[str, str] = {}
    if args.write_foxglove_targets:
        for context in contexts:
            selection = selections.get(context.spec.robot_id)
            target_path = context.spec.snapshot_dir / "shadow_target.json"
            if selection is None:
                atomic_write_json(target_path, {
                    "schema_version": SHADOW_SCHEMA_VERSION,
                    "status": "shadow_no_allocation",
                    "robot_id": context.spec.robot_id,
                    "created_at_ns": completed_at_ns,
                    "authority": "display_only_never_robot_command",
                })
                continue
            robot_xy = (
                float(context.T_shared_camera[0, 3]),
                float(context.T_shared_camera[1, 3]),
            )
            yaw_rad = math.atan2(
                float(selection["y_m"]) - robot_xy[1],
                float(selection["x_m"]) - robot_xy[0],
            )
            payload = build_shadow_target_payload(
                robot_id=context.spec.robot_id,
                frontier_id=str(selection["target_id"]),
                goal_category=args.goal_category,
                target_xy_m=(
                    float(selection["x_m"]),
                    float(selection["y_m"]),
                ),
                yaw_rad=yaw_rad,
                snapshot=context.snapshot,
                created_at_ns=completed_at_ns,
                expires_at_ns=(
                    completed_at_ns + int(args.display_expiry_s * 1e9)
                ),
                run_manifest=str(manifest_path),
                map_snapshot_sha256=context.map_sha256,
            )
            payload["target_kind"] = selection["kind"]
            payload["source_find_goal"] = bool(
                selection["kind"] == "semantic_goal"
            )
            atomic_write_json(target_path, payload)
            target_files[context.spec.robot_id] = str(target_path.resolve())

    scene_state_after_artifact: dict[str, object] | None = None
    source_episode_round_status = "one_shot_compatibility"
    if scene_state is not None:
        scene_state.fused_origin_xy_m = fused_origin
        scene_state.resolution_m = resolution_m
        scene_state.fused_shape_hw = (
            int(decision_grid.shape[1]),
            int(decision_grid.shape[2]),
        )
        scene_state.round_index += 1
        state_after_path = output / "scene_state_after.json"
        atomic_write_json(state_after_path, scene_state.to_dict())
        scene_state_after_artifact = {
            "source_path": str(scene_state_path),
            "preserved_path": str(state_after_path.resolve()),
            "size_bytes": state_after_path.stat().st_size,
            "sha256": sha256_file(state_after_path),
            "status": "source-derived persistent HPC episode state after this round",
        }
        # Commit state only after every VLM call, HOLD publication and display
        # write above succeeded.  A partial round never advances the episode.
        atomic_write_json(scene_state_path, scene_state.to_dict())
        source_episode_round_status = (
            "target_found_awaiting_robot_local_planner_stop"
            if any(scene_state.source_find_goal.values())
            else "exploration_continues"
        )

    manifest.update({
        "status": "complete_shadow_only",
        "completed_at_ns": completed_at_ns,
        "elapsed_s": (completed_at_ns - started_at_ns) / 1e9,
        "remaining_frontiers": [frontier_record(item) for item in remaining],
        "final_shadow_selections": selections,
        "source_episode_round_status": source_episode_round_status,
        "hub_hold_publications": publish_results,
        "foxglove_target_files": target_files,
        "safety": {
            "robot_commands_sent": False,
            "goal_publication_code_path_present": False,
            "hub_decision_mode_if_published": "HOLD",
            "allow_goal_changed": False,
        },
    })
    if scene_state is not None:
        source_episode_manifest = manifest["source_episode"]
        if not isinstance(source_episode_manifest, dict):
            raise RuntimeError("source episode manifest became malformed")
        source_episode_manifest["scene_state_after"] = scene_state_after_artifact
        source_episode_manifest["next_round_index"] = scene_state.round_index
        source_episode_manifest["next_logical_l_step"] = scene_state.source_step
    atomic_write_json(manifest_path, manifest)
    print(json.dumps({
        "run_id": run_id,
        "status": manifest["status"],
        "hidden_untrusted_semantic_cells": hidden_semantic_counts,
        "allocations": {
            robot_id: frontier_record(frontier)
            for robot_id, frontier in allocations.items()
        },
        "final_shadow_selections": selections,
        "source_episode_round_status": source_episode_round_status,
        "hub_hold_publications": publish_results,
        "manifest": str(manifest_path),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
