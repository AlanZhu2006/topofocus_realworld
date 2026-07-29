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
    filter_semantic_categories,
    heading_deg_from_base_pose,
    sha256_file,
    shared_base_pose_from_camera,
    validate_shadow_input_timing,
    validated_yolo_source,
    world_to_cell,
)
from focus_hub.source_behavior_contract import (  # noqa: E402
    SOURCE_BEHAVIOR_CONTRACT_VERSION,
    observe_reviewed_source_artifacts,
)
from focus_hub.source_episode import (  # noqa: E402
    SOURCE_EARLY_FRONTIER_STEP,
    SOURCE_HM3D_OBJECTNAV_GOALS,
    SourceEpisodeState,
    extract_source_goal_component,
)
from focus_hub.vlm_decision import (  # noqa: E402
    run_decision_cascade,
    validate_glm_server_contract,
)
from focus_hub.vlm_prompts import (  # noqa: E402
    extract_scene_objects,
    format_scene_objects_for_prompt,
    semantic_label_points,
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
    T_shared_base: np.ndarray
    yolo_model_provenance: dict[str, object]
    robot_trajectory_xy_m: tuple[tuple[float, float], ...]
    robot_trajectory_provenance: dict[str, object]
    artifacts: list[dict[str, object]]


def require_complete_cascade_result(
    cascade: object, *, robot_id: str
) -> None:
    """Fail a strict round instead of disguising a VLM fallback as success."""

    errors = tuple(str(item) for item in getattr(cascade, "errors", ()))
    if errors:
        raise RuntimeError(
            f"{robot_id} VLM cascade was incomplete: " + "; ".join(errors)
        )


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


def semantic_input_contract(
    contexts: list[RobotContext],
) -> dict[str, object]:
    """Require one explicit pixel/fusion semantic contract across robots.

    Every backend writes the same HM3D-15 channel names, but mixing pixel
    models or temporal fusion policies in one max-fused decision map would
    make the evidence incomparable.  The executable source's semantic path
    is also more specific than "RedNet": it applies a Detectron2 Mask R-CNN
    override to six COCO categories in ``_preprocess_obs_rednet``.  The Hub
    records that distinction instead of calling a RedNet-only or SegFormer
    map source-identical.
    """

    if not contexts:
        raise ValueError("semantic input contract requires robot contexts")
    robots: dict[str, dict[str, object]] = {}
    backends: set[str] = set()
    fusion_modes: set[str] = set()
    map_reinforcement_modes: set[bool] = set()
    for context in contexts:
        summary = context.map_summary
        semantic_mapping = summary.get("semantic_mapping")
        pixel_segmenter = (
            semantic_mapping.get("pixel_segmenter")
            if isinstance(semantic_mapping, dict)
            else None
        )
        yolo = (
            semantic_mapping.get("yolo_reinforcement")
            if isinstance(semantic_mapping, dict)
            else None
        )
        backend = (
            pixel_segmenter.get("backend")
            if isinstance(pixel_segmenter, dict)
            else None
        )
        fusion_mode = summary.get("semantic_fusion_mode")
        map_reinforcement = (
            yolo.get("map_reinforcement_enabled")
            if isinstance(yolo, dict)
            else None
        )
        if not isinstance(backend, str) or not backend:
            raise ValueError(
                f"{context.spec.robot_id} map lacks pixel-segmenter identity"
            )
        if fusion_mode not in {"max", "multi_view"}:
            raise ValueError(
                f"{context.spec.robot_id} map has invalid semantic fusion mode"
            )
        if not isinstance(map_reinforcement, bool):
            raise ValueError(
                f"{context.spec.robot_id} map lacks YOLO map policy"
            )
        backends.add(backend)
        fusion_modes.add(str(fusion_mode))
        map_reinforcement_modes.add(map_reinforcement)
        robots[context.spec.robot_id] = {
            "pixel_segmenter_backend": backend,
            "semantic_fusion_mode": fusion_mode,
            "yolo_map_reinforcement_enabled": map_reinforcement,
            "pixel_segmenter_status": pixel_segmenter.get("status"),
        }
    if len(backends) != 1:
        raise ValueError(
            "robot maps use different pixel semantic backends: "
            f"{sorted(backends)}"
        )
    if len(fusion_modes) != 1:
        raise ValueError(
            "robot maps use different semantic fusion modes: "
            f"{sorted(fusion_modes)}"
        )
    if len(map_reinforcement_modes) != 1:
        raise ValueError(
            "robot maps use different YOLO map-reinforcement policies"
        )

    backend = next(iter(backends))
    source_maskrcnn_available = (
        backend == "source_rednet_detectron2_hm3d15"
    )
    if source_maskrcnn_available:
        pixel_model_classification = (
            "executable-source RedNet MP3D-40 plus Detectron2 Mask-RCNN "
            "six-category pixel override"
        )
    elif backend == "rednet_mp3d40":
        pixel_model_classification = (
            "source RedNet backbone; executable source Detectron2 "
            "Mask-RCNN six-category pixel override is not present"
        )
    elif backend == "segformer_b0_ade20k_to_mp3d40":
        pixel_model_classification = (
            "checksum-pinned real-camera deployment adapter; not the "
            "executable source pixel model"
        )
    else:
        pixel_model_classification = (
            "explicit non-source pixel backend recorded from map producer"
        )
    return {
        "schema_version": "focus-vlm-semantic-input-contract-v1",
        "uniform_across_robots": True,
        "pixel_segmenter_backend": backend,
        "semantic_fusion_mode": next(iter(fusion_modes)),
        "yolo_map_reinforcement_enabled": next(
            iter(map_reinforcement_modes)
        ),
        "hm3d_category_order": list(HM3D_CATEGORY_NAMES),
        "pixel_model_classification": pixel_model_classification,
        "source_pixel_model": (
            "RedNet MP3D-40 plus Detectron2 Mask-RCNN COCO overrides in "
            "agents/vlm_agents.py::_preprocess_obs_rednet"
        ),
        "source_maskrcnn_override_available_in_hub": (
            source_maskrcnn_available
        ),
        "robots": robots,
    }


def frozen_robot_trajectory(
    map_path: Path,
    map_summary: dict[str, object],
    live_status: dict[str, object],
) -> tuple[
    tuple[tuple[float, float], ...],
    dict[str, object],
]:
    """Load an observed base trajectory with explicit temporal provenance.

    New map snapshots carry the trajectory in the same atomic NPZ generation.
    Historical snapshots remain replayable from their independently frozen
    live status, but that fallback is never described as map-atomic.
    """

    with np.load(map_path, allow_pickle=False) as payload:
        has_trajectory = "robot_trajectory_xy_m" in payload.files
        related_fields = {
            "robot_trajectory_last_observation_sequence",
            "robot_trajectory_pose_source",
        }
        if has_trajectory and not related_fields.issubset(payload.files):
            raise ValueError(
                "atomic map trajectory is missing provenance fields"
            )
        if has_trajectory:
            raw = np.asarray(
                payload["robot_trajectory_xy_m"],
                dtype=np.float64,
            )
            raw_sequence = np.asarray(
                payload[
                    "robot_trajectory_last_observation_sequence"
                ]
            )
            raw_pose_source = np.asarray(
                payload["robot_trajectory_pose_source"]
            )
        else:
            raw = None
            raw_sequence = None
            raw_pose_source = None

    if raw is not None:
        if (
            raw.ndim != 2
            or raw.shape[1:] != (2,)
            or not np.isfinite(raw).all()
        ):
            raise ValueError("atomic map base trajectory is malformed")
        sequence = int(raw_sequence.item())
        pose_source = str(raw_pose_source.item())
        summary_sequence = map_summary.get("last_observation_sequence")
        summary_record = map_summary.get("robot_trajectory_snapshot")
        if (
            isinstance(summary_sequence, bool)
            or not isinstance(summary_sequence, int)
            or sequence != summary_sequence
            or not pose_source
            or not isinstance(summary_record, dict)
            or summary_record.get("container") != "central_map.npz"
            or summary_record.get("field") != "robot_trajectory_xy_m"
            or summary_record.get("point_count") != int(raw.shape[0])
            or summary_record.get("last_observation_sequence") != sequence
            or summary_record.get("pose_source") != pose_source
        ):
            raise ValueError(
                "atomic map trajectory differs from its summary contract"
            )
        points = tuple(
            (float(point[0]), float(point[1])) for point in raw
        )
        return points, {
            "source_container": "central_map.npz",
            "source_field": "robot_trajectory_xy_m",
            "point_count": len(points),
            "last_observation_sequence": sequence,
            "pose_source": pose_source,
            "classification": (
                "observed base trajectory in the same atomic map snapshot "
                "generation"
            ),
            "temporal_alignment": "map_atomic",
        }

    raw_fallback = live_status.get("robot_trajectory_xy_m")
    if not isinstance(raw_fallback, list):
        raise ValueError(
            "legacy map lacks both atomic and live-status base trajectory"
        )
    points_list: list[tuple[float, float]] = []
    for raw_point in raw_fallback:
        if (
            not isinstance(raw_point, list)
            or len(raw_point) != 2
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in raw_point
            )
        ):
            raise ValueError("legacy live-status trajectory is malformed")
        points_list.append((float(raw_point[0]), float(raw_point[1])))
    return tuple(points_list), {
        "source_container": "live_status.json",
        "source_field": "robot_trajectory_xy_m",
        "point_count": len(points_list),
        "map_last_observation_sequence": map_summary.get(
            "last_observation_sequence"
        ),
        "live_status_last_observation_sequence": live_status.get(
            "last_observation_sequence"
        ),
        "classification": (
            "observed independently frozen live-status base trajectory; "
            "not claimed as the atomic map generation"
        ),
        "temporal_alignment": "recorded_but_not_map_atomic",
    }


def load_context(
    spec: RobotSpec,
    spool: Path,
    output: Path,
    *,
    allow_blocked_shadow_input: bool,
    expected_goal_category: str,
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
    if map_summary.get("robot_id") != spec.robot_id:
        raise RuntimeError(f"{spec.name} map summary robot identity mismatch")
    if map_summary.get("frame_id") != snapshot.frame_id:
        raise RuntimeError(f"{spec.name} summary/snapshot frame mismatch")
    if (
        map_summary.get("shared_frame_calibration_id")
        != snapshot.shared_frame_calibration_id
    ):
        raise RuntimeError(
            f"{spec.name} summary/snapshot calibration mismatch"
        )
    if map_summary.get("transform_version") != snapshot.transform_version:
        raise RuntimeError(f"{spec.name} summary/snapshot transform mismatch")
    if map_summary.get("map_format_version") != snapshot.map_format_version:
        raise RuntimeError(f"{spec.name} summary/snapshot format mismatch")
    robot_trajectory, robot_trajectory_provenance = (
        frozen_robot_trajectory(map_copy, map_summary, live_status)
    )
    if (
        live_status.get("mapping_blocked_reason") is not None
        and not allow_blocked_shadow_input
    ):
        raise RuntimeError(
            f"{spec.name} map is blocked: {live_status['mapping_blocked_reason']}"
        )
    if live_status.get("transform_version") != snapshot.transform_version:
        raise RuntimeError(f"{spec.name} status/snapshot transform mismatch")

    try:
        source_sequence, detections, yolo_model_provenance = (
            validated_yolo_source(map_summary)
        )
    except RuntimeError as exc:
        raise RuntimeError(f"{spec.name} {exc}") from exc

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
    if metadata.object_goal.category != expected_goal_category:
        raise RuntimeError(
            f"{spec.name} source goal category "
            f"{metadata.object_goal.category!r} is not "
            f"{expected_goal_category!r}"
        )
    if metadata.base_T_camera is None:
        raise RuntimeError(
            f"{spec.name} source lacks measured base_T_camera; "
            "the VLM base pose cannot be reconstructed"
        )
    expected_rgb_suffix = ".jpg" if metadata.rgb_encoding == "jpeg" else ".png"
    if rgb_copy.suffix.lower() != expected_rgb_suffix:
        raise RuntimeError(f"{spec.name} RGB encoding/path mismatch")
    for payload_path, expected_size, expected_sha, label in (
        (
            rgb_copy,
            metadata.rgb_size_bytes,
            metadata.rgb_sha256,
            "RGB",
        ),
        (
            depth_copy,
            metadata.depth_size_bytes,
            metadata.depth_sha256,
            "depth",
        ),
    ):
        if payload_path.stat().st_size != expected_size:
            raise RuntimeError(f"{spec.name} {label} byte count mismatch")
        if sha256_file(payload_path) != expected_sha:
            raise RuntimeError(f"{spec.name} {label} SHA-256 mismatch")
    rgb_bgr = cv2.imread(str(rgb_copy), cv2.IMREAD_COLOR)
    if rgb_bgr is None:
        raise RuntimeError(f"failed to decode {rgb_copy}")
    T_shared_camera = np.asarray(
        metadata.pose.shared_T_camera.matrix, dtype=np.float64
    ).reshape(4, 4)
    base_T_camera = np.asarray(
        metadata.base_T_camera.matrix, dtype=np.float64
    ).reshape(4, 4)
    T_shared_base = shared_base_pose_from_camera(
        T_shared_camera,
        base_T_camera,
    )

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
        T_shared_base=T_shared_base,
        yolo_model_provenance=yolo_model_provenance,
        robot_trajectory_xy_m=robot_trajectory,
        robot_trajectory_provenance=robot_trajectory_provenance,
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


def source_visited_paths(
    contexts: list[RobotContext],
    *,
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    shape_hw: tuple[int, int],
) -> tuple[
    list[list[tuple[int, int]]],
    dict[str, dict[str, object]],
]:
    """Convert frozen observed base trajectories to source visited masks."""

    paths: list[list[tuple[int, int]]] = []
    report: dict[str, dict[str, object]] = {}
    for context in contexts:
        raw_path = context.robot_trajectory_xy_m
        cells: list[tuple[int, int]] = []
        for raw_point in raw_path:
            if (
                not isinstance(raw_point, (tuple, list))
                or len(raw_point) != 2
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in raw_point
                )
            ):
                raise ValueError(
                    f"{context.spec.robot_id} trajectory point is malformed"
                )
            cell = world_to_cell(
                (float(raw_point[0]), float(raw_point[1])),
                origin_xy_m,
                resolution_m,
                shape_hw,
            )
            if not cells or cell != cells[-1]:
                cells.append(cell)
        paths.append(cells)
        report[context.spec.robot_id] = {
            "observed_world_point_count": len(raw_path),
            "source_cell_path_count": len(cells),
            "trajectory_provenance": (
                context.robot_trajectory_provenance
            ),
            "classification": (
                "observed base trajectory converted to source-derived "
                "visited_vis decision-map layer"
            ),
        }
    return paths, report


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
    parser.add_argument(
        "--realworld-session-id",
        default=None,
        help="bind this frozen shadow run to one persistent deployment session",
    )
    parser.add_argument(
        "--realworld-session-contract-sha256",
        default=None,
        help="bind this run to the immutable portion of that session manifest",
    )
    parser.add_argument("--vlm-timeout-s", type=float, default=300.0)
    parser.add_argument(
        "--require-complete-vlm",
        action="store_true",
        help=(
            "fail the round if any Perception/Judgment/Decision request "
            "falls back after an error"
        ),
    )
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
    if (args.realworld_session_id is None) != (
        args.realworld_session_contract_sha256 is None
    ):
        parser.error(
            "real-world session ID and contract SHA-256 must be supplied together"
        )
    if args.realworld_session_contract_sha256 is not None and (
        len(args.realworld_session_contract_sha256) != 64
        or any(
            character not in "0123456789abcdef"
            for character in args.realworld_session_contract_sha256.lower()
        )
    ):
        parser.error("real-world session contract must be a SHA-256 value")
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
        "realworld_session_id": args.realworld_session_id,
        "realworld_session_contract_sha256": (
            args.realworld_session_contract_sha256
        ),
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
        "source_execution_profile": {
            "profile": "authoritative_default_unpruned_path",
            "enable_pruning": False,
            "vlm_image_transport": {
                "byte_encoding": "PNG",
                "data_uri_media_type": "image/jpeg",
                "camera_array": "RGB",
                "semantic_map_array": "source BGR passed to PIL unchanged",
            },
            "vlm_generation_request": {
                "model": "cogvlm2",
                "temperature": 0.8,
                "top_p": 0.8,
                "max_tokens": 1,
                "source_max_tokens": 2048,
                "max_tokens_adaptation": (
                    "consume only the first generated label token and its "
                    "first-step candidate scores"
                ),
            },
            "decision_effect": (
                "Perception VLM, Judgment VLM, source gate, unpruned "
                "Decision VLM score argmax, directional history, sequential "
                "shared-frontier removal, source palette/480px decision "
                "canvas and semantic target override"
            ),
            "optional_mechanisms_without_selection_effect": {
                "room_segmentation_and_room_semantics": (
                    "computed by source for visualization/analysis and "
                    "active-patch bookkeeping; not supplied to the Decision "
                    "VLM while enable_pruning is false"
                ),
                "attention_dod": (
                    "post-decision logging/visualization only"
                ),
                "active_patches": (
                    "post-decision state for a possible future pruning call; "
                    "does not alter selection while pruning is disabled"
                ),
            },
            "source_argument_note": (
                "arguments.py declares run_mode='pruned', but the executed "
                "main.py decision branch checks only enable_pruning and "
                "parse_args does not translate run_mode into that flag"
            ),
            "source_paths": [
                "source/Focus_realworld/arguments.py",
                "source/Focus_realworld/main.py",
            ],
        },
        "source_behavior_contract_version": (
            SOURCE_BEHAVIOR_CONTRACT_VERSION
        ),
        "source_code_artifacts": observe_reviewed_source_artifacts(
            WORKSPACE
        ),
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
            expected_goal_category=args.goal_category,
        )
        for spec in specs
    ]
    semantic_contract = semantic_input_contract(contexts)
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
    extracted_scene_objects = extract_scene_objects(
        decision_grid[2 : 2 + len(HM3D_CATEGORY_NAMES)],
        HM3D_CATEGORY_NAMES,
    )
    scene_objects = format_scene_objects_for_prompt(
        extracted_scene_objects,
        shape_hw=(
            int(decision_grid.shape[1]),
            int(decision_grid.shape[2]),
        ),
    )
    scene_labels = semantic_label_points(extracted_scene_objects)
    visited_paths_rc, visited_path_report = source_visited_paths(
        contexts,
        origin_xy_m=fused_origin,
        resolution_m=resolution_m,
        shape_hw=(
            int(decision_grid.shape[1]),
            int(decision_grid.shape[2]),
        ),
    )
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
        "vlm_frontier_contract": {
            "scope": "one shared fused-map A-D set",
            "label_identity": "stable across image, prompt, score vector and target",
            "per_robot_view": "remaining shared candidates in canonical A-D order",
            "allocation": "selected frontier removed before the next robot",
            "duplicate_physical_frontier_targets": False,
            "stable_id_binding": (
                "one shared component ID is preserved across rendered letter, "
                "prompt coordinate, requested score token, selected target "
                "and transport provenance"
            ),
            "source_later_agent_image_prompt_mismatch_corrected": True,
            "semantic_polygon_binding": (
                "prompt polygons and rendered labels share source-flipped "
                "480px display coordinates"
            ),
            "history_label_binding": (
                "a-z then A-Z IDs are stable across Judgment image, prompt "
                "and source-score selection"
            ),
            "source_history_image_prompt_mismatch_corrected": True,
            "source_single_frontier_reuse_suppressed": True,
            "extraction": (
                "source main.py::Frontiers largest explored contour, 5x5 "
                "close, 3x3 obstacle dilation, 8-connected components"
            ),
            "minimum_component_cells": 5,
            "source_first_region_property_skipped": True,
            "decision_canvas_px": 480,
            "decision_palette": "source constants.py color_palette",
        },
        "scene_objects_for_vlm": scene_objects,
        "source_visited_paths": visited_path_report,
        "source_semantic_goals": semantic_goal_records,
        "semantic_input_contract": semantic_contract,
        "input_timing": timing,
        "input_artifacts": [
            artifact for context in contexts for artifact in context.artifacts
        ],
        "yolo_model_provenance": {
            context.spec.robot_id: context.yolo_model_provenance
            for context in contexts
        },
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

    try:
        glm_server_contract = validate_glm_server_contract(args.glm_url)
    except Exception as exc:
        manifest.update(
            {
                "status": "failed_glm_server_contract",
                "glm_server_contract_error": f"{type(exc).__name__}: {exc}",
            }
        )
        atomic_write_json(manifest_path, manifest)
        raise
    manifest["glm_server_contract"] = glm_server_contract
    atomic_write_json(manifest_path, manifest)

    remaining = list(frontiers)
    robot_results: list[dict[str, object]] = []
    allocations: dict[str, Frontier] = {}
    selections: dict[str, dict[str, object]] = {}
    shared_memory = None if scene_state is None else scene_state.memory
    remaining_history_indices: list[int] | None = None
    remaining_history_scores: dict[int, float] | None = None
    for index, context in enumerate(contexts, start=1):
        # ABCD belongs to the shared fused map, not independently to each
        # robot.  Each robot receives the still-unallocated stable subset.  The
        # simulator source reuses a sole frontier, but duplicate physical
        # targets can make two robots converge on one point, so the real-world
        # coordination adapter holds the later robot instead.
        frontier_reused = False
        decision_frontiers = list(remaining)
        candidate_frontiers = list(decision_frontiers)
        robot_xy = (
            float(context.T_shared_base[0, 3]),
            float(context.T_shared_base[1, 3]),
        )
        robot_rc = world_to_cell(
            robot_xy,
            fused_origin,
            resolution_m,
            decision_grid.shape[1:],
        )
        heading_deg = heading_deg_from_base_pose(context.T_shared_base)
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
            semantic_labels=scene_labels,
            goal_category=args.goal_category,
            visited_paths_rc=visited_paths_rc,
        )
        decision_map = render_semantic_decision_map(
            decision_grid,
            HM3D_CATEGORY_NAMES,
            decision_frontiers,
            robot_rc,
            heading_deg,
            pre_goal_rc=pre_goal_point,
            semantic_labels=scene_labels,
            goal_category=args.goal_category,
            visited_paths_rc=visited_paths_rc,
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
            fail_fast=args.require_complete_vlm,
        )
        elapsed_s = time.perf_counter() - call_started
        if args.require_complete_vlm:
            try:
                require_complete_cascade_result(
                    cascade, robot_id=context.spec.robot_id
                )
            except RuntimeError as exc:
                manifest.update(
                    {
                        "status": "failed_incomplete_vlm",
                        "failed_robot_id": context.spec.robot_id,
                        "vlm_errors": list(cascade.errors),
                    }
                )
                atomic_write_json(manifest_path, manifest)
                raise
        chosen = cascade.frontier_choice.frontier if cascade.frontier_choice else None
        exploration_selection: dict[str, object] | None = None
        if chosen is not None:
            if all(item.frontier_id != chosen.frontier_id for item in decision_frontiers):
                raise RuntimeError("VLM returned a frontier outside its candidate set")
            allocations[context.spec.robot_id] = chosen
            remaining = [
                item for item in remaining if item.frontier_id != chosen.frontier_id
            ]
            exploration_selection = {
                "kind": "frontier",
                "target_id": chosen.frontier_id,
                **frontier_record(chosen),
                "source_behavior": "sequential frontier removed before next robot",
            }
        candidate_history_nodes: list[dict[str, object]] = []
        if scene_state is not None and remaining_history_indices is None:
            # This is the source's shared ``history_nodes_copy`` snapshot,
            # created during agent 0's pass and consumed sequentially.
            remaining_history_indices = list(range(len(memory.history_nodes)))
            remaining_history_scores = {
                history_index: memory.history_score[history_index]
                for history_index in remaining_history_indices
            }
        if (
            scene_state is not None
            and remaining_history_indices is not None
            and remaining_history_scores is not None
        ):
            # Preserve the exact candidate/score snapshot supplied to this
            # robot's source history branch.  Physical replanning can then use
            # the next source-ranked history point without inventing an A-D
            # fallback after a rejected history leg.
            for history_index in remaining_history_indices:
                history_row, history_col = memory.history_nodes[history_index]
                candidate_history_nodes.append(
                    {
                        "frontier_id": f"history-{history_index}",
                        "history_index": history_index,
                        "row": history_row,
                        "col": history_col,
                        "x_m": (
                            fused_origin[0]
                            + (history_col + 0.5) * resolution_m
                        ),
                        "y_m": (
                            fused_origin[1]
                            + (history_row + 0.5) * resolution_m
                        ),
                        "history_score": remaining_history_scores[
                            history_index
                        ],
                    }
                )
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
            "heading_deg_base_forward": heading_deg,
            "robot_pose_source": (
                "shared_T_camera @ inverse(measured base_T_camera)"
            ),
            "detections": context.detections,
            "candidate_frontiers": [
                frontier_record(frontier)
                for frontier in candidate_frontiers
            ],
            "candidate_history_nodes": candidate_history_nodes,
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
                float(context.T_shared_base[0, 3]),
                float(context.T_shared_base[1, 3]),
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
        "remaining_history_nodes": (
            []
            if (
                scene_state is None
                or remaining_history_indices is None
                or remaining_history_scores is None
            )
            else [
                {
                    "frontier_id": f"history-{history_index}",
                    "history_index": history_index,
                    "row": scene_state.memory.history_nodes[
                        history_index
                    ][0],
                    "col": scene_state.memory.history_nodes[
                        history_index
                    ][1],
                    "x_m": (
                        fused_origin[0]
                        + (
                            scene_state.memory.history_nodes[
                                history_index
                            ][1]
                            + 0.5
                        )
                        * resolution_m
                    ),
                    "y_m": (
                        fused_origin[1]
                        + (
                            scene_state.memory.history_nodes[
                                history_index
                            ][0]
                            + 0.5
                        )
                        * resolution_m
                    ),
                    "history_score": remaining_history_scores[
                        history_index
                    ],
                }
                for history_index in remaining_history_indices
            ]
        ),
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
