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
from typing import Any, Iterable

import cv2
import numpy as np

from .central_mapping import HM3D_CATEGORY_NAMES
from .map_snapshot import MapSnapshot, load_map_snapshot
from .models import ObservationMetadata
from .shadow_coordination import (
    heading_deg_from_base_pose,
    shared_base_pose_from_camera,
)
from .source_behavior_contract import (
    SOURCE_BEHAVIOR_CONTRACT_VERSION,
    validate_source_artifact_records,
)
from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2
from .vlm_decision import GLM_SERVER_MODEL_ID


_FRONTIER_LABELS = ("A", "B", "C", "D")


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


def _validated_frontier_record(
    raw: object,
    *,
    context: str,
) -> dict[str, object]:
    if not isinstance(raw, dict):
        raise ValueError(f"{context} frontier record is not an object")
    frontier_id = raw.get("frontier_id")
    row = raw.get("row")
    col = raw.get("col")
    size_cells = raw.get("size_cells")
    x_m = raw.get("x_m")
    y_m = raw.get("y_m")
    if frontier_id not in _FRONTIER_LABELS:
        raise ValueError(f"{context} frontier label is not in A-D")
    for name, value in (("row", row), ("col", col), ("size_cells", size_cells)):
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < (1 if name == "size_cells" else 0)
        ):
            raise ValueError(f"{context} frontier {name} is invalid")
    for name, value in (("x_m", x_m), ("y_m", y_m)):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise ValueError(f"{context} frontier {name} is invalid")
    return {
        "frontier_id": str(frontier_id),
        "row": int(row),
        "col": int(col),
        "x_m": float(x_m),
        "y_m": float(y_m),
        "size_cells": int(size_cells),
    }


def _validated_frontier_list(
    raw: object,
    *,
    context: str,
    require_prefix: bool = False,
) -> list[dict[str, object]]:
    if not isinstance(raw, list):
        raise ValueError(f"{context} frontier list is malformed")
    if len(raw) > len(_FRONTIER_LABELS):
        raise ValueError(f"{context} has more than four frontier candidates")
    records = [
        _validated_frontier_record(item, context=context)
        for item in raw
    ]
    labels = tuple(str(item["frontier_id"]) for item in records)
    if len(set(labels)) != len(labels):
        raise ValueError(f"{context} repeats a frontier label")
    canonical = tuple(
        label for label in _FRONTIER_LABELS if label in set(labels)
    )
    if labels != canonical:
        raise ValueError(f"{context} does not preserve canonical A-D order")
    if require_prefix and labels != _FRONTIER_LABELS[: len(labels)]:
        raise ValueError(f"{context} is not a contiguous A-D prefix")
    return records


def _validated_history_list(
    raw: object,
    *,
    context: str,
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    shape_hw: tuple[int, int],
) -> list[dict[str, object]]:
    if not isinstance(raw, list):
        raise ValueError(f"{context} history list is malformed")
    records: list[dict[str, object]] = []
    seen: set[int] = set()
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError(f"{context} history record is malformed")
        history_index = item.get("history_index")
        row = item.get("row")
        col = item.get("col")
        score = item.get("history_score")
        frontier_id = item.get("frontier_id")
        if (
            isinstance(history_index, bool)
            or not isinstance(history_index, int)
            or history_index < 0
            or history_index in seen
            or frontier_id != f"history-{history_index}"
        ):
            raise ValueError(f"{context} history identity is invalid")
        if (
            isinstance(row, bool)
            or isinstance(col, bool)
            or not isinstance(row, int)
            or not isinstance(col, int)
            or not 0 <= row < shape_hw[0]
            or not 0 <= col < shape_hw[1]
        ):
            raise ValueError(f"{context} history cell is invalid")
        if (
            isinstance(score, bool)
            or not isinstance(score, (int, float))
            or not math.isfinite(float(score))
        ):
            raise ValueError(f"{context} history score is invalid")
        expected_x = origin_xy_m[0] + (col + 0.5) * resolution_m
        expected_y = origin_xy_m[1] + (row + 0.5) * resolution_m
        x_m = item.get("x_m")
        y_m = item.get("y_m")
        if (
            isinstance(x_m, bool)
            or isinstance(y_m, bool)
            or not isinstance(x_m, (int, float))
            or not isinstance(y_m, (int, float))
            or not math.isclose(
                float(x_m), expected_x, rel_tol=0.0, abs_tol=1e-9
            )
            or not math.isclose(
                float(y_m), expected_y, rel_tol=0.0, abs_tol=1e-9
            )
        ):
            raise ValueError(
                f"{context} history world/grid binding is invalid"
            )
        seen.add(history_index)
        records.append(
            {
                "frontier_id": frontier_id,
                "history_index": history_index,
                "row": row,
                "col": col,
                "x_m": float(x_m),
                "y_m": float(y_m),
                "history_score": float(score),
            }
        )
    return records


def _fused_grid_geometry(
    manifest: dict[str, Any],
) -> tuple[tuple[float, float], float, tuple[int, int]]:
    raw_origin = manifest.get("fused_origin_xy_m")
    raw_resolution = manifest.get("resolution_m")
    raw_shape = manifest.get("fused_shape")
    if (
        not isinstance(raw_origin, list)
        or len(raw_origin) != 2
        or not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in raw_origin
        )
        or isinstance(raw_resolution, bool)
        or not isinstance(raw_resolution, (int, float))
        or not math.isfinite(float(raw_resolution))
        or float(raw_resolution) <= 0.0
        or not isinstance(raw_shape, list)
        or len(raw_shape) != 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value <= 0
            for value in raw_shape
        )
    ):
        raise ValueError("VLM manifest has malformed fused-grid geometry")
    return (
        (float(raw_origin[0]), float(raw_origin[1])),
        float(raw_resolution),
        (int(raw_shape[1]), int(raw_shape[2])),
    )


def _validate_frontier_grid_binding(
    frontiers: list[dict[str, object]],
    *,
    origin_xy_m: tuple[float, float],
    resolution_m: float,
    shape_hw: tuple[int, int],
) -> None:
    """Prove every A-D marker cell and world target are the same point."""

    height, width = shape_hw
    for frontier in frontiers:
        row = int(frontier["row"])
        col = int(frontier["col"])
        if not 0 <= row < height or not 0 <= col < width:
            raise ValueError(
                f"VLM frontier {frontier['frontier_id']} is outside fused grid"
            )
        expected_x = origin_xy_m[0] + (col + 0.5) * resolution_m
        expected_y = origin_xy_m[1] + (row + 0.5) * resolution_m
        if not math.isclose(
            float(frontier["x_m"]),
            expected_x,
            rel_tol=0.0,
            abs_tol=1e-9,
        ) or not math.isclose(
            float(frontier["y_m"]),
            expected_y,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(
                f"VLM frontier {frontier['frontier_id']} cell/world "
                "coordinates differ"
            )


def _validate_robot_pose_binding(
    manifest: dict[str, Any],
    result: dict[str, Any],
    metadata: ObservationMetadata,
) -> None:
    """Bind the VLM red arrow and target bearing to measured base_link."""

    if metadata.base_T_camera is None:
        return
    shared_camera = np.asarray(
        metadata.pose.shared_T_camera.matrix, dtype=np.float64
    ).reshape(4, 4)
    base_camera = np.asarray(
        metadata.base_T_camera.matrix, dtype=np.float64
    ).reshape(4, 4)
    shared_base = shared_base_pose_from_camera(shared_camera, base_camera)
    expected_xy = (
        float(shared_base[0, 3]),
        float(shared_base[1, 3]),
    )
    raw_xy = result.get("robot_xy_m")
    if (
        not isinstance(raw_xy, list)
        or len(raw_xy) != 2
        or not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in raw_xy
        )
        or not np.allclose(
            np.asarray(raw_xy, dtype=np.float64),
            np.asarray(expected_xy),
            rtol=0.0,
            atol=1e-9,
        )
    ):
        raise ValueError(
            f"{metadata.robot_id} VLM position is not the measured base pose"
        )
    if result.get("robot_pose_source") != (
        "shared_T_camera @ inverse(measured base_T_camera)"
    ):
        raise ValueError(f"{metadata.robot_id} VLM base-pose source is invalid")
    raw_heading = result.get("heading_deg_base_forward")
    if (
        isinstance(raw_heading, bool)
        or not isinstance(raw_heading, (int, float))
        or not math.isfinite(float(raw_heading))
    ):
        raise ValueError(f"{metadata.robot_id} VLM base heading is invalid")
    expected_heading = heading_deg_from_base_pose(shared_base)
    heading_error = (
        float(raw_heading) - expected_heading + 180.0
    ) % 360.0 - 180.0
    if abs(heading_error) > 1e-9:
        raise ValueError(
            f"{metadata.robot_id} VLM heading is not the measured base yaw"
        )

    origin, resolution, shape_hw = _fused_grid_geometry(manifest)
    expected_rc = (
        math.floor((expected_xy[1] - origin[1]) / resolution),
        math.floor((expected_xy[0] - origin[0]) / resolution),
    )
    raw_rc = result.get("robot_rc")
    if (
        not isinstance(raw_rc, list)
        or len(raw_rc) != 2
        or any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in raw_rc
        )
        or tuple(raw_rc) != expected_rc
        or not (
            0 <= expected_rc[0] < shape_hw[0]
            and 0 <= expected_rc[1] < shape_hw[1]
        )
    ):
        raise ValueError(
            f"{metadata.robot_id} VLM map cell is not the measured base cell"
        )


def _frontier_record_matches(
    left: dict[str, object],
    right: dict[str, object],
) -> bool:
    return (
        left["frontier_id"] == right["frontier_id"]
        and left["row"] == right["row"]
        and left["col"] == right["col"]
        and left["size_cells"] == right["size_cells"]
        and math.isclose(
            float(left["x_m"]),
            float(right["x_m"]),
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        and math.isclose(
            float(left["y_m"]),
            float(right["y_m"]),
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    )


def _validate_semantic_input_contract(
    manifest: dict[str, Any],
    *,
    robot_ids: set[str],
    required: bool = False,
) -> None:
    """Validate the optional v1 semantic-provenance contract.

    Older frozen rounds predate this record and remain replayable.  New
    rounds must prove that both sides used the same pixel backend, temporal
    fusion policy and YOLO map-mutation policy before their channels were
    max-fused.
    """

    contract = manifest.get("semantic_input_contract")
    if contract is None:
        if required:
            raise ValueError(
                "new source behavior contract requires semantic input "
                "provenance"
            )
        return
    robots = (
        contract.get("robots") if isinstance(contract, dict) else None
    )
    backend = (
        contract.get("pixel_segmenter_backend")
        if isinstance(contract, dict)
        else None
    )
    fusion_mode = (
        contract.get("semantic_fusion_mode")
        if isinstance(contract, dict)
        else None
    )
    reinforcement = (
        contract.get("yolo_map_reinforcement_enabled")
        if isinstance(contract, dict)
        else None
    )
    expected_maskrcnn_availability = (
        backend == "source_rednet_detectron2_hm3d15"
    )
    if (
        not isinstance(contract, dict)
        or contract.get("schema_version")
        != "focus-vlm-semantic-input-contract-v1"
        or contract.get("uniform_across_robots") is not True
        or not isinstance(backend, str)
        or not backend
        or fusion_mode not in {"max", "multi_view"}
        or not isinstance(reinforcement, bool)
        or contract.get("hm3d_category_order")
        != list(HM3D_CATEGORY_NAMES)
        or contract.get("source_maskrcnn_override_available_in_hub")
        is not expected_maskrcnn_availability
        or not isinstance(
            contract.get("pixel_model_classification"), str
        )
        or not isinstance(robots, dict)
        or set(robots) != robot_ids
    ):
        raise ValueError("shadow semantic input contract is malformed")
    for robot_id, record in robots.items():
        if (
            not isinstance(record, dict)
            or record.get("pixel_segmenter_backend") != backend
            or record.get("semantic_fusion_mode") != fusion_mode
            or record.get("yolo_map_reinforcement_enabled")
            is not reinforcement
        ):
            raise ValueError(
                f"{robot_id} semantic input differs from the shared contract"
            )


def _validate_source_behavior_contract(
    manifest: dict[str, Any],
) -> bool:
    """Validate a new pinned source contract; identify legacy replay input."""

    version = manifest.get("source_behavior_contract_version")
    if version is None:
        modern_fields = {
            "source_code_artifacts",
            "source_execution_profile",
            "semantic_input_contract",
        }
        if modern_fields.intersection(manifest):
            raise ValueError(
                "shadow manifest has modern source records without their "
                "versioned behavior contract"
            )
        return False
    if version != SOURCE_BEHAVIOR_CONTRACT_VERSION:
        raise ValueError("shadow source behavior contract version is unsupported")
    validate_source_artifact_records(manifest.get("source_code_artifacts"))
    return True


def _validate_vlm_selection_bindings(
    manifest: dict[str, Any],
    robot_results_raw: list[dict[str, Any]],
    selections: dict[str, Any],
) -> None:
    """Prove the VLM image/prompt/score/selection ABCD mapping end to end."""

    glm_contract = manifest.get("glm_server_contract")
    if (
        not isinstance(glm_contract, dict)
        or glm_contract.get("model_id") != GLM_SERVER_MODEL_ID
        or glm_contract.get("candidate_score_contract")
        != (
            "sum exact label unspaced+leading-space token mass; "
            "zero/non-finite mass raises"
        )
        or glm_contract.get("verification")
        != "observed local /models response"
    ):
        raise ValueError(
            "shadow manifest lacks the reviewed GLM candidate-score contract"
        )
    pinned_source_contract = _validate_source_behavior_contract(manifest)
    _validate_semantic_input_contract(
        manifest,
        robot_ids={
            str(record.get("robot_id", ""))
            for record in robot_results_raw
        },
        required=pinned_source_contract,
    )
    execution_profile = manifest.get("source_execution_profile")
    if pinned_source_contract and execution_profile is None:
        raise ValueError(
            "new source behavior contract requires an execution profile"
        )
    if execution_profile is not None:
        optional = (
            execution_profile.get(
                "optional_mechanisms_without_selection_effect"
            )
            if isinstance(execution_profile, dict)
            else None
        )
        image_transport = (
            execution_profile.get("vlm_image_transport")
            if isinstance(execution_profile, dict)
            else None
        )
        generation_request = (
            execution_profile.get("vlm_generation_request")
            if isinstance(execution_profile, dict)
            else None
        )
        if (
            not isinstance(execution_profile, dict)
            or execution_profile.get("profile")
            != "authoritative_default_unpruned_path"
            or execution_profile.get("enable_pruning") is not False
            or not isinstance(optional, dict)
            or set(optional)
            != {
                "room_segmentation_and_room_semantics",
                "attention_dod",
                "active_patches",
            }
            or execution_profile.get("source_paths")
            != [
                "source/Focus_realworld/arguments.py",
                "source/Focus_realworld/main.py",
            ]
            or (
                pinned_source_contract
                and image_transport
                != {
                    "byte_encoding": "PNG",
                    "data_uri_media_type": "image/jpeg",
                    "camera_array": "RGB",
                    "semantic_map_array": (
                        "source BGR passed to PIL unchanged"
                    ),
                }
            )
            or (
                pinned_source_contract
                and generation_request
                != {
                    "model": "cogvlm2",
                    "temperature": 0.8,
                    "top_p": 0.8,
                    "max_tokens": 1,
                    "source_max_tokens": 2048,
                    "max_tokens_adaptation": (
                        "consume only the first generated label token and "
                        "its first-step candidate scores"
                    ),
                }
            )
        ):
            raise ValueError(
                "shadow manifest has an unsupported source execution profile"
            )
    contract = manifest.get("vlm_frontier_contract")
    if not isinstance(contract, dict):
        raise ValueError("shadow manifest lacks the VLM frontier contract")
    per_robot_view = contract.get("per_robot_view")
    owner_filtered_frontiers = per_robot_view == (
        "locally supported subset of remaining shared candidates "
        "in canonical A-D order"
    )
    if (
        contract.get("scope") != "one shared fused-map A-D set"
        or contract.get("label_identity")
        != "stable across image, prompt, score vector and target"
        or per_robot_view
        not in {
            "remaining shared candidates in canonical A-D order",
            (
                "locally supported subset of remaining shared candidates "
                "in canonical A-D order"
            ),
        }
        or contract.get("allocation")
        != "selected frontier removed before the next robot"
        or contract.get("duplicate_physical_frontier_targets") is not False
        or (
            owner_filtered_frontiers
            and (
                contract.get("frontier_ownership_filter") is not True
                or contract.get("disconnected_component_balance") is not True
            )
        )
        or (
            not owner_filtered_frontiers
            and (
                contract.get("frontier_ownership_filter", False) is not False
                or contract.get("disconnected_component_balance", False)
                is not False
            )
        )
        or (
            "source_later_agent_image_prompt_mismatch_corrected" in contract
            and contract.get(
                "source_later_agent_image_prompt_mismatch_corrected"
            )
            is not True
        )
    ):
        raise ValueError("shadow manifest has an unsupported VLM frontier contract")
    source_geometry_fields = {
        "extraction",
        "minimum_component_cells",
        "source_first_region_property_skipped",
        "decision_canvas_px",
        "decision_palette",
        "stable_id_binding",
        "source_later_agent_image_prompt_mismatch_corrected",
        "semantic_polygon_binding",
        "history_label_binding",
        "source_history_image_prompt_mismatch_corrected",
        "source_single_frontier_reuse_suppressed",
    }
    if (
        pinned_source_contract
        or source_geometry_fields.intersection(contract)
    ) and (
        contract.get("extraction")
        != (
            "source main.py::Frontiers largest explored contour, 5x5 "
            "close, 3x3 obstacle dilation, 8-connected components"
        )
        or contract.get("minimum_component_cells") != 5
        or contract.get("source_first_region_property_skipped") is not True
        or contract.get("decision_canvas_px") != 480
        or contract.get("decision_palette")
        != "source constants.py color_palette"
        or contract.get("stable_id_binding")
        != (
            "one shared component ID is preserved across rendered letter, "
            "prompt coordinate, requested score token, selected target "
            "and transport provenance"
        )
        or contract.get(
            "source_later_agent_image_prompt_mismatch_corrected"
        )
        is not True
        or contract.get("semantic_polygon_binding")
        != (
            "prompt polygons and rendered labels share source-flipped "
            "480px display coordinates"
        )
        or contract.get("history_label_binding")
        != (
            "a-z then A-Z IDs are stable across Judgment image, prompt "
            "and source-score selection"
        )
        or contract.get(
            "source_history_image_prompt_mismatch_corrected"
        )
        is not True
        or contract.get("source_single_frontier_reuse_suppressed")
        is not True
    ):
        raise ValueError(
            "shadow manifest has unsupported source frontier geometry"
        )

    shared = _validated_frontier_list(
        manifest.get("frontiers"),
        context="shared VLM",
        require_prefix=True,
    )
    origin_xy_m, resolution_m, shape_hw = _fused_grid_geometry(manifest)
    _validate_frontier_grid_binding(
        shared,
        origin_xy_m=origin_xy_m,
        resolution_m=resolution_m,
        shape_hw=shape_hw,
    )
    frontier_ownership: dict[str, set[str]] = {}
    if owner_filtered_frontiers:
        ownership_raw = manifest.get("frontier_ownership")
        shared_ids = {
            str(record["frontier_id"]) for record in shared
        }
        robot_ids = {
            str(record.get("robot_id", ""))
            for record in robot_results_raw
        }
        if (
            not isinstance(ownership_raw, dict)
            or set(ownership_raw) != shared_ids
            or "" in robot_ids
        ):
            raise ValueError(
                "owner-filtered VLM frontier provenance is malformed"
            )
        for frontier_id in shared_ids:
            ownership_record = ownership_raw[frontier_id]
            eligible_raw = (
                ownership_record.get("eligible_robot_ids")
                if isinstance(ownership_record, dict)
                else None
            )
            source_records = (
                ownership_record.get("source_local_frontiers")
                if isinstance(ownership_record, dict)
                else None
            )
            if (
                not isinstance(ownership_record, dict)
                or ownership_record.get("frontier_id") != frontier_id
                or not isinstance(eligible_raw, list)
                or not eligible_raw
                or any(
                    not isinstance(robot_id, str) or not robot_id
                    for robot_id in eligible_raw
                )
                or len(set(eligible_raw)) != len(eligible_raw)
                or not set(eligible_raw).issubset(robot_ids)
                or ownership_record.get("fabricated") is not False
                or not isinstance(source_records, list)
                or not source_records
            ):
                raise ValueError(
                    f"owner-filtered VLM frontier {frontier_id} "
                    "provenance is malformed"
                )
            source_robot_ids = {
                str(record.get("robot_id", ""))
                for record in source_records
                if isinstance(record, dict)
            }
            if (
                not set(eligible_raw).issubset(source_robot_ids)
                or any(
                    not isinstance(record, dict)
                    or record.get("classification")
                    != (
                        "source-derived from observed frozen "
                        "robot-local map"
                    )
                    for record in source_records
                )
            ):
                raise ValueError(
                    f"owner-filtered VLM frontier {frontier_id} lacks "
                    "observed local-map provenance"
                )
            frontier_ownership[frontier_id] = set(eligible_raw)
    remaining = {
        str(record["frontier_id"]): record
        for record in shared
    }
    remaining_history: list[dict[str, object]] | None = None
    expected_orders = list(range(1, len(robot_results_raw) + 1))
    actual_orders = [record.get("allocation_order") for record in robot_results_raw]
    if actual_orders != expected_orders:
        raise ValueError("VLM robot allocation order is missing or ambiguous")

    for result in robot_results_raw:
        robot_id = str(result.get("robot_id", ""))
        errors = result.get("errors")
        if not isinstance(errors, list) or errors:
            raise ValueError(f"{robot_id} has an incomplete VLM cascade")
        candidates = _validated_frontier_list(
            result.get("candidate_frontiers"),
            context=f"{robot_id} candidate",
        )
        expected_candidates = [
            record
            for frontier_id, record in remaining.items()
            if (
                not owner_filtered_frontiers
                or robot_id in frontier_ownership[frontier_id]
            )
        ]
        if len(candidates) != len(expected_candidates) or any(
            not _frontier_record_matches(actual, expected)
            for actual, expected in zip(
                candidates,
                expected_candidates,
                strict=True,
            )
        ):
            raise ValueError(
                f"{robot_id} VLM candidates do not match the remaining shared ABCD set"
            )

        allocated_raw = result.get("allocated_frontier")
        probabilities = result.get("choice_probabilities")
        if not isinstance(probabilities, dict):
            raise ValueError(f"{robot_id} VLM choice probabilities are malformed")
        history_candidates: list[dict[str, object]] | None = None
        if "candidate_history_nodes" in result:
            history_candidates = _validated_history_list(
                result.get("candidate_history_nodes"),
                context=f"{robot_id} candidate",
                origin_xy_m=origin_xy_m,
                resolution_m=resolution_m,
                shape_hw=shape_hw,
            )
            if remaining_history is None:
                remaining_history = list(history_candidates)
            elif history_candidates != remaining_history:
                raise ValueError(
                    f"{robot_id} history candidates do not match the "
                    "remaining shared source-history snapshot"
                )
        allocated: dict[str, object] | None = None
        if allocated_raw is not None:
            allocated = _validated_frontier_record(
                allocated_raw,
                context=f"{robot_id} allocated",
            )
            candidate_by_id = {
                str(item["frontier_id"]): item for item in candidates
            }
            candidate = candidate_by_id.get(str(allocated["frontier_id"]))
            if candidate is None or not _frontier_record_matches(
                allocated,
                candidate,
            ):
                raise ValueError(
                    f"{robot_id} allocated frontier is outside its ABCD candidates"
                )
            if set(probabilities) != set(candidate_by_id):
                raise ValueError(
                    f"{robot_id} VLM score labels differ from its ABCD candidates"
                )
            score_values: list[float] = []
            for label in candidate_by_id:
                value = probabilities[label]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    or float(value) < 0.0
                ):
                    raise ValueError(
                        f"{robot_id} VLM score for {label} is invalid"
                    )
                score_values.append(float(value))
            if not math.isclose(
                math.fsum(score_values),
                1.0,
                rel_tol=0.0,
                abs_tol=1e-6,
            ):
                raise ValueError(
                    f"{robot_id} VLM scores are not a normalized candidate vector"
                )
            chosen_label = max(
                candidate_by_id,
                key=lambda label: float(probabilities[label]),
            )
            if chosen_label != allocated["frontier_id"]:
                raise ValueError(
                    f"{robot_id} allocated frontier differs from score argmax"
                )
            del remaining[str(allocated["frontier_id"])]
        elif probabilities:
            raise ValueError(
                f"{robot_id} has VLM choice scores but no allocated frontier"
            )

        selection = selections.get(robot_id)
        if result.get("final_shadow_selection") != selection:
            raise ValueError(
                f"{robot_id} final VLM selection differs across manifest sections"
            )

        # The source cascade commits its exploration choice before a
        # semantic target may override the final destination.  Validate and
        # consume that first-stage choice independently; otherwise a history
        # node selected immediately before a semantic override is left in the
        # validator's remaining set even though the source episode correctly
        # removed it.
        exploration = result.get(
            "exploration_selection_before_target_override"
        )
        if exploration is not None and not isinstance(exploration, dict):
            raise ValueError(
                f"{robot_id} exploration VLM selection is malformed"
            )
        exploration_kind = (
            None if exploration is None else exploration.get("kind")
        )
        if exploration_kind == "frontier":
            if allocated is None:
                raise ValueError(
                    f"{robot_id} frontier target has no score-selected ABCD source"
                )
            selected_record = _validated_frontier_record(
                exploration,
                context=f"{robot_id} selected",
            )
            if (
                exploration.get("target_id")
                != selected_record["frontier_id"]
                or not _frontier_record_matches(selected_record, allocated)
                or result.get("selected_history_index") is not None
            ):
                raise ValueError(
                    f"{robot_id} frontier target is not bound to its ABCD choice"
                )
        elif exploration_kind == "history":
            if (
                exploration.get("history_index")
                != result.get("selected_history_index")
            ):
                raise ValueError(
                    f"{robot_id} history target differs from the source gate choice"
                )
            if history_candidates is None:
                raise ValueError(
                    f"{robot_id} history target has no frozen candidates"
                )
            # Source history selections carry their stable identity as
            # ``target_id``; frozen candidate records carry the same identity
            # as ``frontier_id``. Normalize only that alias before validating
            # the complete grid/score binding.
            exploration_history_record = dict(exploration)
            exploration_history_record["frontier_id"] = exploration.get(
                "target_id"
            )
            validated_selection = _validated_history_list(
                [exploration_history_record],
                context=f"{robot_id} selected",
                origin_xy_m=origin_xy_m,
                resolution_m=resolution_m,
                shape_hw=shape_hw,
            )[0]
            selected_index = validated_selection["history_index"]
            selected = next(
                (
                    item
                    for item in history_candidates
                    if item["history_index"] == selected_index
                ),
                None,
            )
            if (
                selected is None
                or validated_selection != selected
                or exploration.get("target_id")
                != validated_selection["frontier_id"]
            ):
                raise ValueError(
                    f"{robot_id} history target is outside its frozen "
                    "source-history candidates"
                )
            if remaining_history is not None and len(remaining_history) > 1:
                remaining_history = [
                    item
                    for item in remaining_history
                    if item["history_index"] != selected_index
                ]
        elif exploration_kind is None:
            if result.get("selected_history_index") is not None:
                raise ValueError(
                    f"{robot_id} source history index has no exploration target"
                )
        else:
            raise ValueError(
                f"{robot_id} has unsupported exploration VLM selection kind"
            )

        if selection is None:
            if (
                exploration is not None
                or result.get("semantic_goal_override") is not None
            ):
                raise ValueError(
                    f"{robot_id} has an intermediate VLM target but no final selection"
                )
            continue
        if not isinstance(selection, dict):
            raise ValueError(f"{robot_id} final VLM selection is malformed")
        kind = selection.get("kind")
        if kind in {"frontier", "history"}:
            if (
                selection != exploration
                or result.get("semantic_goal_override") is not None
            ):
                raise ValueError(
                    f"{robot_id} final exploration target differs from "
                    "the source gate choice"
                )
        elif kind == "semantic_goal":
            if result.get("semantic_goal_override") != selection:
                raise ValueError(
                    f"{robot_id} semantic target differs across VLM result fields"
                )
        else:
            raise ValueError(f"{robot_id} has unsupported VLM selection kind")

    recorded_remaining = _validated_frontier_list(
        manifest.get("remaining_frontiers"),
        context="remaining VLM",
    )
    if len(recorded_remaining) != len(remaining) or any(
        not _frontier_record_matches(actual, expected)
        for actual, expected in zip(
            recorded_remaining,
            remaining.values(),
            strict=True,
        )
    ):
        raise ValueError("remaining VLM frontier set does not match allocations")
    if "remaining_history_nodes" in manifest:
        recorded_history = _validated_history_list(
            manifest.get("remaining_history_nodes"),
            context="remaining VLM",
            origin_xy_m=origin_xy_m,
            resolution_m=resolution_m,
            shape_hw=shape_hw,
        )
        if recorded_history != (remaining_history or []):
            raise ValueError(
                "remaining VLM history set does not match allocations"
            )


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
    forced_hold_robot_ids: Iterable[str] = (),
) -> SceneBatchBuild:
    """Build and preflight a two-robot v2 batch without publishing it.

    ``forced_hold_robot_ids`` is an execution-scope restriction, not a VLM
    rewrite. The original selections remain preserved and fully validated in
    the shadow manifest; listed robots are converted to explicit HOLD
    decisions before any command-capable batch is constructed.
    """

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
    selections_raw = manifest.get("final_shadow_selections", {})
    if not isinstance(selections_raw, dict):
        raise ValueError("final_shadow_selections is malformed")
    _validate_vlm_selection_bindings(
        manifest,
        robot_results_raw,
        selections_raw,
    )
    robot_ids = tuple(record["robot_id"] for record in robot_results_raw)
    forced_holds = frozenset(str(value) for value in forced_hold_robot_ids)
    unknown_forced_holds = forced_holds.difference(robot_ids)
    if unknown_forced_holds:
        raise ValueError(
            "forced HOLD contains unknown robot IDs: "
            + ", ".join(sorted(unknown_forced_holds))
        )
    execution_selections = {
        robot_id: selection
        for robot_id, selection in selections_raw.items()
        if robot_id not in forced_holds
    }
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
        if metadata.object_goal.category != manifest.get("goal_category"):
            raise ValueError(
                f"{robot_id} frozen observation goal category differs "
                "from the VLM round"
            )
        _validate_robot_pose_binding(manifest, result, metadata)
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
            if (
                robot_id in execution_selections
                and not bool(policy.get("allow_goal", False))
            ):
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

    active_robot_ids = tuple(
        robot_id for robot_id in robot_ids if robot_id in execution_selections
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
        selection_raw = execution_selections.get(robot_id)
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
                (
                    "operator-scoped diagnostic forced HOLD; original VLM "
                    f"selection preserved in frozen manifest {run_id}"
                )
                if robot_id in forced_holds
                else (
                    "source-faithful VLM "
                    f"{selection.get('kind') if selection else 'no-selection'} "
                    f"from frozen manifest {run_id}"
                )
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
        "source_active_robot_ids": [
            robot_id for robot_id in robot_ids if robot_id in selections_raw
        ],
        "forced_hold_robot_ids": sorted(forced_holds),
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
