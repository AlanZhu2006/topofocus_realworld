from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from focus_hub.models import ObservationMetadata
from focus_hub.v2_scene_batch import (
    build_batch_from_shadow_manifest,
    sha256_file,
)


ROBOTS = ("robot-0", "robot-1")
NAMES = {"robot-0": "wsj", "robot-1": "yunji"}
TRANSFORMS = {"robot-0": "wsj-test-v1", "robot-1": "yunji-test-v1"}
CALIBRATION = "shared-test-v1"


def artifact(path: Path, status: str = "observed test input") -> dict[str, object]:
    return {
        "source_path": str(path),
        "preserved_path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "status": status,
    }


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def prepare_round(
    tmp_path,
    observation_factory,
    *,
    mapping_only=False,
    health_ready: bool | None = None,
):
    now = 100_000_000_000
    artifacts = []
    results = []
    registry = {"robots": {}}
    for index, robot_id in enumerate(ROBOTS):
        name = NAMES[robot_id]
        input_dir = tmp_path / "inputs" / name
        input_dir.mkdir(parents=True)
        grid = np.zeros((17, 25, 25), dtype=np.float32)
        map_path = input_dir / "central_map.npz"
        np.savez_compressed(
            map_path,
            grid=grid,
            origin_xy_m=np.asarray([-0.625, -0.625]),
            resolution_m=np.asarray(0.05),
            frame_id=np.asarray("shared_world"),
            transform_version=np.asarray(TRANSFORMS[robot_id]),
            shared_frame_calibration_id=np.asarray(CALIBRATION),
            map_format_version=np.asarray("focus-hub-central-map-v3"),
        )
        sequence = 10 + index
        rgb_array = np.full((8, 8, 3), 30 + index, dtype=np.uint8)
        depth_array = np.full((8, 8), 1000, dtype=np.uint16)
        ok_rgb, rgb_encoded = cv2.imencode(".jpg", rgb_array)
        ok_depth, depth_encoded = cv2.imencode(".png", depth_array)
        assert ok_rgb and ok_depth
        rgb = rgb_encoded.tobytes()
        depth = depth_encoded.tobytes()
        rgb_path = input_dir / f"source_{sequence}.jpg"
        depth_path = input_dir / f"source_{sequence}_depth.png"
        rgb_path.write_bytes(rgb)
        depth_path.write_bytes(depth)
        raw = observation_factory(
            robot_id=robot_id,
            sequence=sequence,
            now_ns=now,
            mapping_only=mapping_only,
            health_ready=(
                not mapping_only if health_ready is None else health_ready
            ),
        ).model_dump(mode="json")
        raw["pose"]["transform_version"] = TRANSFORMS[robot_id]
        raw["rgb_size_bytes"] = len(rgb)
        raw["depth_size_bytes"] = len(depth)
        raw["rgb_sha256"] = hashlib.sha256(rgb).hexdigest()
        raw["depth_sha256"] = hashlib.sha256(depth).hexdigest()
        metadata = ObservationMetadata.model_validate(raw)
        metadata_path = input_dir / f"source_{sequence}_metadata.json"
        metadata_path.write_text(metadata.model_dump_json(), encoding="utf-8")
        for path in (map_path, metadata_path, rgb_path, depth_path):
            artifacts.append(artifact(path))
        payload_digest = hashlib.sha256(
            metadata.model_dump_json().encode("utf-8")
            + metadata.rgb_sha256.encode("ascii")
            + metadata.depth_sha256.encode("ascii")
        ).hexdigest()
        registry["robots"][robot_id] = {
            "last_sequence": sequence,
            "last_payload_digest": payload_digest,
            "map_version": 3,
        }
        results.append({
            "robot_id": robot_id,
            "name": name,
            "source_sequence": sequence,
            "source_capture_time_ns": metadata.capture_time_ns,
            "robot_xy_m": [float(index), 0.0],
            "map_transform_version": TRANSFORMS[robot_id],
            "map_snapshot_sha256": sha256_file(map_path),
            "input_mapping_blocked_reason": None,
        })

    mask = np.zeros((25, 25), dtype=np.uint8)
    mask[12, 12] = 255
    mask_path = tmp_path / "source_goal_masks" / "wsj_chair.png"
    mask_path.parent.mkdir()
    assert cv2.imwrite(str(mask_path), mask)
    semantic = {
        "kind": "semantic_goal",
        "target_id": "target-chair",
        "category": "chair",
        "evidence_status": "model_inference_map_projected_unverified",
        "mask_path": str(mask_path),
        "mask_size_bytes": mask_path.stat().st_size,
        "mask_sha256": sha256_file(mask_path),
        "size_cells": 1,
        "x_m": 0.0,
        "y_m": 0.0,
    }
    frontier = {
        "kind": "frontier",
        "target_id": "B",
        "x_m": 2.0,
        "y_m": 1.0,
    }
    fused_path = tmp_path / "fused_decision_map.npz"
    np.savez_compressed(
        fused_path,
        grid=np.zeros((17, 25, 25), dtype=np.float32),
        origin_xy_m=np.asarray([-0.625, -0.625]),
        resolution_m=np.asarray(0.05),
        frame_id=np.asarray("shared_world"),
        transform_version=np.asarray("multi-robot-source-derived"),
        shared_frame_calibration_id=np.asarray(CALIBRATION),
        map_format_version=np.asarray("focus-hub-central-map-v3"),
    )
    manifest = {
        "schema_version": "focus-vlm-shadow-v1",
        "run_id": "test-shadow-round",
        "status": "complete_shadow_only",
        "goal_category": "chair",
        "shared_frame_calibration_id": CALIBRATION,
        "safety": {"robot_commands_sent": False},
        "source_episode": {"logical_l_step": 24, "next_round_index": 2},
        "input_artifacts": artifacts,
        "decision_map_artifact": artifact(
            fused_path, "source-derived frozen fused VLM decision map"
        ),
        "robots": results,
        "final_shadow_selections": {
            "robot-0": semantic,
            "robot-1": frontier,
        },
    }
    manifest_path = tmp_path / "shadow_manifest.json"
    registry_path = tmp_path / "registry_state.json"
    config_path = tmp_path / "robots.json"
    write_json(manifest_path, manifest)
    write_json(registry_path, registry)
    write_json(config_path, {
        "robots": {
            robot_id: {
                "transform_version": TRANSFORMS[robot_id],
                "allow_goal": True,
            }
            for robot_id in ROBOTS
        }
    })
    return now, manifest_path, registry_path, config_path


def test_builds_semantic_and_frontier_concurrent_batch(tmp_path, observation_factory):
    now, manifest, registry, config = prepare_round(tmp_path, observation_factory)
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=4,
        now_ns=now,
        robot_config_path=config,
    )

    assert built.report["preflight_ready"] is True
    assert built.report["active_robot_ids"] == ["robot-0", "robot-1"]
    assert [decision.target.kind for decision in built.batch.decisions] == [
        "SEMANTIC_REGION",
        "FRONTIER_POINT",
    ]
    assert built.batch.decisions[0].target.region.component_size_cells == 1
    assert (
        built.batch.decisions[1].map_provenance.map_snapshot_sha256
        == sha256_file(tmp_path / "fused_decision_map.npz")
    )


def test_mapping_only_inputs_build_but_fail_preflight(tmp_path, observation_factory):
    now, manifest, registry, config = prepare_round(
        tmp_path, observation_factory, mapping_only=True
    )
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=0,
        now_ns=now,
        robot_config_path=config,
    )

    assert built.report["preflight_ready"] is False
    codes = [row["code"] for row in built.report["blockers"]]
    assert codes.count("INPUT_MAPPING_ONLY") == 2
    assert codes.count("BASE_T_CAMERA_ABSENT") == 2


def test_command_metadata_defers_nonfatal_health_to_live_receiver(
    tmp_path, observation_factory
):
    now, manifest, registry, config = prepare_round(
        tmp_path,
        observation_factory,
        mapping_only=False,
        health_ready=False,
    )
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=0,
        now_ns=now,
        robot_config_path=config,
    )

    assert built.report["preflight_ready"] is True
    unverified_codes = [
        row["code"] for row in built.report["unverified_runtime_checks"]
    ]
    assert unverified_codes.count("RUNTIME_HEALTH_RECHECK_REQUIRED") == 2
