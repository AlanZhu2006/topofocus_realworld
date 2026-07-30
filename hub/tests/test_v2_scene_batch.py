from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from focus_hub.models import ObservationMetadata
from focus_hub.source_behavior_contract import (
    SOURCE_BEHAVIOR_CONTRACT_VERSION,
    expected_source_artifact_records,
)
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
        base_x = 0.5 * index
        raw["pose"]["shared_T_camera"]["matrix"][3] = base_x
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
            "robot_xy_m": [base_x, 0.0],
            "robot_rc": [12, 12 + 10 * index],
            "heading_deg_base_forward": 0.0,
            "robot_pose_source": (
                "shared_T_camera @ inverse(measured base_T_camera)"
            ),
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
    frontier_a_record = {
        "frontier_id": "A",
        "row": 10,
        "col": 12,
        "x_m": 0.0,
        "y_m": -0.1,
        "size_cells": 30,
    }
    frontier_b_record = {
        "frontier_id": "B",
        "row": 20,
        "col": 22,
        "x_m": 0.5,
        "y_m": 0.4,
        "size_cells": 20,
    }
    frontier_a = {
        "kind": "frontier",
        "target_id": "A",
        **frontier_a_record,
        "source_behavior": "sequential frontier removed before next robot",
    }
    frontier = {
        "kind": "frontier",
        "target_id": "B",
        **frontier_b_record,
        "source_behavior": "sequential frontier removed before next robot",
    }
    results[0].update({
        "allocation_order": 1,
        "candidate_frontiers": [frontier_a_record, frontier_b_record],
        "allocated_frontier": frontier_a_record,
        "choice_probabilities": {"A": 0.8, "B": 0.2},
        "errors": [],
        "selected_history_index": None,
        "exploration_selection_before_target_override": frontier_a,
        "semantic_goal_override": semantic,
        "final_shadow_selection": semantic,
    })
    results[1].update({
        "allocation_order": 2,
        "candidate_frontiers": [frontier_b_record],
        "allocated_frontier": frontier_b_record,
        "choice_probabilities": {"B": 1.0},
        "errors": [],
        "selected_history_index": None,
        "exploration_selection_before_target_override": frontier,
        "semantic_goal_override": None,
        "final_shadow_selection": frontier,
    })
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
        "resolution_m": 0.05,
        "fused_origin_xy_m": [-0.625, -0.625],
        "fused_shape": [17, 25, 25],
        "glm_server_contract": {
            "model_id": "cogvlm2-19b-focus-score-contract-v1",
            "candidate_score_contract": (
                "sum exact label unspaced+leading-space token mass; "
                "zero/non-finite mass raises"
            ),
            "verification": "observed local /models response",
        },
        "safety": {"robot_commands_sent": False},
        "source_episode": {"logical_l_step": 24, "next_round_index": 2},
        "source_behavior_contract_version": (
            SOURCE_BEHAVIOR_CONTRACT_VERSION
        ),
        "source_code_artifacts": expected_source_artifact_records(),
        "source_execution_profile": {
            "profile": "authoritative_default_unpruned_path",
            "enable_pruning": False,
            "vlm_image_transport": {
                "byte_encoding": "PNG",
                "data_uri_media_type": "image/jpeg",
                "camera_array": "RGB",
                "semantic_map_array": (
                    "source BGR passed to PIL unchanged"
                ),
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
            "optional_mechanisms_without_selection_effect": {
                "room_segmentation_and_room_semantics": (
                    "test source-derived selection-neutral record"
                ),
                "attention_dod": (
                    "test source-derived selection-neutral record"
                ),
                "active_patches": (
                    "test source-derived selection-neutral record"
                ),
            },
            "source_paths": [
                "source/Focus_realworld/arguments.py",
                "source/Focus_realworld/main.py",
            ],
        },
        "semantic_input_contract": {
            "schema_version": "focus-vlm-semantic-input-contract-v1",
            "uniform_across_robots": True,
            "pixel_segmenter_backend": (
                "segformer_b0_ade20k_to_mp3d40"
            ),
            "semantic_fusion_mode": "multi_view",
            "yolo_map_reinforcement_enabled": False,
            "hm3d_category_order": [
                "chair",
                "sofa",
                "plant",
                "bed",
                "toilet",
                "tv",
                "bathtub",
                "shower",
                "fireplace",
                "appliances",
                "towel",
                "sink",
                "chest_of_drawers",
                "table",
                "stairs",
            ],
            "pixel_model_classification": (
                "checksum-pinned real-camera deployment adapter; not the "
                "executable source pixel model"
            ),
            "source_maskrcnn_override_available_in_hub": False,
            "robots": {
                robot_id: {
                    "pixel_segmenter_backend": (
                        "segformer_b0_ade20k_to_mp3d40"
                    ),
                    "semantic_fusion_mode": "multi_view",
                    "yolo_map_reinforcement_enabled": False,
                    "pixel_segmenter_status": (
                        "deployment_adapter_model_inference_unverified"
                    ),
                }
                for robot_id in ROBOTS
            },
        },
        "frontiers": [frontier_a_record, frontier_b_record],
        "remaining_frontiers": [],
        "vlm_frontier_contract": {
            "scope": "one shared fused-map A-D set",
            "label_identity": (
                "stable across image, prompt, score vector and target"
            ),
            "per_robot_view": (
                "remaining shared candidates in canonical A-D order"
            ),
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


def test_single_shared_frontier_builds_one_goal_and_one_explicit_hold(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path, observation_factory
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    first = manifest["robots"][0]
    second = manifest["robots"][1]
    frontier = first["candidate_frontiers"][0]
    frontier_selection = {
        "kind": "frontier",
        "target_id": frontier["frontier_id"],
        **frontier,
        "source_behavior": "sequential frontier removed before next robot",
    }
    manifest["frontiers"] = [frontier]
    manifest["remaining_frontiers"] = []
    manifest["source_undercomplete_frontier_adapter"] = {
        "observed_frontier_count": 1,
        "robot_count": 2,
        "source_behavior": (
            "a sole frontier may be reused by a later simulator agent"
        ),
        "real_robot_adapter": (
            "allocate each observed shared frontier at most once; "
            "robots without a remaining frontier explicitly HOLD"
        ),
        "fabricated_frontiers": False,
    }
    first.update({
        "candidate_frontiers": [frontier],
        "allocated_frontier": frontier,
        "choice_probabilities": {frontier["frontier_id"]: 1.0},
        "exploration_selection_before_target_override": frontier_selection,
        "semantic_goal_override": None,
        "final_shadow_selection": frontier_selection,
    })
    second.update({
        "candidate_frontiers": [],
        "allocated_frontier": None,
        "choice_probabilities": {},
        "exploration_selection_before_target_override": None,
        "semantic_goal_override": None,
        "final_shadow_selection": None,
        "vlm_execution_status": "not_called_no_remaining_candidate",
        "adapter_hold_reason": (
            "initial shared frontier set exhausted by earlier allocation; "
            "duplicate physical target suppressed; explicit HOLD"
        ),
    })
    manifest["final_shadow_selections"] = {
        "robot-0": frontier_selection,
    }
    write_json(manifest_path, manifest)

    built = build_batch_from_shadow_manifest(
        manifest_path,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-single-frontier",
        execution_epoch=4,
        now_ns=now,
        robot_config_path=config,
    )

    by_robot = {
        decision.robot_id: decision for decision in built.batch.decisions
    }
    assert built.report["preflight_ready"] is True
    assert built.report["active_robot_ids"] == ["robot-0"]
    assert by_robot["robot-0"].mode == "GOAL"
    assert by_robot["robot-1"].mode == "HOLD"
    assert by_robot["robot-1"].target is None


def test_forced_hold_preserves_vlm_selection_but_scopes_physical_authority(
    tmp_path,
    observation_factory,
):
    now, manifest, registry, config = prepare_round(
        tmp_path, observation_factory
    )
    config_payload = json.loads(config.read_text(encoding="utf-8"))
    config_payload["robots"]["robot-0"]["allow_goal"] = False
    write_json(config, config_payload)

    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-yunji-only",
        execution_epoch=4,
        now_ns=now,
        robot_config_path=config,
        forced_hold_robot_ids=("robot-0",),
    )

    assert built.report["preflight_ready"] is True
    assert built.report["source_active_robot_ids"] == ["robot-0", "robot-1"]
    assert built.report["active_robot_ids"] == ["robot-1"]
    assert built.report["forced_hold_robot_ids"] == ["robot-0"]
    by_robot = {
        decision.robot_id: decision for decision in built.batch.decisions
    }
    assert by_robot["robot-0"].mode == "HOLD"
    assert by_robot["robot-0"].target is None
    assert by_robot["robot-1"].mode == "GOAL"
    assert by_robot["robot-1"].target.kind == "FRONTIER_POINT"


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


def test_rejects_frontier_coordinate_drift_between_vlm_and_target(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["final_shadow_selections"]["robot-1"]["x_m"] = 99.0
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="differs across manifest sections"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_rejects_abcd_score_labels_that_do_not_match_candidate_view(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["robots"][1]["choice_probabilities"] = {"A": 1.0}
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="score labels differ"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_rejects_later_agent_image_prompt_binding_regression(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["vlm_frontier_contract"][
        "source_later_agent_image_prompt_mismatch_corrected"
    ] = False
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="frontier contract"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_rejects_abcd_cell_world_coordinate_mismatch(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["frontiers"][0]["x_m"] = 0.05
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="cell/world coordinates differ"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_rejects_vlm_red_arrow_that_uses_camera_instead_of_base_pose(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["robots"][1]["robot_xy_m"] = [0.6, 0.0]
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="measured base pose"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_rejects_mixed_robot_semantic_contract(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["semantic_input_contract"]["robots"]["robot-1"][
        "pixel_segmenter_backend"
    ] = "rednet_mp3d40"
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="semantic input differs"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_new_source_contract_rejects_missing_semantic_provenance(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    del manifest["semantic_input_contract"]
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="semantic input provenance"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_new_source_contract_rejects_source_identity_drift(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["source_code_artifacts"][0]["sha256"] = "0" * 64
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="differs from review"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_new_source_contract_cannot_be_downgraded_by_removing_version(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    del manifest["source_behavior_contract_version"]
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="without their versioned"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )


def test_new_source_contract_rejects_missing_geometry_binding(
    tmp_path,
    observation_factory,
):
    now, manifest_path, registry, config = prepare_round(
        tmp_path,
        observation_factory,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    del manifest["vlm_frontier_contract"]["stable_id_binding"]
    write_json(manifest_path, manifest)

    with pytest.raises(ValueError, match="source frontier geometry"):
        build_batch_from_shadow_manifest(
            manifest_path,
            registry,
            scene_id="scene-1",
            episode_id="scene-1-trial-1",
            execution_epoch=4,
            now_ns=now,
            robot_config_path=config,
        )
