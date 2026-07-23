from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from focus_hub.realworld_session import (
    CalibrationIdentity,
    CodeIdentity,
    DebugValidation,
    RealworldSession,
    RobotSession,
    RuntimeIdentity,
    artifact_identity,
    atomic_write_json,
    calibration_validation_kind,
    expected_map_session_contract,
    expected_robot_config,
    session_contract_sha256,
    validate_debug_manifest,
    validate_session,
)


def calibration_payload() -> dict:
    return {
        "schema_version": 2,
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "transform_version": "yunji-transform-v1",
        "shared_frame_calibration_id": "shared-calibration-v1",
        "calibration_frame": {
            "reference": {"transform_version": "wsj-transform-v1"},
            "other": {"transform_version": "yunji-raw-v1"},
        },
        "shared_world_from_other_odom": {
            "matrix": [
                1, 0, 0, 1,
                0, 1, 0, 2,
                0, 0, 1, 0,
                0, 0, 0, 1,
            ]
        },
        "holdout_validation": {
            "checks": {
                "sync_skew": True,
                "board_center_residual": True,
                "board_normal_residual": True,
                "board_moved_independently": True,
            }
        },
    }


def build_session(tmp_path: Path) -> RealworldSession:
    calibration_path = tmp_path / "calibration.json"
    atomic_write_json(calibration_path, calibration_payload())
    robots = (
        RobotSession(
            robot_id="robot-0",
            name="wsj",
            transform_version="wsj-transform-v1",
            map_dir="hub/runtime/map_out_wsj_test-session",
            map_start_after_sequence=10,
            remote_root="/robot/wsj/release",
            remote_calibration_path="/robot/wsj/calibration.json",
            remote_base_camera_calibration_path="/robot/wsj/base-camera.json",
            remote_hub_url="http://127.0.0.1:18089",
            remote_preview_url="http://127.0.0.1:18766",
            ssh_tmux_target="wsj:ssh",
        ),
        RobotSession(
            robot_id="robot-1",
            name="yunji",
            transform_version="yunji-transform-v1",
            map_dir="hub/runtime/map_out_yunji_test-session",
            map_start_after_sequence=20,
            remote_root="/robot/yunji/release",
            remote_calibration_path="/robot/yunji/calibration.json",
            remote_base_camera_calibration_path="/robot/yunji/base-camera.json",
            remote_hub_url="http://127.0.0.1:18089",
            ssh_tmux_target="yunji:ssh",
        ),
    )
    for robot in robots:
        directory = tmp_path / robot.map_dir
        directory.mkdir(parents=True)
        np.savez_compressed(
            directory / "central_map.npz",
            grid=np.zeros((2, 2, 2), dtype=np.float32),
            origin_xy_m=np.zeros(2, dtype=np.float64),
            resolution_m=np.asarray(0.05),
            frame_id=np.asarray("shared_world"),
            transform_version=np.asarray(robot.transform_version),
            shared_frame_calibration_id=np.asarray(
                "shared-calibration-v1"
            ),
            map_format_version=np.asarray("focus-hub-central-map-v3"),
        )
        common = {
            "robot_id": robot.robot_id,
            "frame_id": "shared_world",
            "transform_version": robot.transform_version,
            "shared_frame_calibration_id": "shared-calibration-v1",
            "mapping_blocked_reason": None,
        }
        atomic_write_json(
            directory / "map_summary.json",
            {
                **common,
                "last_observation_sequence": 30,
                "semantic_mapping": {
                    "yolo_reinforcement": {
                        "enabled": True,
                        "last_sequence": 30,
                    }
                },
            },
        )
        atomic_write_json(
            directory / "live_status.json",
            {**common, "last_capture_time_ns": 1_000_000_000},
        )

    pending_runtime = RuntimeIdentity.model_construct(
        hub_port=8188,
        hub_session="hub-test",
        glm_url="http://127.0.0.1:31511/v1",
        glm_session="glm-test",
        map_session="shared_maps_test-session",
        foxglove_session="foxglove_relay_test-session",
        foxglove_port=8765,
        preview_port=8766,
        map_goal_category="chair",
        spool_dir="spool",
        admin_token_file="admin-token",
        debug_robot_config=None,
        live_robot_config=None,
    )
    calibration = CalibrationIdentity(
        calibration_id="shared-calibration-v1",
        artifact=artifact_identity(
            tmp_path,
            calibration_path,
            classification="observed board calibration",
        ),
        validation_kind="independent_moved_board_holdout",
    )
    provisional = RealworldSession.model_construct(
        schema_version="focus-realworld-session-v1",
        session_id="test-session",
        created_at_ns=1,
        code=CodeIdentity(git_commit="a" * 40, working_tree_clean=True),
        calibration=calibration,
        robots=robots,
        runtime=pending_runtime,
        debug_validation=None,
    )
    config_dir = tmp_path / "configs"
    debug_config = config_dir / "debug.json"
    live_config = config_dir / "live.json"
    atomic_write_json(
        debug_config, expected_robot_config(provisional, allow_goal=False)
    )
    atomic_write_json(
        live_config, expected_robot_config(provisional, allow_goal=True)
    )
    runtime = RuntimeIdentity(
        hub_port=8188,
        hub_session="hub-test",
        glm_url="http://127.0.0.1:31511/v1",
        glm_session="glm-test",
        map_session="shared_maps_test-session",
        foxglove_session="foxglove_relay_test-session",
        foxglove_port=8765,
        preview_port=8766,
        map_goal_category="chair",
        spool_dir="spool",
        admin_token_file="admin-token",
        debug_robot_config=artifact_identity(
            tmp_path,
            debug_config,
            classification="debug policy",
        ),
        live_robot_config=artifact_identity(
            tmp_path,
            live_config,
            classification="live policy",
        ),
    )
    session = RealworldSession(
        session_id="test-session",
        created_at_ns=1,
        code=CodeIdentity(git_commit="a" * 40, working_tree_clean=True),
        calibration=calibration,
        robots=robots,
        runtime=runtime,
    )
    for robot in robots:
        atomic_write_json(
            tmp_path / robot.map_dir / "map_session_contract.json",
            expected_map_session_contract(session, robot),
        )
    return session


def attach_debug(
    tmp_path: Path,
    session: RealworldSession,
    *,
    allow_stale: bool = False,
    allow_blocked: bool = False,
) -> RealworldSession:
    path = tmp_path / "debug/shadow_manifest.json"
    atomic_write_json(
        path,
        {
            "schema_version": "focus-vlm-shadow-v1",
            "status": "complete_shadow_only",
            "goal_category": "chair",
            "shared_frame_calibration_id": "shared-calibration-v1",
            "realworld_session_id": session.session_id,
            "realworld_session_contract_sha256": session_contract_sha256(
                session
            ),
            "allow_stale_shadow_input": allow_stale,
            "allow_blocked_shadow_input": allow_blocked,
            "input_timing": {"status": "accepted_fresh"},
            "safety": {
                "robot_commands_sent": False,
                "hub_decision_mode_if_published": "HOLD",
                "allow_goal_changed": False,
                "goal_publication_code_path_present": False,
            },
            "hub_hold_publications": {
                "robot-0": {"mode": "HOLD", "status_code": 200},
                "robot-1": {"mode": "HOLD", "status_code": 200},
            },
        },
    )
    debug = DebugValidation(
        passed_at_ns=2,
        code_git_commit=session.code.git_commit,
        session_contract_sha256=session_contract_sha256(session),
        shadow_manifest=artifact_identity(
            tmp_path,
            path,
            classification="strict debug evidence",
        ),
        goal_category="chair",
        strict_freshness=True,
        strict_mapping_health=True,
        hub_goal_output_disabled=True,
        robot_command_paths_disabled=True,
    )
    return session.model_copy(update={"debug_validation": debug})


def test_session_contract_validates_calibration_maps_and_debug(tmp_path):
    session = attach_debug(tmp_path, build_session(tmp_path))

    report = validate_session(
        tmp_path,
        session,
        require_maps=True,
        require_debug=True,
    )

    assert report["calibration_id"] == "shared-calibration-v1"
    assert report["debug_validation"]["goal_category"] == "chair"


@pytest.mark.parametrize(
    ("allow_stale", "allow_blocked", "message"),
    [
        (True, False, "stale-input override"),
        (False, True, "blocked-map override"),
    ],
)
def test_debug_override_cannot_unlock_live(
    tmp_path, allow_stale, allow_blocked, message
):
    session = attach_debug(
        tmp_path,
        build_session(tmp_path),
        allow_stale=allow_stale,
        allow_blocked=allow_blocked,
    )

    with pytest.raises(ValueError, match=message):
        validate_debug_manifest(
            tmp_path, session, session.debug_validation
        )


def test_artifact_drift_is_rejected(tmp_path):
    session = build_session(tmp_path)
    calibration = tmp_path / session.calibration.artifact.path
    calibration.write_text(calibration.read_text() + " ")

    with pytest.raises(ValueError, match="size drift"):
        validate_session(tmp_path, session)


def test_live_requires_debug_for_same_contract(tmp_path):
    session = attach_debug(tmp_path, build_session(tmp_path))
    changed_robot = session.robots[0].model_copy(
        update={"map_start_after_sequence": 99}
    )
    changed = session.model_copy(
        update={"robots": (changed_robot, session.robots[1])}
    )

    with pytest.raises(ValueError, match="different session contract"):
        validate_session(tmp_path, changed, require_debug=True)


def test_calibration_without_independent_validation_is_rejected():
    payload = calibration_payload()
    payload["holdout_validation"] = None

    with pytest.raises(ValueError, match="independent moved-board holdout"):
        calibration_validation_kind(payload)


def test_calibration_cannot_replace_named_holdout_checks():
    payload = calibration_payload()
    payload["holdout_validation"]["checks"] = {"unrelated_check": True}

    with pytest.raises(ValueError, match="independent moved-board holdout"):
        calibration_validation_kind(payload)


def test_debug_manifest_rejects_any_true_command_alias(tmp_path):
    session = attach_debug(tmp_path, build_session(tmp_path))
    manifest = tmp_path / session.debug_validation.shadow_manifest.path
    payload = json.loads(manifest.read_text())
    payload["safety"]["robot_commands_issued"] = True
    atomic_write_json(manifest, payload)
    debug = session.debug_validation.model_copy(
        update={
            "shadow_manifest": artifact_identity(
                tmp_path,
                manifest,
                classification="strict debug evidence",
            )
        }
    )

    with pytest.raises(ValueError, match="claims a robot command"):
        validate_debug_manifest(tmp_path, session, debug)


def test_map_session_boundary_drift_is_rejected(tmp_path):
    session = build_session(tmp_path)
    robot = session.robots[0]
    contract = tmp_path / robot.map_dir / "map_session_contract.json"
    payload = json.loads(contract.read_text())
    payload["start_after_sequence"] += 1
    atomic_write_json(contract, payload)

    with pytest.raises(ValueError, match="map session contract"):
        validate_session(tmp_path, session, require_maps=True)
