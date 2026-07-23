from __future__ import annotations

import json
import math

import pytest

from focus_hub.geometry import compose_rigid, invert_rigid
from focus_hub.robot_map_alignment import (
    IDENTITY,
    alignment_artifact,
    derive_shared_T_map_from_tracking_map,
    derive_shared_T_robot_map,
    load_shared_tracking_calibration,
    planar_pose_matrix,
    planarize_rigid,
    yaw_from_matrix,
)


def test_two_pose_alignment_recovers_nonidentity_robot_map():
    shared_T_tracking = planar_pose_matrix(2.0, -1.0, math.pi / 2)
    tracking_T_body = planar_pose_matrix(1.0, 0.0, 0.2)
    expected_shared_T_map = planar_pose_matrix(-0.5, 1.5, -0.4)
    map_T_body = compose_rigid(
        invert_rigid(expected_shared_T_map),
        compose_rigid(shared_T_tracking, tracking_T_body),
    )

    actual = derive_shared_T_robot_map(
        shared_T_tracking=shared_T_tracking,
        tracking_T_body=tracking_T_body,
        robot_map_T_body=map_T_body,
    )

    assert actual == pytest.approx(expected_shared_T_map)
    assert yaw_from_matrix(actual) == pytest.approx(-0.4)


def test_direct_tinynav_tracking_map_composition():
    shared_T_world = planar_pose_matrix(1.0, 2.0, 0.3)
    world_T_map = planar_pose_matrix(-0.5, 0.25, -0.1)

    result = derive_shared_T_map_from_tracking_map(
        shared_T_tracking=shared_T_world,
        tracking_T_map=world_T_map,
    )

    assert result == pytest.approx(compose_rigid(shared_T_world, world_T_map))


def test_planar_projection_rejects_large_tilt():
    tilted = list(IDENTITY)
    angle = math.radians(10)
    tilted[5], tilted[6] = math.cos(angle), -math.sin(angle)
    tilted[9], tilted[10] = math.sin(angle), math.cos(angle)

    with pytest.raises(ValueError, match="tilt"):
        planarize_rigid(tilted, label="tilted", max_tilt_deg=5)


def test_board_artifact_loads_reference_and_other_with_provenance(tmp_path):
    path = tmp_path / "board.json"
    raw = {
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "shared_frame_calibration_id": "board-v1",
        "transform_version": "other-v1",
        "calibration_frame": {
            "reference": {"transform_version": "reference-v1"}
        },
        "shared_world_from_other_odom": {
            "child_frame": "odin_odom",
            "matrix": list(planar_pose_matrix(1, 2, 0.4)),
        },
    }
    path.write_text(json.dumps(raw))

    reference = load_shared_tracking_calibration(
        path,
        robot_id="robot-0",
        expected_transform_version="reference-v1",
        expected_calibration_id="board-v1",
    )
    other = load_shared_tracking_calibration(
        path,
        robot_id="robot-1",
        expected_transform_version="other-v1",
        expected_calibration_id="board-v1",
    )

    assert reference.shared_T_tracking == IDENTITY
    assert other.shared_T_tracking == pytest.approx(planar_pose_matrix(1, 2, 0.4))
    assert other.source_size_bytes == path.stat().st_size
    assert len(other.source_sha256) == 64

    artifact = alignment_artifact(
        calibration=other,
        local_map_frame="water_map",
        shared_T_robot_map=planar_pose_matrix(3, 4, 0.5),
        captured_at_ns=10,
        sample_skew_ns=5,
        max_sample_skew_ns=10,
        observed_inputs={"water": {"status": "observed"}},
    )
    assert artifact["result_status"] == "source_derived_from_observed_localization_samples"
    assert artifact["robot_commands_issued"] is False


def test_reference_tracking_restart_loads_explicit_stationary_handover(tmp_path):
    path = tmp_path / "board-reanchored.json"
    handover = planar_pose_matrix(0.11, 0.03, math.radians(1.7), z=-0.05)
    path.write_text(json.dumps({
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "shared_frame_calibration_id": "board-v1",
        "transform_version": "other-v1",
        "calibration_frame": {
            "reference": {"transform_version": "reference-restart-v2"}
        },
        "shared_world_from_reference_tracking": {
            "parent_frame": "shared_world",
            "child_frame": "robot-0_tracking",
            "matrix": list(handover),
        },
        "shared_world_from_other_odom": {
            "child_frame": "robot-1_odom",
            "matrix": list(planar_pose_matrix(1, 2, 0.4)),
        },
    }))

    reference = load_shared_tracking_calibration(
        path,
        robot_id="robot-0",
        expected_transform_version="reference-restart-v2",
        expected_calibration_id="board-v1",
    )

    assert reference.shared_T_tracking == pytest.approx(handover)
    assert reference.tracking_frame == "robot-0_tracking"
    assert reference.provenance_status == (
        "observed_stationary_pose_handover_source_derived_alignment"
    )


def test_reference_tracking_handover_rejects_non_object(tmp_path):
    path = tmp_path / "bad-board-reanchored.json"
    path.write_text(json.dumps({
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "shared_frame_calibration_id": "board-v1",
        "transform_version": "other-v1",
        "calibration_frame": {
            "reference": {"transform_version": "reference-restart-v2"}
        },
        "shared_world_from_reference_tracking": list(IDENTITY),
    }))

    with pytest.raises(
        ValueError, match="shared_world_from_reference_tracking must be an object"
    ):
        load_shared_tracking_calibration(
            path,
            robot_id="robot-0",
            expected_transform_version="reference-restart-v2",
            expected_calibration_id="board-v1",
        )


def test_alignment_artifact_rejects_sample_skew(tmp_path):
    path = tmp_path / "board.json"
    path.write_text(json.dumps({
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "shared_frame_calibration_id": "board-v1",
        "transform_version": "other-v1",
        "calibration_frame": {"reference": {"transform_version": "reference-v1"}},
        "shared_world_from_other_odom": {
            "child_frame": "odom", "matrix": list(IDENTITY)
        },
    }))
    calibration = load_shared_tracking_calibration(
        path,
        robot_id="robot-0",
        expected_transform_version="reference-v1",
        expected_calibration_id="board-v1",
    )
    with pytest.raises(ValueError, match="skew"):
        alignment_artifact(
            calibration=calibration,
            local_map_frame="map",
            shared_T_robot_map=IDENTITY,
            captured_at_ns=10,
            sample_skew_ns=11,
            max_sample_skew_ns=10,
            observed_inputs={},
        )
