from __future__ import annotations

import math

import pytest

from focus_hub.geometry import compose_rigid
from focus_hub.relocalized_odometry import (
    ExactStampPosePairs,
    RelocalizationConsensus,
    map_pose,
    pose_matrix,
    quaternion_xyzw,
    rotate_pose_covariance,
    transform_error,
)
from focus_hub.robot_map_alignment import planar_pose_matrix


def test_map_pose_removes_tracking_drift() -> None:
    tracking_T_map = planar_pose_matrix(1.7, -0.4, math.radians(12.0))
    expected_map_T_camera = planar_pose_matrix(2.0, 0.5, math.radians(20.0))
    tracking_T_camera = compose_rigid(
        tracking_T_map, expected_map_T_camera
    )

    actual = map_pose(
        tracking_T_map=tracking_T_map,
        tracking_T_camera=tracking_T_camera,
    )

    assert actual == pytest.approx(expected_map_T_camera)


def test_pose_quaternion_round_trip() -> None:
    original = pose_matrix(
        (1.0, -2.0, 0.4),
        (0.1, -0.2, 0.3, 0.9),
    )
    quaternion = quaternion_xyzw(original)
    restored = pose_matrix(
        (original[3], original[7], original[11]),
        quaternion,
    )

    assert restored == pytest.approx(original)


def test_exact_stamp_pairing_rejects_approximate_match() -> None:
    pairs = ExactStampPosePairs(maximum_entries=3)
    tracking = planar_pose_matrix(1.0, 0.0, 0.0)
    map_camera = planar_pose_matrix(0.0, 0.0, 0.0)

    assert pairs.add_tracking(
        stamp_ns=10, tracking_T_camera=tracking
    ) is None
    assert pairs.add_map(stamp_ns=11, map_T_camera=map_camera) is None
    paired = pairs.add_map(stamp_ns=10, map_T_camera=map_camera)

    assert paired is not None
    assert paired[0] == pytest.approx(tracking)
    assert paired[1] == pytest.approx(map_camera)


def test_consensus_requires_multiple_consistent_exact_pairs() -> None:
    gate = RelocalizationConsensus(
        minimum_support=3,
        candidate_window_s=10.0,
        maximum_supported_age_s=5.0,
    )
    tracking_T_map = planar_pose_matrix(1.5, -0.3, math.radians(4.0))
    now_ns = 10_000_000_000
    for index in range(2):
        map_T_camera = planar_pose_matrix(
            0.5 + index * 0.1, 0.2, math.radians(15.0)
        )
        tracking_T_camera = compose_rigid(
            tracking_T_map, map_T_camera
        )
        gate.add_pair(
            tracking_T_camera=tracking_T_camera,
            map_T_camera=map_T_camera,
            stamp_ns=index + 1,
            observed_ns=now_ns - (2 - index) * 100_000_000,
        )

    decision = gate.evaluate(
        source_tracking_T_map=tracking_T_map,
        now_ns=now_ns,
    )
    assert not decision.ready
    assert decision.reason == "INSUFFICIENT_CONSISTENT_RELOCALIZATIONS"

    map_T_camera = planar_pose_matrix(0.8, 0.2, math.radians(15.0))
    gate.add_pair(
        tracking_T_camera=compose_rigid(
            tracking_T_map, map_T_camera
        ),
        map_T_camera=map_T_camera,
        stamp_ns=3,
        observed_ns=now_ns,
    )
    decision = gate.evaluate(
        source_tracking_T_map=tracking_T_map,
        now_ns=now_ns,
    )

    assert decision.ready
    assert decision.reason == "READY"
    assert decision.support == 3


def test_consensus_rejects_source_tf_jump_despite_old_good_cluster() -> None:
    gate = RelocalizationConsensus(minimum_support=3)
    tracking_T_map = planar_pose_matrix(1.5, -0.3, math.radians(4.0))
    now_ns = 20_000_000_000
    for index in range(3):
        map_T_camera = planar_pose_matrix(0.1 * index, 0.0, 0.0)
        gate.add_pair(
            tracking_T_camera=compose_rigid(
                tracking_T_map, map_T_camera
            ),
            map_T_camera=map_T_camera,
            stamp_ns=index + 1,
            observed_ns=now_ns - index,
        )

    jumped = planar_pose_matrix(2.4, -0.3, math.radians(30.0))
    decision = gate.evaluate(
        source_tracking_T_map=jumped,
        now_ns=now_ns,
    )

    assert not decision.ready
    assert decision.reason == "MAP_TF_DISAGREES_WITH_RELOCALIZATION_PAIRS"
    assert decision.source_translation_error_m is not None
    assert decision.source_translation_error_m > 0.3


def test_consensus_expires_without_recent_supported_match() -> None:
    gate = RelocalizationConsensus(
        minimum_support=2,
        candidate_window_s=20.0,
        maximum_supported_age_s=1.0,
    )
    tracking_T_map = planar_pose_matrix(1.0, 0.0, 0.0)
    for index in range(2):
        map_T_camera = planar_pose_matrix(float(index), 0.0, 0.0)
        gate.add_pair(
            tracking_T_camera=compose_rigid(
                tracking_T_map, map_T_camera
            ),
            map_T_camera=map_T_camera,
            stamp_ns=index + 1,
            observed_ns=1_000_000_000 + index,
        )

    decision = gate.evaluate(
        source_tracking_T_map=tracking_T_map,
        now_ns=3_000_000_000,
    )

    assert not decision.ready
    assert decision.reason == "RELOCALIZATION_STALE"


def test_large_alignment_tilt_is_rejected() -> None:
    gate = RelocalizationConsensus(minimum_support=2)
    tilted = list(planar_pose_matrix(0.0, 0.0, 0.0))
    angle = math.radians(30.0)
    tilted[5] = math.cos(angle)
    tilted[6] = -math.sin(angle)
    tilted[9] = math.sin(angle)
    tilted[10] = math.cos(angle)

    with pytest.raises(ValueError, match="tilt exceeds"):
        gate.add_pair(
            tracking_T_camera=tilted,
            map_T_camera=planar_pose_matrix(0.0, 0.0, 0.0),
            stamp_ns=1,
            observed_ns=1,
        )


def test_pose_covariance_rotates_parent_axes() -> None:
    covariance = [0.0] * 36
    covariance[0] = 4.0
    covariance[7] = 1.0
    covariance[14] = 9.0
    covariance[21] = 0.4
    covariance[28] = 0.1
    covariance[35] = 0.9
    map_T_tracking = planar_pose_matrix(0.0, 0.0, math.pi / 2)

    rotated = rotate_pose_covariance(
        covariance, map_T_tracking=map_T_tracking
    )

    assert rotated[0] == pytest.approx(1.0)
    assert rotated[7] == pytest.approx(4.0)
    assert rotated[14] == pytest.approx(9.0)
    assert rotated[21] == pytest.approx(0.1)
    assert rotated[28] == pytest.approx(0.4)
    assert rotated[35] == pytest.approx(0.9)


def test_transform_error_reports_full_rotation() -> None:
    first = pose_matrix((0, 0, 0), (0, 0, 0, 1))
    half = math.sin(math.radians(10.0) / 2.0)
    second = pose_matrix(
        (0.3, 0.4, 0.0),
        (half, 0.0, 0.0, math.cos(math.radians(10.0) / 2.0)),
    )

    translation_m, rotation_deg = transform_error(first, second)

    assert translation_m == pytest.approx(0.5)
    assert rotation_deg == pytest.approx(10.0)
