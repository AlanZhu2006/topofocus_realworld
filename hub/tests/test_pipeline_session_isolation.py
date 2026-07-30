from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from focus_hub.central_mapping import MapperConfig
from focus_hub.ground_plane import GroundCandidate, GroundPlaneConfig
from focus_hub.pipeline import SpoolMappingPipeline, SpooledObservation
from focus_hub.pose_gate import KeyframeConfig


class _Segmenter:
    def __init__(self) -> None:
        self.calls = 0

    def segment(self, _rgb, _depth):
        self.calls += 1
        return object()


class _Mapper:
    def __init__(self, config: MapperConfig) -> None:
        self.calls = 0
        self.last_floor_plane = None
        self.semantic_vote_enabled: list[bool] = []
        self.config = config
        self.map = SimpleNamespace(floor_plane_coefficients=(0.0, 0.0, 0.0))

    def integrate(
        self,
        _frame,
        _prediction,
        *,
        floor_plane_coefficients=None,
        semantic_vote_enabled=True,
    ) -> None:
        self.calls += 1
        self.last_floor_plane = floor_plane_coefficients
        self.semantic_vote_enabled.append(semantic_vote_enabled)


class _Detector:
    def __init__(self) -> None:
        self.calls = 0

    def detect_boxes(self, _rgb):
        self.calls += 1
        return [
            SimpleNamespace(
                class_name="chair",
                confidence=0.8,
                xyxy=(0.0, 0.0, 2.0, 2.0),
            )
        ]


def _pipeline(
    expected_version=None,
    *,
    keyframe_config=None,
    ground_guard=False,
    ground_drift_consecutive_frames=3,
    ground_drift_min_duration_s=5.0,
    allow_ground_height_translation_for_2d=False,
    semantic_detector=None,
    semantic_yolo_reinforce_map=True,
    semantic_fusion_mode="max",
):
    segmenter = _Segmenter()
    K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
    config = MapperConfig(semantic_fusion_mode=semantic_fusion_mode)
    pipeline = SpoolMappingPipeline(
        segmenter,
        K,
        config,
        (0.0, 0.0),
        0.0,
        expected_transform_version=expected_version,
        keyframe_config=keyframe_config,
        ground_plane_config=GroundPlaneConfig() if ground_guard else None,
        ground_drift_consecutive_frames=ground_drift_consecutive_frames,
        ground_drift_min_duration_s=ground_drift_min_duration_s,
        allow_ground_height_translation_for_2d=(
            allow_ground_height_translation_for_2d
        ),
        semantic_detector=semantic_detector,
        semantic_yolo_reinforce_map=semantic_yolo_reinforce_map,
    )
    pipeline.mapper = _Mapper(config)
    return pipeline, segmenter


def _observation(sequence: int, version: str) -> SpooledObservation:
    metadata = SimpleNamespace(
        capture_time_ns=sequence * 1_000_000_000,
        base_T_camera=None,
        pose=SimpleNamespace(
            transform_version=version,
            shared_T_camera=SimpleNamespace(parent_frame="shared_world"),
        ),
    )
    return SpooledObservation(
        sequence=sequence,
        metadata=metadata,
        rgb_bgr=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.ones((2, 2), dtype=np.float32),
        T_shared_camera=np.eye(4),
    )


def test_pipeline_binds_to_first_transform_version():
    pipeline, _ = _pipeline()

    pipeline.process(_observation(10, "session-a"))
    latest = _observation(11, "session-a")
    pipeline.process(latest)

    assert pipeline.transform_version == "session-a"
    assert pipeline.first_sequence == 10
    assert pipeline.last_sequence == 11
    assert pipeline.last_integrated_capture_time_ns == (
        latest.metadata.capture_time_ns
    )
    assert pipeline.frames_processed == 2


def test_stage1_yolo_evidence_does_not_mutate_pixel_semantics():
    detector = _Detector()
    pipeline, segmenter = _pipeline(
        "session-a",
        semantic_detector=detector,
        semantic_yolo_reinforce_map=False,
    )

    decision = pipeline.process(_observation(10, "session-a"))
    status = pipeline.semantic_yolo_status()

    assert decision.accept
    assert segmenter.calls == 1
    assert detector.calls == 1
    assert pipeline.mapper.calls == 1
    assert status["enabled"] is True
    assert status["map_reinforcement_enabled"] is False
    assert status["method"] == "yolov10_image_detections_for_perception_vlm_only"
    assert status["frames_with_detections"] == 1
    assert status["frames_with_evidence"] == 0
    assert status["last_sequence"] == 10
    assert status["last_detections"][0]["class_name"] == "chair"


def test_pipeline_rejects_version_change_before_segmentation_or_integration():
    pipeline, segmenter = _pipeline("session-a")
    pipeline.process(_observation(10, "session-a"))

    with pytest.raises(ValueError, match="refusing to mix transform versions"):
        pipeline.process(_observation(11, "session-b"))

    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1
    assert pipeline.frames_processed == 1
    assert pipeline.last_sequence == 10


def test_live_keyframe_gate_skips_duplicate_before_segmentation():
    pipeline, segmenter = _pipeline(
        "session-a", keyframe_config=KeyframeConfig(max_interval_sec=5.0)
    )

    assert pipeline.process(_observation(10, "session-a")).accept
    skipped = pipeline.process(_observation(11, "session-a"))

    assert not skipped.accept
    assert skipped.reason == "below_threshold"
    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1
    assert pipeline.frames_processed == 1
    assert pipeline.observations_seen == 2
    assert pipeline.last_observation_sequence == 11


def test_interval_keyframe_refreshes_geometry_without_semantic_vote():
    pipeline, _ = _pipeline(
        "session-a",
        keyframe_config=KeyframeConfig(max_interval_sec=5.0),
        semantic_fusion_mode="multi_view",
    )

    first = pipeline.process(_observation(10, "session-a"))
    interval = pipeline.process(_observation(15, "session-a"))

    assert first.reason == "first"
    assert interval.reason == "interval"
    assert pipeline.mapper.semantic_vote_enabled == [True, False]
    assert pipeline.semantic_vote_frames == 1
    assert pipeline.semantic_interval_frames_without_vote == 1


def test_pipeline_derives_robot_base_pose_from_observed_mount():
    pipeline, _ = _pipeline("session-a")
    observation = _observation(10, "session-a")
    observation.T_shared_camera[0, 3] = 1.0
    observation.T_shared_camera[1, 3] = 2.0
    base_T_camera = np.eye(4)
    base_T_camera[0, 3] = 0.3
    observation.metadata.base_T_camera = SimpleNamespace(
        matrix=tuple(base_T_camera.reshape(-1))
    )

    pipeline.process(observation)

    assert pipeline.last_camera_xy == pytest.approx((1.0, 2.0))
    assert pipeline.last_robot_xy == pytest.approx((0.7, 2.0))
    assert pipeline.robot_trajectory_xy_m == pytest.approx([(0.7, 2.0)])
    assert pipeline.robot_pose_source.startswith("source_derived")


def test_live_keyframe_gate_latches_pose_jump():
    pipeline, segmenter = _pipeline(
        "session-a", keyframe_config=KeyframeConfig(max_interval_sec=5.0)
    )
    pipeline.process(_observation(10, "session-a"))
    jump_observation = _observation(11, "session-a")
    jump_observation.T_shared_camera[0, 3] = 3.0

    jump = pipeline.process(jump_observation)
    after = pipeline.process(_observation(12, "session-a"))

    assert jump.pose_jump
    assert after.reason == "pose_jump_latched"
    assert pipeline.mapping_blocked_reason is not None
    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1
    assert pipeline.trajectory_xy_m == [(0.0, 0.0)]
    assert pipeline.robot_trajectory_xy_m == [(0.0, 0.0)]


def test_ground_rejected_turns_still_advance_pose_continuity(monkeypatch):
    pipeline, segmenter = _pipeline(
        "session-a",
        keyframe_config=KeyframeConfig(max_interval_sec=5.0),
        ground_guard=True,
    )
    accepted = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=0.0,
        plane_coefficients=(0.0, 0.0, 0.0),
    )
    rejected = GroundCandidate(
        accepted=False,
        ground_z_m=None,
        reason="no_valid_plane",
        candidate_points=0,
        inlier_points=0,
        inlier_ratio=0.0,
        tilt_deg=None,
        plane_coefficients=None,
    )
    candidates = iter((accepted, rejected, rejected, accepted))
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate",
        lambda *_args: next(candidates),
    )

    observations = [
        _observation(sequence, "session-a")
        for sequence in range(10, 14)
    ]
    for observation, yaw_deg in zip(
        observations, (0.0, 35.0, 70.0, 105.0), strict=True
    ):
        yaw = np.deg2rad(yaw_deg)
        observation.T_shared_camera[:2, :2] = [
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)],
        ]

    assert pipeline.process(observations[0]).accept
    assert pipeline.process(observations[1]).reason == "ground_no_valid_plane"
    assert pipeline.process(observations[2]).reason == "ground_no_valid_plane"
    final = pipeline.process(observations[3])

    assert final.accept
    assert final.reason == "rotation"
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.pose_jump_events == 0
    assert segmenter.calls == 2
    assert pipeline.mapper.calls == 2


def test_ground_guard_latches_only_after_consecutive_drift_before_segmentation(
    monkeypatch,
):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    candidate = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=8.0,
        plane_coefficients=(0.15, 0.0, 0.0),
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: candidate
    )

    first = pipeline.process(_observation(10, "session-a"))
    second = pipeline.process(_observation(13, "session-a"))
    decision = pipeline.process(_observation(15, "session-a"))
    after = pipeline.process(_observation(16, "session-a"))

    assert first.reason == "ground_drift_pending"
    assert second.reason == "ground_drift_pending"
    assert decision.reason == "ground_drift"
    assert after.reason == "ground_drift_latched"
    assert pipeline.mapping_blocked_kind == "ground_drift"
    assert pipeline.ground_drift_frames == 3
    assert pipeline.ground_drift_events == 1
    assert pipeline.ground_drift_streak == 3
    assert segmenter.calls == 0
    assert pipeline.mapper.calls == 0


def test_ground_guard_does_not_latch_short_stationary_fit_burst(monkeypatch):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    drifting = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=8.0,
        plane_coefficients=(0.15, 0.0, 0.0),
    )
    stable = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=1.0,
        plane_coefficients=(0.01, -0.01, 0.0),
    )
    candidates = iter([drifting, drifting, drifting, stable])
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: next(candidates)
    )

    pending = [
        pipeline.process(_observation(sequence, "session-a"))
        for sequence in (10, 11, 12)
    ]
    recovered = pipeline.process(_observation(13, "session-a"))

    assert [decision.reason for decision in pending] == [
        "ground_drift_pending",
        "ground_drift_pending",
        "ground_drift_pending",
    ]
    assert recovered.accept
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.ground_drift_frames == 3
    assert pipeline.ground_drift_events == 0
    assert pipeline.ground_drift_streak == 0
    assert pipeline.last_ground_drift_duration_s == 0.0
    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1


def test_ground_guard_recovers_after_one_transient_drift(monkeypatch):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    candidates = iter(
        [
            GroundCandidate(
                accepted=True,
                ground_z_m=0.0,
                reason="accepted",
                candidate_points=1000,
                inlier_points=900,
                inlier_ratio=0.9,
                tilt_deg=8.0,
                plane_coefficients=(0.15, 0.0, 0.0),
            ),
            GroundCandidate(
                accepted=True,
                ground_z_m=0.0,
                reason="accepted",
                candidate_points=1000,
                inlier_points=900,
                inlier_ratio=0.9,
                tilt_deg=1.0,
                plane_coefficients=(0.01, -0.01, 0.0),
            ),
        ]
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: next(candidates)
    )

    skipped = pipeline.process(_observation(10, "session-a"))
    recovered = pipeline.process(_observation(11, "session-a"))

    assert skipped.reason == "ground_drift_pending"
    assert recovered.accept
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.ground_drift_frames == 1
    assert pipeline.ground_drift_events == 0
    assert pipeline.ground_drift_streak == 0
    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1


def test_ground_guard_defers_irreversible_latch_while_robot_is_moving(
    monkeypatch,
):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    stable = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=0.0,
        plane_coefficients=(0.0, 0.0, 0.0),
    )
    drifting = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=8.0,
        plane_coefficients=(0.15, 0.0, 0.0),
    )
    candidates = iter([stable, drifting, drifting, drifting, stable])
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate",
        lambda *_args: next(candidates),
    )

    baseline = _observation(10, "session-a")
    moving_one = _observation(11, "session-a")
    moving_one.T_shared_camera[0, 3] = 0.15
    moving_two = _observation(12, "session-a")
    moving_two.T_shared_camera[0, 3] = 0.30
    stationary = _observation(13, "session-a")
    stationary.T_shared_camera[0, 3] = 0.30
    recovered = _observation(14, "session-a")
    recovered.T_shared_camera[0, 3] = 0.30

    assert pipeline.process(baseline).accept
    first = pipeline.process(moving_one)
    second = pipeline.process(moving_two)
    pending = pipeline.process(stationary)
    recovery = pipeline.process(recovered)

    assert first.reason == "ground_drift_motion_deferred"
    assert second.reason == "ground_drift_motion_deferred"
    assert pending.reason == "ground_drift_pending"
    assert recovery.accept
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.ground_drift_frames == 3
    assert pipeline.ground_drift_motion_deferred_frames == 2
    assert pipeline.ground_drift_events == 0
    assert pipeline.ground_drift_streak == 0
    assert segmenter.calls == 2
    assert pipeline.mapper.calls == 2


def test_ground_guard_rebases_consistent_local_plane_after_observed_motion(
    monkeypatch,
):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    stable = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=0.0,
        plane_coefficients=(0.0, 0.0, 0.0),
    )
    locally_tilted = GroundCandidate(
        accepted=True,
        ground_z_m=-0.06,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=4.0,
        plane_coefficients=(0.0, 0.07, -0.06),
    )
    # The motion itself can keep an in-range floor estimate.  The bounded
    # posture offset may appear only after the quadruped stops, so rebase
    # authority must come from all observed pose motion rather than only from
    # moving frames whose floor estimate was already outside the gate.
    candidates = iter(
        [
            stable,
            stable,
            stable,
            locally_tilted,
            locally_tilted,
            locally_tilted,
            locally_tilted,
        ]
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate",
        lambda *_args: next(candidates),
    )

    baseline = _observation(10, "session-a")
    moving = _observation(11, "session-a")
    moving.T_shared_camera[0, 3] = 0.15
    recovered_during_settle = _observation(12, "session-a")
    recovered_during_settle.T_shared_camera[0, 3] = 0.15
    stationary = [
        _observation(sequence, "session-a")
        for sequence in (13, 15, 18, 19)
    ]
    for observation in stationary:
        observation.T_shared_camera[0, 3] = 0.15

    assert pipeline.process(baseline).accept
    assert pipeline.process(moving).accept
    assert pipeline.process(recovered_during_settle).accept
    assert pipeline.process(stationary[0]).reason == "ground_drift_pending"
    assert pipeline.process(stationary[1]).reason == "ground_drift_pending"
    rebased = pipeline.process(stationary[2])
    after = pipeline.process(stationary[3])

    assert rebased.accept
    assert after.accept
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.ground_drift_reference_rebases == 1
    assert pipeline.ground_drift_events == 0
    assert pipeline.last_ground_rebase_sequence == 18
    assert pipeline.last_ground_reason == "accepted"
    np.testing.assert_allclose(
        pipeline.ground_reference_plane_coefficients,
        locally_tilted.plane_coefficients,
    )
    assert segmenter.calls == 5
    assert pipeline.mapper.calls == 5


def test_2d_ground_guard_tolerates_pure_world_z_translation(monkeypatch):
    pipeline, segmenter = _pipeline(
        "session-a",
        ground_guard=True,
        allow_ground_height_translation_for_2d=True,
    )
    candidate = GroundCandidate(
        accepted=True,
        ground_z_m=0.09,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=0.0,
        plane_coefficients=(0.0, 0.0, 0.09),
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: candidate
    )

    decision = pipeline.process(_observation(10, "session-a"))

    assert decision.accept
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.ground_drift_frames == 0
    assert pipeline.ground_drift_streak == 0
    assert pipeline.ground_height_translation_frames == 1
    assert pipeline.max_ground_height_translation_m == pytest.approx(0.09)
    assert pipeline.last_ground_reason == "height_translation_tolerated_2d"
    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1
    assert pipeline.mapper.last_floor_plane == candidate.plane_coefficients


def test_ground_guard_no_floor_breaks_consecutive_drift_run(monkeypatch):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    drifting = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=8.0,
        plane_coefficients=(0.15, 0.0, 0.0),
    )
    no_floor = GroundCandidate(
        accepted=False,
        ground_z_m=None,
        reason="insufficient_candidates",
        candidate_points=10,
        inlier_points=0,
        inlier_ratio=0.0,
        tilt_deg=None,
        plane_coefficients=None,
    )
    candidates = iter([drifting, no_floor, drifting])
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: next(candidates)
    )

    first = pipeline.process(_observation(10, "session-a"))
    missing = pipeline.process(_observation(11, "session-a"))
    second = pipeline.process(_observation(12, "session-a"))

    assert first.reason == "ground_drift_pending"
    assert missing.reason == "ground_insufficient_candidates"
    assert second.reason == "ground_drift_pending"
    assert pipeline.mapping_blocked_reason is None
    assert pipeline.ground_drift_frames == 2
    assert pipeline.ground_drift_events == 0
    assert pipeline.ground_drift_streak == 1
    assert segmenter.calls == 0
    assert pipeline.mapper.calls == 0


def test_stage1_yolo_stays_current_when_wall_frame_cannot_update_map(
    monkeypatch,
):
    detector = _Detector()
    pipeline, segmenter = _pipeline(
        "session-a",
        ground_guard=True,
        semantic_detector=detector,
        semantic_yolo_reinforce_map=False,
    )
    no_floor = GroundCandidate(
        accepted=False,
        ground_z_m=None,
        reason="insufficient_candidates",
        candidate_points=10,
        inlier_points=0,
        inlier_ratio=0.0,
        tilt_deg=None,
        plane_coefficients=None,
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: no_floor
    )

    decision = pipeline.process(_observation(10, "session-a"))
    status = pipeline.semantic_yolo_status()

    assert decision.reason == "ground_insufficient_candidates"
    assert detector.calls == 1
    assert segmenter.calls == 0
    assert pipeline.mapper.calls == 0
    assert pipeline.last_sequence is None
    assert status["last_sequence"] == 10
    assert status["last_detections"][0]["class_name"] == "chair"
    assert status["inference_policy"] == "every_current_observation_for_stage1"


def test_ground_guard_passes_frame_plane_to_mapper(monkeypatch):
    pipeline, segmenter = _pipeline("session-a", ground_guard=True)
    candidate = GroundCandidate(
        accepted=True,
        ground_z_m=0.0,
        reason="accepted",
        candidate_points=1000,
        inlier_points=900,
        inlier_ratio=0.9,
        tilt_deg=1.0,
        plane_coefficients=(0.01, -0.01, 0.0),
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.depth_points_world", lambda *_args: np.zeros((3, 3))
    )
    monkeypatch.setattr(
        "focus_hub.pipeline.fit_ground_candidate", lambda *_args: candidate
    )

    decision = pipeline.process(_observation(10, "session-a"))

    assert decision.accept
    assert segmenter.calls == 1
    assert pipeline.mapper.calls == 1
    assert pipeline.mapper.last_floor_plane == candidate.plane_coefficients
