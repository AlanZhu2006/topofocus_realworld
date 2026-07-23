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
    def __init__(self) -> None:
        self.calls = 0
        self.last_floor_plane = None
        self.map = SimpleNamespace(floor_plane_coefficients=(0.0, 0.0, 0.0))

    def integrate(self, _frame, _prediction, *, floor_plane_coefficients=None) -> None:
        self.calls += 1
        self.last_floor_plane = floor_plane_coefficients


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
    semantic_detector=None,
    semantic_yolo_reinforce_map=True,
):
    segmenter = _Segmenter()
    K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
    pipeline = SpoolMappingPipeline(
        segmenter,
        K,
        MapperConfig(),
        (0.0, 0.0),
        0.0,
        expected_transform_version=expected_version,
        keyframe_config=keyframe_config,
        ground_plane_config=GroundPlaneConfig() if ground_guard else None,
        ground_drift_consecutive_frames=ground_drift_consecutive_frames,
        semantic_detector=semantic_detector,
        semantic_yolo_reinforce_map=semantic_yolo_reinforce_map,
    )
    pipeline.mapper = _Mapper()
    return pipeline, segmenter


def _observation(sequence: int, version: str) -> SpooledObservation:
    metadata = SimpleNamespace(
        capture_time_ns=sequence * 1_000_000_000,
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
    pipeline.process(_observation(11, "session-a"))

    assert pipeline.transform_version == "session-a"
    assert pipeline.first_sequence == 10
    assert pipeline.last_sequence == 11
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
    second = pipeline.process(_observation(11, "session-a"))
    decision = pipeline.process(_observation(12, "session-a"))
    after = pipeline.process(_observation(13, "session-a"))

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
