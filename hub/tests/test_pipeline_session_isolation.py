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


def _pipeline(expected_version=None, *, keyframe_config=None, ground_guard=False):
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


def test_ground_guard_latches_drift_before_segmentation(monkeypatch):
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

    decision = pipeline.process(_observation(10, "session-a"))

    assert decision.reason == "ground_drift"
    assert pipeline.mapping_blocked_kind == "ground_drift"
    assert pipeline.ground_drift_events == 1
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
