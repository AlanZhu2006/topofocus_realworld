from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from focus_hub.central_mapping import MapperConfig
from focus_hub.pipeline import SpoolMappingPipeline, SpooledObservation


class _Segmenter:
    def __init__(self) -> None:
        self.calls = 0

    def segment(self, _rgb, _depth):
        self.calls += 1
        return object()


class _Mapper:
    def __init__(self) -> None:
        self.calls = 0

    def integrate(self, _frame, _prediction) -> None:
        self.calls += 1


def _pipeline(expected_version=None):
    segmenter = _Segmenter()
    K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])
    pipeline = SpoolMappingPipeline(
        segmenter,
        K,
        MapperConfig(),
        (0.0, 0.0),
        0.0,
        expected_transform_version=expected_version,
    )
    pipeline.mapper = _Mapper()
    return pipeline, segmenter


def _observation(sequence: int, version: str) -> SpooledObservation:
    metadata = SimpleNamespace(
        pose=SimpleNamespace(transform_version=version),
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
