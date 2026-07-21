from __future__ import annotations

import numpy as np

from focus_hub.pose_gate import (
    KeyframeConfig,
    KeyframeSelector,
    StartupPoseConfig,
    StartupPoseGate,
)


def pose(x: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    angle = np.deg2rad(yaw_deg)
    result = np.eye(4)
    result[:2, :2] = [
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)],
    ]
    result[0, 3] = x
    return result


def test_startup_gate_discards_stale_outlier_before_map_origin_is_chosen():
    gate = StartupPoseGate(StartupPoseConfig(required_consecutive=3))

    assert not gate.evaluate(pose(3.944), 1_000_000_000).ready
    reset = gate.evaluate(pose(-0.377), 758_000_000_000)
    assert reset.reset
    assert reset.consecutive == 1
    assert reset.translation_m > 4.0

    assert not gate.evaluate(pose(-0.376), 759_000_000_000).ready
    ready = gate.evaluate(pose(-0.375), 760_000_000_000)
    assert ready.ready
    assert ready.consecutive == 3


def test_startup_gate_allows_plausible_motion_inside_window():
    gate = StartupPoseGate(StartupPoseConfig(required_consecutive=3))

    gate.evaluate(pose(0.0), 0)
    assert not gate.evaluate(pose(0.6), 1_000_000_000).reset
    assert gate.evaluate(pose(1.2), 2_000_000_000).ready


def test_keyframe_selector_suppresses_duplicates_and_reports_jump():
    selector = KeyframeSelector(KeyframeConfig(max_interval_sec=5.0))

    assert selector.evaluate(pose(), 0).reason == "first"
    assert selector.evaluate(pose(0.01), 1_000_000_000).reason == "below_threshold"
    assert selector.evaluate(pose(0.2), 2_000_000_000).reason == "translation"
    assert selector.evaluate(pose(0.2), 8_000_000_000).reason == "interval"

    jump = selector.evaluate(pose(3.0), 9_000_000_000)
    assert jump.pose_jump
    assert not jump.accept
    assert jump.translation_m > 2.0
