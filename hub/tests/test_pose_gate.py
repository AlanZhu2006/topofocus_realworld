from __future__ import annotations

import numpy as np
from types import SimpleNamespace

from focus_hub.pose_gate import (
    KeyframeConfig,
    KeyframeSelector,
    StartupPoseConfig,
    StartupPoseGate,
)
from focus_hub.rate_aware_keyframes import RateAwareKeyframeSelector


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


def test_rate_aware_selector_accepts_bounded_motion_after_slow_mapping_pair():
    config = SimpleNamespace(
        translation_threshold_m=0.20,
        rotation_threshold_deg=10.0,
        max_interval_sec=1.0,
        pose_jump_translation_m=1.0,
        pose_jump_rotation_deg=90.0,
        pause_frames_after_jump=0,
    )
    selector = RateAwareKeyframeSelector(config)

    assert selector.evaluate(pose(), 0).accept
    delayed_motion = selector.evaluate(pose(1.089), 7_000_000_000)

    assert delayed_motion.accept
    assert delayed_motion.reason == "translation"
    assert not delayed_motion.pose_jump


def test_rate_aware_selector_still_rejects_fast_or_large_discontinuity():
    config = SimpleNamespace(
        translation_threshold_m=0.20,
        rotation_threshold_deg=10.0,
        max_interval_sec=1.0,
        pose_jump_translation_m=1.0,
        pose_jump_rotation_deg=90.0,
        pause_frames_after_jump=0,
    )
    fast = RateAwareKeyframeSelector(config)
    fast.evaluate(pose(), 0)
    assert fast.evaluate(pose(1.089), 500_000_000).pose_jump

    large = RateAwareKeyframeSelector(config)
    large.evaluate(pose(), 0)
    jump = large.evaluate(pose(3.93), 7_000_000_000)
    assert jump.pose_jump
    assert jump.reason == "pose_jump"
    # A true discontinuity is re-anchored, and pause=0 lets the next stable
    # observation republish geometry instead of starving two slow frames.
    recovered = large.evaluate(pose(4.14), 8_000_000_000)
    assert recovered.accept
    assert recovered.reason == "translation"
