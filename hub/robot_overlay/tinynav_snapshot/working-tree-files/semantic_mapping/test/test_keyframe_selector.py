import numpy as np

from semantic_mapping.keyframe_selector import KeyframeConfig, KeyframeSelector


def pose(x: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    angle = np.deg2rad(yaw_deg)
    result = np.eye(4)
    result[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    result[0, 3] = x
    return result


def test_keyframe_or_thresholds() -> None:
    selector = KeyframeSelector(
        KeyframeConfig(
            translation_threshold_m=0.2,
            rotation_threshold_deg=10.0,
            max_interval_sec=1.0,
            pose_jump_translation_m=2.0,
            pose_jump_rotation_deg=90.0,
        )
    )
    assert selector.evaluate(pose(), 0).reason == "first"
    assert not selector.evaluate(pose(0.1), 100_000_000).accept
    assert selector.evaluate(pose(0.2), 200_000_000).reason == "translation"
    assert selector.evaluate(pose(0.2, 11.0), 300_000_000).reason == "rotation"
    assert selector.evaluate(pose(0.2, 11.0), 1_300_000_000).reason == "interval"


def test_pose_jump_is_rejected_and_paused() -> None:
    selector = KeyframeSelector(
        KeyframeConfig(
            pose_jump_translation_m=0.5,
            pose_jump_rotation_deg=20.0,
            pause_frames_after_jump=2,
        )
    )
    assert selector.evaluate(pose(), 0).accept
    jump = selector.evaluate(pose(1.0), 100_000_000)
    assert jump.pose_jump and not jump.accept
    assert selector.evaluate(pose(1.01), 200_000_000).reason == "post_jump_pause"
    assert selector.evaluate(pose(1.02), 300_000_000).reason == "post_jump_pause"
    assert not selector.evaluate(pose(1.03), 400_000_000).accept
