"""Pose-based keyframe selection and relocalization jump detection."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class KeyframeConfig:
    translation_threshold_m: float = 0.20
    rotation_threshold_deg: float = 10.0
    max_interval_sec: float = 1.0
    pose_jump_translation_m: float = 0.50
    pose_jump_rotation_deg: float = 20.0
    pause_frames_after_jump: int = 2

    def __post_init__(self) -> None:
        values = (
            self.translation_threshold_m,
            self.rotation_threshold_deg,
            self.max_interval_sec,
            self.pose_jump_translation_m,
            self.pose_jump_rotation_deg,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("Keyframe thresholds must be finite and non-negative")
        if self.max_interval_sec <= 0.0:
            raise ValueError("max_interval_sec must be positive")
        if self.pause_frames_after_jump < 0:
            raise ValueError("pause_frames_after_jump must be non-negative")


@dataclass(frozen=True)
class KeyframeDecision:
    accept: bool
    reason: str
    translation_m: float
    rotation_deg: float
    elapsed_sec: float
    pose_jump: bool = False


def _pose_matrix(value: NDArray) -> NDArray[np.float64]:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (4, 4):
        raise ValueError(f"Pose must have shape (4, 4), got {pose.shape}")
    if not np.all(np.isfinite(pose)):
        raise ValueError("Pose must contain finite values")
    if not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError("Pose must be homogeneous")
    return pose


def pose_delta(first: NDArray, second: NDArray) -> tuple[float, float]:
    """Return translation distance and full 3D rotation angle in degrees."""
    pose_first = _pose_matrix(first)
    pose_second = _pose_matrix(second)
    translation = float(np.linalg.norm(pose_second[:3, 3] - pose_first[:3, 3]))
    relative_rotation = pose_first[:3, :3].T @ pose_second[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
    rotation = math.degrees(math.acos(cosine))
    return translation, rotation


class KeyframeSelector:
    """Stateful OR-threshold gate with a short pause after pose jumps."""

    def __init__(self, config: KeyframeConfig) -> None:
        self.config = config
        self._last_observed_pose: NDArray[np.float64] | None = None
        self._last_integrated_pose: NDArray[np.float64] | None = None
        self._last_integrated_timestamp_ns: int | None = None
        self._pause_remaining = 0

    def evaluate(self, pose: NDArray, timestamp_ns: int) -> KeyframeDecision:
        current = _pose_matrix(pose).copy()
        stamp = int(timestamp_ns)
        if stamp < 0:
            raise ValueError("timestamp_ns must be non-negative")

        if self._last_observed_pose is not None:
            observed_translation, observed_rotation = pose_delta(
                self._last_observed_pose, current
            )
            if (
                observed_translation > self.config.pose_jump_translation_m
                or observed_rotation > self.config.pose_jump_rotation_deg
            ):
                self._last_observed_pose = current
                self._last_integrated_pose = current
                self._last_integrated_timestamp_ns = stamp
                self._pause_remaining = self.config.pause_frames_after_jump
                return KeyframeDecision(
                    accept=False,
                    reason="pose_jump",
                    translation_m=observed_translation,
                    rotation_deg=observed_rotation,
                    elapsed_sec=0.0,
                    pose_jump=True,
                )
        self._last_observed_pose = current

        if self._pause_remaining > 0:
            self._pause_remaining -= 1
            return KeyframeDecision(
                accept=False,
                reason="post_jump_pause",
                translation_m=0.0,
                rotation_deg=0.0,
                elapsed_sec=0.0,
            )

        if self._last_integrated_pose is None:
            self._accept(current, stamp)
            return KeyframeDecision(True, "first", 0.0, 0.0, 0.0)

        if self._last_integrated_timestamp_ns is None:
            raise RuntimeError("Keyframe selector timestamp state is inconsistent")
        if stamp < self._last_integrated_timestamp_ns:
            return KeyframeDecision(False, "out_of_order", 0.0, 0.0, 0.0)

        translation, rotation = pose_delta(self._last_integrated_pose, current)
        elapsed = (stamp - self._last_integrated_timestamp_ns) * 1e-9
        if translation >= self.config.translation_threshold_m:
            reason = "translation"
        elif rotation >= self.config.rotation_threshold_deg:
            reason = "rotation"
        elif elapsed >= self.config.max_interval_sec:
            reason = "interval"
        else:
            return KeyframeDecision(
                False, "below_threshold", translation, rotation, elapsed
            )

        self._accept(current, stamp)
        return KeyframeDecision(True, reason, translation, rotation, elapsed)

    def _accept(self, pose: NDArray[np.float64], timestamp_ns: int) -> None:
        self._last_integrated_pose = pose.copy()
        self._last_integrated_timestamp_ns = timestamp_ns
