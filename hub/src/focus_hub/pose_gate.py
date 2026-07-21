"""Pose continuity and keyframe gates for the live Hub mapper.

The policy is source-derived from the TinyNav semantic-mapping experiment
preserved at
``hub/robot_overlay/tinynav_snapshot/working-tree-files/semantic_mapping/``.
It lives in the Hub package so the central mapper does not import or mutate
the archived robot-side snapshot.

Two separate gates are intentional:

* :class:`StartupPoseGate` waits for a short, mutually-consistent pose window
  before a fixed map extent is chosen.  A lone stale observation therefore
  cannot place the entire map in the wrong part of the world.
* :class:`KeyframeSelector` suppresses near-duplicate stationary frames and
  reports discontinuities before irreversible semantic evidence is fused.

Neither class sends commands or has any robot-control interface.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _pose_matrix(value: np.ndarray) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (4, 4):
        raise ValueError(f"pose must have shape (4, 4), got {pose.shape}")
    if not np.all(np.isfinite(pose)):
        raise ValueError("pose must contain finite values")
    if not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError("pose must be homogeneous")
    return pose


def pose_delta(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    """Return full 3-D translation and rotation deltas in metres/degrees."""
    pose_first = _pose_matrix(first)
    pose_second = _pose_matrix(second)
    translation = float(np.linalg.norm(pose_second[:3, 3] - pose_first[:3, 3]))
    relative_rotation = pose_first[:3, :3].T @ pose_second[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
    return translation, math.degrees(math.acos(cosine))


@dataclass(frozen=True)
class StartupPoseConfig:
    required_consecutive: int = 3
    max_translation_delta_m: float = 2.0
    max_rotation_delta_deg: float = 90.0
    max_interval_s: float = 10.0

    def __post_init__(self) -> None:
        if self.required_consecutive < 2:
            raise ValueError("required_consecutive must be at least 2")
        values = (
            self.max_translation_delta_m,
            self.max_rotation_delta_deg,
            self.max_interval_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("startup pose thresholds must be finite and positive")


@dataclass(frozen=True)
class StartupPoseDecision:
    ready: bool
    reset: bool
    reason: str
    consecutive: int
    translation_m: float = 0.0
    rotation_deg: float = 0.0
    elapsed_sec: float = 0.0


class StartupPoseGate:
    """Require a short continuous pose window before map initialization."""

    def __init__(self, config: StartupPoseConfig) -> None:
        self.config = config
        self._last_pose: np.ndarray | None = None
        self._last_timestamp_ns: int | None = None
        self._consecutive = 0

    def reset(self) -> None:
        self._last_pose = None
        self._last_timestamp_ns = None
        self._consecutive = 0

    def evaluate(self, pose: np.ndarray, timestamp_ns: int) -> StartupPoseDecision:
        current = _pose_matrix(pose).copy()
        stamp = int(timestamp_ns)
        if stamp < 0:
            raise ValueError("timestamp_ns must be non-negative")

        if self._last_pose is None:
            self._last_pose = current
            self._last_timestamp_ns = stamp
            self._consecutive = 1
            return StartupPoseDecision(False, False, "first_candidate", 1)

        if self._last_timestamp_ns is None:
            raise RuntimeError("startup pose gate timestamp state is inconsistent")
        translation, rotation = pose_delta(self._last_pose, current)
        elapsed = (stamp - self._last_timestamp_ns) * 1e-9
        discontinuous = (
            elapsed < 0.0
            or elapsed > self.config.max_interval_s
            or translation > self.config.max_translation_delta_m
            or rotation > self.config.max_rotation_delta_deg
        )
        self._last_pose = current
        self._last_timestamp_ns = stamp
        if discontinuous:
            self._consecutive = 1
            if elapsed < 0.0:
                reason = "out_of_order"
            elif elapsed > self.config.max_interval_s:
                reason = "time_gap"
            else:
                reason = "pose_jump"
            return StartupPoseDecision(
                False, True, reason, 1, translation, rotation, elapsed
            )

        self._consecutive += 1
        ready = self._consecutive >= self.config.required_consecutive
        return StartupPoseDecision(
            ready,
            False,
            "stable" if ready else "collecting",
            self._consecutive,
            translation,
            rotation,
            elapsed,
        )


@dataclass(frozen=True)
class KeyframeConfig:
    translation_threshold_m: float = 0.20
    rotation_threshold_deg: float = 10.0
    max_interval_sec: float = 5.0
    pose_jump_translation_m: float = 2.0
    pose_jump_rotation_deg: float = 90.0

    def __post_init__(self) -> None:
        values = (
            self.translation_threshold_m,
            self.rotation_threshold_deg,
            self.max_interval_sec,
            self.pose_jump_translation_m,
            self.pose_jump_rotation_deg,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("keyframe thresholds must be finite and non-negative")
        if self.max_interval_sec <= 0.0:
            raise ValueError("max_interval_sec must be positive")
        if self.pose_jump_translation_m <= self.translation_threshold_m:
            raise ValueError("pose jump translation must exceed keyframe translation")
        if self.pose_jump_rotation_deg <= self.rotation_threshold_deg:
            raise ValueError("pose jump rotation must exceed keyframe rotation")


@dataclass(frozen=True)
class KeyframeDecision:
    accept: bool
    reason: str
    translation_m: float
    rotation_deg: float
    elapsed_sec: float
    pose_jump: bool = False


class KeyframeSelector:
    """Stateful OR-threshold keyframe gate with discontinuity reporting."""

    def __init__(self, config: KeyframeConfig) -> None:
        self.config = config
        self._last_observed_pose: np.ndarray | None = None
        self._last_integrated_pose: np.ndarray | None = None
        self._last_integrated_timestamp_ns: int | None = None

    def evaluate(self, pose: np.ndarray, timestamp_ns: int) -> KeyframeDecision:
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
                return KeyframeDecision(
                    False,
                    "pose_jump",
                    observed_translation,
                    observed_rotation,
                    0.0,
                    pose_jump=True,
                )
        self._last_observed_pose = current

        if self._last_integrated_pose is None:
            self._accept(current, stamp)
            return KeyframeDecision(True, "first", 0.0, 0.0, 0.0)

        if self._last_integrated_timestamp_ns is None:
            raise RuntimeError("keyframe selector timestamp state is inconsistent")
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

    def _accept(self, pose: np.ndarray, timestamp_ns: int) -> None:
        self._last_integrated_pose = pose.copy()
        self._last_integrated_timestamp_ns = timestamp_ns
