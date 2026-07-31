"""Rate-aware keyframe continuity for low-rate robot occupancy mapping.

The archived TinyNav mapper uses a fixed adjacent-pose displacement as a
relocalization-jump test.  That is safe only while the input rate is also
fixed: a physically valid robot displacement can exceed the fixed threshold
when an expensive mapping callback delays the next observation.  This module
keeps the fixed threshold as the short-interval floor, then relaxes it only by
the platform's bounded physical speed and the observed source-stamp interval.

It has no ROS or actuator dependency.  The robot-side deployment adapter uses
it without modifying the provenance-preserving TinyNav snapshot.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

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


def _pose_delta(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    pose_first = _pose_matrix(first)
    pose_second = _pose_matrix(second)
    translation = float(np.linalg.norm(pose_second[:3, 3] - pose_first[:3, 3]))
    relative_rotation = pose_first[:3, :3].T @ pose_second[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
    return translation, math.degrees(math.acos(cosine))


@dataclass(frozen=True)
class RateAwareKeyframeDecision:
    accept: bool
    reason: str
    translation_m: float
    rotation_deg: float
    elapsed_sec: float
    pose_jump: bool = False


class RateAwareKeyframeSelector:
    """TinyNav-compatible selector with source-time-aware jump limits.

    ``config`` deliberately uses structural typing so the deployment can pass
    the archived semantic mapper's unchanged ``KeyframeConfig``.  Dynamic
    limits are bounded above: a long input outage therefore cannot turn an
    arbitrary localization discontinuity into valid motion.
    """

    def __init__(
        self,
        config: Any,
        *,
        maximum_translation_speed_mps: float = 0.25,
        translation_margin_m: float = 0.20,
        maximum_dynamic_translation_m: float = 1.50,
        maximum_rotation_speed_degps: float = 30.0,
        rotation_margin_deg: float = 15.0,
        maximum_dynamic_rotation_deg: float = 150.0,
    ) -> None:
        self.config = config
        limits = (
            maximum_translation_speed_mps,
            translation_margin_m,
            maximum_dynamic_translation_m,
            maximum_rotation_speed_degps,
            rotation_margin_deg,
            maximum_dynamic_rotation_deg,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in limits):
            raise ValueError("rate-aware jump limits must be finite and non-negative")
        if maximum_dynamic_translation_m < float(config.pose_jump_translation_m):
            raise ValueError("dynamic translation cap is below the fixed jump limit")
        if maximum_dynamic_rotation_deg < float(config.pose_jump_rotation_deg):
            raise ValueError("dynamic rotation cap is below the fixed jump limit")
        self.maximum_translation_speed_mps = maximum_translation_speed_mps
        self.translation_margin_m = translation_margin_m
        self.maximum_dynamic_translation_m = maximum_dynamic_translation_m
        self.maximum_rotation_speed_degps = maximum_rotation_speed_degps
        self.rotation_margin_deg = rotation_margin_deg
        self.maximum_dynamic_rotation_deg = maximum_dynamic_rotation_deg
        self._last_observed_pose: np.ndarray | None = None
        self._last_observed_timestamp_ns: int | None = None
        self._last_integrated_pose: np.ndarray | None = None
        self._last_integrated_timestamp_ns: int | None = None
        self._pause_remaining = 0

    def _jump_limits(self, elapsed_sec: float) -> tuple[float, float]:
        translation = min(
            self.maximum_dynamic_translation_m,
            max(
                float(self.config.pose_jump_translation_m),
                self.maximum_translation_speed_mps * elapsed_sec
                + self.translation_margin_m,
            ),
        )
        rotation = min(
            self.maximum_dynamic_rotation_deg,
            max(
                float(self.config.pose_jump_rotation_deg),
                self.maximum_rotation_speed_degps * elapsed_sec
                + self.rotation_margin_deg,
            ),
        )
        return translation, rotation

    def evaluate(
        self, pose: np.ndarray, timestamp_ns: int
    ) -> RateAwareKeyframeDecision:
        current = _pose_matrix(pose).copy()
        stamp = int(timestamp_ns)
        if stamp < 0:
            raise ValueError("timestamp_ns must be non-negative")

        if self._last_observed_timestamp_ns is not None:
            if stamp < self._last_observed_timestamp_ns:
                return RateAwareKeyframeDecision(
                    False, "out_of_order", 0.0, 0.0, 0.0
                )
            if self._last_observed_pose is None:
                raise RuntimeError("observed keyframe timestamp has no pose")
            observed_elapsed = (
                stamp - self._last_observed_timestamp_ns
            ) * 1e-9
            observed_translation, observed_rotation = _pose_delta(
                self._last_observed_pose, current
            )
            translation_limit, rotation_limit = self._jump_limits(
                observed_elapsed
            )
            if (
                observed_translation > translation_limit
                or observed_rotation > rotation_limit
            ):
                self._last_observed_pose = current
                self._last_observed_timestamp_ns = stamp
                self._last_integrated_pose = current
                self._last_integrated_timestamp_ns = stamp
                self._pause_remaining = int(
                    getattr(self.config, "pause_frames_after_jump", 0)
                )
                return RateAwareKeyframeDecision(
                    False,
                    "pose_jump",
                    observed_translation,
                    observed_rotation,
                    observed_elapsed,
                    pose_jump=True,
                )
        self._last_observed_pose = current
        self._last_observed_timestamp_ns = stamp

        if self._pause_remaining > 0:
            self._pause_remaining -= 1
            return RateAwareKeyframeDecision(
                False, "post_jump_pause", 0.0, 0.0, 0.0
            )

        if self._last_integrated_pose is None:
            self._accept(current, stamp)
            return RateAwareKeyframeDecision(True, "first", 0.0, 0.0, 0.0)
        if self._last_integrated_timestamp_ns is None:
            raise RuntimeError("integrated keyframe timestamp has no pose")
        if stamp < self._last_integrated_timestamp_ns:
            return RateAwareKeyframeDecision(
                False, "out_of_order", 0.0, 0.0, 0.0
            )

        translation, rotation = _pose_delta(
            self._last_integrated_pose, current
        )
        elapsed = (stamp - self._last_integrated_timestamp_ns) * 1e-9
        if translation >= float(self.config.translation_threshold_m):
            reason = "translation"
        elif rotation >= float(self.config.rotation_threshold_deg):
            reason = "rotation"
        elif elapsed >= float(self.config.max_interval_sec):
            reason = "interval"
        else:
            return RateAwareKeyframeDecision(
                False, "below_threshold", translation, rotation, elapsed
            )
        self._accept(current, stamp)
        return RateAwareKeyframeDecision(
            True, reason, translation, rotation, elapsed
        )

    def _accept(self, pose: np.ndarray, timestamp_ns: int) -> None:
        self._last_integrated_pose = pose.copy()
        self._last_integrated_timestamp_ns = timestamp_ns
