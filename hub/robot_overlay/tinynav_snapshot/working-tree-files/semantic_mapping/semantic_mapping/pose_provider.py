"""Timestamped pose interpolation and TF2-backed pose lookup."""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass

from geometry_msgs.msg import Transform
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class PoseSample:
    """One source-to-target pose at a ROS timestamp."""

    timestamp_ns: int
    matrix: NDArray[np.float64]


@dataclass(frozen=True)
class PoseLookupResult:
    """Pose lookup result with an explicit temporal error bound."""

    matrix: NDArray[np.float64]
    time_error_ns: int
    interpolated: bool


class PoseLookupError(RuntimeError):
    """Raised when no pose satisfies the configured time bound."""


def make_transform_matrix(
    translation_xyz: NDArray | list[float] | tuple[float, float, float],
    quaternion_xyzw: NDArray | list[float] | tuple[float, float, float, float],
) -> NDArray[np.float64]:
    """Create a homogeneous transform from translation and XYZW quaternion."""
    translation = np.asarray(translation_xyz, dtype=np.float64)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if translation.shape != (3,):
        raise ValueError(f"Translation must have shape (3,), got {translation.shape}")
    if quaternion.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {quaternion.shape}")
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
        raise ValueError("Transform components must be finite")
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-12:
        raise ValueError("Quaternion norm must be non-zero")

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def transform_components(
    matrix: NDArray,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return translation XYZ and normalized quaternion XYZW from a transform."""
    transform = _validated_matrix(matrix)
    translation = transform[:3, 3].copy()
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return translation, quaternion.astype(np.float64, copy=False)


def interpolate_transform(
    first: NDArray,
    second: NDArray,
    fraction: float,
) -> NDArray[np.float64]:
    """Interpolate translation linearly and rotation with quaternion SLERP."""
    matrix_first = _validated_matrix(first)
    matrix_second = _validated_matrix(second)
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("Interpolation fraction must be in [0, 1]")

    rotations = Rotation.from_matrix(
        np.stack((matrix_first[:3, :3], matrix_second[:3, :3]))
    )
    rotation = Slerp([0.0, 1.0], rotations)([fraction]).as_matrix()[0]
    translation = (1.0 - fraction) * matrix_first[:3, 3] + fraction * matrix_second[
        :3, 3
    ]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _validated_matrix(matrix: NDArray) -> NDArray[np.float64]:
    result = np.asarray(matrix, dtype=np.float64)
    if result.shape != (4, 4):
        raise ValueError(f"Pose must have shape (4, 4), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError("Pose must be finite")
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError("Pose must be homogeneous")
    return result.copy()


class PoseBuffer:
    """Bounded sorted pose buffer with interpolation and nearest fallback."""

    def __init__(self, max_samples: int = 2000) -> None:
        if max_samples < 2:
            raise ValueError("max_samples must be at least 2")
        self.max_samples = max_samples
        self._samples: list[PoseSample] = []

    def __len__(self) -> int:
        return len(self._samples)

    def add(self, timestamp_ns: int, matrix: NDArray) -> None:
        """Insert or replace a timestamped pose while preserving time order."""
        sample = PoseSample(int(timestamp_ns), _validated_matrix(matrix))
        timestamps = [item.timestamp_ns for item in self._samples]
        index = bisect_left(timestamps, sample.timestamp_ns)
        if (
            index < len(self._samples)
            and self._samples[index].timestamp_ns == timestamp_ns
        ):
            self._samples[index] = sample
        else:
            self._samples.insert(index, sample)
        overflow = len(self._samples) - self.max_samples
        if overflow > 0:
            del self._samples[:overflow]

    def lookup(self, timestamp_ns: int, max_time_error_ns: int) -> PoseLookupResult:
        """Interpolate a pose or return the nearest sample within the error bound."""
        if max_time_error_ns < 0:
            raise ValueError("max_time_error_ns must be non-negative")
        if not self._samples:
            raise PoseLookupError("Pose buffer is empty")

        target = int(timestamp_ns)
        timestamps = [item.timestamp_ns for item in self._samples]
        index = bisect_left(timestamps, target)
        if index < len(self._samples) and timestamps[index] == target:
            return PoseLookupResult(self._samples[index].matrix.copy(), 0, False)

        before = self._samples[index - 1] if index > 0 else None
        after = self._samples[index] if index < len(self._samples) else None
        if before is not None and after is not None:
            before_error = target - before.timestamp_ns
            after_error = after.timestamp_ns - target
            if max(before_error, after_error) <= max_time_error_ns:
                fraction = before_error / (after.timestamp_ns - before.timestamp_ns)
                return PoseLookupResult(
                    interpolate_transform(before.matrix, after.matrix, fraction),
                    max(before_error, after_error),
                    True,
                )

        candidates = [item for item in (before, after) if item is not None]
        nearest = min(candidates, key=lambda item: abs(item.timestamp_ns - target))
        error = abs(nearest.timestamp_ns - target)
        if error <= max_time_error_ns:
            return PoseLookupResult(nearest.matrix.copy(), error, False)
        raise PoseLookupError(
            f"Nearest pose is {error * 1e-6:.3f} ms from target; "
            f"limit is {max_time_error_ns * 1e-6:.3f} ms"
        )


def matrix_from_transform_message(transform: Transform) -> NDArray[np.float64]:
    """Convert a geometry_msgs Transform-like object to a matrix."""
    translation = transform.translation
    rotation = transform.rotation
    return make_transform_matrix(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )
