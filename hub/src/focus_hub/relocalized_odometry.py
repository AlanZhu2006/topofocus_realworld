"""Fail-closed helpers for converting tracking odometry into a saved map.

TinyNav's :class:`MapNode` publishes ``tracking_T_map`` as a TF after a
successful image/depth relocalization.  The continuous odometry streams remain
in the drifting tracking frame, so consumers must explicitly compute

``map_T_camera = inverse(tracking_T_map) @ tracking_T_camera``.

The helpers in this module keep that coordinate conversion independent of ROS
and require multiple mutually consistent, exact-stamp relocalization pairs
before a runtime TF is accepted.  A ROS deployment wrapper lives in
``hub/robot_overlay``; no upstream TinyNav source is modified.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import compose_rigid, invert_rigid


Rigid = tuple[float, ...]


def _validate_rigid(
    matrix: Sequence[float],
    *,
    label: str,
    tolerance: float = 2e-4,
) -> Rigid:
    if len(matrix) != 16:
        raise ValueError(f"{label} must contain 16 row-major values")
    values = tuple(float(value) for value in matrix)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains a non-finite value")
    if any(
        abs(values[index] - expected) > tolerance
        for index, expected in zip(
            (12, 13, 14, 15),
            (0.0, 0.0, 0.0, 1.0),
            strict=True,
        )
    ):
        raise ValueError(f"{label} is not homogeneous")
    rotation = tuple(
        tuple(values[row * 4 + column] for column in range(3))
        for row in range(3)
    )
    for row in range(3):
        for column in range(3):
            dot = sum(
                rotation[index][row] * rotation[index][column]
                for index in range(3)
            )
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > tolerance:
                raise ValueError(f"{label} rotation is not orthonormal")
    determinant = (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(f"{label} rotation determinant is not +1")
    return values


def pose_matrix(
    position_xyz: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> Rigid:
    """Build a checked row-major rigid transform from a ROS-style pose."""

    if len(position_xyz) != 3 or len(quaternion_xyzw) != 4:
        raise ValueError("pose requires XYZ and XYZW")
    x, y, z = (float(value) for value in position_xyz)
    qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
    if not all(math.isfinite(value) for value in (x, y, z, qx, qy, qz, qw)):
        raise ValueError("pose contains a non-finite value")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise ValueError("pose quaternion has zero norm")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return _validate_rigid(
        (
            1.0 - 2.0 * (yy + zz),
            2.0 * (xy - wz),
            2.0 * (xz + wy),
            x,
            2.0 * (xy + wz),
            1.0 - 2.0 * (xx + zz),
            2.0 * (yz - wx),
            y,
            2.0 * (xz - wy),
            2.0 * (yz + wx),
            1.0 - 2.0 * (xx + yy),
            z,
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        label="pose",
    )


def quaternion_xyzw(matrix: Sequence[float]) -> tuple[float, float, float, float]:
    """Return a normalized ROS-order quaternion from a checked transform."""

    value = _validate_rigid(matrix, label="transform")
    r00, r01, r02 = value[0], value[1], value[2]
    r10, r11, r12 = value[4], value[5], value[6]
    r20, r21, r22 = value[8], value[9], value[10]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale
    quaternion = (qx, qy, qz, qw)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("rotation produced an invalid quaternion")
    return tuple(component / norm for component in quaternion)


def transform_error(
    first: Sequence[float],
    second: Sequence[float],
) -> tuple[float, float]:
    """Return translation metres and full 3-D rotation degrees."""

    a = _validate_rigid(first, label="first transform")
    b = _validate_rigid(second, label="second transform")
    relative = compose_rigid(invert_rigid(a), b)
    translation_m = math.sqrt(
        relative[3] * relative[3]
        + relative[7] * relative[7]
        + relative[11] * relative[11]
    )
    cosine = max(
        -1.0,
        min(1.0, (relative[0] + relative[5] + relative[10] - 1.0) / 2.0),
    )
    return translation_m, math.degrees(math.acos(cosine))


def alignment_tilt_deg(tracking_T_map: Sequence[float]) -> float:
    """Return the angle between the tracking and map +Z axes."""

    transform = _validate_rigid(tracking_T_map, label="tracking_T_map")
    cosine = max(-1.0, min(1.0, transform[10]))
    return math.degrees(math.acos(cosine))


def map_pose(
    *,
    tracking_T_map: Sequence[float],
    tracking_T_camera: Sequence[float],
) -> Rigid:
    """Convert a continuous camera pose from tracking coordinates to map."""

    tracking_map = _validate_rigid(
        tracking_T_map, label="tracking_T_map"
    )
    tracking_camera = _validate_rigid(
        tracking_T_camera, label="tracking_T_camera"
    )
    return _validate_rigid(
        compose_rigid(invert_rigid(tracking_map), tracking_camera),
        label="map_T_camera",
    )


def rotate_pose_covariance(
    covariance: Sequence[float],
    *,
    map_T_tracking: Sequence[float],
) -> tuple[float, ...]:
    """Rotate a ROS 6x6 pose covariance into the saved-map axes.

    Position and small-angle orientation errors are both rotated by the same
    parent-frame rotation.  The transform translation is deliberately absent:
    covariance belongs to the camera pose, not to a spatial twist adjoint.
    """

    if len(covariance) != 36:
        raise ValueError("pose covariance must contain 36 values")
    values = tuple(float(value) for value in covariance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose covariance contains a non-finite value")
    transform = _validate_rigid(map_T_tracking, label="map_T_tracking")
    rotation = (
        (transform[0], transform[1], transform[2]),
        (transform[4], transform[5], transform[6]),
        (transform[8], transform[9], transform[10]),
    )
    block = tuple(
        tuple(
            rotation[row % 3][column % 3]
            if row // 3 == column // 3
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )
    source = tuple(
        tuple(values[row * 6 + column] for column in range(6))
        for row in range(6)
    )
    left = tuple(
        tuple(
            sum(block[row][index] * source[index][column] for index in range(6))
            for column in range(6)
        )
        for row in range(6)
    )
    rotated = tuple(
        sum(left[row][index] * block[column][index] for index in range(6))
        for row in range(6)
        for column in range(6)
    )
    return rotated


@dataclass(frozen=True)
class RelocalizationCandidate:
    stamp_ns: int
    observed_ns: int
    tracking_T_map: Rigid


@dataclass(frozen=True)
class RelocalizationDecision:
    ready: bool
    reason: str
    support: int
    latest_supported_observed_ns: int
    consensus_tracking_T_map: Rigid | None
    source_translation_error_m: float | None = None
    source_rotation_error_deg: float | None = None


class RelocalizationConsensus:
    """Validate TinyNav's runtime map transform against exact-stamp pairs."""

    def __init__(
        self,
        *,
        minimum_support: int = 3,
        history_size: int = 12,
        candidate_window_s: float = 45.0,
        maximum_supported_age_s: float = 30.0,
        maximum_cluster_translation_m: float = 0.25,
        maximum_cluster_rotation_deg: float = 7.0,
        maximum_source_translation_m: float = 0.30,
        maximum_source_rotation_deg: float = 8.0,
        maximum_alignment_tilt_deg: float = 15.0,
    ) -> None:
        if minimum_support < 2:
            raise ValueError("minimum_support must be at least two")
        if history_size < minimum_support:
            raise ValueError("history_size must cover minimum_support")
        if min(
            candidate_window_s,
            maximum_supported_age_s,
            maximum_cluster_translation_m,
            maximum_cluster_rotation_deg,
            maximum_source_translation_m,
            maximum_source_rotation_deg,
            maximum_alignment_tilt_deg,
        ) <= 0.0:
            raise ValueError("relocalization limits must be positive")
        self.minimum_support = minimum_support
        self.candidate_window_ns = int(candidate_window_s * 1e9)
        self.maximum_supported_age_ns = int(maximum_supported_age_s * 1e9)
        self.maximum_cluster_translation_m = maximum_cluster_translation_m
        self.maximum_cluster_rotation_deg = maximum_cluster_rotation_deg
        self.maximum_source_translation_m = maximum_source_translation_m
        self.maximum_source_rotation_deg = maximum_source_rotation_deg
        self.maximum_alignment_tilt_deg = maximum_alignment_tilt_deg
        self._candidates: deque[RelocalizationCandidate] = deque(
            maxlen=history_size
        )

    @property
    def candidate_count(self) -> int:
        return len(self._candidates)

    def add_pair(
        self,
        *,
        tracking_T_camera: Sequence[float],
        map_T_camera: Sequence[float],
        stamp_ns: int,
        observed_ns: int,
    ) -> RelocalizationCandidate:
        if stamp_ns <= 0 or observed_ns <= 0:
            raise ValueError("relocalization timestamps must be positive")
        tracking_camera = _validate_rigid(
            tracking_T_camera, label="tracking_T_camera"
        )
        map_camera = _validate_rigid(map_T_camera, label="map_T_camera")
        tracking_map = _validate_rigid(
            compose_rigid(tracking_camera, invert_rigid(map_camera)),
            label="candidate tracking_T_map",
        )
        tilt = alignment_tilt_deg(tracking_map)
        if tilt > self.maximum_alignment_tilt_deg:
            raise ValueError(
                "candidate tracking_T_map tilt exceeds limit: "
                f"{tilt:.3f} > {self.maximum_alignment_tilt_deg:.3f} deg"
            )
        candidate = RelocalizationCandidate(
            stamp_ns=int(stamp_ns),
            observed_ns=int(observed_ns),
            tracking_T_map=tracking_map,
        )
        self._candidates.append(candidate)
        return candidate

    def _cluster(
        self, *, now_ns: int
    ) -> tuple[RelocalizationCandidate | None, tuple[RelocalizationCandidate, ...]]:
        fresh = tuple(
            candidate
            for candidate in self._candidates
            if 0 <= now_ns - candidate.observed_ns <= self.candidate_window_ns
        )
        if not fresh:
            return None, ()
        best_center: RelocalizationCandidate | None = None
        best_members: tuple[RelocalizationCandidate, ...] = ()
        best_cost = math.inf
        for center in fresh:
            members: list[RelocalizationCandidate] = []
            cost = 0.0
            for candidate in fresh:
                translation_m, rotation_deg = transform_error(
                    center.tracking_T_map, candidate.tracking_T_map
                )
                if (
                    translation_m <= self.maximum_cluster_translation_m
                    and rotation_deg <= self.maximum_cluster_rotation_deg
                ):
                    members.append(candidate)
                    cost += (
                        translation_m / self.maximum_cluster_translation_m
                        + rotation_deg / self.maximum_cluster_rotation_deg
                    )
            members_tuple = tuple(members)
            if (
                len(members_tuple) > len(best_members)
                or (
                    len(members_tuple) == len(best_members)
                    and (
                        cost < best_cost
                        or (
                            math.isclose(cost, best_cost)
                            and best_center is not None
                            and center.observed_ns > best_center.observed_ns
                        )
                    )
                )
            ):
                best_center = center
                best_members = members_tuple
                best_cost = cost
        return best_center, best_members

    def evaluate(
        self,
        *,
        source_tracking_T_map: Sequence[float] | None,
        now_ns: int,
    ) -> RelocalizationDecision:
        if now_ns <= 0:
            raise ValueError("now_ns must be positive")
        center, members = self._cluster(now_ns=now_ns)
        support = len(members)
        latest_ns = max(
            (candidate.observed_ns for candidate in members),
            default=0,
        )
        if center is None or support < self.minimum_support:
            return RelocalizationDecision(
                ready=False,
                reason="INSUFFICIENT_CONSISTENT_RELOCALIZATIONS",
                support=support,
                latest_supported_observed_ns=latest_ns,
                consensus_tracking_T_map=(
                    None if center is None else center.tracking_T_map
                ),
            )
        if (
            latest_ns <= 0
            or now_ns - latest_ns > self.maximum_supported_age_ns
        ):
            return RelocalizationDecision(
                ready=False,
                reason="RELOCALIZATION_STALE",
                support=support,
                latest_supported_observed_ns=latest_ns,
                consensus_tracking_T_map=center.tracking_T_map,
            )
        if source_tracking_T_map is None:
            return RelocalizationDecision(
                ready=False,
                reason="MAP_TF_UNAVAILABLE",
                support=support,
                latest_supported_observed_ns=latest_ns,
                consensus_tracking_T_map=center.tracking_T_map,
            )
        try:
            source = _validate_rigid(
                source_tracking_T_map, label="source tracking_T_map"
            )
            source_tilt = alignment_tilt_deg(source)
        except ValueError:
            return RelocalizationDecision(
                ready=False,
                reason="MAP_TF_INVALID",
                support=support,
                latest_supported_observed_ns=latest_ns,
                consensus_tracking_T_map=center.tracking_T_map,
            )
        if source_tilt > self.maximum_alignment_tilt_deg:
            return RelocalizationDecision(
                ready=False,
                reason="MAP_TF_TILT_REJECTED",
                support=support,
                latest_supported_observed_ns=latest_ns,
                consensus_tracking_T_map=center.tracking_T_map,
            )
        translation_m, rotation_deg = transform_error(
            center.tracking_T_map, source
        )
        ready = (
            translation_m <= self.maximum_source_translation_m
            and rotation_deg <= self.maximum_source_rotation_deg
        )
        return RelocalizationDecision(
            ready=ready,
            reason=(
                "READY"
                if ready
                else "MAP_TF_DISAGREES_WITH_RELOCALIZATION_PAIRS"
            ),
            support=support,
            latest_supported_observed_ns=latest_ns,
            consensus_tracking_T_map=center.tracking_T_map,
            source_translation_error_m=translation_m,
            source_rotation_error_deg=rotation_deg,
        )


class ExactStampPosePairs:
    """Pair tracking and map camera poses without approximate-time guessing."""

    def __init__(self, *, maximum_entries: int = 40) -> None:
        if maximum_entries < 2:
            raise ValueError("maximum_entries must be at least two")
        self.maximum_entries = maximum_entries
        self._tracking: dict[int, Rigid] = {}
        self._map: dict[int, Rigid] = {}

    def _trim(self, values: dict[int, Rigid]) -> None:
        while len(values) > self.maximum_entries:
            del values[min(values)]

    def _add(
        self,
        own: dict[int, Rigid],
        other: dict[int, Rigid],
        *,
        stamp_ns: int,
        matrix: Sequence[float],
        label: str,
    ) -> tuple[Rigid, Rigid] | None:
        if stamp_ns <= 0:
            raise ValueError("pose timestamp must be positive")
        value = _validate_rigid(matrix, label=label)
        own[int(stamp_ns)] = value
        self._trim(own)
        paired = other.pop(int(stamp_ns), None)
        if paired is None:
            return None
        own.pop(int(stamp_ns), None)
        if own is self._tracking:
            return value, paired
        return paired, value

    def add_tracking(
        self, *, stamp_ns: int, tracking_T_camera: Sequence[float]
    ) -> tuple[Rigid, Rigid] | None:
        return self._add(
            self._tracking,
            self._map,
            stamp_ns=stamp_ns,
            matrix=tracking_T_camera,
            label="tracking_T_camera",
        )

    def add_map(
        self, *, stamp_ns: int, map_T_camera: Sequence[float]
    ) -> tuple[Rigid, Rigid] | None:
        return self._add(
            self._map,
            self._tracking,
            stamp_ns=stamp_ns,
            matrix=map_T_camera,
            label="map_T_camera",
        )
