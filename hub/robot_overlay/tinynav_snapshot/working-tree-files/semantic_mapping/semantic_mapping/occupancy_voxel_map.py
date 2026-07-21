"""Sparse log-odds occupancy voxels independent of ROS I/O."""

from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from enum import IntEnum
import math

import numpy as np
from numpy.typing import NDArray

from semantic_mapping.raycasting import (
    VoxelIndex,
    batch_raycast_free_voxels,
    point_to_voxel,
    voxel_center,
)


class OccupancyState(IntEnum):
    """Planner-compatible occupancy state values."""

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100


@dataclass(frozen=True)
class OccupancyVoxelConfig:
    """Fixed geometry and inverse sensor model parameters."""

    resolution_m: float = 0.05
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    free_update: float = -0.40
    occupied_update: float = 0.85
    min_log_odds: float = -4.0
    max_log_odds: float = 4.0
    free_threshold: float = 0.30
    occupied_threshold: float = 0.70
    truncation_distance_m: float = 0.05

    def __post_init__(self) -> None:
        scalar_values = (
            self.resolution_m,
            self.free_update,
            self.occupied_update,
            self.min_log_odds,
            self.max_log_odds,
            self.free_threshold,
            self.occupied_threshold,
            self.truncation_distance_m,
        )
        if not all(math.isfinite(value) for value in scalar_values):
            raise ValueError("Voxel configuration values must be finite")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if len(self.origin_xyz) != 3 or not np.all(np.isfinite(self.origin_xyz)):
            raise ValueError("origin_xyz must contain three finite values")
        if self.free_update >= 0.0 or self.occupied_update <= 0.0:
            raise ValueError("Expected free_update < 0 and occupied_update > 0")
        if self.min_log_odds >= self.max_log_odds:
            raise ValueError("min_log_odds must be below max_log_odds")
        if not 0.0 < self.free_threshold < self.occupied_threshold < 1.0:
            raise ValueError(
                "Expected 0 < free_threshold < occupied_threshold < 1"
            )
        if self.truncation_distance_m < 0.0:
            raise ValueError("truncation_distance_m must be non-negative")


@dataclass
class OccupancyVoxel:
    """Accumulated geometric evidence for one allocated voxel."""

    log_odds: float = 0.0
    observation_count: int = 0
    free_observation_count: int = 0
    occupied_observation_count: int = 0
    last_seen_timestamp_ns: int = 0


@dataclass(frozen=True)
class IntegrationStats:
    """Counts from one frame-level point integration."""

    input_points: int
    valid_rays: int
    rejected_rays: int
    traversed_free_cells: int
    unique_free_voxels: int
    unique_occupied_voxels: int


@dataclass(frozen=True)
class VoxelMapCounts:
    """Current classified map sizes."""

    active: int
    free: int
    occupied: int
    uncertain: int


def probability_from_log_odds(log_odds: float) -> float:
    """Convert bounded log odds to an occupancy probability."""
    value = float(log_odds)
    if not math.isfinite(value):
        raise ValueError("log_odds must be finite")
    if value >= 0.0:
        return 1.0 / (1.0 + math.exp(-value))
    exponent = math.exp(value)
    return exponent / (1.0 + exponent)


class SparseOccupancyVoxelMap:
    """Dictionary-backed occupancy map with one update per voxel per frame."""

    def __init__(self, config: OccupancyVoxelConfig) -> None:
        self.config = config
        self._origin = np.asarray(config.origin_xyz, dtype=np.float64)
        self._voxels: dict[VoxelIndex, OccupancyVoxel] = {}
        self.revision = 0

    def __len__(self) -> int:
        return len(self._voxels)

    @property
    def voxels(self) -> Mapping[VoxelIndex, OccupancyVoxel]:
        """Return a read-only typing view of allocated voxels."""
        return self._voxels

    @property
    def origin(self) -> NDArray[np.float64]:
        return self._origin.copy()

    def point_to_index(self, point: Sequence[float] | NDArray) -> VoxelIndex:
        return point_to_voxel(point, self._origin, self.config.resolution_m)

    def index_to_center(self, index: Sequence[int] | NDArray) -> NDArray[np.float64]:
        return voxel_center(index, self._origin, self.config.resolution_m)

    def iter_voxels(self) -> Iterator[tuple[VoxelIndex, OccupancyVoxel]]:
        return iter(self._voxels.items())

    def probability(self, index: VoxelIndex) -> float | None:
        voxel = self._voxels.get(index)
        if voxel is None:
            return None
        return probability_from_log_odds(voxel.log_odds)

    def state(self, index: VoxelIndex) -> OccupancyState:
        probability = self.probability(index)
        if probability is None:
            return OccupancyState.UNKNOWN
        if probability < self.config.free_threshold:
            return OccupancyState.FREE
        if probability > self.config.occupied_threshold:
            return OccupancyState.OCCUPIED
        return OccupancyState.UNKNOWN

    def update_free(self, index: VoxelIndex, timestamp_ns: int) -> None:
        self._update(index, self.config.free_update, timestamp_ns, occupied=False)

    def update_occupied(self, index: VoxelIndex, timestamp_ns: int) -> None:
        self._update(
            index, self.config.occupied_update, timestamp_ns, occupied=True
        )

    def _update(
        self,
        index: VoxelIndex,
        delta: float,
        timestamp_ns: int,
        *,
        occupied: bool,
    ) -> None:
        if len(index) != 3:
            raise ValueError("Voxel index must have three components")
        stamp = int(timestamp_ns)
        if stamp < 0:
            raise ValueError("timestamp_ns must be non-negative")
        normalized_index = tuple(int(value) for value in index)
        voxel = self._voxels.setdefault(normalized_index, OccupancyVoxel())
        voxel.log_odds = float(
            min(
                self.config.max_log_odds,
                max(self.config.min_log_odds, voxel.log_odds + delta),
            )
        )
        voxel.observation_count += 1
        if occupied:
            voxel.occupied_observation_count += 1
        else:
            voxel.free_observation_count += 1
        voxel.last_seen_timestamp_ns = max(voxel.last_seen_timestamp_ns, stamp)

    def integrate_points(
        self,
        camera_origin: Sequence[float] | NDArray,
        endpoints: NDArray,
        timestamp_ns: int,
    ) -> IntegrationStats:
        """Fuse a frame of surface endpoints and carve free space.

        Duplicate evidence is collapsed within a frame. If a voxel is both a
        surface endpoint and traversed by another ray, occupied evidence wins
        for that frame and the voxel is removed from the free update set.
        """
        origin = np.asarray(camera_origin, dtype=np.float64)
        if origin.shape != (3,) or not np.all(np.isfinite(origin)):
            raise ValueError("camera_origin must contain three finite values")
        points = np.asarray(endpoints, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"endpoints must have shape (N, 3), got {points.shape}")
        stamp = int(timestamp_ns)
        if stamp < 0:
            raise ValueError("timestamp_ns must be non-negative")

        raycast = batch_raycast_free_voxels(
            origin,
            points,
            self._origin,
            self.config.resolution_m,
            self.config.truncation_distance_m,
        )
        for raw_index in raycast.free_voxels:
            self.update_free(tuple(int(value) for value in raw_index), stamp)
        for raw_index in raycast.occupied_voxels:
            self.update_occupied(tuple(int(value) for value in raw_index), stamp)
        if raycast.free_voxels.size or raycast.occupied_voxels.size:
            self.revision += 1

        return IntegrationStats(
            input_points=int(points.shape[0]),
            valid_rays=raycast.valid_rays,
            rejected_rays=raycast.rejected_rays,
            traversed_free_cells=raycast.traversed_free_cells,
            unique_free_voxels=int(raycast.free_voxels.shape[0]),
            unique_occupied_voxels=int(raycast.occupied_voxels.shape[0]),
        )

    def counts(self) -> VoxelMapCounts:
        free = 0
        occupied = 0
        for voxel in self._voxels.values():
            probability = probability_from_log_odds(voxel.log_odds)
            if probability < self.config.free_threshold:
                free += 1
            elif probability > self.config.occupied_threshold:
                occupied += 1
        active = len(self._voxels)
        return VoxelMapCounts(
            active=active,
            free=free,
            occupied=occupied,
            uncertain=active - free - occupied,
        )

    def occupied_points(self) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """Return centers and probabilities of currently occupied voxels."""
        indices: list[VoxelIndex] = []
        probabilities: list[float] = []
        for index, voxel in sorted(self._voxels.items()):
            probability = probability_from_log_odds(voxel.log_odds)
            if probability > self.config.occupied_threshold:
                indices.append(index)
                probabilities.append(probability)
        if not indices:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
            )
        centers = np.stack([self.index_to_center(index) for index in indices])
        return (
            centers.astype(np.float32, copy=False),
            np.asarray(probabilities, dtype=np.float32),
        )

    def projection_arrays(
        self,
    ) -> tuple[NDArray[np.int64], NDArray[np.float64]]:
        """Return unsorted voxel indices and log odds for vectorized projection."""
        if not self._voxels:
            return (
                np.empty((0, 3), dtype=np.int64),
                np.empty((0,), dtype=np.float64),
            )
        indices = np.asarray(list(self._voxels), dtype=np.int64)
        log_odds = np.fromiter(
            (voxel.log_odds for voxel in self._voxels.values()),
            dtype=np.float64,
            count=len(self._voxels),
        )
        return indices, log_odds

    def to_arrays(self) -> dict[str, NDArray]:
        """Export deterministic arrays suitable for compressed NPZ storage."""
        ordered = sorted(self._voxels.items())
        count = len(ordered)
        indices = np.empty((count, 3), dtype=np.int32)
        log_odds = np.empty(count, dtype=np.float32)
        observations = np.empty(count, dtype=np.uint32)
        free_observations = np.empty(count, dtype=np.uint32)
        occupied_observations = np.empty(count, dtype=np.uint32)
        last_seen = np.empty(count, dtype=np.int64)
        for position, (index, voxel) in enumerate(ordered):
            indices[position] = index
            log_odds[position] = voxel.log_odds
            observations[position] = voxel.observation_count
            free_observations[position] = voxel.free_observation_count
            occupied_observations[position] = voxel.occupied_observation_count
            last_seen[position] = voxel.last_seen_timestamp_ns
        return {
            "indices": indices,
            "log_odds": log_odds,
            "observation_count": observations,
            "free_observation_count": free_observations,
            "occupied_observation_count": occupied_observations,
            "last_seen_timestamp_ns": last_seen,
        }

    @classmethod
    def from_arrays(
        cls, config: OccupancyVoxelConfig, arrays: Mapping[str, NDArray]
    ) -> "SparseOccupancyVoxelMap":
        """Restore a map from arrays produced by :meth:`to_arrays`."""
        required = {
            "indices",
            "log_odds",
            "observation_count",
            "free_observation_count",
            "occupied_observation_count",
            "last_seen_timestamp_ns",
        }
        missing = required.difference(arrays)
        if missing:
            raise ValueError(f"Missing voxel arrays: {sorted(missing)}")
        indices = np.asarray(arrays["indices"], dtype=np.int64)
        if indices.ndim != 2 or indices.shape[1] != 3:
            raise ValueError("indices must have shape (N, 3)")
        count = indices.shape[0]
        one_dimensional = {
            name: np.asarray(arrays[name]).reshape(-1)
            for name in required
            if name != "indices"
        }
        invalid = {
            name: values.shape
            for name, values in one_dimensional.items()
            if values.shape != (count,)
        }
        if invalid:
            raise ValueError(f"Voxel array lengths do not match indices: {invalid}")

        result = cls(config)
        for position, raw_index in enumerate(indices):
            index = tuple(int(value) for value in raw_index)
            voxel = OccupancyVoxel(
                log_odds=float(one_dimensional["log_odds"][position]),
                observation_count=int(
                    one_dimensional["observation_count"][position]
                ),
                free_observation_count=int(
                    one_dimensional["free_observation_count"][position]
                ),
                occupied_observation_count=int(
                    one_dimensional["occupied_observation_count"][position]
                ),
                last_seen_timestamp_ns=int(
                    one_dimensional["last_seen_timestamp_ns"][position]
                ),
            )
            if not config.min_log_odds <= voxel.log_odds <= config.max_log_odds:
                raise ValueError(f"Voxel {index} has out-of-range log odds")
            if voxel.observation_count != (
                voxel.free_observation_count + voxel.occupied_observation_count
            ):
                raise ValueError(f"Voxel {index} has inconsistent observation counts")
            result._voxels[index] = voxel
        result.revision = 1 if count else 0
        return result
