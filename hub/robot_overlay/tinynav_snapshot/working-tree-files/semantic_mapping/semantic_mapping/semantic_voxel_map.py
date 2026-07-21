"""Sparse confidence-vote semantic voxel layer independent of occupancy."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterator, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from semantic_mapping.raycasting import VoxelIndex, point_to_voxel, voxel_center


@dataclass(frozen=True)
class SemanticVoxelConfig:
    """Metric indexing and class-confirmation contract for semantic voxels."""

    resolution_m: float = 0.05
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    class_count: int = 11
    valid_class_ids: tuple[int, ...] = tuple(range(11))
    unknown_class_id: int = 0
    dynamic_class_ids: tuple[int, ...] = (10,)
    min_observations: int = 2
    confirmation_threshold: float = 0.50

    def __post_init__(self) -> None:
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be finite and positive")
        if len(self.origin_xyz) != 3 or not all(
            math.isfinite(value) for value in self.origin_xyz
        ):
            raise ValueError("origin_xyz must contain three finite values")
        if self.class_count <= 0:
            raise ValueError("class_count must be positive")
        valid = set(self.valid_class_ids)
        if not valid or any(value < 0 or value >= self.class_count for value in valid):
            raise ValueError("valid_class_ids must fit class_count")
        if self.unknown_class_id not in valid:
            raise ValueError("unknown_class_id must be a valid class")
        if not set(self.dynamic_class_ids).issubset(valid):
            raise ValueError("dynamic_class_ids must be valid classes")
        if self.min_observations <= 0:
            raise ValueError("min_observations must be positive")
        if not 0.0 <= self.confirmation_threshold <= 1.0:
            raise ValueError("confirmation_threshold must be in [0, 1]")


@dataclass
class SemanticVoxel:
    """Accumulated class evidence for one allocated metric voxel."""

    semantic_scores: NDArray[np.float32]
    observation_count: int = 0
    last_seen_timestamp_ns: int = 0


@dataclass(frozen=True)
class SemanticIntegrationStats:
    """One-frame semantic integration accounting."""

    input_points: int
    integrated_points: int
    unique_voxels: int
    skipped_unknown: int
    skipped_dynamic: int
    skipped_zero_weight: int


@dataclass(frozen=True)
class SemanticMapCounts:
    """Current allocated and confirmed semantic map sizes."""

    active: int
    confirmed: int
    unconfirmed: int
    total_observations: int
    by_class: dict[int, int] = field(default_factory=dict)


class SparseSemanticVoxelMap:
    """Dictionary-backed semantic scores with one observation per voxel/frame."""

    def __init__(self, config: SemanticVoxelConfig) -> None:
        self.config = config
        self._origin = np.asarray(config.origin_xyz, dtype=np.float64)
        self._valid_lookup = np.zeros(config.class_count, dtype=np.bool_)
        self._valid_lookup[list(config.valid_class_ids)] = True
        self._dynamic_lookup = np.zeros(config.class_count, dtype=np.bool_)
        self._dynamic_lookup[list(config.dynamic_class_ids)] = True
        self._voxels: dict[VoxelIndex, SemanticVoxel] = {}
        self.revision = 0

    def __len__(self) -> int:
        return len(self._voxels)

    @property
    def voxels(self) -> Mapping[VoxelIndex, SemanticVoxel]:
        return self._voxels

    def point_to_index(self, point: Sequence[float] | NDArray) -> VoxelIndex:
        return point_to_voxel(point, self._origin, self.config.resolution_m)

    def index_to_center(self, index: Sequence[int] | NDArray) -> NDArray[np.float64]:
        return voxel_center(index, self._origin, self.config.resolution_m)

    def iter_voxels(self) -> Iterator[tuple[VoxelIndex, SemanticVoxel]]:
        return iter(self._voxels.items())

    def integrate_observations(
        self,
        points: NDArray,
        labels: NDArray,
        weights: NDArray,
        timestamp_ns: int,
    ) -> SemanticIntegrationStats:
        """Fuse static surface labels after per-frame voxel density normalization."""
        point_array = np.asarray(points, dtype=np.float64)
        label_array = np.asarray(labels)
        weight_array = np.asarray(weights, dtype=np.float64)
        if point_array.ndim != 2 or point_array.shape[1] != 3:
            raise ValueError(f"points must have shape (N, 3), got {point_array.shape}")
        count = point_array.shape[0]
        if label_array.shape != (count,) or not np.issubdtype(
            label_array.dtype, np.integer
        ):
            raise ValueError("labels must be an integer array with shape (N,)")
        if weight_array.shape != (count,):
            raise ValueError("weights must have shape (N,)")
        if np.any(~np.isfinite(point_array)):
            raise ValueError("points must be finite")
        if np.any(~np.isfinite(weight_array)) or np.any(
            (weight_array < 0.0) | (weight_array > 1.0)
        ):
            raise ValueError("weights must be finite and in [0, 1]")
        labels_i64 = label_array.astype(np.int64, copy=False)
        if np.any(labels_i64 < 0) or np.any(labels_i64 >= self.config.class_count):
            raise ValueError("labels contain IDs outside class_count")
        if np.any(~self._valid_lookup[labels_i64]):
            raise ValueError("labels contain IDs outside valid_class_ids")
        stamp = int(timestamp_ns)
        if stamp < 0:
            raise ValueError("timestamp_ns must be non-negative")

        unknown = labels_i64 == self.config.unknown_class_id
        dynamic = self._dynamic_lookup[labels_i64]
        zero_weight = weight_array <= 0.0
        keep = ~unknown & ~dynamic & ~zero_weight
        if not np.any(keep):
            return SemanticIntegrationStats(
                input_points=count,
                integrated_points=0,
                unique_voxels=0,
                skipped_unknown=int(np.count_nonzero(unknown)),
                skipped_dynamic=int(np.count_nonzero(dynamic)),
                skipped_zero_weight=int(np.count_nonzero(zero_weight)),
            )

        kept_points = point_array[keep]
        kept_labels = labels_i64[keep]
        kept_weights = weight_array[keep]
        indices = np.floor(
            (kept_points - self._origin) / self.config.resolution_m
        ).astype(np.int64)
        unique_indices, inverse, points_per_voxel = np.unique(
            indices, axis=0, return_inverse=True, return_counts=True
        )
        frame_scores = np.zeros(
            (unique_indices.shape[0], self.config.class_count), dtype=np.float64
        )
        np.add.at(frame_scores, (inverse, kept_labels), kept_weights)
        frame_scores /= points_per_voxel[:, None]

        for raw_index, score_update in zip(unique_indices, frame_scores):
            index = tuple(int(value) for value in raw_index)
            voxel = self._voxels.get(index)
            if voxel is None:
                voxel = SemanticVoxel(
                    semantic_scores=np.zeros(
                        self.config.class_count, dtype=np.float32
                    )
                )
                self._voxels[index] = voxel
            voxel.semantic_scores += score_update.astype(np.float32)
            voxel.observation_count += 1
            voxel.last_seen_timestamp_ns = max(voxel.last_seen_timestamp_ns, stamp)
        self.revision += 1
        return SemanticIntegrationStats(
            input_points=count,
            integrated_points=int(kept_points.shape[0]),
            unique_voxels=int(unique_indices.shape[0]),
            skipped_unknown=int(np.count_nonzero(unknown)),
            skipped_dynamic=int(np.count_nonzero(dynamic)),
            skipped_zero_weight=int(np.count_nonzero(zero_weight)),
        )

    def label_and_confidence(self, index: VoxelIndex) -> tuple[int, float]:
        voxel = self._voxels.get(index)
        if voxel is None or voxel.observation_count < self.config.min_observations:
            return self.config.unknown_class_id, 0.0
        score_sum = float(np.sum(voxel.semantic_scores, dtype=np.float64))
        if score_sum <= 0.0:
            return self.config.unknown_class_id, 0.0
        label = int(np.argmax(voxel.semantic_scores))
        confidence = float(voxel.semantic_scores[label] / score_sum)
        if confidence < self.config.confirmation_threshold:
            return self.config.unknown_class_id, confidence
        return label, confidence

    def confirmed_arrays(
        self,
    ) -> tuple[
        NDArray[np.float32],
        NDArray[np.uint8],
        NDArray[np.float32],
        NDArray[np.uint32],
    ]:
        centers: list[NDArray[np.float64]] = []
        labels: list[int] = []
        confidences: list[float] = []
        observations: list[int] = []
        for index, voxel in sorted(self._voxels.items()):
            label, confidence = self.label_and_confidence(index)
            if label == self.config.unknown_class_id:
                continue
            centers.append(self.index_to_center(index))
            labels.append(label)
            confidences.append(confidence)
            observations.append(voxel.observation_count)
        if not centers:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0,), dtype=np.uint8),
                np.empty((0,), dtype=np.float32),
                np.empty((0,), dtype=np.uint32),
            )
        return (
            np.asarray(centers, dtype=np.float32),
            np.asarray(labels, dtype=np.uint8),
            np.asarray(confidences, dtype=np.float32),
            np.asarray(observations, dtype=np.uint32),
        )

    def confirmed_score_arrays(
        self,
    ) -> tuple[NDArray[np.float32], NDArray[np.uint8], NDArray[np.float32]]:
        """Return confirmed voxel centers, labels, and unnormalized class votes."""
        centers: list[NDArray[np.float64]] = []
        labels: list[int] = []
        scores: list[NDArray[np.float32]] = []
        for index, voxel in sorted(self._voxels.items()):
            label, _ = self.label_and_confidence(index)
            if label == self.config.unknown_class_id:
                continue
            centers.append(self.index_to_center(index))
            labels.append(label)
            scores.append(voxel.semantic_scores.copy())
        if not centers:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0,), dtype=np.uint8),
                np.empty((0, self.config.class_count), dtype=np.float32),
            )
        return (
            np.asarray(centers, dtype=np.float32),
            np.asarray(labels, dtype=np.uint8),
            np.asarray(scores, dtype=np.float32),
        )

    def counts(self) -> SemanticMapCounts:
        by_class = {class_id: 0 for class_id in self.config.valid_class_ids}
        confirmed = 0
        total_observations = 0
        for index, voxel in self._voxels.items():
            total_observations += voxel.observation_count
            label, _ = self.label_and_confidence(index)
            if label != self.config.unknown_class_id:
                confirmed += 1
                by_class[label] += 1
        return SemanticMapCounts(
            active=len(self._voxels),
            confirmed=confirmed,
            unconfirmed=len(self._voxels) - confirmed,
            total_observations=total_observations,
            by_class=by_class,
        )

    def to_arrays(self) -> dict[str, NDArray]:
        """Export deterministic arrays suitable for compressed NPZ storage."""
        ordered = sorted(self._voxels.items())
        count = len(ordered)
        indices = np.empty((count, 3), dtype=np.int32)
        scores = np.empty((count, self.config.class_count), dtype=np.float32)
        observations = np.empty(count, dtype=np.uint32)
        last_seen = np.empty(count, dtype=np.int64)
        for position, (index, voxel) in enumerate(ordered):
            indices[position] = index
            scores[position] = voxel.semantic_scores
            observations[position] = voxel.observation_count
            last_seen[position] = voxel.last_seen_timestamp_ns
        return {
            "indices": indices,
            "semantic_scores": scores,
            "observation_count": observations,
            "last_seen_timestamp_ns": last_seen,
        }

    @classmethod
    def from_arrays(
        cls, config: SemanticVoxelConfig, arrays: Mapping[str, NDArray]
    ) -> "SparseSemanticVoxelMap":
        """Restore a semantic map from arrays produced by :meth:`to_arrays`."""
        required = {
            "indices",
            "semantic_scores",
            "observation_count",
            "last_seen_timestamp_ns",
        }
        missing = required.difference(arrays)
        if missing:
            raise ValueError(f"Missing semantic voxel arrays: {sorted(missing)}")
        indices = np.asarray(arrays["indices"], dtype=np.int64)
        scores = np.asarray(arrays["semantic_scores"], dtype=np.float32)
        observations = np.asarray(arrays["observation_count"]).reshape(-1)
        last_seen = np.asarray(arrays["last_seen_timestamp_ns"]).reshape(-1)
        if indices.ndim != 2 or indices.shape[1] != 3:
            raise ValueError("indices must have shape (N, 3)")
        count = indices.shape[0]
        if scores.shape != (count, config.class_count):
            raise ValueError("semantic_scores shape does not match map config")
        if observations.shape != (count,) or last_seen.shape != (count,):
            raise ValueError("semantic observation arrays must have shape (N,)")
        if np.any(~np.isfinite(scores)) or np.any(scores < 0.0):
            raise ValueError("semantic_scores must be finite and non-negative")

        semantic_map = cls(config)
        for position, raw_index in enumerate(indices):
            index = tuple(int(value) for value in raw_index)
            if index in semantic_map._voxels:
                raise ValueError(f"Duplicate semantic voxel index: {index}")
            observation_count = int(observations[position])
            timestamp_ns = int(last_seen[position])
            if observation_count < 0 or timestamp_ns < 0:
                raise ValueError("Semantic counts and timestamps must be non-negative")
            semantic_map._voxels[index] = SemanticVoxel(
                semantic_scores=scores[position].copy(),
                observation_count=observation_count,
                last_seen_timestamp_ns=timestamp_ns,
            )
        semantic_map.revision = 1 if count else 0
        return semantic_map
