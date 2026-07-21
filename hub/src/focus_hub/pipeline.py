"""Hub-side observation pipeline: spool -> decode -> central semantic map.

Consumes observations exactly as they arrived over the wire (the append-only
spool written by the API), so the mapping input is the transported data, not a
side channel.  Depth on the wire is already aligned to the RGB frame, so the
mapper runs with the RGB intrinsics and an identity depth-to-RGB extrinsic.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .central_mapping import CentralMapper, MapperConfig, RedNetSegmenter
from .depth_align import decode_depth_png16
from .models import ObservationMetadata


@dataclass(frozen=True)
class SpooledObservation:
    sequence: int
    metadata: ObservationMetadata
    rgb_bgr: np.ndarray
    depth_m: np.ndarray
    T_shared_camera: np.ndarray


def iter_spooled_observations(spool_dir: Path, robot_id: str, *, after_sequence: int = -1):
    """Yield spooled observations in sequence order.

    ``after_sequence`` filters by directory name before any parsing, so
    incremental consumers can tail a large spool cheaply.
    """
    robot_root = spool_dir / robot_id
    if not robot_root.is_dir():
        return
    for entry in sorted(robot_root.iterdir()):
        if entry.name.startswith(".") or not entry.is_dir():
            continue
        if int(entry.name) <= after_sequence:
            continue
        metadata = ObservationMetadata.model_validate_json(
            (entry / "metadata.json").read_text(encoding="utf-8")
        )
        rgb_path = entry / ("rgb.jpg" if metadata.rgb_encoding == "jpeg" else "rgb.png")
        rgb = cv2.imdecode(np.frombuffer(rgb_path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        if rgb is None:
            raise ValueError(f"undecodable RGB payload in {entry}")
        depth = decode_depth_png16((entry / "depth.png").read_bytes(), metadata.depth_scale_m)
        if rgb.shape[:2] != depth.shape:
            raise ValueError(f"RGB/depth shape mismatch in {entry}")
        pose = np.array(metadata.pose.shared_T_camera.matrix, dtype=np.float64).reshape(4, 4)
        yield SpooledObservation(
            sequence=metadata.sequence,
            metadata=metadata,
            rgb_bgr=rgb,
            depth_m=depth,
            T_shared_camera=pose,
        )


@dataclass
class _MapperFrame:
    depth_m: np.ndarray
    T_world_infra1: np.ndarray  # depth is RGB-aligned, so this is T_shared_camera


class SpoolMappingPipeline:
    """Builds the central semantic map for one robot from its spool."""

    def __init__(
        self,
        segmenter: RedNetSegmenter,
        K_rgb: np.ndarray,
        config: MapperConfig,
        origin_xy_m: tuple[float, float],
        floor_z_m: float,
        expected_transform_version: str | None = None,
    ) -> None:
        self.segmenter = segmenter
        self.mapper = CentralMapper(
            config=config,
            K_infra1=K_rgb,                # depth arrives aligned to the RGB frame
            K_rgb=K_rgb,
            T_rgb_to_infra1=np.eye(4),
            origin_xy_m=origin_xy_m,
            floor_z_m=floor_z_m,
        )
        self.last_camera_xy: tuple[float, float] | None = None
        self.last_camera_T: np.ndarray | None = None
        self.last_rgb_bgr: np.ndarray | None = None
        self.frames_processed = 0
        self.transform_version = expected_transform_version
        self.first_sequence: int | None = None
        self.last_sequence: int | None = None

    def process(self, observation: SpooledObservation) -> None:
        observation_version = observation.metadata.pose.transform_version
        if self.transform_version is None:
            self.transform_version = observation_version
        elif observation_version != self.transform_version:
            raise ValueError(
                "refusing to mix transform versions in one map: "
                f"bound={self.transform_version!r}, observation={observation_version!r}, "
                f"sequence={observation.sequence}"
            )
        pred = self.segmenter.segment(observation.rgb_bgr, observation.depth_m)
        self.mapper.integrate(
            _MapperFrame(
                depth_m=observation.depth_m,
                T_world_infra1=observation.T_shared_camera,
            ),
            pred,
        )
        self.last_camera_xy = (
            float(observation.T_shared_camera[0, 3]),
            float(observation.T_shared_camera[1, 3]),
        )
        self.last_camera_T = observation.T_shared_camera
        # Kept for the Perception VLM stage (needs the latest raw RGB, not
        # just the accumulated semantic grid) — see vlm_decision.py.
        self.last_rgb_bgr = observation.rgb_bgr
        if self.first_sequence is None:
            self.first_sequence = observation.sequence
        self.last_sequence = observation.sequence
        self.frames_processed += 1

    def run(self, spool_dir: Path, robot_id: str) -> int:
        for observation in iter_spooled_observations(spool_dir, robot_id):
            self.process(observation)
        return self.frames_processed

    def save(self, out_dir: Path) -> None:
        out_dir.mkdir(parents=True, exist_ok=True)
        # Atomic write: a concurrent reader (e.g. foxglove_relay.py polling
        # this same directory while the daemon periodically re-saves) must
        # never observe a partially-written file. np.savez_compressed writes
        # directly to its target path with no such guarantee, so write to a
        # sibling temp file first and os.replace() it into place -- POSIX
        # rename is atomic, readers see either the old or the new file whole,
        # never a torn one. The temp name must itself end in .npz: savez
        # silently APPENDS .npz to any path that doesn't already end with
        # it, so a naive "central_map.npz.tmp" actually gets written as
        # "central_map.npz.tmp.npz" and os.replace() then fails looking for
        # a file that was never created (hit this for real, crashed the
        # daemon -- not a hypothetical).
        tmp_path = out_dir / "central_map.tmp.npz"
        np.savez_compressed(
            tmp_path,
            grid=self.mapper.map.grid,
            origin_xy_m=np.array(self.mapper.map.origin_xy_m),
            floor_z_m=np.array(self.mapper.map.floor_z_m),
            resolution_m=np.array(self.mapper.config.resolution_m),
        )
        os.replace(tmp_path, out_dir / "central_map.npz")
        summary = {
            "frames_processed": self.frames_processed,
            "transform_version": self.transform_version,
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "obstacle_cells": int((self.mapper.map.grid[0] > 0.5).sum()),
            "explored_cells": int((self.mapper.map.grid[1] > 0.5).sum()),
        }
        (out_dir / "map_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
