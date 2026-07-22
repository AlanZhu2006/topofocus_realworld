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
from .ground_plane import (
    GroundPlaneConfig,
    depth_points_world,
    fit_ground_candidate,
    plane_angle_deg,
    plane_height_at,
)
from .models import ObservationMetadata
from .pose_gate import KeyframeConfig, KeyframeDecision, KeyframeSelector


@dataclass(frozen=True)
class SpooledObservation:
    sequence: int
    metadata: ObservationMetadata
    rgb_bgr: np.ndarray
    depth_m: np.ndarray
    T_shared_camera: np.ndarray


def iter_spooled_observations(
    spool_dir: Path, robot_id: str, *, after_sequence: int = -1
):
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
        rgb = cv2.imdecode(
            np.frombuffer(rgb_path.read_bytes(), np.uint8), cv2.IMREAD_COLOR
        )
        if rgb is None:
            raise ValueError(f"undecodable RGB payload in {entry}")
        depth = decode_depth_png16(
            (entry / "depth.png").read_bytes(), metadata.depth_scale_m
        )
        if rgb.shape[:2] != depth.shape:
            raise ValueError(f"RGB/depth shape mismatch in {entry}")
        pose = np.array(metadata.pose.shared_T_camera.matrix, dtype=np.float64).reshape(
            4, 4
        )
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
        *,
        floor_plane_coefficients: tuple[float, float, float] | None = None,
        ground_plane_config: GroundPlaneConfig | None = None,
        max_ground_tilt_delta_deg: float = 3.0,
        max_ground_height_delta_m: float = 0.08,
        ground_drift_consecutive_frames: int = 3,
        frame_id: str = "shared_world",
        robot_id: str | None = None,
        shared_frame_calibration_id: str | None = None,
        floor_source: str = "caller_provided_unverified",
        keyframe_config: KeyframeConfig | None = None,
        halt_on_pose_jump: bool = True,
    ) -> None:
        if not frame_id:
            raise ValueError("frame_id must be non-empty")
        self.segmenter = segmenter
        self.mapper = CentralMapper(
            config=config,
            K_infra1=K_rgb,  # depth arrives aligned to the RGB frame
            K_rgb=K_rgb,
            T_rgb_to_infra1=np.eye(4),
            origin_xy_m=origin_xy_m,
            floor_z_m=floor_z_m,
            floor_plane_coefficients=floor_plane_coefficients,
        )
        if max_ground_tilt_delta_deg <= 0.0 or not np.isfinite(
            max_ground_tilt_delta_deg
        ):
            raise ValueError("max_ground_tilt_delta_deg must be finite and positive")
        if max_ground_height_delta_m <= 0.0 or not np.isfinite(
            max_ground_height_delta_m
        ):
            raise ValueError("max_ground_height_delta_m must be finite and positive")
        if (
            isinstance(ground_drift_consecutive_frames, bool)
            or not isinstance(ground_drift_consecutive_frames, int)
            or ground_drift_consecutive_frames <= 0
        ):
            raise ValueError("ground_drift_consecutive_frames must be a positive integer")
        self.K_rgb = np.asarray(K_rgb, dtype=np.float64)
        self.ground_plane_config = ground_plane_config
        self.max_ground_tilt_delta_deg = float(max_ground_tilt_delta_deg)
        self.max_ground_height_delta_m = float(max_ground_height_delta_m)
        self.ground_drift_consecutive_frames = ground_drift_consecutive_frames
        self.last_camera_xy: tuple[float, float] | None = None
        self.last_camera_T: np.ndarray | None = None
        self.last_rgb_bgr: np.ndarray | None = None
        self.frames_processed = 0
        self.observations_seen = 0
        self.skipped_non_keyframes = 0
        self.pose_jump_events = 0
        self.ground_rejected_frames = 0
        self.ground_drift_frames = 0
        self.ground_drift_events = 0
        self.ground_drift_streak = 0
        self.last_ground_sequence: int | None = None
        self.last_ground_reason: str | None = None
        self.last_ground_tilt_delta_deg: float | None = None
        self.last_ground_height_delta_m: float | None = None
        self.transform_version = expected_transform_version
        self.frame_id = frame_id
        self.robot_id = robot_id
        self.shared_frame_calibration_id = shared_frame_calibration_id
        self.floor_source = floor_source
        self.keyframes = KeyframeSelector(keyframe_config) if keyframe_config else None
        self.halt_on_pose_jump = halt_on_pose_jump
        self.mapping_blocked_reason: str | None = None
        self.mapping_blocked_kind: str | None = None
        self.first_sequence: int | None = None
        self.last_sequence: int | None = None
        self.last_observation_sequence: int | None = None

    def process(self, observation: SpooledObservation) -> KeyframeDecision:
        observation_version = observation.metadata.pose.transform_version
        if self.transform_version is None:
            self.transform_version = observation_version
        elif observation_version != self.transform_version:
            raise ValueError(
                "refusing to mix transform versions in one map: "
                f"bound={self.transform_version!r}, observation={observation_version!r}, "
                f"sequence={observation.sequence}"
            )
        observation_frame = observation.metadata.pose.shared_T_camera.parent_frame
        if observation_frame != self.frame_id:
            raise ValueError(
                "refusing to mix coordinate frames in one map: "
                f"bound={self.frame_id!r}, observation={observation_frame!r}, "
                f"sequence={observation.sequence}"
            )

        self.observations_seen += 1
        self.last_observation_sequence = observation.sequence
        self.last_camera_xy = (
            float(observation.T_shared_camera[0, 3]),
            float(observation.T_shared_camera[1, 3]),
        )
        self.last_camera_T = observation.T_shared_camera
        # The dashboard camera remains current even when geometry integration
        # is skipped by the keyframe gate or latched after a pose jump.
        self.last_rgb_bgr = observation.rgb_bgr

        if self.mapping_blocked_reason is not None:
            self.skipped_non_keyframes += 1
            return KeyframeDecision(
                False,
                f"{self.mapping_blocked_kind or 'mapping_blocked'}_latched",
                0.0,
                0.0,
                0.0,
            )

        # Validate gravity/floor geometry before either the keyframe selector
        # commits this pose or RedNet spends GPU time.  A frame with no
        # trustworthy visible floor is skipped.  A fitted plane that moved
        # materially from the startup consensus latches the session: allowing
        # it into max-fused semantic layers would make the corruption
        # irreversible and usually indicates a bad mount/shared transform.
        ground_candidate = None
        if self.ground_plane_config is not None:
            ground_candidate = fit_ground_candidate(
                depth_points_world(
                    observation,
                    self.K_rgb,
                    self.ground_plane_config,
                ),
                observation.T_shared_camera[:3, 3],
                self.ground_plane_config,
            )
            self.last_ground_sequence = observation.sequence
            self.last_ground_reason = ground_candidate.reason
            if (
                not ground_candidate.accepted
                or ground_candidate.plane_coefficients is None
            ):
                self.ground_rejected_frames += 1
                # A missing/invalid plane breaks consecutiveness.  It gives
                # no evidence that drift persists, and the frame is already
                # excluded from both the pose gate and map integration.
                self.ground_drift_streak = 0
                self.skipped_non_keyframes += 1
                return KeyframeDecision(
                    False,
                    f"ground_{ground_candidate.reason}",
                    0.0,
                    0.0,
                    0.0,
                )

            startup_plane = self.mapper.map.floor_plane_coefficients
            camera_xy = observation.T_shared_camera[:2, 3]
            tilt_delta = plane_angle_deg(
                startup_plane,
                ground_candidate.plane_coefficients,
            )
            startup_height = plane_height_at(startup_plane, camera_xy)
            height_delta = abs(float(ground_candidate.ground_z_m) - startup_height)
            self.last_ground_tilt_delta_deg = tilt_delta
            self.last_ground_height_delta_m = height_delta
            if (
                tilt_delta > self.max_ground_tilt_delta_deg
                or height_delta > self.max_ground_height_delta_m
            ):
                # Do not integrate any outlying frame.  A single fit can be
                # transiently biased during a turn (RGB-D/pose timing, body
                # dynamics, or reduced visible floor), so only latch after a
                # configurable run of accepted-but-drifting floor fits.  A
                # subsequent in-range fit proves recovery and resets the run.
                self.ground_drift_frames += 1
                self.ground_drift_streak += 1
                self.skipped_non_keyframes += 1
                if self.ground_drift_streak < self.ground_drift_consecutive_frames:
                    self.last_ground_reason = "drift_pending"
                    return KeyframeDecision(
                        False,
                        "ground_drift_pending",
                        0.0,
                        0.0,
                        0.0,
                    )
                self.ground_drift_events += 1
                self.last_ground_reason = "drift_latched"
                self.mapping_blocked_kind = "ground_drift"
                self.mapping_blocked_reason = (
                    "ground plane drift requires a fresh calibrated map session: "
                    f"sequence={observation.sequence}, "
                    f"consecutive_frames={self.ground_drift_streak}, "
                    f"tilt_delta_deg={tilt_delta:.3f}, "
                    f"height_delta_m={height_delta:.3f}"
                )
                return KeyframeDecision(False, "ground_drift", 0.0, 0.0, 0.0)
            self.ground_drift_streak = 0

        if self.keyframes is None:
            decision = KeyframeDecision(True, "unfiltered", 0.0, 0.0, 0.0)
        else:
            decision = self.keyframes.evaluate(
                observation.T_shared_camera, observation.metadata.capture_time_ns
            )
        if decision.pose_jump:
            self.pose_jump_events += 1
            self.skipped_non_keyframes += 1
            if self.halt_on_pose_jump:
                self.mapping_blocked_kind = "pose_jump"
                self.mapping_blocked_reason = (
                    "pose discontinuity requires a fresh map session: "
                    f"sequence={observation.sequence}, "
                    f"translation_m={decision.translation_m:.3f}, "
                    f"rotation_deg={decision.rotation_deg:.2f}"
                )
            return decision
        if not decision.accept:
            self.skipped_non_keyframes += 1
            return decision

        pred = self.segmenter.segment(observation.rgb_bgr, observation.depth_m)
        self.mapper.integrate(
            _MapperFrame(
                depth_m=observation.depth_m,
                T_world_infra1=observation.T_shared_camera,
            ),
            pred,
            floor_plane_coefficients=(
                None
                if ground_candidate is None
                else ground_candidate.plane_coefficients
            ),
        )
        if self.first_sequence is None:
            self.first_sequence = observation.sequence
        self.last_sequence = observation.sequence
        self.frames_processed += 1
        return decision

    def run(self, spool_dir: Path, robot_id: str) -> int:
        for observation in iter_spooled_observations(spool_dir, robot_id):
            self.process(observation)
        return self.frames_processed

    def save(self, out_dir: Path) -> None:
        if not self.transform_version:
            raise ValueError("cannot save a map before binding a transform_version")
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
            floor_plane_coefficients=np.asarray(
                self.mapper.map.floor_plane_coefficients,
                dtype=np.float64,
            ),
            floor_source=np.asarray(self.floor_source),
            resolution_m=np.array(self.mapper.config.resolution_m),
            frame_id=np.asarray(self.frame_id),
            transform_version=np.asarray(self.transform_version or ""),
            shared_frame_calibration_id=np.asarray(
                self.shared_frame_calibration_id or ""
            ),
            map_format_version=np.asarray("focus-hub-central-map-v3"),
            obstacle_fusion_mode=np.asarray(self.mapper.config.obstacle_fusion_mode),
            obstacle_band_m=np.asarray(
                [
                    self.mapper.config.obstacle_band_low_m,
                    self.mapper.config.obstacle_band_high_m,
                ],
                dtype=np.float64,
            ),
            obstacle_min_hits=np.asarray(self.mapper.config.obstacle_min_hits),
        )
        os.replace(tmp_path, out_dir / "central_map.npz")
        summary = {
            "robot_id": self.robot_id,
            "source_kind": "focus_hub_incremental_rgbd",
            "source_status": "observed_spooled_observations",
            "map_format_version": "focus-hub-central-map-v3",
            "frames_processed": self.frames_processed,
            "observations_seen": self.observations_seen,
            "skipped_non_keyframes": self.skipped_non_keyframes,
            "pose_jump_events": self.pose_jump_events,
            "ground_rejected_frames": self.ground_rejected_frames,
            "ground_drift_frames": self.ground_drift_frames,
            "ground_drift_events": self.ground_drift_events,
            "ground_drift_streak": self.ground_drift_streak,
            "mapping_blocked_reason": self.mapping_blocked_reason,
            "mapping_blocked_kind": self.mapping_blocked_kind,
            "transform_version": self.transform_version,
            "frame_id": self.frame_id,
            "shared_frame_calibration_id": self.shared_frame_calibration_id,
            "floor_z_m": self.mapper.map.floor_z_m,
            "floor_plane_coefficients": list(self.mapper.map.floor_plane_coefficients),
            "floor_source": self.floor_source,
            "ground_guard": {
                "enabled": self.ground_plane_config is not None,
                "max_tilt_delta_deg": self.max_ground_tilt_delta_deg,
                "max_height_delta_m": self.max_ground_height_delta_m,
                "consecutive_frames_to_latch": self.ground_drift_consecutive_frames,
                "last_sequence": self.last_ground_sequence,
                "last_reason": self.last_ground_reason,
                "last_tilt_delta_deg": self.last_ground_tilt_delta_deg,
                "last_height_delta_m": self.last_ground_height_delta_m,
            },
            "obstacle_fusion_mode": self.mapper.config.obstacle_fusion_mode,
            "obstacle_band_m": [
                self.mapper.config.obstacle_band_low_m,
                self.mapper.config.obstacle_band_high_m,
            ],
            "obstacle_min_hits": self.mapper.config.obstacle_min_hits,
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "last_observation_sequence": self.last_observation_sequence,
            "obstacle_cells": int((self.mapper.map.grid[0] > 0.5).sum()),
            "explored_cells": int((self.mapper.map.grid[1] > 0.5).sum()),
        }
        summary_tmp = out_dir / "map_summary.json.tmp"
        summary_tmp.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(summary_tmp, out_dir / "map_summary.json")
