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
import time
from typing import Protocol

import cv2
import numpy as np

from .central_mapping import CentralMapper, MapperConfig
from .depth_align import decode_depth_png16
from .ground_plane import (
    GroundPlaneConfig,
    depth_points_world,
    fit_ground_candidate,
    plane_angle_deg,
    plane_height_at,
)
from .models import ObservationMetadata
from .pose_gate import (
    KeyframeConfig,
    KeyframeDecision,
    KeyframeSelector,
    pose_delta,
)
from .semantic_yolo import SemanticYoloConfig, reinforce_rednet_prediction


POST_MOTION_GROUND_REBASE_GATE_MULTIPLIER = 3.0
DEFAULT_POST_MOTION_GROUND_REBASE_WINDOW_S = 120.0


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


class SemanticSegmenter(Protocol):
    """Pixel backend contract.

    Backends may return either MP3D-40 IDs (``H x W``) or the executable
    source's multi-hot HM3D tensor (``H x W x 15``).
    """

    def segment(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> np.ndarray: ...


class SpoolMappingPipeline:
    """Builds the central semantic map for one robot from its spool."""

    def __init__(
        self,
        segmenter: SemanticSegmenter,
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
        ground_drift_min_duration_s: float = 5.0,
        ground_drift_post_motion_rebase_window_s: float = (
            DEFAULT_POST_MOTION_GROUND_REBASE_WINDOW_S
        ),
        ground_drift_stationary_translation_m: float = 0.03,
        ground_drift_stationary_rotation_deg: float = 2.0,
        allow_ground_height_translation_for_2d: bool = False,
        frame_id: str = "shared_world",
        robot_id: str | None = None,
        shared_frame_calibration_id: str | None = None,
        floor_source: str = "caller_provided_unverified",
        keyframe_config: KeyframeConfig | None = None,
        halt_on_pose_jump: bool = True,
        semantic_detector=None,
        semantic_yolo_config: SemanticYoloConfig | None = None,
        semantic_yolo_reinforce_map: bool = True,
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
        if (
            not np.isfinite(ground_drift_min_duration_s)
            or ground_drift_min_duration_s <= 0.0
        ):
            raise ValueError("ground_drift_min_duration_s must be finite and positive")
        if (
            not np.isfinite(ground_drift_post_motion_rebase_window_s)
            or ground_drift_post_motion_rebase_window_s
            < ground_drift_min_duration_s
        ):
            raise ValueError(
                "ground_drift_post_motion_rebase_window_s must be finite and "
                "at least ground_drift_min_duration_s"
            )
        for value, name in (
            (
                ground_drift_stationary_translation_m,
                "ground_drift_stationary_translation_m",
            ),
            (
                ground_drift_stationary_rotation_deg,
                "ground_drift_stationary_rotation_deg",
            ),
        ):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not isinstance(allow_ground_height_translation_for_2d, bool):
            raise ValueError(
                "allow_ground_height_translation_for_2d must be a boolean"
            )
        self.K_rgb = np.asarray(K_rgb, dtype=np.float64)
        self.ground_plane_config = ground_plane_config
        self.max_ground_tilt_delta_deg = float(max_ground_tilt_delta_deg)
        self.max_ground_height_delta_m = float(max_ground_height_delta_m)
        self.ground_drift_consecutive_frames = ground_drift_consecutive_frames
        self.ground_drift_min_duration_s = float(ground_drift_min_duration_s)
        self.ground_drift_post_motion_rebase_window_s = float(
            ground_drift_post_motion_rebase_window_s
        )
        self.ground_drift_stationary_translation_m = float(
            ground_drift_stationary_translation_m
        )
        self.ground_drift_stationary_rotation_deg = float(
            ground_drift_stationary_rotation_deg
        )
        self.allow_ground_height_translation_for_2d = (
            allow_ground_height_translation_for_2d
        )
        self.last_camera_xy: tuple[float, float] | None = None
        self.last_camera_T: np.ndarray | None = None
        self.last_robot_xy: tuple[float, float] | None = None
        self.last_robot_T: np.ndarray | None = None
        self.last_rgb_bgr: np.ndarray | None = None
        self.trajectory_xy_m: list[tuple[float, float]] = []
        self.robot_trajectory_xy_m: list[tuple[float, float]] = []
        self.robot_pose_source = "camera_pose_fallback_no_base_T_camera"
        self.frames_processed = 0
        self.observations_seen = 0
        self.skipped_non_keyframes = 0
        self.pose_jump_events = 0
        self.ground_rejected_frames = 0
        self.ground_rejection_streak = 0
        self.ground_rejection_streak_start_capture_time_ns: int | None = None
        self.last_ground_rejection_duration_s = 0.0
        self.ground_drift_frames = 0
        self.ground_drift_events = 0
        self.ground_drift_streak = 0
        self.ground_drift_streak_start_capture_time_ns: int | None = None
        self.last_ground_drift_duration_s = 0.0
        self.ground_drift_motion_deferred_frames = 0
        self.ground_drift_reference_rebases = 0
        self.ground_drift_motion_translation_m = 0.0
        self.ground_drift_motion_rotation_deg = 0.0
        self.ground_drift_last_motion_capture_time_ns: int | None = None
        self.ground_drift_candidate_planes: list[tuple[float, float, float]] = []
        self.ground_reference_plane_coefficients = tuple(
            float(value) for value in self.mapper.map.floor_plane_coefficients
        )
        self.last_ground_rebase_sequence: int | None = None
        self.last_ground_rebase_tilt_delta_deg: float | None = None
        self.last_ground_rebase_height_delta_m: float | None = None
        self.ground_height_translation_frames = 0
        self.max_ground_height_translation_m = 0.0
        self.last_ground_sequence: int | None = None
        self.last_ground_reason: str | None = None
        self.last_ground_tilt_delta_deg: float | None = None
        self.last_ground_height_delta_m: float | None = None
        self.last_ground_pose_translation_m: float | None = None
        self.last_ground_pose_rotation_deg: float | None = None
        self.last_ground_pose_moving: bool | None = None
        self.transform_version = expected_transform_version
        self.frame_id = frame_id
        self.robot_id = robot_id
        self.shared_frame_calibration_id = shared_frame_calibration_id
        self.floor_source = floor_source
        self.keyframes = KeyframeSelector(keyframe_config) if keyframe_config else None
        self.halt_on_pose_jump = halt_on_pose_jump
        self.semantic_detector = semantic_detector
        self.semantic_yolo_config = semantic_yolo_config or SemanticYoloConfig()
        self.semantic_yolo_reinforce_map = bool(semantic_yolo_reinforce_map)
        segmenter_provenance = getattr(segmenter, "provenance", None)
        if (
            semantic_detector is not None
            and self.semantic_yolo_reinforce_map
            and isinstance(segmenter_provenance, dict)
            and segmenter_provenance.get("backend")
            == "source_rednet_detectron2_hm3d15"
        ):
            raise ValueError(
                "the executable-source pixel backend requires YOLO "
                "evidence-only mode"
            )
        self.semantic_yolo_frames_inferred = 0
        self.semantic_yolo_frames_with_detections = 0
        self.semantic_yolo_frames_with_evidence = 0
        self.semantic_yolo_failures = 0
        self.semantic_yolo_evidence_pixels = 0
        self.semantic_yolo_category_counts: dict[str, int] = {}
        self.last_semantic_yolo_sequence: int | None = None
        self.last_semantic_yolo_detections: list[dict[str, object]] = []
        self.last_semantic_yolo_evidence: list[dict[str, object]] = []
        self.last_semantic_yolo_error: str | None = None
        self.mapping_blocked_reason: str | None = None
        self.mapping_blocked_kind: str | None = None
        self.first_sequence: int | None = None
        self.last_sequence: int | None = None
        self.last_integrated_capture_time_ns: int | None = None
        self.last_observation_sequence: int | None = None
        self.semantic_vote_frames = 0
        self.semantic_interval_frames_without_vote = 0

    def _reset_ground_drift_confirmation(self) -> None:
        self.ground_drift_streak = 0
        self.ground_drift_streak_start_capture_time_ns = None
        self.last_ground_drift_duration_s = 0.0
        self.ground_drift_candidate_planes = []

    def _record_ground_rejection(self, capture_time_ns: int) -> None:
        """Track consecutive failed floor fits on the source clock.

        A cumulative rejection counter cannot distinguish one intermittent
        bad frame from a mapper which has stopped accepting geometry.  The
        input freezer uses this bounded consecutive interval to fail closed
        instead of pairing current RGB/VLM evidence with an indefinitely old
        BEV.
        """

        start_ns = self.ground_rejection_streak_start_capture_time_ns
        if (
            self.ground_rejection_streak == 0
            or start_ns is None
            or capture_time_ns < start_ns
        ):
            self.ground_rejection_streak = 1
            self.ground_rejection_streak_start_capture_time_ns = capture_time_ns
            self.last_ground_rejection_duration_s = 0.0
            return
        self.ground_rejection_streak += 1
        self.last_ground_rejection_duration_s = (
            capture_time_ns - start_ns
        ) / 1e9

    def _reset_ground_rejection(self) -> None:
        self.ground_rejection_streak = 0
        self.ground_rejection_streak_start_capture_time_ns = None
        self.last_ground_rejection_duration_s = 0.0

    def _record_ground_drift_motion(
        self,
        *,
        capture_time_ns: int,
        translation_m: float,
        rotation_deg: float,
    ) -> None:
        previous_ns = self.ground_drift_last_motion_capture_time_ns
        motion_window_ns = int(
            self.ground_drift_post_motion_rebase_window_s * 1e9
        )
        if (
            previous_ns is None
            or capture_time_ns < previous_ns
            or capture_time_ns - previous_ns > motion_window_ns
        ):
            self.ground_drift_motion_translation_m = 0.0
            self.ground_drift_motion_rotation_deg = 0.0
        self.ground_drift_motion_translation_m += translation_m
        self.ground_drift_motion_rotation_deg += rotation_deg
        self.ground_drift_last_motion_capture_time_ns = capture_time_ns

    def _post_motion_ground_rebase_allowed(
        self,
        *,
        capture_time_ns: int,
        tilt_delta_deg: float,
        height_delta_m: float,
    ) -> bool:
        last_motion_ns = self.ground_drift_last_motion_capture_time_ns
        if last_motion_ns is None or capture_time_ns < last_motion_ns:
            return False
        motion_age_s = (capture_time_ns - last_motion_ns) / 1e9
        motion_recent = (
            motion_age_s <= self.ground_drift_post_motion_rebase_window_s
        )
        motion_material = (
            self.ground_drift_motion_translation_m
            >= self.ground_drift_stationary_translation_m * 2.0
            or self.ground_drift_motion_rotation_deg
            >= self.ground_drift_stationary_rotation_deg * 2.0
        )
        bounded_tilt = (
            tilt_delta_deg
            <= (
                self.max_ground_tilt_delta_deg
                * POST_MOTION_GROUND_REBASE_GATE_MULTIPLIER
            )
        )
        # A shared-world Z translation cancels when this pipeline projects
        # every frame relative to its accepted local floor plane into a 2-D
        # map.  Keep the bounded-tilt, observed-motion, temporal-consistency,
        # and pose-jump gates, but do not reintroduce a 3-D height bound in the
        # post-motion rebase path when that 2-D policy is explicitly enabled.
        bounded_height = (
            self.allow_ground_height_translation_for_2d
            or height_delta_m
            <= (
                self.max_ground_height_delta_m
                * POST_MOTION_GROUND_REBASE_GATE_MULTIPLIER
            )
        )
        bounded_local_plane = bounded_tilt and bounded_height
        return motion_recent and motion_material and bounded_local_plane

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
        previous_robot_T = (
            None if self.last_robot_T is None else self.last_robot_T.copy()
        )
        self.last_camera_xy = (
            float(observation.T_shared_camera[0, 3]),
            float(observation.T_shared_camera[1, 3]),
        )
        self.last_camera_T = observation.T_shared_camera
        base_T_camera = getattr(observation.metadata, "base_T_camera", None)
        if base_T_camera is None:
            shared_T_robot = observation.T_shared_camera
            self.robot_pose_source = (
                "camera_pose_fallback_no_base_T_camera"
            )
        else:
            base_matrix = np.asarray(
                base_T_camera.matrix,
                dtype=np.float64,
            ).reshape(4, 4)
            shared_T_robot = (
                observation.T_shared_camera @ np.linalg.inv(base_matrix)
            )
            self.robot_pose_source = (
                "source_derived_shared_T_camera_times_inverse_"
                "observed_base_T_camera"
            )
        self.last_robot_T = shared_T_robot
        self.last_robot_xy = (
            float(shared_T_robot[0, 3]),
            float(shared_T_robot[1, 3]),
        )
        if previous_robot_T is None:
            ground_pose_translation_m = 0.0
            ground_pose_rotation_deg = 0.0
            ground_pose_moving = False
        else:
            (
                ground_pose_translation_m,
                ground_pose_rotation_deg,
            ) = pose_delta(previous_robot_T, shared_T_robot)
            ground_pose_moving = (
                ground_pose_translation_m
                > self.ground_drift_stationary_translation_m
                or ground_pose_rotation_deg
                > self.ground_drift_stationary_rotation_deg
            )
        self.last_ground_pose_translation_m = ground_pose_translation_m
        self.last_ground_pose_rotation_deg = ground_pose_rotation_deg
        self.last_ground_pose_moving = ground_pose_moving
        # The dashboard camera/pose remains current even when map integration
        # is latched.  Do not extend a trajectory inside a blocked coordinate
        # session: repeatedly appending those poses can draw a convincing but
        # false line across the discontinuity that caused the latch.
        self.last_rgb_bgr = observation.rgb_bgr
        # The executable source feeds the Perception VLM with the *current*
        # RGB plus YOLO detections.  Pixel-map integration is a separate,
        # geometry-sensitive operation.  In evidence-only mode, therefore,
        # keep Stage-1 perception current even when a wall-facing frame has no
        # trustworthy visible floor and must not be fused into the BEV.
        #
        # Map-reinforcement mode remains keyframe-only because its detections
        # mutate the pixel labels that are projected into the map below.
        if (
            self.semantic_detector is not None
            and not self.semantic_yolo_reinforce_map
        ):
            self.semantic_yolo_frames_inferred += 1
            self.last_semantic_yolo_sequence = observation.sequence
            try:
                detections = self.semantic_detector.detect_boxes(
                    observation.rgb_bgr
                )
                self.last_semantic_yolo_detections = [
                    {
                        "class_name": item.class_name,
                        "confidence": item.confidence,
                        "xyxy": list(item.xyxy),
                        "status": "model_inference_unverified",
                    }
                    for item in detections
                ]
                self.last_semantic_yolo_evidence = []
                self.last_semantic_yolo_error = None
                if detections:
                    self.semantic_yolo_frames_with_detections += 1
            except Exception as exc:
                # Preserve the exact source frame and an explicit failure
                # rather than silently falling back to an older RGB frame.
                self.semantic_yolo_failures += 1
                self.last_semantic_yolo_detections = []
                self.last_semantic_yolo_evidence = []
                self.last_semantic_yolo_error = (
                    f"{type(exc).__name__}: {exc}"
                )[:300]
        if self.mapping_blocked_reason is not None:
            # Keep the last continuous, accepted trajectory prefix intact.
            # ``last_*`` above deliberately follows the newest observation so
            # the dashboard can expose the discontinuity, but appending that
            # pose would draw a false connecting segment and replacing the
            # list would erase observed motion that happened before the
            # latch.  A fresh map session is the only authority that may
            # start a new continuous trajectory.
            self.skipped_non_keyframes += 1
            return KeyframeDecision(
                False,
                f"{self.mapping_blocked_kind or 'mapping_blocked'}_latched",
                0.0,
                0.0,
                0.0,
            )
        if self.keyframes is not None:
            discontinuity = self.keyframes.observe(
                observation.T_shared_camera
            )
            if discontinuity is not None:
                self.pose_jump_events += 1
                self.skipped_non_keyframes += 1
                if self.halt_on_pose_jump:
                    self.mapping_blocked_kind = "pose_jump"
                    self.mapping_blocked_reason = (
                        "pose discontinuity requires a fresh map session: "
                        f"sequence={observation.sequence}, "
                        f"translation_m={discontinuity.translation_m:.3f}, "
                        f"rotation_deg={discontinuity.rotation_deg:.2f}"
                    )
                return discontinuity
        if (
            not self.trajectory_xy_m
            or np.linalg.norm(
                np.asarray(self.last_camera_xy)
                - np.asarray(self.trajectory_xy_m[-1])
            )
            >= 0.05
        ):
            self.trajectory_xy_m.append(self.last_camera_xy)
            if len(self.trajectory_xy_m) > 2000:
                self.trajectory_xy_m = self.trajectory_xy_m[-2000:]
        if (
            not self.robot_trajectory_xy_m
            or np.linalg.norm(
                np.asarray(self.last_robot_xy)
                - np.asarray(self.robot_trajectory_xy_m[-1])
            )
            >= 0.05
        ):
            self.robot_trajectory_xy_m.append(self.last_robot_xy)
            if len(self.robot_trajectory_xy_m) > 2000:
                self.robot_trajectory_xy_m = (
                    self.robot_trajectory_xy_m[-2000:]
                )

        # Validate gravity/floor geometry before either the keyframe selector
        # commits this pose or RedNet spends GPU time. A frame with no
        # trustworthy visible floor is skipped. A persistent same-pose plane
        # change latches the session; a bounded local plane observed after
        # motion can become a new reference only after temporal and mutual
        # consistency checks.
        ground_candidate = None
        if self.ground_plane_config is not None:
            capture_time_ns = int(observation.metadata.capture_time_ns)
            if ground_pose_moving:
                # Post-motion floor rebasing must be justified by the
                # observed robot trajectory, not only by those moving frames
                # whose plane fit also happened to cross the drift gate.
                # A quadruped can walk with an in-gate floor estimate and
                # settle into a bounded, stable pitch/height offset only
                # after stopping. Recording all material pose motion lets
                # that stable local plane use the existing temporal,
                # consistency and bounded rebase checks below.
                self._record_ground_drift_motion(
                    capture_time_ns=capture_time_ns,
                    translation_m=ground_pose_translation_m,
                    rotation_deg=ground_pose_rotation_deg,
                )
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
                self._record_ground_rejection(capture_time_ns)
                # A missing/invalid plane breaks consecutiveness.  It gives
                # no evidence that drift persists, and the frame is already
                # excluded from both the pose gate and map integration.
                self._reset_ground_drift_confirmation()
                self.skipped_non_keyframes += 1
                return KeyframeDecision(
                    False,
                    f"ground_{ground_candidate.reason}",
                    0.0,
                    0.0,
                    0.0,
                )

            self._reset_ground_rejection()

            reference_plane = self.ground_reference_plane_coefficients
            camera_xy = observation.T_shared_camera[:2, 3]
            tilt_delta = plane_angle_deg(
                reference_plane,
                ground_candidate.plane_coefficients,
            )
            reference_height = plane_height_at(reference_plane, camera_xy)
            height_delta = abs(
                float(ground_candidate.ground_z_m) - reference_height
            )
            self.last_ground_tilt_delta_deg = tilt_delta
            self.last_ground_height_delta_m = height_delta
            tilt_outside_gate = tilt_delta > self.max_ground_tilt_delta_deg
            height_outside_gate = height_delta > self.max_ground_height_delta_m
            if (
                self.allow_ground_height_translation_for_2d
                and height_outside_gate
                and not tilt_outside_gate
            ):
                # This mapper stores only shared XY cells and classifies point
                # heights relative to this frame's accepted floor plane below.
                # A pure world-Z translation therefore cancels out and cannot
                # move an obstacle or semantic cell in the 2-D map.  This is
                # the observed Go2 tracking behavior: the fitted camera-to-floor
                # height stayed stable while both camera and floor Z translated.
                # Keep tilt and full pose-jump gates active; this exception is
                # deliberately invalid for a 3-D map.
                self.ground_height_translation_frames += 1
                self.max_ground_height_translation_m = max(
                    self.max_ground_height_translation_m,
                    height_delta,
                )
                self._reset_ground_drift_confirmation()
                self.last_ground_reason = "height_translation_tolerated_2d"
            elif tilt_outside_gate or height_outside_gate:
                # Do not integrate any outlying frame.  A single fit can be
                # transiently biased during a turn (RGB-D/pose timing, body
                # dynamics, or reduced visible floor).  Robot odometry is
                # planar and therefore cannot compensate camera pitch/roll
                # caused by quadruped gait.  Moving outliers are rejected but
                # cannot advance the irreversible latch; a true mount or
                # calibration change remains out of range after the robot
                # stops and then latches on the configured stationary run.
                self.ground_drift_frames += 1
                self.skipped_non_keyframes += 1
                if ground_pose_moving:
                    self.ground_drift_motion_deferred_frames += 1
                    self._reset_ground_drift_confirmation()
                    self.last_ground_reason = "drift_deferred_while_moving"
                    return KeyframeDecision(
                        False,
                        "ground_drift_motion_deferred",
                        ground_pose_translation_m,
                        ground_pose_rotation_deg,
                        0.0,
                    )
                candidate_plane = tuple(
                    float(value)
                    for value in ground_candidate.plane_coefficients
                )
                if self.ground_drift_candidate_planes:
                    consensus = tuple(
                        float(value)
                        for value in np.median(
                            np.asarray(self.ground_drift_candidate_planes),
                            axis=0,
                        )
                    )
                    candidate_spread_tilt_deg = plane_angle_deg(
                        consensus,
                        candidate_plane,
                    )
                    candidate_spread_height_m = abs(
                        plane_height_at(consensus, camera_xy)
                        - float(ground_candidate.ground_z_m)
                    )
                    if (
                        candidate_spread_tilt_deg
                        > self.max_ground_tilt_delta_deg * 0.5
                        or candidate_spread_height_m
                        > self.max_ground_height_delta_m * 0.5
                    ):
                        # Unrelated RANSAC modes cannot accumulate into
                        # persistent drift evidence.
                        self._reset_ground_drift_confirmation()
                if self.ground_drift_streak == 0:
                    self.ground_drift_streak_start_capture_time_ns = capture_time_ns
                self.ground_drift_candidate_planes.append(candidate_plane)
                self.ground_drift_streak += 1
                streak_start_ns = self.ground_drift_streak_start_capture_time_ns
                if streak_start_ns is None or capture_time_ns < streak_start_ns:
                    # A non-monotonic source timestamp cannot prove persistence.
                    self.ground_drift_streak = 1
                    self.ground_drift_streak_start_capture_time_ns = capture_time_ns
                    self.last_ground_drift_duration_s = 0.0
                    self.ground_drift_candidate_planes = [candidate_plane]
                else:
                    self.last_ground_drift_duration_s = (
                        capture_time_ns - streak_start_ns
                    ) / 1e9
                if (
                    self.ground_drift_streak
                    < self.ground_drift_consecutive_frames
                    or self.last_ground_drift_duration_s
                    < self.ground_drift_min_duration_s
                ):
                    self.last_ground_reason = "drift_pending"
                    return KeyframeDecision(
                        False,
                        "ground_drift_pending",
                        0.0,
                        0.0,
                        0.0,
                    )
                if self._post_motion_ground_rebase_allowed(
                    capture_time_ns=capture_time_ns,
                    tilt_delta_deg=tilt_delta,
                    height_delta_m=height_delta,
                ):
                    # The startup floor is a local observation, not a global
                    # calibration invariant. A quadruped can settle at a
                    # different stable pitch/height after walking, and a new
                    # floor patch can have a small real slope. Rebase only
                    # after observed motion plus a bounded, mutually
                    # consistent, time-confirmed plane. A same-pose change
                    # still takes the fail-closed latch below.
                    consensus = np.median(
                        np.asarray(self.ground_drift_candidate_planes),
                        axis=0,
                    )
                    self.ground_reference_plane_coefficients = tuple(
                        float(value) for value in consensus
                    )
                    self.ground_drift_reference_rebases += 1
                    self.last_ground_rebase_sequence = observation.sequence
                    self.last_ground_rebase_tilt_delta_deg = tilt_delta
                    self.last_ground_rebase_height_delta_m = height_delta
                    self.ground_drift_motion_translation_m = 0.0
                    self.ground_drift_motion_rotation_deg = 0.0
                    self.ground_drift_last_motion_capture_time_ns = None
                    self._reset_ground_drift_confirmation()
                    self.last_ground_reason = (
                        "post_motion_local_plane_rebased"
                    )
                else:
                    self.ground_drift_events += 1
                    self.last_ground_reason = "drift_latched"
                    self.mapping_blocked_kind = "ground_drift"
                    self.mapping_blocked_reason = (
                        "ground plane drift requires a fresh calibrated map session: "
                        f"sequence={observation.sequence}, "
                        f"consecutive_frames={self.ground_drift_streak}, "
                        f"duration_s={self.last_ground_drift_duration_s:.3f}, "
                        f"tilt_delta_deg={tilt_delta:.3f}, "
                        f"height_delta_m={height_delta:.3f}"
                    )
                    return KeyframeDecision(
                        False,
                        "ground_drift",
                        0.0,
                        0.0,
                        0.0,
                    )
            else:
                self._reset_ground_drift_confirmation()

        if self.keyframes is None:
            decision = KeyframeDecision(True, "unfiltered", 0.0, 0.0, 0.0)
        else:
            decision = self.keyframes.select_keyframe(
                observation.T_shared_camera, observation.metadata.capture_time_ns
            )
        if not decision.accept:
            self.skipped_non_keyframes += 1
            return decision

        pred = self.segmenter.segment(observation.rgb_bgr, observation.depth_m)
        if (
            self.semantic_detector is not None
            and self.semantic_yolo_reinforce_map
        ):
            self.semantic_yolo_frames_inferred += 1
            self.last_semantic_yolo_sequence = observation.sequence
            try:
                detections = self.semantic_detector.detect_boxes(observation.rgb_bgr)
                self.last_semantic_yolo_detections = [
                    {
                        "class_name": item.class_name,
                        "confidence": item.confidence,
                        "xyxy": list(item.xyxy),
                        "status": "model_inference_unverified",
                    }
                    for item in detections
                ]
                if detections:
                    self.semantic_yolo_frames_with_detections += 1
                pred, yolo_evidence = reinforce_rednet_prediction(
                    pred,
                    observation.depth_m,
                    detections,
                    self.semantic_yolo_config,
                )
                self.last_semantic_yolo_evidence = [
                    item.to_dict() for item in yolo_evidence
                ]
                self.last_semantic_yolo_error = None
                if yolo_evidence:
                    self.semantic_yolo_frames_with_evidence += 1
                    for item in yolo_evidence:
                        self.semantic_yolo_evidence_pixels += item.labelled_pixels
                        self.semantic_yolo_category_counts[item.map_category] = (
                            self.semantic_yolo_category_counts.get(item.map_category, 0) + 1
                        )
            except Exception as exc:  # keep RedNet/geometry live if detector fails
                self.semantic_yolo_failures += 1
                self.last_semantic_yolo_detections = []
                self.last_semantic_yolo_evidence = []
                self.last_semantic_yolo_error = f"{type(exc).__name__}: {exc}"[:300]
        semantic_vote_enabled = not (
            self.mapper.config.semantic_fusion_mode == "multi_view"
            and decision.reason == "interval"
        )
        if self.mapper.config.semantic_fusion_mode == "multi_view":
            if semantic_vote_enabled:
                self.semantic_vote_frames += 1
            else:
                self.semantic_interval_frames_without_vote += 1
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
            semantic_vote_enabled=semantic_vote_enabled,
        )
        if self.first_sequence is None:
            self.first_sequence = observation.sequence
        self.last_sequence = observation.sequence
        self.last_integrated_capture_time_ns = (
            observation.metadata.capture_time_ns
        )
        self.frames_processed += 1
        return decision

    def run(self, spool_dir: Path, robot_id: str) -> int:
        for observation in iter_spooled_observations(spool_dir, robot_id):
            self.process(observation)
        return self.frames_processed

    def semantic_yolo_status(self) -> dict[str, object]:
        provenance = None
        if self.semantic_detector is not None:
            provenance = getattr(self.semantic_detector, "provenance", None)
        return {
            "enabled": self.semantic_detector is not None,
            "method": (
                "yolov10_bbox_central_depth_cluster_to_pixel_label_projection"
                if self.semantic_yolo_reinforce_map
                else "yolov10_image_detections_for_perception_vlm_only"
            ),
            "status": (
                (
                    "model_inference_depth_projected_unverified"
                    if self.semantic_yolo_reinforce_map
                    else "model_inference_unverified_stage1_only"
                )
                if self.semantic_detector is not None
                else "disabled"
            ),
            "map_reinforcement_enabled": (
                self.semantic_detector is not None
                and self.semantic_yolo_reinforce_map
            ),
            "inference_policy": (
                "integrated_keyframes_only_for_map_reinforcement"
                if self.semantic_yolo_reinforce_map
                else "every_current_observation_for_stage1"
            ),
            "model_provenance": provenance,
            "config": {
                "minimum_confidence": self.semantic_yolo_config.minimum_confidence,
                "depth_anchor_quantile": (
                    self.semantic_yolo_config.depth_anchor_quantile
                ),
                "central_crop_fraction": (
                    self.semantic_yolo_config.central_crop_fraction
                ),
                "depth_tolerance_m": self.semantic_yolo_config.depth_tolerance_m,
                "minimum_valid_pixels": self.semantic_yolo_config.minimum_valid_pixels,
                "minimum_depth_m": self.semantic_yolo_config.minimum_depth_m,
                "maximum_depth_m": self.semantic_yolo_config.maximum_depth_m,
                "allowed_map_categories": list(
                    self.semantic_yolo_config.allowed_map_categories
                ),
            },
            "frames_inferred": self.semantic_yolo_frames_inferred,
            "frames_with_detections": self.semantic_yolo_frames_with_detections,
            "frames_with_evidence": self.semantic_yolo_frames_with_evidence,
            "failures": self.semantic_yolo_failures,
            "evidence_pixels_total": self.semantic_yolo_evidence_pixels,
            "category_detection_counts": dict(
                sorted(self.semantic_yolo_category_counts.items())
            ),
            "last_sequence": self.last_semantic_yolo_sequence,
            "last_detections": self.last_semantic_yolo_detections,
            "last_evidence": self.last_semantic_yolo_evidence,
            "last_error": self.last_semantic_yolo_error,
        }

    def semantic_backend_status(self) -> dict[str, object]:
        provenance = getattr(self.segmenter, "provenance", None)
        if isinstance(provenance, dict):
            return dict(provenance)
        return {
            "backend": "rednet_mp3d40",
            "status": "source_derived_model_inference_unverified",
            "method": "mp3d40_rednet_rgbd_confidence_0.8",
            "source_maskrcnn_override_applied": False,
            "source_compatibility": "rednet_backbone_only",
        }

    def save(self, out_dir: Path) -> None:
        if not self.transform_version:
            raise ValueError("cannot save a map before binding a transform_version")
        out_dir.mkdir(parents=True, exist_ok=True)
        snapshot_id = (
            f"{self.robot_id or 'unknown'}:"
            f"{self.last_observation_sequence}:{time.time_ns()}"
        )
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
            ground_reference_plane_coefficients=np.asarray(
                self.ground_reference_plane_coefficients,
                dtype=np.float64,
            ),
            ground_drift_reference_rebases=np.asarray(
                self.ground_drift_reference_rebases,
                dtype=np.int64,
            ),
            floor_source=np.asarray(self.floor_source),
            resolution_m=np.array(self.mapper.config.resolution_m),
            frame_id=np.asarray(self.frame_id),
            transform_version=np.asarray(self.transform_version or ""),
            shared_frame_calibration_id=np.asarray(
                self.shared_frame_calibration_id or ""
            ),
            map_format_version=np.asarray("focus-hub-central-map-v3"),
            snapshot_id=np.asarray(snapshot_id),
            obstacle_fusion_mode=np.asarray(self.mapper.config.obstacle_fusion_mode),
            obstacle_band_m=np.asarray(
                [
                    self.mapper.config.obstacle_band_low_m,
                    self.mapper.config.obstacle_band_high_m,
                ],
                dtype=np.float64,
            ),
            obstacle_min_hits=np.asarray(self.mapper.config.obstacle_min_hits),
            semantic_fusion_mode=np.asarray(
                self.mapper.config.semantic_fusion_mode
            ),
            semantic_min_hits=np.asarray(self.mapper.config.semantic_min_hits),
            semantic_winner_margin_hits=np.asarray(
                self.mapper.config.semantic_winner_margin_hits
            ),
            semantic_vote_policy=np.asarray(
                (
                    "pose_change_keyframes_only_interval_geometry_refresh"
                    if self.mapper.config.semantic_fusion_mode == "multi_view"
                    else "every_integrated_keyframe"
                )
            ),
            semantic_fusion=np.asarray(
                f"{self.semantic_backend_status()['backend']}+yolov10_bbox_depth"
                if (
                    self.semantic_detector is not None
                    and self.semantic_yolo_reinforce_map
                )
                else str(self.semantic_backend_status()["backend"])
            ),
            semantic_yolo_model_sha256=np.asarray(
                ""
                if self.semantic_detector is None
                else str(
                    getattr(self.semantic_detector, "weights_sha256", "")
                )
            ),
            last_map_sequence=np.asarray(
                -1 if self.last_sequence is None else self.last_sequence
            ),
            last_map_capture_time_ns=np.asarray(
                -1
                if self.last_integrated_capture_time_ns is None
                else self.last_integrated_capture_time_ns
            ),
            robot_trajectory_xy_m=np.asarray(
                self.robot_trajectory_xy_m,
                dtype=np.float64,
            ).reshape((-1, 2)),
            robot_trajectory_last_observation_sequence=np.asarray(
                -1
                if self.last_observation_sequence is None
                else self.last_observation_sequence,
                dtype=np.int64,
            ),
            robot_trajectory_pose_source=np.asarray(
                self.robot_pose_source
            ),
        )
        os.replace(tmp_path, out_dir / "central_map.npz")
        summary = {
            "robot_id": self.robot_id,
            "source_kind": "focus_hub_incremental_rgbd",
            "source_status": "observed_spooled_observations",
            "semantic_status": "model_inference_unverified",
            "map_format_version": "focus-hub-central-map-v3",
            "snapshot_id": snapshot_id,
            "frames_processed": self.frames_processed,
            "observations_seen": self.observations_seen,
            "skipped_non_keyframes": self.skipped_non_keyframes,
            "pose_jump_events": self.pose_jump_events,
            "ground_rejected_frames": self.ground_rejected_frames,
            "ground_rejection_streak": self.ground_rejection_streak,
            "ground_rejection_duration_s": (
                self.last_ground_rejection_duration_s
            ),
            "ground_drift_frames": self.ground_drift_frames,
            "ground_drift_events": self.ground_drift_events,
            "ground_drift_streak": self.ground_drift_streak,
            "ground_drift_duration_s": self.last_ground_drift_duration_s,
            "ground_drift_motion_deferred_frames": (
                self.ground_drift_motion_deferred_frames
            ),
            "ground_drift_reference_rebases": (
                self.ground_drift_reference_rebases
            ),
            "ground_height_translation_frames": (
                self.ground_height_translation_frames
            ),
            "max_ground_height_translation_m": (
                self.max_ground_height_translation_m
            ),
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
                "minimum_duration_s_to_latch": self.ground_drift_min_duration_s,
                "stationary_translation_threshold_m": (
                    self.ground_drift_stationary_translation_m
                ),
                "stationary_rotation_threshold_deg": (
                    self.ground_drift_stationary_rotation_deg
                ),
                "post_motion_rebase_policy": {
                    "motion_window_s": (
                        self.ground_drift_post_motion_rebase_window_s
                    ),
                    "minimum_translation_m": (
                        self.ground_drift_stationary_translation_m * 2.0
                    ),
                    "minimum_rotation_deg": (
                        self.ground_drift_stationary_rotation_deg * 2.0
                    ),
                    "maximum_tilt_delta_deg": (
                        self.max_ground_tilt_delta_deg
                        * POST_MOTION_GROUND_REBASE_GATE_MULTIPLIER
                    ),
                    "maximum_height_delta_m": (
                        self.max_ground_height_delta_m
                        * POST_MOTION_GROUND_REBASE_GATE_MULTIPLIER
                    ),
                    "plane_consistency_tolerance_deg": (
                        self.max_ground_tilt_delta_deg * 0.5
                    ),
                    "height_consistency_tolerance_m": (
                        self.max_ground_height_delta_m * 0.5
                    ),
                },
                "current_reference_plane_coefficients": list(
                    self.ground_reference_plane_coefficients
                ),
                "reference_rebases": self.ground_drift_reference_rebases,
                "motion_evidence_translation_m": (
                    self.ground_drift_motion_translation_m
                ),
                "motion_evidence_rotation_deg": (
                    self.ground_drift_motion_rotation_deg
                ),
                "last_motion_capture_time_ns": (
                    self.ground_drift_last_motion_capture_time_ns
                ),
                "last_rebase_sequence": self.last_ground_rebase_sequence,
                "last_rebase_tilt_delta_deg": (
                    self.last_ground_rebase_tilt_delta_deg
                ),
                "last_rebase_height_delta_m": (
                    self.last_ground_rebase_height_delta_m
                ),
                "height_translation_policy": (
                    "tolerate_for_2d_with_frame_local_floor_plane"
                    if self.allow_ground_height_translation_for_2d
                    else "latch_after_consecutive_frames"
                ),
                "last_sequence": self.last_ground_sequence,
                "last_reason": self.last_ground_reason,
                "rejection_streak": self.ground_rejection_streak,
                "rejection_duration_s": (
                    self.last_ground_rejection_duration_s
                ),
                "last_tilt_delta_deg": self.last_ground_tilt_delta_deg,
                "last_height_delta_m": self.last_ground_height_delta_m,
                "last_drift_duration_s": self.last_ground_drift_duration_s,
                "last_pose_translation_m": (
                    self.last_ground_pose_translation_m
                ),
                "last_pose_rotation_deg": self.last_ground_pose_rotation_deg,
                "last_pose_moving": self.last_ground_pose_moving,
            },
            "obstacle_fusion_mode": self.mapper.config.obstacle_fusion_mode,
            "obstacle_band_m": [
                self.mapper.config.obstacle_band_low_m,
                self.mapper.config.obstacle_band_high_m,
            ],
            "obstacle_min_hits": self.mapper.config.obstacle_min_hits,
            "semantic_fusion_mode": self.mapper.config.semantic_fusion_mode,
            "semantic_min_hits": self.mapper.config.semantic_min_hits,
            "semantic_winner_margin_hits": (
                self.mapper.config.semantic_winner_margin_hits
            ),
            "semantic_vote_policy": (
                "pose_change_keyframes_only_interval_geometry_refresh"
                if self.mapper.config.semantic_fusion_mode == "multi_view"
                else "every_integrated_keyframe"
            ),
            "semantic_vote_frames": self.semantic_vote_frames,
            "semantic_interval_frames_without_vote": (
                self.semantic_interval_frames_without_vote
            ),
            "robot_trajectory_snapshot": {
                "container": "central_map.npz",
                "field": "robot_trajectory_xy_m",
                "point_count": len(self.robot_trajectory_xy_m),
                "status": (
                    "frozen_last_continuous_prefix"
                    if self.mapping_blocked_reason is not None
                    else "live_continuous_session"
                ),
                "last_observation_sequence": (
                    self.last_observation_sequence
                ),
                "pose_source": self.robot_pose_source,
                "classification": (
                    "observed continuous base-trajectory prefix in the same "
                    "atomic map snapshot generation; a discontinuous current "
                    "pose is never appended or allowed to erase that prefix"
                ),
            },
            "semantic_mapping": {
                "rednet": {
                    "enabled": (
                        self.semantic_backend_status().get("backend")
                        in {
                            "rednet_mp3d40",
                            "source_rednet_detectron2_hm3d15",
                        }
                    ),
                    "method": "mp3d40_rednet_depth_projection",
                    "status": (
                        "source_exact_model_inference_unverified"
                        if self.semantic_backend_status().get("backend")
                        == "source_rednet_detectron2_hm3d15"
                        else "source_derived_model_inference_unverified"
                    ),
                },
                "pixel_segmenter": self.semantic_backend_status(),
                "yolo_reinforcement": self.semantic_yolo_status(),
            },
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "last_map_capture_time_ns": (
                self.last_integrated_capture_time_ns
            ),
            "last_observation_sequence": self.last_observation_sequence,
            "obstacle_cells": int((self.mapper.map.grid[0] > 0.5).sum()),
            "explored_cells": int((self.mapper.map.grid[1] > 0.5).sum()),
            "semantic_cells": int(
                np.any(self.mapper.map.grid[2:] > 0.1, axis=0).sum()
            ),
        }
        summary_tmp = out_dir / "map_summary.json.tmp"
        summary_tmp.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(summary_tmp, out_dir / "map_summary.json")
