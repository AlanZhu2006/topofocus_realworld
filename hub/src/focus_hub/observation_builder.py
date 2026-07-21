"""Builds wire observations (metadata + encoded payloads) from replay frames.

Shared by the replay sender tool and the end-to-end runner so both emit the
identical contract the future ROS sender must implement.
"""
from __future__ import annotations

import hashlib
import time

import cv2

from .depth_align import align_depth_to_rgb, encode_depth_png16
from .models import ObservationMetadata
from .tinynav_replay import TinyNavReplayReader

DEPTH_SCALE_M = 0.001
CAMERA_FRAME = "camera_color_optical_frame"

# Placeholder Go2 mount used ONLY by the explicit command-capable test lane:
# optical frame looking along base +x from 0.35 m up, 0.2 m forward.  This is
# not a measured calibration and must never be used for real motion.
PLACEHOLDER_BASE_T_CAMERA = (
    0.0, 0.0, 1.0, 0.2,
    -1.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.35,
    0.0, 0.0, 0.0, 1.0,
)


def encode_frame(frame, reader: TinyNavReplayReader) -> tuple[bytes, bytes]:
    """Return (JPEG RGB, PNG16 depth-aligned-to-RGB) wire payloads."""
    ok, jpeg = cv2.imencode(".jpg", frame.rgb_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    aligned = align_depth_to_rgb(
        frame.depth_m,
        reader.calibration.K_infra1,
        reader.calibration.K_rgb,
        reader.calibration.T_rgb_to_infra1,
        frame.rgb_bgr.shape[:2],
    )
    return jpeg.tobytes(), encode_depth_png16(aligned, DEPTH_SCALE_M)


def build_metadata(
    *,
    robot_id: str,
    sequence: int,
    frame,
    reader: TinyNavReplayReader,
    rgb_bytes: bytes,
    depth_bytes: bytes,
    transform_version: str,
    mapping_only: bool,
    base_T_camera: tuple[float, ...] | None,
    health_ready: bool,
    goal_category: str,
) -> ObservationMetadata:
    now_ns = time.time_ns()
    K = reader.calibration.K_rgb
    h, w = frame.rgb_bgr.shape[:2]
    return ObservationMetadata.model_validate(
        {
            "robot_id": robot_id,
            "sequence": sequence,
            "capture_time_ns": now_ns - 50_000_000,
            "sent_time_ns": now_ns,
            "pose": {
                "shared_T_camera": {
                    "parent_frame": "shared_world",
                    "child_frame": CAMERA_FRAME,
                    "matrix": tuple(frame.T_world_rgb.reshape(-1).tolist()),
                },
                "covariance_6x6": (0.0,) * 36,
                "transform_version": transform_version,
            },
            "base_T_camera": None if base_T_camera is None else {
                "parent_frame": "base_link",
                "child_frame": CAMERA_FRAME,
                "matrix": base_T_camera,
            },
            "intrinsics": {
                "width": w,
                "height": h,
                "fx": float(K[0, 0]),
                "fy": float(K[1, 1]),
                "cx": float(K[0, 2]),
                "cy": float(K[1, 2]),
                "distortion_model": "none",
                "distortion": [],
            },
            "depth_scale_m": DEPTH_SCALE_M,
            "depth_min_m": 0.3,
            "depth_max_m": 5.0,
            "rgb_encoding": "jpeg",
            "depth_encoding": "png16",
            "rgb_size_bytes": len(rgb_bytes),
            "depth_size_bytes": len(depth_bytes),
            "rgb_sha256": hashlib.sha256(rgb_bytes).hexdigest(),
            "depth_sha256": hashlib.sha256(depth_bytes).hexdigest(),
            "object_goal": {"goal_id": "e2e-goal-1", "category": goal_category},
            "health": {
                "safety_state": "READY" if health_ready else "UNKNOWN",
                "localization_state": "TRACKING" if health_ready else "UNKNOWN",
                "estop_engaged": False,
                "collision_avoidance_ready": health_ready,
                "motor_controller_ready": health_ready,
            },
            "mapping_only": mapping_only,
        }
    )
