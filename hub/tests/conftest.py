from __future__ import annotations

import hashlib
import time

import pytest

from focus_hub.models import ObservationMetadata


IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
ZERO_COVARIANCE = (0.0,) * 36


@pytest.fixture
def observation_factory():
    def make(
        *,
        robot_id: str = "robot-0",
        sequence: int = 0,
        now_ns: int | None = None,
        mapping_only: bool = True,
        health_ready: bool = False,
    ) -> ObservationMetadata:
        now_ns = time.time_ns() if now_ns is None else now_ns
        rgb = b"rgb"
        depth = b"depth"
        return ObservationMetadata.model_validate(
            {
                "robot_id": robot_id,
                "sequence": sequence,
                "capture_time_ns": now_ns - 100_000_000,
                "sent_time_ns": now_ns - 10_000_000,
                "pose": {
                    "shared_T_camera": {
                        "parent_frame": "shared_world",
                        "child_frame": "camera_color_optical_frame",
                        "matrix": IDENTITY,
                    },
                    "covariance_6x6": ZERO_COVARIANCE,
                    "transform_version": "calib-test-v1",
                },
                "base_T_camera": None if mapping_only else {
                    "parent_frame": "base_link",
                    "child_frame": "camera_color_optical_frame",
                    "matrix": IDENTITY,
                },
                "intrinsics": {
                    "width": 848,
                    "height": 480,
                    "fx": 420.0,
                    "fy": 420.0,
                    "cx": 424.0,
                    "cy": 240.0,
                    "distortion_model": "none",
                    "distortion": [],
                },
                "depth_scale_m": 0.001,
                "depth_min_m": 0.1,
                "depth_max_m": 10.0,
                "rgb_encoding": "jpeg",
                "depth_encoding": "png16",
                "rgb_size_bytes": len(rgb),
                "depth_size_bytes": len(depth),
                "rgb_sha256": hashlib.sha256(rgb).hexdigest(),
                "depth_sha256": hashlib.sha256(depth).hexdigest(),
                "object_goal": {"goal_id": "goal-1", "category": "chair"},
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

    return make

