"""Reader for TinyNav ``map_record`` directories extracted for hub replay.

A replay sample is the pair of
  * the original record directory (read-only evidence copied from the robot):
    ``poses.npy``, ``intrinsics.npy``, ``rgb_camera_intrinsics.npy``,
    ``T_rgb_to_infra1.npy``, ``rgb_images_db/{video.mp4,meta.json}``
  * an ``*_extracted`` directory produced by ``hub/tools/extract_tinynav_record.py``
    holding one raw pickle per keyframe depth plus ``manifest.json`` hashes.

Conventions (verified against TinyNav source on the robot, see
``hub/docs/ROBOT_WSJ_AUDIT.md`` and audit notes):
  * ``poses[ts]`` is camera-to-world for the infra1 optical frame
    (``p_world = pose @ p_infra1``); the TinyNav world is gravity aligned
    with +z up.
  * depth images are float32 metres in the rectified infra1 frame with
    intrinsics ``K_infra1``; invalid pixels are exactly 0.
  * ``T_rgb_to_infra1`` maps RGB-optical points into the infra1 optical frame
    (``p_infra1 = T @ p_rgb``), hence
    ``T_world_rgb = poses[ts] @ T_rgb_to_infra1``.
"""
from __future__ import annotations

import json
import pickle
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class ReplayFrame:
    timestamp_ns: int
    index: int
    rgb_bgr: np.ndarray            # (H, W, 3) uint8
    depth_m: np.ndarray            # (H, W) float32, metres, 0 = invalid
    T_world_infra1: np.ndarray     # (4, 4) float64 camera-to-world
    T_world_rgb: np.ndarray        # (4, 4) float64 camera-to-world


@dataclass(frozen=True)
class ReplayCalibration:
    K_infra1: np.ndarray           # (3, 3) rectified stereo intrinsics (depth frame)
    K_rgb: np.ndarray              # (3, 3) color intrinsics
    T_rgb_to_infra1: np.ndarray    # (4, 4) p_infra1 = T @ p_rgb
    baseline_m: float


class TinyNavReplayReader:
    """Iterates keyframes of one extracted TinyNav record in timestamp order."""

    def __init__(self, record_dir: Path | str, extracted_dir: Path | str) -> None:
        self.record_dir = Path(record_dir)
        self.extracted_dir = Path(extracted_dir)
        manifest_path = self.extracted_dir / "manifest.json"
        with manifest_path.open("r", encoding="utf-8") as f:
            self.manifest = json.load(f)

        self.poses: dict[int, np.ndarray] = {
            int(k): np.asarray(v, dtype=np.float64)
            for k, v in np.load(self.record_dir / "poses.npy", allow_pickle=True).item().items()
        }
        with (self.record_dir / "rgb_images_db" / "meta.json").open("r", encoding="utf-8") as f:
            ts_to_idx = {int(k): int(v) for k, v in json.load(f)["ts_to_idx"].items()}

        depth_keys = [int(k) for k in self.manifest["depth_keys"]]
        pose_keys = set(self.poses)
        rgb_keys = set(ts_to_idx)
        if set(depth_keys) != pose_keys or set(depth_keys) != rgb_keys:
            raise ValueError(
                "record keyframe sets disagree: "
                f"{len(depth_keys)} depths, {len(pose_keys)} poses, {len(rgb_keys)} rgb frames"
            )
        self.timestamps: list[int] = sorted(depth_keys)
        indices = [ts_to_idx[t] for t in self.timestamps]
        if indices != sorted(indices):
            raise ValueError("rgb frame indices are not monotonic in timestamp order")
        self._ts_to_idx = ts_to_idx

        self.calibration = ReplayCalibration(
            K_infra1=np.load(self.record_dir / "intrinsics.npy").astype(np.float64),
            K_rgb=np.asarray(
                np.load(self.record_dir / "rgb_camera_intrinsics.npy", allow_pickle=True),
                dtype=np.float64,
            ),
            T_rgb_to_infra1=np.load(self.record_dir / "T_rgb_to_infra1.npy").astype(np.float64),
            baseline_m=float(np.load(self.record_dir / "baseline.npy")),
        )

    def __len__(self) -> int:
        return len(self.timestamps)

    def frames(self):
        """Yield ReplayFrame in timestamp order, decoding the video sequentially."""
        capture = cv2.VideoCapture(str(self.record_dir / "rgb_images_db" / "video.mp4"))
        if not capture.isOpened():
            raise RuntimeError("failed to open rgb_images_db/video.mp4")
        try:
            decoded = 0
            for timestamp in self.timestamps:
                target = self._ts_to_idx[timestamp]
                frame = None
                while decoded <= target:
                    ok, frame = capture.read()
                    if not ok:
                        raise RuntimeError(
                            f"video ended at frame {decoded} before index {target}"
                        )
                    decoded += 1
                depth_path = self.extracted_dir / "depths_pkl" / f"{timestamp}.pkl"
                with depth_path.open("rb") as f:
                    depth = pickle.loads(f.read())
                depth = np.asarray(depth, dtype=np.float32)
                pose = self.poses[timestamp]
                yield ReplayFrame(
                    timestamp_ns=timestamp,
                    index=target,
                    rgb_bgr=frame,
                    depth_m=depth,
                    T_world_infra1=pose,
                    T_world_rgb=pose @ self.calibration.T_rgb_to_infra1,
                )
        finally:
            capture.release()
