"""Depth reprojection from the rectified infra1 frame into the RGB frame.

The production robot publishes ``/camera/camera/aligned_depth_to_color`` — depth
resampled into the color camera frame.  TinyNav map-records instead store the
raw rectified infra1 depth, so the replay sender performs the same alignment
the RealSense driver would: lift every valid depth pixel to 3-D, transform it
into the RGB optical frame, project with the RGB intrinsics and keep the
nearest hit per output pixel (z-buffer).

Also provides the lossless 16-bit PNG wire encoding used by the observation
contract (`depth_encoding: png16`, metres = value * depth_scale_m).
"""
from __future__ import annotations

import cv2
import numpy as np


def align_depth_to_rgb(
    depth_infra1_m: np.ndarray,
    K_infra1: np.ndarray,
    K_rgb: np.ndarray,
    T_rgb_to_infra1: np.ndarray,
    rgb_shape: tuple[int, int],
) -> np.ndarray:
    """Return depth in metres resampled onto the RGB pixel grid (0 = no data).

    ``T_rgb_to_infra1`` maps RGB-optical points into the infra1 optical frame
    (the TinyNav record convention), so points go the other way here.
    """
    h_ir, w_ir = depth_infra1_m.shape
    valid = depth_infra1_m > 0
    if not np.any(valid):
        return np.zeros(rgb_shape, dtype=np.float32)

    vs, us = np.nonzero(valid)
    z = depth_infra1_m[vs, us].astype(np.float64)
    fx, fy = K_infra1[0, 0], K_infra1[1, 1]
    cx, cy = K_infra1[0, 2], K_infra1[1, 2]
    points = np.stack(((us - cx) / fx * z, (vs - cy) / fy * z, z), axis=-1)

    T_infra1_to_rgb = np.linalg.inv(T_rgb_to_infra1)
    p_rgb = points @ T_infra1_to_rgb[:3, :3].T + T_infra1_to_rgb[:3, 3]
    in_front = p_rgb[:, 2] > 1e-6
    p_rgb = p_rgb[in_front]

    h_rgb, w_rgb = rgb_shape
    u = np.round(K_rgb[0, 0] * p_rgb[:, 0] / p_rgb[:, 2] + K_rgb[0, 2]).astype(np.int64)
    v = np.round(K_rgb[1, 1] * p_rgb[:, 1] / p_rgb[:, 2] + K_rgb[1, 2]).astype(np.int64)
    in_image = (u >= 0) & (u < w_rgb) & (v >= 0) & (v < h_rgb)
    u, v = u[in_image], v[in_image]
    z_rgb = p_rgb[in_image, 2]

    aligned = np.full(h_rgb * w_rgb, np.inf, dtype=np.float64)
    np.minimum.at(aligned, v * w_rgb + u, z_rgb)
    aligned[~np.isfinite(aligned)] = 0.0
    return aligned.reshape(h_rgb, w_rgb).astype(np.float32)


def encode_depth_png16(depth_m: np.ndarray, depth_scale_m: float = 0.001) -> bytes:
    """Encode metres to the png16 wire format (uint16 counts of depth_scale_m)."""
    counts = np.round(depth_m.astype(np.float64) / depth_scale_m)
    counts = np.clip(counts, 0, np.iinfo(np.uint16).max).astype(np.uint16)
    ok, buffer = cv2.imencode(".png", counts)
    if not ok:
        raise RuntimeError("PNG16 depth encoding failed")
    return buffer.tobytes()


def decode_depth_png16(payload: bytes, depth_scale_m: float) -> np.ndarray:
    """Decode the png16 wire format back to metres (float32, 0 = no data)."""
    counts = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if counts is None or counts.dtype != np.uint16:
        raise ValueError("payload is not a 16-bit PNG depth image")
    return (counts.astype(np.float32)) * np.float32(depth_scale_m)
