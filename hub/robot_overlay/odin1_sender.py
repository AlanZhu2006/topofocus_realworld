#!/usr/bin/env python3
"""Read-only Odin1 RGB/SLAM-cloud adapter for the Yunji Hub lane.

The Odin1 driver does not publish a RealSense-style aligned depth image in its
deployed SLAM mode.  It publishes a distorted RGB image, a colored point cloud
already expressed in ``odom``, and ``T_odom_imu`` odometry.  This sender keeps
the existing Hub RGB-D transport contract by:

1. rectifying RGB with the device's FishPoly factory calibration;
2. composing the factory ``T_imu_camera`` with live Odin odometry;
3. transforming ``cloud_slam`` back into the camera frame;
4. z-buffering it into the same rectified pinhole image.

Odin header stamps are device-boot time, not Unix time.  They are used only to
synchronize the three local ROS messages.  ``capture_time_ns`` is the
NTP-synchronized host receipt time recorded for the RGB message.  The source
remains mapping-only and never calls a WATER motion endpoint.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import cv2
import numpy as np

HUB_SRC = Path(__file__).resolve().parents[1] / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.base_camera_calibration import load_base_camera_calibration

from yunji_sender import (
    DEPTH_SCALE_M,
    HeartbeatThread,
    HubTransport,
    LatestLocalizationState,
    WaterTcpClient,
    build_metadata,
    classify_localization_state,
)


# The deployed driver emits matching image/cloud/odometry device stamps.  A
# 100 ms period is one complete Odin frame at its observed ~10 Hz rate, so a
# looser gate can accidentally combine adjacent frames while the robot moves.
DEFAULT_SYNC_SLOP_S = 0.02


def _rigid_matrix(values: Any, *, label: str) -> np.ndarray:
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16:
        raise ValueError(f"{label} must contain 16 values")
    matrix = matrix.reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{label} must contain finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{label} must be homogeneous")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5):
        raise ValueError(f"{label} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-5):
        raise ValueError(f"{label} rotation must have determinant +1")
    return matrix


@dataclass(frozen=True)
class OdinCalibration:
    serial: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    skew: float
    distortion: tuple[float, float, float, float, float, float]
    T_imu_camera: np.ndarray
    camera_frame: str
    odometry_frame: str


def load_odin_calibration(path: str, expected_serial: str | None = None) -> OdinCalibration:
    with open(path, encoding="utf-8") as handle:
        artifact = json.load(handle)
    if artifact.get("sensor_model") != "odin1":
        raise ValueError("Odin calibration sensor_model must be 'odin1'")
    serial = str(artifact.get("sensor_serial", ""))
    if expected_serial and serial != expected_serial:
        raise ValueError(
            f"Odin serial mismatch: expected {expected_serial!r}, artifact declares {serial!r}"
        )
    camera = artifact["camera"]
    distortion = camera["distortion"]
    result = OdinCalibration(
        serial=serial,
        width=int(camera["image_width"]),
        height=int(camera["image_height"]),
        fx=float(camera["fx"]),
        fy=float(camera["fy"]),
        cx=float(camera["cx"]),
        cy=float(camera["cy"]),
        skew=float(camera["skew"]),
        distortion=tuple(
            float(distortion[name]) for name in ("k2", "k3", "k4", "k5", "k6", "k7")
        ),
        T_imu_camera=_rigid_matrix(
            artifact["imu_from_camera"]["matrix"], label="imu_from_camera"
        ),
        camera_frame=str(artifact["camera_frame"]),
        odometry_frame=str(artifact["odometry_frame"]),
    )
    if result.width <= 0 or result.height <= 0:
        raise ValueError("Odin image dimensions must be positive")
    if min(result.fx, result.fy) <= 0.0:
        raise ValueError("Odin focal lengths must be positive")
    return result


def load_shared_transform(
    path: str | None, *, expected_transform_version: str
) -> tuple[np.ndarray | None, str | None]:
    """Load only a calibration explicitly generated for this Odin session.

    Refusing a D435/D455 transform is essential: Odin owns a new odometry frame
    and its camera/IMU extrinsic is unrelated to the removed RealSense mount.
    """
    if not path:
        return None, None
    with open(path, encoding="utf-8") as handle:
        artifact = json.load(handle)
    actual_version = artifact.get("transform_version")
    if actual_version != expected_transform_version:
        raise ValueError(
            "shared-frame transform version mismatch: "
            f"sender={expected_transform_version!r}, artifact={actual_version!r}"
        )
    matrix = _rigid_matrix(
        artifact["shared_world_from_other_odom"]["matrix"],
        label="shared_world_from_other_odom",
    )
    calibration_id = artifact.get("shared_frame_calibration_id")
    if not isinstance(calibration_id, str) or not calibration_id:
        raise ValueError("shared-frame artifact has no calibration ID")
    return matrix, calibration_id


def quaternion_matrix(position: Any, orientation: Any) -> np.ndarray:
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("Odin odometry quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [float(position.x), float(position.y), float(position.z)]
    return matrix


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def decode_bgr8(message: Any) -> np.ndarray:
    if message.encoding.lower() not in {"bgr8", "rgb8"}:
        raise ValueError(f"unsupported Odin image encoding {message.encoding!r}")
    row_bytes = int(message.width) * 3
    if int(message.step) < row_bytes:
        raise ValueError("Odin image step is smaller than width*3")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.step) * int(message.height)
    if raw.size < expected:
        raise ValueError("truncated Odin image payload")
    image = raw[:expected].reshape(int(message.height), int(message.step))
    image = image[:, :row_bytes].reshape(int(message.height), int(message.width), 3)
    if message.encoding.lower() == "rgb8":
        image = image[..., ::-1]
    return np.ascontiguousarray(image)


def decode_cloud_xyz_bgr(message: Any) -> tuple[np.ndarray, np.ndarray | None]:
    fields = {field.name: field for field in message.fields}
    missing = {"x", "y", "z"} - fields.keys()
    if missing:
        raise ValueError(f"Odin cloud is missing fields: {sorted(missing)}")
    endian = ">" if message.is_bigendian else "<"
    names = ["x", "y", "z"]
    formats: list[str] = [endian + "f4", endian + "f4", endian + "f4"]
    offsets = [fields[name].offset for name in names]
    if "rgb" in fields:
        names.append("rgb")
        formats.append(endian + "f4")
        offsets.append(fields["rgb"].offset)
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": message.point_step}
    )
    rows = []
    payload = memoryview(message.data)
    for row in range(int(message.height)):
        offset = row * int(message.row_step)
        rows.append(
            np.ndarray(
                shape=(int(message.width),), dtype=dtype, buffer=payload, offset=offset
            )
        )
    records = rows[0] if len(rows) == 1 else np.concatenate(rows)
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64, copy=False
    )
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    bgr = None
    if "rgb" in records.dtype.names:
        packed = records["rgb"].view(endian + "u4")[finite]
        bgr = np.column_stack(
            (packed & 255, (packed >> 8) & 255, (packed >> 16) & 255)
        ).astype(np.uint8)
    return xyz, bgr


class OdinProjector:
    def __init__(
        self,
        calibration: OdinCalibration,
        *,
        output_width: int = 800,
        splat_radius: int = 1,
        depth_min_m: float = 0.2,
        depth_max_m: float = 8.0,
    ) -> None:
        if output_width <= 0 or output_width > calibration.width:
            raise ValueError("output_width must be within the calibrated image width")
        if splat_radius < 0 or splat_radius > 3:
            raise ValueError("splat_radius must be between 0 and 3")
        if not 0.0 < depth_min_m < depth_max_m:
            raise ValueError("invalid Odin depth range")
        self.calibration = calibration
        self.scale = output_width / calibration.width
        self.width = output_width
        self.height = int(round(calibration.height * self.scale))
        self.fx = calibration.fx * self.scale
        self.fy = calibration.fy * self.scale
        self.cx = calibration.cx * self.scale
        self.cy = calibration.cy * self.scale
        self.splat_radius = splat_radius
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m
        self.map_x, self.map_y = self._build_undistort_map()

    def _build_undistort_map(self) -> tuple[np.ndarray, np.ndarray]:
        # The output is deliberately zero-skew pinhole geometry because the
        # Hub intrinsics contract contains fx/fy/cx/cy but no skew term.
        u, v = np.meshgrid(
            np.arange(self.width, dtype=np.float64),
            np.arange(self.height, dtype=np.float64),
        )
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        radius = np.hypot(x, y)
        theta = np.arctan(radius)
        k2, k3, k4, k5, k6, k7 = self.calibration.distortion
        theta_d = (
            theta
            + k2 * theta**2
            + k3 * theta**3
            + k4 * theta**4
            + k5 * theta**5
            + k6 * theta**6
            + k7 * theta**7
        )
        factor = np.ones_like(radius)
        np.divide(theta_d, radius, out=factor, where=radius > 1e-12)
        xd = x * factor
        yd = y * factor
        map_x = (
            xd * self.calibration.fx
            + yd * self.calibration.skew
            + self.calibration.cx
        ).astype(np.float32)
        map_y = (yd * self.calibration.fy + self.calibration.cy).astype(np.float32)
        return map_x, map_y

    def rectify(self, raw_bgr: np.ndarray) -> np.ndarray:
        expected = (self.calibration.height, self.calibration.width)
        if raw_bgr.shape[:2] != expected:
            raise ValueError(
                f"Odin RGB shape {raw_bgr.shape[:2]} does not match calibration {expected}"
            )
        return cv2.remap(
            raw_bgr,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def project(
        self,
        points_odom: np.ndarray,
        T_odom_camera: np.ndarray,
        *,
        point_bgr: np.ndarray | None = None,
        rectified_bgr: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, float | int | None]]:
        transform = _rigid_matrix(T_odom_camera, label="T_odom_camera")
        points = np.asarray(points_odom, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_odom must have shape (N,3)")
        # Row-vector form of R^T * (p_odom - t).
        points_camera = (points - transform[:3, 3]) @ transform[:3, :3]
        z = points_camera[:, 2]
        valid = (
            np.isfinite(points_camera).all(axis=1)
            & (z >= self.depth_min_m)
            & (z <= self.depth_max_m)
        )
        points_camera = points_camera[valid]
        z = z[valid]
        colors = point_bgr[valid] if point_bgr is not None else None
        u = np.rint(self.fx * points_camera[:, 0] / z + self.cx).astype(np.int64)
        v = np.rint(self.fy * points_camera[:, 1] / z + self.cy).astype(np.int64)

        depth_flat = np.full(self.height * self.width, np.inf, dtype=np.float32)
        center_valid = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        center_u = u[center_valid]
        center_v = v[center_valid]
        center_z = z[center_valid].astype(np.float32)
        for dy in range(-self.splat_radius, self.splat_radius + 1):
            for dx in range(-self.splat_radius, self.splat_radius + 1):
                uu = center_u + dx
                vv = center_v + dy
                inside = (uu >= 0) & (uu < self.width) & (vv >= 0) & (vv < self.height)
                indices = vv[inside] * self.width + uu[inside]
                np.minimum.at(depth_flat, indices, center_z[inside])
        depth = depth_flat.reshape(self.height, self.width)
        depth[~np.isfinite(depth)] = 0.0

        color_error = None
        if colors is not None and rectified_bgr is not None and center_u.size:
            sampled = rectified_bgr[center_v, center_u].astype(np.int16)
            expected_colors = colors[center_valid].astype(np.int16)
            color_error = float(np.median(np.abs(sampled - expected_colors)))
        positive = depth[depth > 0.0]
        diagnostics: dict[str, float | int | None] = {
            "source_points": int(points.shape[0]),
            "range_valid_points": int(points_camera.shape[0]),
            "projected_points": int(center_u.size),
            "depth_valid_pixels": int(positive.size),
            "depth_valid_fraction": float(positive.size / depth.size),
            "depth_median_m": float(np.median(positive)) if positive.size else None,
            "depth_p95_m": float(np.percentile(positive, 95)) if positive.size else None,
            "projected_color_abs_error_median": color_error,
        }
        return depth, diagnostics


@dataclass(frozen=True)
class OdinFrame:
    bgr: np.ndarray
    depth_m: np.ndarray
    capture_time_ns: int
    T_odom_camera: np.ndarray
    covariance_6x6: list[float]
    diagnostics: dict[str, float | int | None]


class OdinRos2Source:
    """Small ROS2 subscriber with device-stamp synchronization and wall-time receipt."""

    def __init__(self, args: argparse.Namespace, projector: OdinProjector) -> None:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy._rclpy_pybind11 import RCLError
        from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, PointCloud2

        self.projector = projector
        self.sync_slop_ns = int(args.sync_slop_s * 1e9)
        self.read_timeout_s = args.read_timeout_s
        self._condition = threading.Condition()
        self._images: deque[tuple[int, int, Any]] = deque(maxlen=6)
        self._clouds: deque[tuple[int, int, Any]] = deque(maxlen=6)
        self._odometry: deque[tuple[int, int, Any]] = deque(maxlen=30)
        self._last_cloud_stamp = -1
        self.node = Node("focus_odin1_source")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.node.create_subscription(Image, args.rgb_topic, self._on_image, qos),
            self.node.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, qos),
            self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, qos),
        ]
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        def spin() -> None:
            try:
                self.executor.spin()
            except ExternalShutdownException:
                # SIGINT can shut down the global ROS context before main's
                # finally block gets a chance to stop this executor.
                pass
            except RCLError:
                # Humble may report the same normal shutdown as an invalid
                # wait-set context.  Preserve genuine runtime failures.
                if rclpy.ok():
                    raise

        self.thread = threading.Thread(
            target=spin, daemon=True, name="odin1-ros2-source"
        )
        self.thread.start()
        self._rclpy = rclpy

    def _store(self, queue: deque, message: Any) -> None:
        with self._condition:
            queue.append((stamp_ns(message), time.time_ns(), message))
            self._condition.notify_all()

    def _on_image(self, message: Any) -> None:
        self._store(self._images, message)

    def _on_cloud(self, message: Any) -> None:
        self._store(self._clouds, message)

    def _on_odom(self, message: Any) -> None:
        self._store(self._odometry, message)

    @staticmethod
    def _nearest(queue: deque, target_ns: int) -> tuple[int, int, Any] | None:
        if not queue:
            return None
        return min(queue, key=lambda entry: abs(entry[0] - target_ns))

    def _select_locked(self) -> tuple[Any, ...] | None:
        for cloud in reversed(self._clouds):
            cloud_stamp, _, _ = cloud
            # The queue still contains older clouds after a newer one has
            # been consumed.  Only accepting a different stamp would replay
            # those old frames on the next read; require strict monotonicity.
            if cloud_stamp <= self._last_cloud_stamp:
                continue
            image = self._nearest(self._images, cloud_stamp)
            odom = self._nearest(self._odometry, cloud_stamp)
            if image is None or odom is None:
                continue
            if max(abs(image[0] - cloud_stamp), abs(odom[0] - cloud_stamp)) > self.sync_slop_ns:
                continue
            self._last_cloud_stamp = cloud_stamp
            return image, cloud, odom
        return None

    def read(self) -> OdinFrame:
        deadline = time.monotonic() + self.read_timeout_s
        selected = None
        with self._condition:
            while selected is None:
                selected = self._select_locked()
                if selected is not None:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("no synchronized Odin RGB/cloud/odometry tuple")
                self._condition.wait(remaining)
        image_entry, cloud_entry, odom_entry = selected
        image_stamp, image_receipt_ns, image_message = image_entry
        cloud_stamp, _, cloud_message = cloud_entry
        odom_stamp, _, odom_message = odom_entry
        if cloud_message.header.frame_id != "odom":
            raise ValueError(
                f"Odin cloud frame changed from 'odom' to {cloud_message.header.frame_id!r}"
            )
        if odom_message.header.frame_id != "odom" or odom_message.child_frame_id != "odin1_base_link":
            raise ValueError(
                "Odin odometry frame contract changed: "
                f"{odom_message.header.frame_id!r}->{odom_message.child_frame_id!r}"
            )
        raw_bgr = decode_bgr8(image_message)
        rectified_bgr = self.projector.rectify(raw_bgr)
        points_odom, point_bgr = decode_cloud_xyz_bgr(cloud_message)
        T_odom_imu = quaternion_matrix(
            odom_message.pose.pose.position, odom_message.pose.pose.orientation
        )
        T_odom_camera = T_odom_imu @ self.projector.calibration.T_imu_camera
        depth_m, diagnostics = self.projector.project(
            points_odom,
            T_odom_camera,
            point_bgr=point_bgr,
            rectified_bgr=rectified_bgr,
        )
        diagnostics.update(
            {
                "image_cloud_skew_ms": abs(image_stamp - cloud_stamp) / 1e6,
                "odom_cloud_skew_ms": abs(odom_stamp - cloud_stamp) / 1e6,
                "device_image_stamp_ns": image_stamp,
                "device_cloud_stamp_ns": cloud_stamp,
                "device_odom_stamp_ns": odom_stamp,
            }
        )
        return OdinFrame(
            bgr=rectified_bgr,
            depth_m=depth_m,
            capture_time_ns=image_receipt_ns,
            T_odom_camera=T_odom_camera,
            covariance_6x6=list(odom_message.pose.covariance),
            diagnostics=diagnostics,
        )

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.thread.join(timeout=2.0)


def _write_evidence(
    directory: Path,
    frame_index: int,
    frame: OdinFrame,
    metadata: dict[str, Any],
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"frame_{frame_index:04d}"
    ok_rgb, rgb = cv2.imencode(".jpg", frame.bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    depth_counts = np.clip(
        np.rint(frame.depth_m / DEPTH_SCALE_M), 0, np.iinfo(np.uint16).max
    ).astype(np.uint16)
    ok_depth, depth = cv2.imencode(".png", depth_counts)
    if not ok_rgb or not ok_depth:
        raise RuntimeError("failed to encode Odin evidence")
    (directory / f"{stem}_rgb.jpg").write_bytes(rgb.tobytes())
    (directory / f"{stem}_depth.png").write_bytes(depth.tobytes())
    (directory / f"{stem}_metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-file", required=True)
    parser.add_argument("--expected-serial", default="O1-P070100205")
    parser.add_argument("--rgb-topic", default="/odin1/image")
    parser.add_argument("--cloud-topic", default="/odin1/cloud_slam")
    parser.add_argument("--odom-topic", default="/odin1/odometry")
    parser.add_argument("--robot-host", default="192.168.10.10")
    parser.add_argument("--tcp-port", type=int, default=31001)
    parser.add_argument(
        "--base-url",
        default=os.environ.get("FOCUS_HUB_BASE_URL", "http://127.0.0.1:18089"),
    )
    parser.add_argument(
        "--robot-id", default=os.environ.get("FOCUS_ROBOT_ID", "robot-1")
    )
    parser.add_argument(
        "--transform-version",
        default=os.environ.get(
            "FOCUS_ODIN1_TRANSFORM_VERSION",
            "yunji-odin1-local-odom-20260722-v1",
        ),
    )
    parser.add_argument("--shared-frame-transform-file", default=None)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--output-width", type=int, default=800)
    parser.add_argument("--splat-radius", type=int, default=1)
    parser.add_argument("--depth-min-m", type=float, default=0.2)
    parser.add_argument("--depth-max-m", type=float, default=8.0)
    parser.add_argument("--sync-slop-s", type=float, default=DEFAULT_SYNC_SLOP_S)
    parser.add_argument("--read-timeout-s", type=float, default=8.0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--png-level", type=int, default=1)
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument("--heartbeat-hz", type=float, default=2.0)
    parser.add_argument(
        "--camera-preview-url",
        default=os.environ.get("FOCUS_ODIN1_CAMERA_PREVIEW_URL"),
    )
    parser.add_argument("--camera-preview-token", default=None)
    parser.add_argument("--metrics-out", default="odin1_sender_metrics.json")
    parser.add_argument("--evidence-dir", default=None)
    parser.add_argument("--dry-run", action="store_true", help="process frames but do not upload")
    parser.add_argument("--base-camera-calibration-file", type=Path)
    parser.add_argument("--enable-command-capable-observations", action="store_true")
    parser.add_argument("--activation-confirmation", default="")
    args = parser.parse_args()

    if args.rate_hz < 0.0:
        parser.error("--rate-hz must be non-negative")
    if args.sync_slop_s <= 0.0 or args.read_timeout_s <= 0.0:
        parser.error("sync slop and read timeout must be positive")
    calibration = load_odin_calibration(args.calibration_file, args.expected_serial)
    base_camera_calibration = None
    if args.enable_command_capable_observations:
        if args.activation_confirmation != "COMMAND_CAPABLE_OBSERVATION_ONLY":
            parser.error(
                "command-capable observations require --activation-confirmation "
                "COMMAND_CAPABLE_OBSERVATION_ONLY"
            )
        if args.base_camera_calibration_file is None:
            parser.error(
                "--base-camera-calibration-file is required for command-capable observations"
            )
        if args.heartbeat_hz != 0:
            parser.error(
                "set --heartbeat-hz 0; the armed v2 receiver owns command health heartbeats"
            )
        base_camera_calibration = load_base_camera_calibration(
            args.base_camera_calibration_file,
            expected_robot_id=args.robot_id,
            expected_camera_frame=calibration.camera_frame,
        )
    elif args.base_camera_calibration_file is not None:
        parser.error(
            "--base-camera-calibration-file requires --enable-command-capable-observations"
        )
    shared_transform_path = args.shared_frame_transform_file or os.environ.get(
        "FOCUS_ODIN1_SHARED_TRANSFORM_FILE"
    )
    shared_transform, shared_calibration_id = load_shared_transform(
        shared_transform_path,
        expected_transform_version=args.transform_version,
    )
    if args.enable_command_capable_observations and shared_transform is None:
        parser.error("command-capable observations require a passed shared-frame transform")
    parent_frame = "shared_world"
    if shared_transform is None:
        print(
            "no Odin shared-frame calibration: protocol shared_world is defined as the "
            f"session-local {calibration.odometry_frame!r}; cross-robot fusion must remain "
            "disabled"
        )
    else:
        print(f"loaded shared-frame calibration {shared_calibration_id}")

    token = os.environ.get("FOCUS_ROBOT_TOKEN", "")
    if not args.dry_run and not token:
        print("FOCUS_ROBOT_TOKEN is not set", file=sys.stderr)
        return 2
    preview_token = (
        args.camera_preview_token
        or os.environ.get("FOCUS_CAMERA_PREVIEW_TOKEN")
        or token
    )
    if args.camera_preview_url and not preview_token:
        parser.error("camera preview URL requires a preview token")

    import rclpy

    rclpy.init()
    projector = OdinProjector(
        calibration,
        output_width=args.output_width,
        splat_radius=args.splat_radius,
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
    )
    source = OdinRos2Source(args, projector)
    tcp = WaterTcpClient(args.robot_host, args.tcp_port)
    transport = None if args.dry_run else HubTransport(args.base_url, args.robot_id, token)
    sequence = 0
    if transport is not None:
        sequence = transport.last_sequence() + 1
        print(f"resume: starting at sequence {sequence} [hub]")
    latest_localization = LatestLocalizationState()
    heartbeat = None
    if transport is not None and args.heartbeat_hz > 0.0:
        heartbeat = HeartbeatThread(
            robot_host=args.robot_host,
            tcp_port=args.tcp_port,
            base_url=args.base_url,
            robot_id=args.robot_id,
            token=token,
            localization_state=latest_localization,
            period_s=1.0 / args.heartbeat_hz,
        )
        heartbeat.start()

    frames = 0
    metrics: list[dict[str, Any]] = []
    period_s = 1.0 / args.rate_hz if args.rate_hz > 0.0 else 0.0
    evidence_dir = Path(args.evidence_dir) if args.evidence_dir else None
    try:
        info = tcp.request("/api/robot_info")
        print(f"connected to WATER chassis: {info.get('results', {}).get('product_id')}")
        while not args.max_frames or frames < args.max_frames:
            cycle_started = time.monotonic()
            frame = source.read()
            status = tcp.request("/api/robot_status").get("results", {})
            localization_state, covariance = classify_localization_state(
                frame.covariance_6x6
            )
            latest_localization.set(localization_state)
            pose = frame.T_odom_camera
            if shared_transform is not None:
                pose = shared_transform @ pose

            ok_rgb, jpeg = cv2.imencode(
                ".jpg",
                frame.bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality],
            )
            counts = np.clip(
                np.rint(frame.depth_m / DEPTH_SCALE_M),
                0,
                np.iinfo(np.uint16).max,
            ).astype(np.uint16)
            ok_depth, png = cv2.imencode(
                ".png", counts, [int(cv2.IMWRITE_PNG_COMPRESSION), args.png_level]
            )
            if not ok_rgb or not ok_depth:
                raise RuntimeError("failed to encode Odin RGB-D")
            rgb_bytes = jpeg.tobytes()
            depth_bytes = png.tobytes()
            metadata = build_metadata(
                robot_id=args.robot_id,
                sequence=sequence,
                rgb_bytes=rgb_bytes,
                depth_bytes=depth_bytes,
                pose_matrix=pose.reshape(-1).tolist(),
                transform_version=args.transform_version,
                goal_category=args.goal_category,
                status=status,
                width=projector.width,
                height=projector.height,
                fx=projector.fx,
                fy=projector.fy,
                cx=projector.cx,
                cy=projector.cy,
                capture_time_ns=frame.capture_time_ns,
                localization_state=localization_state,
                covariance_6x6=covariance,
                camera_frame=calibration.camera_frame,
                depth_min_m=args.depth_min_m,
                depth_max_m=args.depth_max_m,
            )
            if base_camera_calibration is not None:
                metadata["base_T_camera"] = base_camera_calibration.wire_transform()
                metadata["mapping_only"] = False
            attempts = 0
            ack_status = "dry_run"
            if transport is not None:
                ack, attempts = transport.upload(
                    metadata,
                    rgb_bytes,
                    depth_bytes,
                    lambda value: value,
                )
                ack_status = ack.get("status")
            if args.camera_preview_url:
                try:
                    import requests

                    requests.post(
                        args.camera_preview_url,
                        headers={
                            "X-Robot-Token": preview_token,
                            "Content-Type": "image/jpeg",
                            "Connection": "close",
                        },
                        data=rgb_bytes,
                        timeout=3.0,
                    ).raise_for_status()
                except Exception as exc:  # noqa: BLE001
                    print(f"preview push failed: {exc}", file=sys.stderr)
            if evidence_dir is not None:
                evidence_metadata = dict(metadata)
                evidence_metadata["odin_diagnostics"] = frame.diagnostics
                _write_evidence(evidence_dir, frames, frame, evidence_metadata)
            record = {
                "sequence": sequence,
                "ack_status": ack_status,
                "attempts": attempts,
                "localization_state": localization_state,
                **frame.diagnostics,
            }
            metrics.append(record)
            print(json.dumps(record, sort_keys=True))
            frames += 1
            sequence += 1
            elapsed = time.monotonic() - cycle_started
            if period_s > elapsed:
                time.sleep(period_s - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        if heartbeat is not None:
            heartbeat.stop()
            heartbeat.join(timeout=2.0)
        tcp.close()
        if rclpy.ok():
            rclpy.shutdown()
        summary = {
            "sensor_serial": calibration.serial,
            "transform_version": args.transform_version,
            "parent_frame": parent_frame,
            "pose_source_frame": calibration.odometry_frame,
            "shared_frame_calibration_id": shared_calibration_id,
            "dry_run": args.dry_run,
            "frames": frames,
            "metrics": metrics,
        }
        Path(args.metrics_out).write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
