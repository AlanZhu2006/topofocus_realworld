"""Publish timestamped RGB-D backprojection in the TinyNav map frame."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
from typing import Any

from builtin_interfaces.msg import Time as TimeMessage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from semantic_mapping.depth_backprojection import (
    CameraIntrinsics,
    backproject_depth,
    depth_image_to_meters,
    transform_points,
)
from semantic_mapping.pose_provider import (
    matrix_from_transform_message,
    transform_components,
)
from semantic_mapping.pointcloud import build_xyzrgb_cloud


@dataclass(frozen=True)
class PendingRgbdFrame:
    """Synchronized RGB-D input retained until its TF pose is available."""

    rgb: Image
    depth: Image
    camera_info: CameraInfo
    enqueued_monotonic: float


def stamp_to_ns(stamp: TimeMessage) -> int:
    """Convert a builtin_interfaces Time-like object to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SemanticPointcloudNode(Node):
    """Synchronize aligned RGB-D and transform points with image-time TF."""

    def __init__(self) -> None:
        super().__init__("semantic_pointcloud_node")
        self._declare_parameters()
        self._read_parameters()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=self.tf_cache_sec))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_publisher = self.create_publisher(
            PointCloud2, self.pointcloud_topic, cloud_qos
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.camera_pose_topic, cloud_qos
        )

        self.rgb_subscriber = message_filters.Subscriber(
            self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_subscriber = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.info_subscriber = message_filters.Subscriber(
            self,
            CameraInfo,
            self.camera_info_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber, self.info_subscriber],
            queue_size=self.sync_queue_size,
            slop=self.max_sync_slop_sec,
        )
        self.synchronizer.registerCallback(self._synchronized_callback)

        self.pending_frames: deque[PendingRgbdFrame] = deque()
        self.pending_timer = self.create_timer(0.02, self._process_pending_frames)
        self.diagnostics_timer = self.create_timer(
            self.diagnostics_interval_sec, self._log_diagnostics
        )
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.pose_lookup_failures = 0
        self.pose_lookup_retries = 0
        self.alignment_failures = 0
        self.total_points = 0
        self.total_processing_sec = 0.0
        self.total_pose_time_error_sec = 0.0
        self.max_pose_time_error_observed_sec = 0.0
        self.first_receive_monotonic: float | None = None
        self.last_accepted_stamp_ns: int | None = None
        self.tracking_from_color: np.ndarray | None = None
        self.used_map_alignment_fallback = 0
        self.target_alignment_ready = self.target_frame == self.odom_frame
        self.target_alignment_wait_frames = 0
        self.published_once = False

        self.get_logger().info(
            "Semantic point cloud ready: "
            f"rgb={self.rgb_topic}, depth={self.depth_topic}, "
            f"camera_info={self.camera_info_topic}, output={self.pointcloud_topic}, "
            f"camera_pose={self.camera_pose_topic}, "
            f"target={self.target_frame}, pose_camera={self.pose_camera_frame}, "
            f"tracking_camera={self.tracking_camera_frame}, rgb_camera={self.camera_frame}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "topics.rgb": "/camera/camera/color/image_raw",
            "topics.depth": "/camera/camera/aligned_depth_to_color/image_raw",
            "topics.camera_info": "/camera/camera/aligned_depth_to_color/camera_info",
            "topics.pointcloud": "/semantic_mapping/semantic_pointcloud",
            "topics.camera_pose": "/semantic_mapping/camera_pose",
            "frames.target_frame": "map",
            "frames.odom_frame": "world",
            "frames.pose_camera_frame": "camera",
            "frames.tracking_camera_frame": "camera_infra1_optical_frame",
            "frames.camera_frame": "camera_color_optical_frame",
            "depth.min_depth_m": 0.25,
            "depth.max_depth_m": 5.0,
            "depth.stride": 2,
            "depth.edge_filter": True,
            "depth.edge_threshold_m": 0.10,
            "sync.queue_size": 20,
            "sync.max_slop_sec": 0.02,
            "pose.lookup_timeout_sec": 0.02,
            "pose.pending_timeout_sec": 2.0,
            "pose.max_time_error_sec": 0.05,
            "pose.tf_cache_sec": 30.0,
            "pose.allow_latest_map_alignment": True,
            "pose.wait_for_target_alignment": True,
            "processing.max_rate_hz": 5.0,
            "processing.publish_once": False,
            "validation.require_frame_ids": True,
            "diagnostics.interval_sec": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        self.rgb_topic = str(value("topics.rgb"))
        self.depth_topic = str(value("topics.depth"))
        self.camera_info_topic = str(value("topics.camera_info"))
        self.pointcloud_topic = str(value("topics.pointcloud"))
        self.camera_pose_topic = str(value("topics.camera_pose"))
        self.target_frame = str(value("frames.target_frame"))
        self.odom_frame = str(value("frames.odom_frame"))
        self.pose_camera_frame = str(value("frames.pose_camera_frame"))
        self.tracking_camera_frame = str(value("frames.tracking_camera_frame"))
        self.camera_frame = str(value("frames.camera_frame"))
        self.min_depth_m = float(value("depth.min_depth_m"))
        self.max_depth_m = float(value("depth.max_depth_m"))
        self.depth_stride = int(value("depth.stride"))
        self.edge_filter = bool(value("depth.edge_filter"))
        self.edge_threshold_m = float(value("depth.edge_threshold_m"))
        self.sync_queue_size = int(value("sync.queue_size"))
        self.max_sync_slop_sec = float(value("sync.max_slop_sec"))
        self.lookup_timeout_sec = float(value("pose.lookup_timeout_sec"))
        self.pending_timeout_sec = float(value("pose.pending_timeout_sec"))
        self.max_pose_time_error_sec = float(value("pose.max_time_error_sec"))
        self.tf_cache_sec = float(value("pose.tf_cache_sec"))
        self.allow_latest_map_alignment = bool(value("pose.allow_latest_map_alignment"))
        self.wait_for_target_alignment = bool(
            value("pose.wait_for_target_alignment")
        )
        self.max_rate_hz = float(value("processing.max_rate_hz"))
        self.publish_once = bool(value("processing.publish_once"))
        self.require_frame_ids = bool(value("validation.require_frame_ids"))
        self.diagnostics_interval_sec = float(value("diagnostics.interval_sec"))

        if self.sync_queue_size <= 0:
            raise ValueError("sync.queue_size must be positive")
        if self.max_sync_slop_sec < 0.0:
            raise ValueError("sync.max_slop_sec must be non-negative")
        if self.pending_timeout_sec <= 0.0 or self.lookup_timeout_sec < 0.0:
            raise ValueError("Pose timeouts are invalid")
        if self.max_pose_time_error_sec < 0.0 or self.tf_cache_sec <= 0.0:
            raise ValueError("Pose cache/error parameters are invalid")
        if self.max_rate_hz < 0.0:
            raise ValueError("processing.max_rate_hz must be non-negative")
        if self.diagnostics_interval_sec <= 0.0:
            raise ValueError("diagnostics.interval_sec must be positive")

    def _synchronized_callback(
        self, rgb_message: Image, depth_message: Image, camera_info: CameraInfo
    ) -> None:
        self.received_frames += 1
        now = time.monotonic()
        if self.first_receive_monotonic is None:
            self.first_receive_monotonic = now
        if self.publish_once and self.published_once:
            return
        if not self._target_frame_is_ready():
            self.target_alignment_wait_frames += 1
            return

        stamps = [
            stamp_to_ns(rgb_message.header.stamp),
            stamp_to_ns(depth_message.header.stamp),
            stamp_to_ns(camera_info.header.stamp),
        ]
        spread_sec = (max(stamps) - min(stamps)) * 1e-9
        if spread_sec > self.max_sync_slop_sec:
            self._drop_alignment(
                f"Synchronized messages exceed slop: {spread_sec * 1e3:.3f} ms"
            )
            return

        stamp_ns = stamps[0]
        if self.max_rate_hz > 0.0 and self.last_accepted_stamp_ns is not None:
            min_period_ns = int(1e9 / self.max_rate_hz)
            if stamp_ns - self.last_accepted_stamp_ns < min_period_ns:
                return
        self.last_accepted_stamp_ns = stamp_ns

        if len(self.pending_frames) >= self.sync_queue_size:
            self.pending_frames.popleft()
            self.dropped_frames += 1
            self.get_logger().warning("Pending RGB-D queue full; dropped oldest frame")
        self.pending_frames.append(
            PendingRgbdFrame(rgb_message, depth_message, camera_info, now)
        )

    def _process_pending_frames(self) -> None:
        if not self.pending_frames or (self.publish_once and self.published_once):
            return

        pending = self.pending_frames[0]
        pose_stamp = Time.from_msg(pending.rgb.header.stamp)
        try:
            target_from_tracking, pose_time_error_sec = self._lookup_tracking_pose(
                pose_stamp
            )
            tracking_from_color = self._lookup_color_extrinsic()
        except TransformException as error:
            self.pose_lookup_retries += 1
            age = time.monotonic() - pending.enqueued_monotonic
            if age < self.pending_timeout_sec:
                return
            self.pending_frames.popleft()
            self.dropped_frames += 1
            self.pose_lookup_failures += 1
            self.get_logger().warning(
                "Dropping RGB-D frame after timestamped TF lookup failure: "
                f"stamp={stamp_to_ns(pending.rgb.header.stamp)}, error={error}"
            )
            return

        if pose_time_error_sec > self.max_pose_time_error_sec:
            self.pending_frames.popleft()
            self.dropped_frames += 1
            self.get_logger().warning(
                f"Dropping frame: pose time error {pose_time_error_sec * 1e3:.3f} ms "
                f"exceeds {self.max_pose_time_error_sec * 1e3:.3f} ms"
            )
            return

        self.total_pose_time_error_sec += pose_time_error_sec
        self.max_pose_time_error_observed_sec = max(
            self.max_pose_time_error_observed_sec, pose_time_error_sec
        )
        self.pending_frames.popleft()
        target_from_color = target_from_tracking @ tracking_from_color
        self._publish_frame(pending, target_from_color)

    def _target_frame_is_ready(self) -> bool:
        if not self.wait_for_target_alignment or self.target_alignment_ready:
            return True
        self.target_alignment_ready = self.tf_buffer.can_transform(
            self.target_frame,
            self.odom_frame,
            Time(),
            timeout=Duration(seconds=0.0),
        )
        if self.target_alignment_ready:
            self.get_logger().info(
                "TinyNav target-frame alignment is ready: "
                f"{self.target_frame} <- {self.odom_frame}"
            )
        return self.target_alignment_ready

    def _lookup_tracking_pose(self, stamp: Time) -> tuple[np.ndarray, float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.pose_camera_frame,
                stamp,
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
            return self._matrix_and_time_error(transform, stamp)
        except TransformException as direct_error:
            if (
                not self.allow_latest_map_alignment
                or self.target_frame == self.odom_frame
            ):
                raise direct_error

        # TinyNav publishes world->camera at the image stamp, but its world->map
        # alignment is sparse. Only that frame-alignment edge may use its latest
        # value; the moving camera pose remains an exact image-time lookup.
        odom_from_tracking = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.pose_camera_frame,
            stamp,
            timeout=Duration(seconds=self.lookup_timeout_sec),
        )
        target_from_odom = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.odom_frame,
            Time(),
            timeout=Duration(seconds=self.lookup_timeout_sec),
        )
        odom_matrix, time_error_sec = self._matrix_and_time_error(
            odom_from_tracking, stamp
        )
        self.used_map_alignment_fallback += 1
        return (
            matrix_from_transform_message(target_from_odom.transform) @ odom_matrix,
            time_error_sec,
        )

    @staticmethod
    def _matrix_and_time_error(
        transform: TransformStamped, stamp: Time
    ) -> tuple[np.ndarray, float]:
        requested_ns = stamp.nanoseconds
        returned_ns = stamp_to_ns(transform.header.stamp)
        time_error_sec = (
            0.0 if returned_ns == 0 else abs(returned_ns - requested_ns) * 1e-9
        )
        return matrix_from_transform_message(transform.transform), time_error_sec

    def _lookup_color_extrinsic(self) -> np.ndarray:
        if self.tracking_from_color is not None:
            return self.tracking_from_color
        if self.tracking_camera_frame == self.camera_frame:
            self.tracking_from_color = np.eye(4, dtype=np.float64)
            return self.tracking_from_color
        transform = self.tf_buffer.lookup_transform(
            self.tracking_camera_frame,
            self.camera_frame,
            Time(),
            timeout=Duration(seconds=self.lookup_timeout_sec),
        )
        self.tracking_from_color = matrix_from_transform_message(transform.transform)
        return self.tracking_from_color

    def _publish_frame(
        self, pending: PendingRgbdFrame, target_from_color: np.ndarray
    ) -> None:
        start = time.perf_counter()
        if not self._alignment_is_valid(pending):
            return

        info = pending.camera_info
        intrinsics = CameraIntrinsics(
            width=int(info.width),
            height=int(info.height),
            fx=float(info.k[0]),
            fy=float(info.k[4]),
            cx=float(info.k[2]),
            cy=float(info.k[5]),
        )
        try:
            rgb = self.bridge.imgmsg_to_cv2(pending.rgb, desired_encoding="rgb8")
            depth_raw = self.bridge.imgmsg_to_cv2(
                pending.depth, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.dropped_frames += 1
            self.get_logger().warning(f"Failed to decode synchronized RGB-D: {error}")
            return

        try:
            depth_m = depth_image_to_meters(depth_raw, pending.depth.encoding)
            result = backproject_depth(
                depth_m,
                intrinsics,
                rgb_image=rgb,
                stride=self.depth_stride,
                min_depth_m=self.min_depth_m,
                max_depth_m=self.max_depth_m,
                edge_filter=self.edge_filter,
                edge_threshold_m=self.edge_threshold_m,
            )
            points_target = transform_points(result.points_camera, target_from_color)
        except ValueError as error:
            self._drop_alignment(str(error))
            return

        if result.colors_rgb is None:
            raise RuntimeError("RGB backprojection unexpectedly returned no colors")

        header = Header(stamp=pending.rgb.header.stamp, frame_id=self.target_frame)
        cloud = build_xyzrgb_cloud(
            points_target, result.colors_rgb, header, result.pixels_uv
        )
        translation, quaternion = transform_components(target_from_color)
        camera_pose = PoseStamped()
        camera_pose.header = header
        camera_pose.pose.position.x = float(translation[0])
        camera_pose.pose.position.y = float(translation[1])
        camera_pose.pose.position.z = float(translation[2])
        camera_pose.pose.orientation.x = float(quaternion[0])
        camera_pose.pose.orientation.y = float(quaternion[1])
        camera_pose.pose.orientation.z = float(quaternion[2])
        camera_pose.pose.orientation.w = float(quaternion[3])
        self.pose_publisher.publish(camera_pose)
        self.cloud_publisher.publish(cloud)
        self.published_once = True
        self.processed_frames += 1
        self.total_points += cloud.width
        self.total_processing_sec += time.perf_counter() - start
        self.get_logger().info(
            f"Published first target-frame RGB-D cloud with {cloud.width} points "
            f"at stamp {stamp_to_ns(header.stamp)} in {header.frame_id}",
            once=True,
        )

    def _alignment_is_valid(self, pending: PendingRgbdFrame) -> bool:
        depth = pending.depth
        rgb = pending.rgb
        info = pending.camera_info
        if rgb.width != depth.width or rgb.height != depth.height:
            self._drop_alignment(
                f"RGB/depth dimensions differ: rgb={rgb.width}x{rgb.height}, "
                f"depth={depth.width}x{depth.height}"
            )
            return False
        if info.width != rgb.width or info.height != rgb.height:
            self._drop_alignment(
                f"CameraInfo dimensions {info.width}x{info.height} do not match "
                f"RGB-D {rgb.width}x{rgb.height}"
            )
            return False
        if self.require_frame_ids:
            frames = {
                "rgb": rgb.header.frame_id,
                "depth": depth.header.frame_id,
                "camera_info": info.header.frame_id,
            }
            invalid = {
                name: frame
                for name, frame in frames.items()
                if frame != self.camera_frame
            }
            if invalid:
                self._drop_alignment(
                    f"Expected aligned RGB-D frame {self.camera_frame!r}, got {invalid}"
                )
                return False
        return True

    def _drop_alignment(self, reason: str) -> None:
        self.alignment_failures += 1
        self.dropped_frames += 1
        self.get_logger().warning(f"Dropping unaligned RGB-D frame: {reason}")

    def _log_diagnostics(self) -> None:
        if self.first_receive_monotonic is None:
            self.get_logger().info(
                "Semantic mapping waiting for synchronized RGB-D input"
            )
            return
        elapsed = max(time.monotonic() - self.first_receive_monotonic, 1e-6)
        mean_points = self.total_points / max(self.processed_frames, 1)
        mean_processing_ms = (
            self.total_processing_sec / max(self.processed_frames, 1) * 1e3
        )
        mean_pose_error_ms = (
            self.total_pose_time_error_sec / max(self.processed_frames, 1) * 1e3
        )
        self.get_logger().info(
            "Semantic mapping diagnostics: "
            f"rgbd_hz={self.received_frames / elapsed:.2f}, "
            f"received={self.received_frames}, processed={self.processed_frames}, "
            f"dropped={self.dropped_frames}, pending={len(self.pending_frames)}, "
            f"pose_lookup_failures={self.pose_lookup_failures}, "
            f"pose_lookup_retries={self.pose_lookup_retries}, "
            f"alignment_wait_frames={self.target_alignment_wait_frames}, "
            f"map_alignment_fallbacks={self.used_map_alignment_fallback}, "
            f"mean_pose_error_ms={mean_pose_error_ms:.3f}, "
            f"max_pose_error_ms={self.max_pose_time_error_observed_sec * 1e3:.3f}, "
            f"alignment_failures={self.alignment_failures}, "
            f"mean_points={mean_points:.0f}, processing_ms={mean_processing_ms:.2f}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticPointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
