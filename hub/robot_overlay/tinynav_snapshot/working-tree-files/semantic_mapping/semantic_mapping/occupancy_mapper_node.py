"""Fuse timestamped map-frame RGB-D points into sparse occupancy and BEV."""

from __future__ import annotations

from dataclasses import replace
import json
import resource
import time
from typing import Any

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseStamped
import message_filters
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from semantic_mapping.bev_projector import (
    BEVProjectionConfig,
    OccupancyBEV,
    project_occupancy_to_bev,
)
from semantic_mapping.keyframe_selector import KeyframeConfig, KeyframeSelector
from semantic_mapping.ground_estimator import (
    GroundEstimate,
    GroundEstimatorConfig,
    GroundHeightEstimator,
)
from semantic_mapping.map_serializer import (
    load_occupancy_voxel_map,
    save_occupancy_map,
)
from semantic_mapping.occupancy_voxel_map import (
    OccupancyVoxelConfig,
    SparseOccupancyVoxelMap,
)
from semantic_mapping.pointcloud import (
    build_xyz_probability_cloud,
    read_xyz_cloud,
)
from semantic_mapping.pose_provider import make_transform_matrix


def stamp_to_ns(stamp: TimeMessage) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _image_message(array: np.ndarray, encoding: str, header: Header) -> Image:
    contiguous = np.ascontiguousarray(array)
    if contiguous.ndim != 2:
        raise ValueError(f"BEV image must be 2D, got {contiguous.shape}")
    message = Image()
    message.header = header
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    message.encoding = encoding
    message.is_bigendian = False
    message.step = int(contiguous.strides[0])
    message.data = contiguous.tobytes()
    return message


class OccupancyMapperNode(Node):
    """ROS wrapper around deterministic Phase-2 geometry algorithms."""

    def __init__(self) -> None:
        super().__init__("occupancy_mapper_node")
        self._declare_parameters()
        self._read_parameters()

        voxel_config = OccupancyVoxelConfig(
            resolution_m=self.voxel_resolution_m,
            origin_xyz=tuple(self.voxel_origin_xyz),
            free_update=self.free_update,
            occupied_update=self.occupied_update,
            min_log_odds=self.min_log_odds,
            max_log_odds=self.max_log_odds,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
            truncation_distance_m=self.truncation_distance_m,
        )
        loaded_metadata: dict[str, Any] | None = None
        if self.input_directory:
            self.voxel_map, loaded_metadata = load_occupancy_voxel_map(
                self.input_directory
            )
            loaded_frame = str(loaded_metadata.get("frame_id", ""))
            if (
                loaded_frame != self.target_frame
                and not self.allow_input_frame_override
            ):
                raise ValueError(
                    f"Loaded occupancy frame {loaded_frame!r} does not match "
                    f"target frame {self.target_frame!r}; set "
                    "input.allow_frame_id_override only for a verified alias"
                )
            loaded_config = self.voxel_map.config
            self.voxel_resolution_m = loaded_config.resolution_m
            self.voxel_origin_xyz = list(loaded_config.origin_xyz)
            self.free_update = loaded_config.free_update
            self.occupied_update = loaded_config.occupied_update
            self.min_log_odds = loaded_config.min_log_odds
            self.max_log_odds = loaded_config.max_log_odds
            self.free_threshold = loaded_config.free_threshold
            self.occupied_threshold = loaded_config.occupied_threshold
            self.truncation_distance_m = loaded_config.truncation_distance_m
            loaded_bev = loaded_metadata.get("bev", {})
            if isinstance(loaded_bev, dict) and "ground_z" in loaded_bev:
                self.ground_z = float(loaded_bev["ground_z"])
        else:
            self.voxel_map = SparseOccupancyVoxelMap(voxel_config)
        self.bev_config = BEVProjectionConfig(
            resolution_m=self.bev_resolution_m,
            ground_z=self.ground_z,
            ground_min_z_relative=self.ground_min_z,
            ground_max_z_relative=self.ground_max_z,
            collision_min_z_relative=self.collision_min_z,
            collision_max_z_relative=self.collision_max_z,
            ignore_above_z_relative=self.ignore_above_z,
            padding_cells=self.bev_padding_cells,
            exclude_ground_band_from_collision=self.exclude_ground_from_collision,
        )
        self.ground_estimator = GroundHeightEstimator(
            self.ground_z,
            GroundEstimatorConfig(
                horizontal_radius_m=self.ground_radius_m,
                search_min_z_relative=self.ground_search_min_z,
                search_max_z_relative=self.ground_search_max_z,
                max_points=self.ground_max_points,
                ransac_iterations=self.ground_ransac_iterations,
                inlier_threshold_m=self.ground_inlier_threshold_m,
                min_candidate_points=self.ground_min_candidate_points,
                min_inlier_points=self.ground_min_inlier_points,
                min_inlier_ratio=self.ground_min_inlier_ratio,
                max_tilt_deg=self.ground_max_tilt_deg,
                max_candidate_jump_m=self.ground_max_candidate_jump_m,
                candidate_window_size=self.ground_candidate_window_size,
                ema_alpha=self.ground_ema_alpha,
                max_update_step_m=self.ground_max_update_step_m,
                random_seed=self.ground_random_seed,
                bootstrap_enabled=self.ground_bootstrap_enabled,
                bootstrap_search_min_z_relative=self.ground_bootstrap_search_min_z,
                bootstrap_search_max_z_relative=self.ground_bootstrap_search_max_z,
                bootstrap_min_camera_height_m=(
                    self.ground_bootstrap_min_camera_height_m
                ),
                bootstrap_max_camera_height_m=(
                    self.ground_bootstrap_max_camera_height_m
                ),
                bootstrap_required_candidates=(
                    self.ground_bootstrap_required_candidates
                ),
                bootstrap_consensus_tolerance_m=(
                    self.ground_bootstrap_consensus_tolerance_m
                ),
            ),
        )
        self.keyframes = KeyframeSelector(
            KeyframeConfig(
                translation_threshold_m=self.keyframe_translation_m,
                rotation_threshold_deg=self.keyframe_rotation_deg,
                max_interval_sec=self.keyframe_interval_sec,
                pose_jump_translation_m=self.pose_jump_translation_m,
                pose_jump_rotation_deg=self.pose_jump_rotation_deg,
                pause_frames_after_jump=self.pause_frames_after_jump,
            )
        )

        input_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cloud_subscriber = message_filters.Subscriber(
            self, PointCloud2, self.pointcloud_topic, qos_profile=input_qos
        )
        self.pose_subscriber = message_filters.Subscriber(
            self, PoseStamped, self.camera_pose_topic, qos_profile=input_qos
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_subscriber, self.pose_subscriber],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
        )
        self.synchronizer.registerCallback(self._synchronized_callback)

        self.occupied_voxel_publisher = self.create_publisher(
            PointCloud2, self.occupied_voxels_topic, map_qos
        )
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid, self.occupancy_bev_topic, map_qos
        )
        self.occupancy_probability_publisher = self.create_publisher(
            Image, self.occupancy_probability_topic, map_qos
        )
        self.free_probability_publisher = self.create_publisher(
            Image, self.free_probability_topic, map_qos
        )
        self.explored_publisher = self.create_publisher(
            Image, self.explored_topic, map_qos
        )
        self.height_max_publisher = self.create_publisher(
            Image, self.height_max_topic, map_qos
        )
        self.metadata_publisher = self.create_publisher(
            String, self.metadata_topic, map_qos
        )
        self.save_service = self.create_service(
            Trigger, self.save_service_name, self._save_service_callback
        )

        self.diagnostics_timer = self.create_timer(
            self.diagnostics_interval_sec, self._log_diagnostics
        )
        self.save_timer = None
        if self.output_directory and self.save_interval_sec > 0.0:
            self.save_timer = self.create_timer(
                self.save_interval_sec, self._save_if_dirty
            )

        self.received_pairs = 0
        self.processed_keyframes = 0
        self.skipped_non_keyframes = 0
        self.dropped_frames = 0
        self.pose_jump_events = 0
        self.total_input_points = 0
        self.total_integrated_rays = 0
        self.total_unique_free_updates = 0
        self.total_unique_occupied_updates = 0
        self.total_integration_sec = 0.0
        self.total_bev_sec = 0.0
        self.bev_updates = 0
        self.first_pair_monotonic: float | None = None
        self.last_bev_monotonic = 0.0
        self.last_bev: OccupancyBEV | None = None
        self.last_bev_revision = -1
        self.last_stamp: TimeMessage | None = None
        self.last_timestamp_ns = (
            0
            if loaded_metadata is None
            else int(loaded_metadata.get("timestamp_ns", 0))
        )
        self.last_saved_revision = -1
        self.last_saved_ground_z: float | None = None
        self.ground_estimation_attempts = 0
        self.last_ground_estimate: GroundEstimate | None = None

        self.get_logger().info(
            "Phase-2 occupancy mapper ready: "
            f"pointcloud={self.pointcloud_topic}, pose={self.camera_pose_topic}, "
            f"frame={self.target_frame}, voxel={self.voxel_resolution_m:.3f} m, "
            f"bev={self.bev_resolution_m:.3f} m, max_rays={self.max_rays_per_frame}, "
            f"ground_estimation={self.ground_estimation_enabled}, "
            f"ground_bootstrap={self.ground_bootstrap_enabled}, "
            f"save_interval={self.save_interval_sec:.1f} s"
        )
        if loaded_metadata is not None:
            loaded_frame = str(loaded_metadata.get("frame_id", ""))
            qualifier = (
                " with explicit frame override"
                if loaded_frame != self.target_frame
                else ""
            )
            self.get_logger().warning(
                f"Loaded {len(self.voxel_map)} occupancy voxels from "
                f"{self.input_directory} ({loaded_frame} -> "
                f"{self.target_frame}{qualifier})"
            )
            self._publish_map(self.get_clock().now().to_msg())

    def _declare_parameters(self) -> None:
        defaults = {
            "topics.pointcloud_input": "/semantic_mapping/semantic_pointcloud",
            "topics.camera_pose": "/semantic_mapping/camera_pose",
            "topics.occupied_voxels": "/semantic_mapping/occupied_voxels",
            "topics.occupancy_bev": "/semantic_mapping/occupancy_bev",
            "topics.occupancy_probability_bev": (
                "/semantic_mapping/occupancy_probability_bev"
            ),
            "topics.free_probability_bev": "/semantic_mapping/free_probability_bev",
            "topics.explored_bev": "/semantic_mapping/explored_bev",
            "topics.height_max_bev": "/semantic_mapping/height_max_bev",
            "topics.map_metadata": "/semantic_mapping/map_metadata",
            "frames.target_frame": "map",
            "sync.queue_size": 10,
            "sync.max_slop_sec": 0.001,
            "voxel.resolution_m": 0.05,
            "voxel.origin_xyz": [0.0, 0.0, 0.0],
            "voxel.free_update": -0.40,
            "voxel.occupied_update": 0.85,
            "voxel.min_log_odds": -4.0,
            "voxel.max_log_odds": 4.0,
            "voxel.truncation_distance_m": 0.05,
            "occupancy.free_threshold": 0.30,
            "occupancy.occupied_threshold": 0.70,
            "integration.max_rays_per_frame": 6000,
            "keyframe.translation_threshold_m": 0.20,
            "keyframe.rotation_threshold_deg": 10.0,
            "keyframe.max_interval_sec": 1.0,
            "keyframe.pose_jump_translation_m": 0.50,
            "keyframe.pose_jump_rotation_deg": 20.0,
            "keyframe.pause_frames_after_jump": 2,
            "bev.resolution_m": 0.05,
            "bev.ground_z": 0.0,
            "bev.ground_min_z_relative": -0.10,
            "bev.ground_max_z_relative": 0.15,
            "bev.collision_min_z_relative": 0.10,
            "bev.collision_max_z_relative": 0.75,
            "bev.ignore_above_z_relative": 1.80,
            "bev.padding_cells": 1,
            "bev.exclude_ground_band_from_collision": True,
            "bev.publish_rate_hz": 1.0,
            "ground_estimation.enabled": True,
            "ground_estimation.update_interval_keyframes": 3,
            "ground_estimation.horizontal_radius_m": 2.0,
            "ground_estimation.search_min_z_relative": -0.25,
            "ground_estimation.search_max_z_relative": 0.25,
            "ground_estimation.max_points": 4000,
            "ground_estimation.ransac_iterations": 128,
            "ground_estimation.inlier_threshold_m": 0.03,
            "ground_estimation.min_candidate_points": 200,
            "ground_estimation.min_inlier_points": 300,
            "ground_estimation.min_inlier_ratio": 0.15,
            "ground_estimation.max_tilt_deg": 12.0,
            "ground_estimation.max_candidate_jump_m": 0.15,
            "ground_estimation.candidate_window_size": 9,
            "ground_estimation.ema_alpha": 0.05,
            "ground_estimation.max_update_step_m": 0.01,
            "ground_estimation.random_seed": 0,
            "ground_estimation.bootstrap.enabled": True,
            "ground_estimation.bootstrap.search_min_z_relative": -1.0,
            "ground_estimation.bootstrap.search_max_z_relative": 0.50,
            "ground_estimation.bootstrap.min_camera_height_m": 0.15,
            "ground_estimation.bootstrap.max_camera_height_m": 1.00,
            "ground_estimation.bootstrap.required_candidates": 3,
            "ground_estimation.bootstrap.consensus_tolerance_m": 0.04,
            "visualization.max_occupied_voxels": 100000,
            "input.directory": "",
            "input.allow_frame_id_override": False,
            "output.directory": "",
            "output.save_on_shutdown": True,
            "output.save_interval_sec": 30.0,
            "services.save_map": "/semantic_mapping/save_map",
            "diagnostics.interval_sec": 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _read_parameters(self) -> None:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        self.pointcloud_topic = str(value("topics.pointcloud_input"))
        self.camera_pose_topic = str(value("topics.camera_pose"))
        self.occupied_voxels_topic = str(value("topics.occupied_voxels"))
        self.occupancy_bev_topic = str(value("topics.occupancy_bev"))
        self.occupancy_probability_topic = str(
            value("topics.occupancy_probability_bev")
        )
        self.free_probability_topic = str(value("topics.free_probability_bev"))
        self.explored_topic = str(value("topics.explored_bev"))
        self.height_max_topic = str(value("topics.height_max_bev"))
        self.metadata_topic = str(value("topics.map_metadata"))
        self.target_frame = str(value("frames.target_frame"))
        self.sync_queue_size = int(value("sync.queue_size"))
        self.sync_slop_sec = float(value("sync.max_slop_sec"))
        self.voxel_resolution_m = float(value("voxel.resolution_m"))
        self.voxel_origin_xyz = [float(item) for item in value("voxel.origin_xyz")]
        self.free_update = float(value("voxel.free_update"))
        self.occupied_update = float(value("voxel.occupied_update"))
        self.min_log_odds = float(value("voxel.min_log_odds"))
        self.max_log_odds = float(value("voxel.max_log_odds"))
        self.truncation_distance_m = float(
            value("voxel.truncation_distance_m")
        )
        self.free_threshold = float(value("occupancy.free_threshold"))
        self.occupied_threshold = float(value("occupancy.occupied_threshold"))
        self.max_rays_per_frame = int(value("integration.max_rays_per_frame"))
        self.keyframe_translation_m = float(
            value("keyframe.translation_threshold_m")
        )
        self.keyframe_rotation_deg = float(value("keyframe.rotation_threshold_deg"))
        self.keyframe_interval_sec = float(value("keyframe.max_interval_sec"))
        self.pose_jump_translation_m = float(
            value("keyframe.pose_jump_translation_m")
        )
        self.pose_jump_rotation_deg = float(
            value("keyframe.pose_jump_rotation_deg")
        )
        self.pause_frames_after_jump = int(
            value("keyframe.pause_frames_after_jump")
        )
        self.bev_resolution_m = float(value("bev.resolution_m"))
        self.ground_z = float(value("bev.ground_z"))
        self.ground_min_z = float(value("bev.ground_min_z_relative"))
        self.ground_max_z = float(value("bev.ground_max_z_relative"))
        self.collision_min_z = float(value("bev.collision_min_z_relative"))
        self.collision_max_z = float(value("bev.collision_max_z_relative"))
        self.ignore_above_z = float(value("bev.ignore_above_z_relative"))
        self.bev_padding_cells = int(value("bev.padding_cells"))
        self.exclude_ground_from_collision = bool(
            value("bev.exclude_ground_band_from_collision")
        )
        self.bev_publish_rate_hz = float(value("bev.publish_rate_hz"))
        self.ground_estimation_enabled = bool(
            value("ground_estimation.enabled")
        )
        self.ground_update_interval_keyframes = int(
            value("ground_estimation.update_interval_keyframes")
        )
        self.ground_radius_m = float(
            value("ground_estimation.horizontal_radius_m")
        )
        self.ground_search_min_z = float(
            value("ground_estimation.search_min_z_relative")
        )
        self.ground_search_max_z = float(
            value("ground_estimation.search_max_z_relative")
        )
        self.ground_max_points = int(value("ground_estimation.max_points"))
        self.ground_ransac_iterations = int(
            value("ground_estimation.ransac_iterations")
        )
        self.ground_inlier_threshold_m = float(
            value("ground_estimation.inlier_threshold_m")
        )
        self.ground_min_candidate_points = int(
            value("ground_estimation.min_candidate_points")
        )
        self.ground_min_inlier_points = int(
            value("ground_estimation.min_inlier_points")
        )
        self.ground_min_inlier_ratio = float(
            value("ground_estimation.min_inlier_ratio")
        )
        self.ground_max_tilt_deg = float(
            value("ground_estimation.max_tilt_deg")
        )
        self.ground_max_candidate_jump_m = float(
            value("ground_estimation.max_candidate_jump_m")
        )
        self.ground_candidate_window_size = int(
            value("ground_estimation.candidate_window_size")
        )
        self.ground_ema_alpha = float(value("ground_estimation.ema_alpha"))
        self.ground_max_update_step_m = float(
            value("ground_estimation.max_update_step_m")
        )
        self.ground_random_seed = int(value("ground_estimation.random_seed"))
        self.ground_bootstrap_enabled = bool(
            value("ground_estimation.bootstrap.enabled")
        )
        self.ground_bootstrap_search_min_z = float(
            value("ground_estimation.bootstrap.search_min_z_relative")
        )
        self.ground_bootstrap_search_max_z = float(
            value("ground_estimation.bootstrap.search_max_z_relative")
        )
        self.ground_bootstrap_min_camera_height_m = float(
            value("ground_estimation.bootstrap.min_camera_height_m")
        )
        self.ground_bootstrap_max_camera_height_m = float(
            value("ground_estimation.bootstrap.max_camera_height_m")
        )
        self.ground_bootstrap_required_candidates = int(
            value("ground_estimation.bootstrap.required_candidates")
        )
        self.ground_bootstrap_consensus_tolerance_m = float(
            value("ground_estimation.bootstrap.consensus_tolerance_m")
        )
        self.max_visualization_voxels = int(
            value("visualization.max_occupied_voxels")
        )
        self.input_directory = str(value("input.directory"))
        self.allow_input_frame_override = bool(
            value("input.allow_frame_id_override")
        )
        self.output_directory = str(value("output.directory"))
        self.save_on_shutdown = bool(value("output.save_on_shutdown"))
        self.save_interval_sec = float(value("output.save_interval_sec"))
        self.save_service_name = str(value("services.save_map"))
        self.diagnostics_interval_sec = float(value("diagnostics.interval_sec"))

        if self.sync_queue_size <= 0 or self.sync_slop_sec < 0.0:
            raise ValueError("Invalid pointcloud/pose synchronization parameters")
        if len(self.voxel_origin_xyz) != 3:
            raise ValueError("voxel.origin_xyz must contain exactly three values")
        if self.max_rays_per_frame <= 0:
            raise ValueError("integration.max_rays_per_frame must be positive")
        if self.bev_publish_rate_hz <= 0.0:
            raise ValueError("bev.publish_rate_hz must be positive")
        if self.max_visualization_voxels <= 0:
            raise ValueError("visualization.max_occupied_voxels must be positive")
        if self.ground_update_interval_keyframes <= 0:
            raise ValueError(
                "ground_estimation.update_interval_keyframes must be positive"
            )
        if not self.save_service_name:
            raise ValueError("services.save_map must not be empty")
        if self.save_interval_sec < 0.0 or self.diagnostics_interval_sec <= 0.0:
            raise ValueError("Invalid output/diagnostics interval")

    def _synchronized_callback(
        self, cloud_message: PointCloud2, pose_message: PoseStamped
    ) -> None:
        self.received_pairs += 1
        now = time.monotonic()
        if self.first_pair_monotonic is None:
            self.first_pair_monotonic = now
        cloud_stamp_ns = stamp_to_ns(cloud_message.header.stamp)
        pose_stamp_ns = stamp_to_ns(pose_message.header.stamp)
        if abs(cloud_stamp_ns - pose_stamp_ns) * 1e-9 > self.sync_slop_sec:
            self.dropped_frames += 1
            self.get_logger().warning(
                "Dropping pointcloud/pose pair with timestamp spread "
                f"{abs(cloud_stamp_ns - pose_stamp_ns) * 1e-6:.3f} ms"
            )
            return
        if (
            cloud_message.header.frame_id != self.target_frame
            or pose_message.header.frame_id != self.target_frame
        ):
            self.dropped_frames += 1
            self.get_logger().warning(
                "Dropping geometry with unexpected frame IDs: "
                f"cloud={cloud_message.header.frame_id!r}, "
                f"pose={pose_message.header.frame_id!r}, "
                f"expected={self.target_frame!r}"
            )
            return

        position = pose_message.pose.position
        orientation = pose_message.pose.orientation
        try:
            target_from_camera = make_transform_matrix(
                [position.x, position.y, position.z],
                [orientation.x, orientation.y, orientation.z, orientation.w],
            )
            decision = self.keyframes.evaluate(target_from_camera, cloud_stamp_ns)
        except ValueError as error:
            self.dropped_frames += 1
            self.get_logger().warning(f"Dropping invalid camera pose: {error}")
            return
        if decision.pose_jump:
            self.pose_jump_events += 1
            self.get_logger().warning(
                "Detected TinyNav pose jump; pausing irreversible integration: "
                f"translation={decision.translation_m:.3f} m, "
                f"rotation={decision.rotation_deg:.2f} deg"
            )
            return
        if not decision.accept:
            self.skipped_non_keyframes += 1
            return

        try:
            points = read_xyz_cloud(cloud_message)
        except ValueError as error:
            self.dropped_frames += 1
            self.get_logger().warning(f"Dropping invalid point cloud: {error}")
            return
        self.total_input_points += int(points.shape[0])
        self._update_ground_estimate(points, target_from_camera[:3, 3])
        if points.shape[0] > self.max_rays_per_frame:
            selection = np.linspace(
                0,
                points.shape[0] - 1,
                num=self.max_rays_per_frame,
                dtype=np.int64,
            )
            points = points[selection]

        integration_start = time.perf_counter()
        stats = self.voxel_map.integrate_points(
            target_from_camera[:3, 3], points, cloud_stamp_ns
        )
        self.total_integration_sec += time.perf_counter() - integration_start
        self.processed_keyframes += 1
        self.total_integrated_rays += stats.valid_rays
        self.total_unique_free_updates += stats.unique_free_voxels
        self.total_unique_occupied_updates += stats.unique_occupied_voxels
        self.last_stamp = cloud_message.header.stamp
        self.last_timestamp_ns = cloud_stamp_ns

        minimum_publish_period = 1.0 / self.bev_publish_rate_hz
        if now - self.last_bev_monotonic >= minimum_publish_period:
            self._publish_map(cloud_message.header.stamp)

    def _update_ground_estimate(
        self, points: np.ndarray, camera_position: np.ndarray
    ) -> None:
        if not self.ground_estimation_enabled:
            return
        if (
            self.processed_keyframes % self.ground_update_interval_keyframes
            != 0
        ):
            return

        previous_ground_z = self.ground_z
        estimate = self.ground_estimator.update(points, camera_position)
        self.ground_estimation_attempts += 1
        self.last_ground_estimate = estimate
        if not estimate.accepted:
            return

        self.ground_z = estimate.ground_z
        self.bev_config = replace(self.bev_config, ground_z=self.ground_z)
        if abs(self.ground_z - previous_ground_z) > 1e-6:
            self.last_bev_revision = -1
        self.get_logger().info(
            "Updated ground height: "
            f"filtered_z={self.ground_z:.3f} m, "
            f"candidate_z={estimate.candidate_ground_z:.3f} m, "
            f"consensus_z={estimate.consensus_ground_z:.3f} m, "
            f"tilt={estimate.tilt_deg:.2f} deg, "
            f"inliers={estimate.inlier_points}/{estimate.candidate_points}",
            throttle_duration_sec=5.0,
        )

    def _publish_map(self, stamp: TimeMessage) -> None:
        start = time.perf_counter()
        bev = project_occupancy_to_bev(self.voxel_map, self.bev_config)
        self.total_bev_sec += time.perf_counter() - start
        self.bev_updates += 1
        self.last_bev_monotonic = time.monotonic()
        self.last_bev = bev
        self.last_bev_revision = self.voxel_map.revision
        if bev.width == 0 or bev.height == 0:
            return

        header = Header(stamp=stamp, frame_id=self.target_frame)
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = header
        occupancy_grid.info.map_load_time = stamp
        occupancy_grid.info.resolution = float(bev.resolution_m)
        occupancy_grid.info.width = bev.width
        occupancy_grid.info.height = bev.height
        occupancy_grid.info.origin.position.x = float(bev.origin_xy[0])
        occupancy_grid.info.origin.position.y = float(bev.origin_xy[1])
        occupancy_grid.info.origin.position.z = float(self.ground_z)
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = bev.occupancy_grid.reshape(-1).astype(int).tolist()
        self.occupancy_grid_publisher.publish(occupancy_grid)
        self.occupancy_probability_publisher.publish(
            _image_message(bev.occupancy_probability, "32FC1", header)
        )
        self.free_probability_publisher.publish(
            _image_message(bev.free_probability, "32FC1", header)
        )
        self.explored_publisher.publish(
            _image_message(bev.explored * np.uint8(255), "mono8", header)
        )
        self.height_max_publisher.publish(
            _image_message(bev.height_max, "32FC1", header)
        )

        occupied_points, occupied_probabilities = self.voxel_map.occupied_points()
        if occupied_points.shape[0] > self.max_visualization_voxels:
            selection = np.linspace(
                0,
                occupied_points.shape[0] - 1,
                num=self.max_visualization_voxels,
                dtype=np.int64,
            )
            occupied_points = occupied_points[selection]
            occupied_probabilities = occupied_probabilities[selection]
        self.occupied_voxel_publisher.publish(
            build_xyz_probability_cloud(
                occupied_points, occupied_probabilities, header
            )
        )
        counts = self.voxel_map.counts()
        metadata = {
            "frame_id": self.target_frame,
            "timestamp_ns": self.last_timestamp_ns,
            "voxel_resolution_m": self.voxel_resolution_m,
            "voxel_origin_xyz": self.voxel_origin_xyz,
            "ground_z": self.ground_z,
            "bev_resolution_m": bev.resolution_m,
            "bev_origin_xy": bev.origin_xy.tolist(),
            "bev_width": bev.width,
            "bev_height": bev.height,
            "active_voxels": counts.active,
            "free_voxels": counts.free,
            "occupied_voxels": counts.occupied,
            "uncertain_voxels": counts.uncertain,
            "processed_keyframes": self.processed_keyframes,
            "ground_estimation_enabled": self.ground_estimation_enabled,
            "ground_estimation_attempts": self.ground_estimation_attempts,
            "ground_estimation_accepted": self.ground_estimator.accepted_updates,
            "ground_estimation_rejected": self.ground_estimator.rejected_updates,
            "ground_estimation_mode": self.ground_estimator.mode,
        }
        self.metadata_publisher.publish(String(data=json.dumps(metadata, sort_keys=True)))

    def _save_if_dirty(self, *, force: bool = False) -> tuple[bool, str]:
        if not self.output_directory:
            return False, "output.directory is empty"
        if len(self.voxel_map) == 0:
            return False, "occupancy map is empty"
        unchanged = (
            self.voxel_map.revision == self.last_saved_revision
            and self.last_saved_ground_z is not None
            and abs(self.ground_z - self.last_saved_ground_z) <= 1e-9
        )
        if unchanged and not force:
            return True, "occupancy map is already checkpointed"
        if (
            self.last_bev is None
            or self.last_bev_revision != self.voxel_map.revision
            or self.last_saved_ground_z is None
            or abs(self.bev_config.ground_z - self.ground_z) > 1e-9
        ):
            self.last_bev = project_occupancy_to_bev(
                self.voxel_map, self.bev_config
            )
            self.last_bev_revision = self.voxel_map.revision
        try:
            output = save_occupancy_map(
                self.output_directory,
                self.voxel_map,
                self.last_bev,
                frame_id=self.target_frame,
                timestamp_ns=self.last_timestamp_ns,
                ground_z=self.ground_z,
            )
        except (OSError, ValueError) as error:
            if rclpy.ok():
                self.get_logger().error(f"Failed to save occupancy map: {error}")
            return False, str(error)
        self.last_saved_revision = self.voxel_map.revision
        self.last_saved_ground_z = self.ground_z
        if rclpy.ok():
            self.get_logger().info(f"Saved Phase-2 occupancy map to {output}")
        return True, str(output)

    def _save_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        success, message = self._save_if_dirty(force=True)
        response.success = success
        response.message = message
        return response

    def _log_diagnostics(self) -> None:
        counts = self.voxel_map.counts()
        elapsed = (
            0.0
            if self.first_pair_monotonic is None
            else max(time.monotonic() - self.first_pair_monotonic, 1e-6)
        )
        input_hz = self.received_pairs / elapsed if elapsed > 0.0 else 0.0
        integration_ms = (
            self.total_integration_sec / max(self.processed_keyframes, 1) * 1e3
        )
        bev_ms = self.total_bev_sec / max(self.bev_updates, 1) * 1e3
        rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        bev_size = (
            "0x0"
            if self.last_bev is None
            else f"{self.last_bev.width}x{self.last_bev.height}"
        )
        ground_status = (
            "none"
            if self.last_ground_estimate is None
            else self.last_ground_estimate.reason
        )
        ground_fit = (
            "0/0"
            if self.last_ground_estimate is None
            else (
                f"{self.last_ground_estimate.inlier_points}/"
                f"{self.last_ground_estimate.candidate_points}"
            )
        )
        self.get_logger().info(
            "Occupancy mapping diagnostics: "
            f"input_hz={input_hz:.2f}, pairs={self.received_pairs}, "
            f"keyframes={self.processed_keyframes}, "
            f"skipped={self.skipped_non_keyframes}, dropped={self.dropped_frames}, "
            f"pose_jumps={self.pose_jump_events}, rays={self.total_integrated_rays}, "
            f"active_voxels={counts.active}, free_voxels={counts.free}, "
            f"occupied_voxels={counts.occupied}, uncertain_voxels={counts.uncertain}, "
            f"free_updates={self.total_unique_free_updates}, "
            f"occupied_updates={self.total_unique_occupied_updates}, "
            f"integration_ms={integration_ms:.2f}, bev={bev_size}, "
            f"bev_ms={bev_ms:.2f}, ground_z={self.ground_z:.3f}, "
            f"ground={ground_status}, ground_fit={ground_fit}, "
            f"ground_mode={self.ground_estimator.mode}, "
            f"ground_updates={self.ground_estimator.accepted_updates}/"
            f"{self.ground_estimation_attempts}, rss_mb={rss_mb:.1f}"
        )

    def close(self) -> None:
        if self.save_on_shutdown:
            self._save_if_dirty()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OccupancyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
