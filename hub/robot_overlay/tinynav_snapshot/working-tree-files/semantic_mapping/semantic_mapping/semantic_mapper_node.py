"""Fuse synchronized map-frame RGB-D semantics into a sparse voxel layer."""

from __future__ import annotations

import json
from pathlib import Path
import resource
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMessage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped
import message_filters
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from semantic_mapping.keyframe_selector import KeyframeConfig, KeyframeSelector
from semantic_mapping.pointcloud import build_xyz_semantic_cloud, read_xyzuv_cloud
from semantic_mapping.pose_provider import make_transform_matrix
from semantic_mapping.semantic_fusion import (
    SemanticWeightConfig,
    build_semantic_observations,
)
from semantic_mapping.semantic_map_serializer import (
    load_semantic_voxel_map,
    save_semantic_voxel_map,
)
from semantic_mapping.semantic_bev_projector import (
    SemanticBEV,
    SemanticBEVGrid,
    SemanticBEVProjectionConfig,
    project_semantic_to_bev,
)
from semantic_mapping.semantic_schema import SemanticClassSchema
from semantic_mapping.semantic_voxel_map import (
    SemanticVoxelConfig,
    SparseSemanticVoxelMap,
)


def stamp_to_ns(stamp: TimeMessage) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SemanticMapperNode(Node):
    """ROS wrapper for confidence-weighted multi-view semantic voxel fusion."""

    def __init__(self) -> None:
        super().__init__("semantic_mapper_node")
        self._declare_parameters()
        self._read_parameters()
        self.schema = SemanticClassSchema.from_yaml(self.semantic_classes_file)
        self.class_count = max(self.schema.class_names) + 1
        self.class_colors = np.zeros((self.class_count, 3), dtype=np.uint8)
        for item in self.schema.classes:
            self.class_colors[item.class_id] = item.color_rgb

        config = SemanticVoxelConfig(
            resolution_m=self.voxel_resolution_m,
            origin_xyz=tuple(self.voxel_origin_xyz),
            class_count=self.class_count,
            valid_class_ids=tuple(self.schema.class_names),
            unknown_class_id=self.schema.unknown_id,
            dynamic_class_ids=tuple(sorted(self.schema.dynamic_class_ids)),
            min_observations=self.min_observations,
            confirmation_threshold=self.confirmation_threshold,
        )
        loaded_metadata: dict[str, Any] | None = None
        semantic_input = self._semantic_input_available()
        if semantic_input:
            self.semantic_map, loaded_metadata = load_semantic_voxel_map(
                self.input_directory
            )
            self._validate_loaded_map(loaded_metadata)
            loaded_config = self.semantic_map.config
            self.voxel_resolution_m = loaded_config.resolution_m
            self.voxel_origin_xyz = list(loaded_config.origin_xyz)
            self.min_observations = loaded_config.min_observations
            self.confirmation_threshold = loaded_config.confirmation_threshold
        else:
            self.semantic_map = SparseSemanticVoxelMap(config)

        self.floor_class_id = next(
            (
                item.class_id
                for item in self.schema.classes
                if item.name == self.bev_floor_class_name
            ),
            None,
        )
        self.bev_grid: SemanticBEVGrid | None = None
        self.bev_ground_z = self.bev_ground_z_config
        self.bev_grid_revision = 0
        if semantic_input and self.bev_use_occupancy_geometry:
            self._load_saved_bev_geometry()
        self.semantic_bev: SemanticBEV | None = None
        self.last_bev_map_revision = -1
        self.last_bev_grid_revision = -1
        self.total_bev_sec = 0.0
        self.bev_updates = 0

        self.weight_config = SemanticWeightConfig(
            min_confidence=self.min_confidence,
            depth_decay_m=self.depth_decay_m,
            edge_margin_px=self.edge_margin_px,
            min_edge_weight=self.min_edge_weight,
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
        self.bridge = CvBridge()

        input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
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
        self.label_subscriber = message_filters.Subscriber(
            self, Image, self.label_topic, qos_profile=input_qos
        )
        self.confidence_subscriber = message_filters.Subscriber(
            self, Image, self.confidence_topic, qos_profile=input_qos
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self.cloud_subscriber,
                self.pose_subscriber,
                self.label_subscriber,
                self.confidence_subscriber,
            ],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
        )
        self.synchronizer.registerCallback(self._synchronized_callback)

        self.voxel_publisher = self.create_publisher(
            PointCloud2, self.semantic_voxels_topic, map_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.semantic_markers_topic, map_qos
        )
        self.semantic_bev_publisher = self.create_publisher(
            Image, self.semantic_bev_topic, map_qos
        )
        self.semantic_bev_confidence_publisher = self.create_publisher(
            Image, self.semantic_bev_confidence_topic, map_qos
        )
        self.semantic_bev_visualization_publisher = self.create_publisher(
            Image, self.semantic_bev_visualization_topic, map_qos
        )
        self.semantic_bev_explored_publisher = self.create_publisher(
            Image, self.semantic_bev_explored_topic, map_qos
        )
        self.semantic_bev_height_min_publisher = self.create_publisher(
            Image, self.semantic_bev_height_min_topic, map_qos
        )
        self.semantic_bev_height_max_publisher = self.create_publisher(
            Image, self.semantic_bev_height_max_topic, map_qos
        )
        self.metadata_publisher = self.create_publisher(
            String, self.metadata_topic, map_qos
        )
        self.occupancy_bev_subscriber = self.create_subscription(
            OccupancyGrid,
            self.occupancy_bev_input_topic,
            self._occupancy_bev_callback,
            map_qos,
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

        self.received_sets = 0
        self.processed_keyframes = 0
        self.skipped_non_keyframes = 0
        self.dropped_frames = 0
        self.pose_jump_events = 0
        self.total_input_points = 0
        self.total_static_points = 0
        self.total_unknown_points = 0
        self.total_dynamic_points = 0
        self.total_low_confidence_points = 0
        self.total_unique_voxels = 0
        self.total_fusion_sec = 0.0
        self.first_set_monotonic: float | None = None
        self.last_publish_monotonic = 0.0
        self.last_timestamp_ns = (
            0
            if loaded_metadata is None
            else int(loaded_metadata.get("timestamp_ns", 0))
        )
        self.last_stamp: TimeMessage | None = None
        self.last_saved_revision = (
            -1 if loaded_metadata is None else self.semantic_map.revision
        )
        self.last_saved_bev_grid_revision = -1

        self.get_logger().info(
            "Phase-4/5 semantic mapper ready: "
            f"cloud={self.pointcloud_topic}, labels={self.label_topic}, "
            f"confidence={self.confidence_topic}, frame={self.target_frame}, "
            f"voxel={self.voxel_resolution_m:.3f} m, "
            f"classes={len(self.schema.classes)}, dynamic_skipped="
            f"{sorted(self.schema.dynamic_class_ids)}"
        )
        if loaded_metadata is not None:
            self.get_logger().info(
                f"Loaded {len(self.semantic_map)} semantic voxels from "
                f"{self.input_directory}"
            )
            self._publish_map(self.get_clock().now().to_msg())

    def _declare_parameters(self) -> None:
        defaults = {
            "topics.pointcloud_input": "/semantic_mapping/semantic_pointcloud",
            "topics.camera_pose": "/semantic_mapping/camera_pose",
            "topics.label": "/semantic_mapping/semantic_label_image",
            "topics.confidence": "/semantic_mapping/semantic_confidence_image",
            "topics.semantic_voxels": "/semantic_mapping/semantic_voxels",
            "topics.semantic_voxel_markers": (
                "/semantic_mapping/semantic_voxel_markers"
            ),
            "topics.semantic_map_metadata": (
                "/semantic_mapping/semantic_map_metadata"
            ),
            "topics.occupancy_bev_input": "/semantic_mapping/occupancy_bev",
            "topics.semantic_bev": "/semantic_mapping/semantic_bev",
            "topics.semantic_bev_confidence": (
                "/semantic_mapping/semantic_bev_confidence"
            ),
            "topics.semantic_bev_visualization": (
                "/semantic_mapping/semantic_bev_visualization"
            ),
            "topics.semantic_bev_explored": "/semantic_mapping/semantic_bev_explored",
            "topics.semantic_bev_height_min": (
                "/semantic_mapping/semantic_bev_height_min"
            ),
            "topics.semantic_bev_height_max": (
                "/semantic_mapping/semantic_bev_height_max"
            ),
            "frames.target_frame": "map",
            "sync.queue_size": 20,
            "sync.max_slop_sec": 0.001,
            "voxel.resolution_m": 0.05,
            "voxel.origin_xyz": [0.0, 0.0, 0.0],
            "semantic.min_confidence": 0.50,
            "semantic.min_observations": 2,
            "semantic.confirmation_threshold": 0.50,
            "semantic.depth_decay_m": 4.0,
            "semantic.edge_margin_px": 3.0,
            "semantic.min_edge_weight": 0.20,
            "semantic_classes.file": "",
            "integration.max_points_per_frame": 100000,
            "keyframe.translation_threshold_m": 0.20,
            "keyframe.rotation_threshold_deg": 10.0,
            "keyframe.max_interval_sec": 1.0,
            "keyframe.pose_jump_translation_m": 0.50,
            "keyframe.pose_jump_rotation_deg": 20.0,
            "keyframe.pause_frames_after_jump": 2,
            "visualization.max_voxels": 50000,
            "visualization.publish_rate_hz": 1.0,
            "visualization.publish_markers": True,
            "bev.use_occupancy_geometry": True,
            "bev.follow_occupancy_ground_z": True,
            "bev.resolution_m": 0.05,
            "bev.ground_z": 0.0,
            "bev.ground_min_z_relative": -0.10,
            "bev.ground_max_z_relative": 0.15,
            "bev.semantic_min_z_relative": 0.05,
            "bev.semantic_max_z_relative": 1.50,
            "bev.ignore_above_z_relative": 1.80,
            "bev.padding_cells": 1,
            "bev.min_cell_confidence": 0.50,
            "bev.floor_class_name": "floor",
            "input.directory": "",
            "input.allow_frame_id_override": False,
            "output.directory": "",
            "output.save_on_shutdown": True,
            "output.save_interval_sec": 30.0,
            "services.save_map": "/semantic_mapping/save_semantic_map",
            "diagnostics.interval_sec": 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _read_parameters(self) -> None:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        self.pointcloud_topic = str(value("topics.pointcloud_input"))
        self.camera_pose_topic = str(value("topics.camera_pose"))
        self.label_topic = str(value("topics.label"))
        self.confidence_topic = str(value("topics.confidence"))
        self.semantic_voxels_topic = str(value("topics.semantic_voxels"))
        self.semantic_markers_topic = str(value("topics.semantic_voxel_markers"))
        self.metadata_topic = str(value("topics.semantic_map_metadata"))
        self.occupancy_bev_input_topic = str(value("topics.occupancy_bev_input"))
        self.semantic_bev_topic = str(value("topics.semantic_bev"))
        self.semantic_bev_confidence_topic = str(
            value("topics.semantic_bev_confidence")
        )
        self.semantic_bev_visualization_topic = str(
            value("topics.semantic_bev_visualization")
        )
        self.semantic_bev_explored_topic = str(value("topics.semantic_bev_explored"))
        self.semantic_bev_height_min_topic = str(
            value("topics.semantic_bev_height_min")
        )
        self.semantic_bev_height_max_topic = str(
            value("topics.semantic_bev_height_max")
        )
        self.target_frame = str(value("frames.target_frame"))
        self.sync_queue_size = int(value("sync.queue_size"))
        self.sync_slop_sec = float(value("sync.max_slop_sec"))
        self.voxel_resolution_m = float(value("voxel.resolution_m"))
        self.voxel_origin_xyz = [float(item) for item in value("voxel.origin_xyz")]
        self.min_confidence = float(value("semantic.min_confidence"))
        self.min_observations = int(value("semantic.min_observations"))
        self.confirmation_threshold = float(
            value("semantic.confirmation_threshold")
        )
        self.depth_decay_m = float(value("semantic.depth_decay_m"))
        self.edge_margin_px = float(value("semantic.edge_margin_px"))
        self.min_edge_weight = float(value("semantic.min_edge_weight"))
        semantic_classes_file = str(value("semantic_classes.file"))
        if not semantic_classes_file:
            semantic_classes_file = str(
                Path(get_package_share_directory("semantic_mapping"))
                / "config"
                / "semantic_classes.yaml"
            )
        self.semantic_classes_file = semantic_classes_file
        self.max_points_per_frame = int(value("integration.max_points_per_frame"))
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
        self.max_visualization_voxels = int(value("visualization.max_voxels"))
        self.publish_rate_hz = float(value("visualization.publish_rate_hz"))
        self.publish_markers = bool(value("visualization.publish_markers"))
        self.bev_use_occupancy_geometry = bool(
            value("bev.use_occupancy_geometry")
        )
        self.bev_follow_occupancy_ground_z = bool(
            value("bev.follow_occupancy_ground_z")
        )
        self.bev_resolution_m = float(value("bev.resolution_m"))
        self.bev_ground_z_config = float(value("bev.ground_z"))
        self.bev_ground_min_z = float(value("bev.ground_min_z_relative"))
        self.bev_ground_max_z = float(value("bev.ground_max_z_relative"))
        self.bev_semantic_min_z = float(value("bev.semantic_min_z_relative"))
        self.bev_semantic_max_z = float(value("bev.semantic_max_z_relative"))
        self.bev_ignore_above_z = float(value("bev.ignore_above_z_relative"))
        self.bev_padding_cells = int(value("bev.padding_cells"))
        self.bev_min_cell_confidence = float(value("bev.min_cell_confidence"))
        self.bev_floor_class_name = str(value("bev.floor_class_name"))
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
            raise ValueError("Invalid semantic synchronization parameters")
        if len(self.voxel_origin_xyz) != 3:
            raise ValueError("voxel.origin_xyz must contain exactly three values")
        if self.max_points_per_frame <= 0:
            raise ValueError("integration.max_points_per_frame must be positive")
        if self.max_visualization_voxels <= 0 or self.publish_rate_hz <= 0.0:
            raise ValueError("Invalid semantic visualization parameters")
        if not self.bev_floor_class_name:
            raise ValueError("bev.floor_class_name must not be empty")
        SemanticBEVProjectionConfig(
            resolution_m=self.bev_resolution_m,
            ground_z=self.bev_ground_z_config,
            ground_min_z_relative=self.bev_ground_min_z,
            ground_max_z_relative=self.bev_ground_max_z,
            semantic_min_z_relative=self.bev_semantic_min_z,
            semantic_max_z_relative=self.bev_semantic_max_z,
            ignore_above_z_relative=self.bev_ignore_above_z,
            padding_cells=self.bev_padding_cells,
            min_cell_confidence=self.bev_min_cell_confidence,
        )
        if not self.save_service_name:
            raise ValueError("services.save_map must not be empty")
        if self.save_interval_sec < 0.0 or self.diagnostics_interval_sec <= 0.0:
            raise ValueError("Invalid semantic output/diagnostics interval")

    def _semantic_input_available(self) -> bool:
        if not self.input_directory:
            return False
        source = Path(self.input_directory).expanduser()
        metadata = source / "semantic_metadata.yaml"
        voxels = source / "semantic_voxels.npz"
        if metadata.is_file() != voxels.is_file():
            raise ValueError(
                "Semantic input requires both semantic_metadata.yaml and "
                "semantic_voxels.npz"
            )
        return metadata.is_file()

    def _validate_loaded_map(self, metadata: dict[str, Any]) -> None:
        loaded_frame = str(metadata.get("frame_id", ""))
        if loaded_frame != self.target_frame and not self.allow_input_frame_override:
            raise ValueError(
                f"Loaded semantic frame {loaded_frame!r} does not match "
                f"target frame {self.target_frame!r}"
            )
        if metadata.get("semantic_schema") != self.schema.to_metadata():
            raise ValueError("Loaded semantic class schema does not match configuration")
        config = self.semantic_map.config
        if config.class_count != self.class_count or tuple(
            config.valid_class_ids
        ) != tuple(self.schema.class_names):
            raise ValueError("Loaded semantic class IDs do not match configuration")

    def _load_saved_bev_geometry(self) -> None:
        """Load the matching Phase-2 grid before transient ROS map data arrives."""
        metadata_path = Path(self.input_directory).expanduser() / "metadata.yaml"
        if not metadata_path.is_file():
            return
        with metadata_path.open(encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        if not isinstance(metadata, dict):
            raise ValueError("metadata.yaml must contain a mapping")
        saved_frame = str(metadata.get("frame_id", ""))
        if saved_frame != self.target_frame and not self.allow_input_frame_override:
            raise ValueError(
                f"Loaded occupancy frame {saved_frame!r} does not match "
                f"target frame {self.target_frame!r}"
            )
        raw_bev = metadata.get("bev")
        if not isinstance(raw_bev, dict):
            raise ValueError("metadata.yaml is missing BEV geometry")
        raw_origin = raw_bev.get("origin_xy")
        if not isinstance(raw_origin, list) or len(raw_origin) != 2:
            raise ValueError("metadata.yaml BEV origin_xy must contain two values")
        self.bev_grid = SemanticBEVGrid(
            origin_xy=(float(raw_origin[0]), float(raw_origin[1])),
            resolution_m=float(raw_bev["resolution_m"]),
            width=int(raw_bev["width"]),
            height=int(raw_bev["height"]),
        )
        if self.bev_follow_occupancy_ground_z:
            self.bev_ground_z = float(raw_bev["ground_z"])
        self.bev_grid_revision += 1

    def _occupancy_bev_callback(self, message: OccupancyGrid) -> None:
        """Adopt the Phase-2 grid so semantic and geometry cells stay aligned."""
        if not self.bev_use_occupancy_geometry:
            return
        if message.header.frame_id != self.target_frame:
            self.get_logger().warning(
                "Ignoring occupancy BEV with unexpected frame "
                f"{message.header.frame_id!r}; expected {self.target_frame!r}"
            )
            return
        try:
            grid = SemanticBEVGrid(
                origin_xy=(
                    float(message.info.origin.position.x),
                    float(message.info.origin.position.y),
                ),
                resolution_m=float(message.info.resolution),
                width=int(message.info.width),
                height=int(message.info.height),
            )
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid occupancy BEV geometry: {error}")
            return
        ground_z = float(message.info.origin.position.z)
        changed = grid != self.bev_grid
        if self.bev_follow_occupancy_ground_z and ground_z != self.bev_ground_z:
            self.bev_ground_z = ground_z
            changed = True
        if not changed:
            return
        self.bev_grid = grid
        self.bev_grid_revision += 1
        self._publish_semantic_bev(message.header.stamp, force=True)

    def _semantic_bev_projection_config(self) -> SemanticBEVProjectionConfig:
        return SemanticBEVProjectionConfig(
            resolution_m=self.bev_resolution_m,
            ground_z=self.bev_ground_z,
            ground_min_z_relative=self.bev_ground_min_z,
            ground_max_z_relative=self.bev_ground_max_z,
            semantic_min_z_relative=self.bev_semantic_min_z,
            semantic_max_z_relative=self.bev_semantic_max_z,
            ignore_above_z_relative=self.bev_ignore_above_z,
            padding_cells=self.bev_padding_cells,
            min_cell_confidence=self.bev_min_cell_confidence,
        )

    def _refresh_semantic_bev(self, *, force: bool = False) -> SemanticBEV:
        if (
            not force
            and self.semantic_bev is not None
            and self.last_bev_map_revision == self.semantic_map.revision
            and self.last_bev_grid_revision == self.bev_grid_revision
        ):
            return self.semantic_bev
        start = time.perf_counter()
        self.semantic_bev = project_semantic_to_bev(
            self.semantic_map,
            self._semantic_bev_projection_config(),
            grid=self.bev_grid if self.bev_use_occupancy_geometry else None,
            floor_class_id=self.floor_class_id,
        )
        self.total_bev_sec += time.perf_counter() - start
        self.bev_updates += 1
        self.last_bev_map_revision = self.semantic_map.revision
        self.last_bev_grid_revision = self.bev_grid_revision
        return self.semantic_bev

    def _image_message(
        self, array: np.ndarray, encoding: str, header: Header
    ) -> Image:
        message = self.bridge.cv2_to_imgmsg(
            np.ascontiguousarray(array), encoding=encoding
        )
        message.header = header
        return message

    def _publish_semantic_bev(self, stamp: TimeMessage, *, force: bool = False) -> None:
        bev = self._refresh_semantic_bev(force=force)
        if bev.width == 0 or bev.height == 0:
            return
        header = Header(stamp=stamp, frame_id=self.target_frame)
        self.semantic_bev_publisher.publish(
            self._image_message(bev.semantic_label, "mono8", header)
        )
        self.semantic_bev_confidence_publisher.publish(
            self._image_message(bev.semantic_confidence, "32FC1", header)
        )
        self.semantic_bev_visualization_publisher.publish(
            self._image_message(
                self.schema.colorize(bev.semantic_label), "rgb8", header
            )
        )
        self.semantic_bev_explored_publisher.publish(
            self._image_message(bev.explored * np.uint8(255), "mono8", header)
        )
        self.semantic_bev_height_min_publisher.publish(
            self._image_message(bev.height_min, "32FC1", header)
        )
        self.semantic_bev_height_max_publisher.publish(
            self._image_message(bev.height_max, "32FC1", header)
        )

    def _synchronized_callback(
        self,
        cloud_message: PointCloud2,
        pose_message: PoseStamped,
        label_message: Image,
        confidence_message: Image,
    ) -> None:
        self.received_sets += 1
        now = time.monotonic()
        if self.first_set_monotonic is None:
            self.first_set_monotonic = now
        stamps = [
            stamp_to_ns(message.header.stamp)
            for message in (
                cloud_message,
                pose_message,
                label_message,
                confidence_message,
            )
        ]
        spread_sec = (max(stamps) - min(stamps)) * 1e-9
        if spread_sec > self.sync_slop_sec:
            self._drop(f"semantic timestamp spread is {spread_sec * 1e3:.3f} ms")
            return
        if (
            cloud_message.header.frame_id != self.target_frame
            or pose_message.header.frame_id != self.target_frame
        ):
            self._drop(
                "semantic geometry has unexpected frame IDs: "
                f"cloud={cloud_message.header.frame_id!r}, "
                f"pose={pose_message.header.frame_id!r}"
            )
            return

        try:
            points, pixels = read_xyzuv_cloud(cloud_message)
            labels = self.bridge.imgmsg_to_cv2(label_message, desired_encoding="mono8")
            confidence = self.bridge.imgmsg_to_cv2(
                confidence_message, desired_encoding="32FC1"
            )
            position = pose_message.pose.position
            orientation = pose_message.pose.orientation
            target_from_camera = make_transform_matrix(
                [position.x, position.y, position.z],
                [orientation.x, orientation.y, orientation.z, orientation.w],
            )
        except (CvBridgeError, ValueError) as error:
            self._drop(str(error))
            return

        timestamp_ns = stamps[0]
        decision = self.keyframes.evaluate(target_from_camera, timestamp_ns)
        if decision.pose_jump:
            self.pose_jump_events += 1
            self.get_logger().warning(
                "Detected TinyNav pose jump; pausing semantic integration: "
                f"translation={decision.translation_m:.3f} m, "
                f"rotation={decision.rotation_deg:.2f} deg"
            )
            return
        if not decision.accept:
            self.skipped_non_keyframes += 1
            return

        if points.shape[0] > self.max_points_per_frame:
            selection = np.linspace(
                0,
                points.shape[0] - 1,
                num=self.max_points_per_frame,
                dtype=np.int64,
            )
            points = points[selection]
            pixels = pixels[selection]

        start = time.perf_counter()
        try:
            observations = build_semantic_observations(
                points,
                pixels,
                labels,
                confidence,
                target_from_camera[:3, 3],
                self.schema,
                self.weight_config,
            )
            stats = self.semantic_map.integrate_observations(
                observations.points,
                observations.labels,
                observations.weights,
                timestamp_ns,
            )
        except ValueError as error:
            self._drop(str(error))
            return
        self.total_fusion_sec += time.perf_counter() - start
        self.processed_keyframes += 1
        self.total_input_points += observations.input_points
        self.total_static_points += stats.integrated_points
        self.total_unknown_points += observations.unknown_points
        self.total_dynamic_points += observations.dynamic_points
        self.total_low_confidence_points += observations.low_confidence_points
        self.total_unique_voxels += stats.unique_voxels
        self.last_timestamp_ns = timestamp_ns
        self.last_stamp = cloud_message.header.stamp

        if now - self.last_publish_monotonic >= 1.0 / self.publish_rate_hz:
            self._publish_map(cloud_message.header.stamp)

    def _drop(self, reason: str) -> None:
        self.dropped_frames += 1
        self.get_logger().warning(f"Dropping semantic fusion frame: {reason}")

    def _publish_map(self, stamp: TimeMessage) -> None:
        points, labels, confidences, observations = (
            self.semantic_map.confirmed_arrays()
        )
        if points.shape[0] > self.max_visualization_voxels:
            selection = np.linspace(
                0,
                points.shape[0] - 1,
                num=self.max_visualization_voxels,
                dtype=np.int64,
            )
            points = points[selection]
            labels = labels[selection]
            confidences = confidences[selection]
            observations = observations[selection]
        header = Header(stamp=stamp, frame_id=self.target_frame)
        colors = self.class_colors[labels]
        self.voxel_publisher.publish(
            build_xyz_semantic_cloud(
                points, labels, confidences, observations, colors, header
            )
        )
        if self.publish_markers:
            self.marker_publisher.publish(
                self._marker_array(points, labels, header)
            )
        self._publish_semantic_bev(stamp)

        counts = self.semantic_map.counts()
        by_name = {
            self.schema.class_names[class_id]: count
            for class_id, count in counts.by_class.items()
        }
        metadata = {
            "frame_id": self.target_frame,
            "timestamp_ns": self.last_timestamp_ns,
            "voxel_resolution_m": self.voxel_resolution_m,
            "voxel_origin_xyz": self.voxel_origin_xyz,
            "active_voxels": counts.active,
            "confirmed_voxels": counts.confirmed,
            "unconfirmed_voxels": counts.unconfirmed,
            "semantic_observations": counts.total_observations,
            "confirmed_by_class": by_name,
            "processed_keyframes": self.processed_keyframes,
        }
        if self.semantic_bev is not None:
            metadata["semantic_bev"] = {
                "resolution_m": self.semantic_bev.resolution_m,
                "origin_xy": self.semantic_bev.origin_xy.tolist(),
                "width": self.semantic_bev.width,
                "height": self.semantic_bev.height,
                "ground_z": self.bev_ground_z,
                "uses_occupancy_geometry": self.bev_grid is not None,
            }
        self.metadata_publisher.publish(String(data=json.dumps(metadata, sort_keys=True)))
        self.last_publish_monotonic = time.monotonic()

    def _marker_array(
        self, points: np.ndarray, labels: np.ndarray, header: Header
    ) -> MarkerArray:
        reset = Marker()
        reset.header = header
        reset.action = Marker.DELETEALL
        markers = [reset]
        for semantic_class in self.schema.classes:
            if semantic_class.class_id == self.schema.unknown_id:
                continue
            class_points = points[labels == semantic_class.class_id]
            if class_points.shape[0] == 0:
                continue
            marker = Marker()
            marker.header = header
            marker.ns = "semantic_voxels"
            marker.id = semantic_class.class_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.voxel_resolution_m * 0.95
            marker.scale.y = self.voxel_resolution_m * 0.95
            marker.scale.z = self.voxel_resolution_m * 0.95
            marker.color.r = semantic_class.color_rgb[0] / 255.0
            marker.color.g = semantic_class.color_rgb[1] / 255.0
            marker.color.b = semantic_class.color_rgb[2] / 255.0
            marker.color.a = 0.85
            marker.points = [
                Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                for point in class_points
            ]
            markers.append(marker)
        return MarkerArray(markers=markers)

    def _save_if_dirty(self, *, force: bool = False) -> tuple[bool, str]:
        if not self.output_directory:
            return False, "output.directory is empty"
        if len(self.semantic_map) == 0:
            return False, "semantic map is empty"
        bev_changed = self.bev_grid_revision != self.last_saved_bev_grid_revision
        if (
            self.semantic_map.revision == self.last_saved_revision
            and not bev_changed
            and not force
        ):
            return True, "semantic map is already checkpointed"
        try:
            semantic_bev = self._refresh_semantic_bev()
            output = save_semantic_voxel_map(
                self.output_directory,
                self.semantic_map,
                self.schema,
                frame_id=self.target_frame,
                timestamp_ns=self.last_timestamp_ns,
                bev=semantic_bev,
            )
        except (OSError, ValueError) as error:
            if rclpy.ok():
                self.get_logger().error(f"Failed to save semantic map: {error}")
            return False, str(error)
        self.last_saved_revision = self.semantic_map.revision
        self.last_saved_bev_grid_revision = self.bev_grid_revision
        if rclpy.ok():
            self.get_logger().info(f"Saved Phase-4/5 semantic map to {output}")
        return True, str(output)

    def _save_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        response.success, response.message = self._save_if_dirty(force=True)
        return response

    def _log_diagnostics(self) -> None:
        counts = self.semantic_map.counts()
        elapsed = (
            0.0
            if self.first_set_monotonic is None
            else max(time.monotonic() - self.first_set_monotonic, 1e-6)
        )
        input_hz = self.received_sets / elapsed if elapsed > 0.0 else 0.0
        fusion_ms = self.total_fusion_sec / max(self.processed_keyframes, 1) * 1e3
        bev_ms = self.total_bev_sec / max(self.bev_updates, 1) * 1e3
        rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        self.get_logger().info(
            "Semantic voxel diagnostics: "
            f"input_hz={input_hz:.2f}, sets={self.received_sets}, "
            f"keyframes={self.processed_keyframes}, "
            f"skipped={self.skipped_non_keyframes}, dropped={self.dropped_frames}, "
            f"pose_jumps={self.pose_jump_events}, input_points="
            f"{self.total_input_points}, static_points={self.total_static_points}, "
            f"unknown_points={self.total_unknown_points}, "
            f"dynamic_points={self.total_dynamic_points}, "
            f"low_confidence_points={self.total_low_confidence_points}, "
            f"voxel_updates={self.total_unique_voxels}, "
            f"active_voxels={counts.active}, confirmed_voxels={counts.confirmed}, "
            f"unconfirmed_voxels={counts.unconfirmed}, fusion_ms={fusion_ms:.2f}, "
            f"semantic_bev={0 if self.semantic_bev is None else self.semantic_bev.width}x"
            f"{0 if self.semantic_bev is None else self.semantic_bev.height}, "
            f"bev_ms={bev_ms:.2f}, "
            f"rss_mb={rss_mb:.1f}"
        )

    def close(self) -> None:
        if self.save_on_shutdown:
            self._save_if_dirty()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticMapperNode()
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
