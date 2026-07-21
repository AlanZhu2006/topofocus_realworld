"""Publish timestamp-aligned 2D semantic labels and confidence images."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from semantic_mapping.precomputed_mask_backend import PrecomputedMaskBackend
from semantic_mapping.semantic_backend import (
    SemanticBackend,
    SemanticFrameUnavailable,
)
from semantic_mapping.semantic_schema import SemanticClassSchema
from semantic_mapping.semantic_visualizer import blend_semantic_overlay


def stamp_to_nanoseconds(stamp: Any) -> int:
    """Convert a ROS Time-like message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SemanticPerceptionNode(Node):
    """Run a replaceable semantic backend against timestamped RGB images."""

    def __init__(self) -> None:
        super().__init__("semantic_perception_node")
        self._declare_parameters()
        self._read_parameters()

        self.schema = SemanticClassSchema.from_yaml(self.semantic_classes_file)
        self.backend = self._create_backend()
        self.bridge = CvBridge()

        image_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        metadata_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.label_publisher = self.create_publisher(
            Image, self.label_topic, image_qos
        )
        self.confidence_publisher = self.create_publisher(
            Image, self.confidence_topic, image_qos
        )
        self.visualization_publisher = self.create_publisher(
            Image, self.visualization_topic, image_qos
        )
        self.metadata_publisher = self.create_publisher(
            String, self.class_metadata_topic, metadata_qos
        )
        self.rgb_subscription = self.create_subscription(
            Image, self.rgb_topic, self._rgb_callback, qos_profile_sensor_data
        )

        self.received_frames = 0
        self.processed_frames = 0
        self.unavailable_frames = 0
        self.invalid_frames = 0
        self.rate_limited_frames = 0
        self.total_inference_sec = 0.0
        self.total_source_time_error_sec = 0.0
        self.max_source_time_error_sec = 0.0
        self.first_receive_monotonic: float | None = None
        self.last_attempt_stamp_ns: int | None = None
        self.last_warning_monotonic: float | None = None
        self.published_once = False
        self.diagnostics_timer = self.create_timer(
            self.diagnostics_interval_sec, self._log_diagnostics
        )
        self._publish_class_metadata()
        self.get_logger().info(
            "Semantic perception ready: "
            f"backend={self.backend_type}, rgb={self.rgb_topic}, "
            f"label={self.label_topic}, confidence={self.confidence_topic}, "
            f"classes={self.semantic_classes_file}"
        )

    def _declare_parameters(self) -> None:
        model_dir = "~/.cache/tinynav/semantic_models/segformer_b0_ade20k"
        defaults = {
            "topics.rgb": "/camera/camera/color/image_raw",
            "topics.label": "/semantic_mapping/semantic_label_image",
            "topics.confidence": "/semantic_mapping/semantic_confidence_image",
            "topics.visualization": "/semantic_mapping/semantic_visualization",
            "topics.class_metadata": "/semantic_mapping/semantic_class_metadata",
            "frames.camera_frame": "camera_color_optical_frame",
            "backend.type": "precomputed",
            "backend.precomputed.directory": "",
            "backend.precomputed.manifest": "manifest.yaml",
            "backend.precomputed.max_time_error_sec": 0.05,
            "backend.precomputed.default_confidence": 1.0,
            "backend.precomputed.unknown_confidence": 0.0,
            "backend.precomputed.cache_size": 8,
            "backend.segformer.engine": f"{model_dir}/model_fp16.engine",
            "backend.segformer.model_config": f"{model_dir}/config.json",
            "backend.segformer.preprocessor_config": (
                f"{model_dir}/preprocessor_config.json"
            ),
            "backend.segformer.label_mapping": "",
            "backend.segformer.min_confidence": 0.35,
            "semantic_classes.file": "",
            "processing.max_rate_hz": 5.0,
            "processing.publish_once": False,
            "validation.require_frame_id": True,
            "visualization.alpha": 0.55,
            "diagnostics.interval_sec": 5.0,
            "diagnostics.warning_interval_sec": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        def value(name: str) -> Any:
            return self.get_parameter(name).value

        self.rgb_topic = str(value("topics.rgb"))
        self.label_topic = str(value("topics.label"))
        self.confidence_topic = str(value("topics.confidence"))
        self.visualization_topic = str(value("topics.visualization"))
        self.class_metadata_topic = str(value("topics.class_metadata"))
        self.camera_frame = str(value("frames.camera_frame"))
        self.backend_type = str(value("backend.type"))
        self.precomputed_directory = str(value("backend.precomputed.directory"))
        self.precomputed_manifest = str(value("backend.precomputed.manifest"))
        self.max_time_error_sec = float(
            value("backend.precomputed.max_time_error_sec")
        )
        self.default_confidence = float(
            value("backend.precomputed.default_confidence")
        )
        self.unknown_confidence = float(
            value("backend.precomputed.unknown_confidence")
        )
        self.cache_size = int(value("backend.precomputed.cache_size"))
        self.segformer_engine = str(value("backend.segformer.engine"))
        self.segformer_model_config = str(value("backend.segformer.model_config"))
        self.segformer_preprocessor_config = str(
            value("backend.segformer.preprocessor_config")
        )
        segformer_label_mapping = str(value("backend.segformer.label_mapping"))
        if not segformer_label_mapping:
            segformer_label_mapping = str(
                Path(get_package_share_directory("semantic_mapping"))
                / "config"
                / "ade20k_navigation_mapping.yaml"
            )
        self.segformer_label_mapping = segformer_label_mapping
        self.segformer_min_confidence = float(
            value("backend.segformer.min_confidence")
        )
        semantic_classes_file = str(value("semantic_classes.file"))
        if not semantic_classes_file:
            semantic_classes_file = str(
                Path(get_package_share_directory("semantic_mapping"))
                / "config"
                / "semantic_classes.yaml"
            )
        self.semantic_classes_file = semantic_classes_file
        self.max_rate_hz = float(value("processing.max_rate_hz"))
        self.publish_once = bool(value("processing.publish_once"))
        self.require_frame_id = bool(value("validation.require_frame_id"))
        self.visualization_alpha = float(value("visualization.alpha"))
        self.diagnostics_interval_sec = float(value("diagnostics.interval_sec"))
        self.warning_interval_sec = float(value("diagnostics.warning_interval_sec"))

        if self.backend_type not in {"precomputed", "segformer_tensorrt"}:
            raise ValueError(
                "Supported semantic backends are 'precomputed' and "
                "'segformer_tensorrt'; "
                f"received {self.backend_type!r}"
            )
        if self.backend_type == "precomputed" and not self.precomputed_directory:
            raise ValueError("backend.precomputed.directory must be configured")
        if self.backend_type == "segformer_tensorrt":
            required_paths = {
                "backend.segformer.engine": self.segformer_engine,
                "backend.segformer.model_config": self.segformer_model_config,
                "backend.segformer.preprocessor_config": (
                    self.segformer_preprocessor_config
                ),
                "backend.segformer.label_mapping": self.segformer_label_mapping,
            }
            missing = [
                name
                for name, path in required_paths.items()
                if not Path(path).expanduser().is_file()
            ]
            if missing:
                raise ValueError(
                    "Missing SegFormer TensorRT files: " + ", ".join(missing)
                )
            if not 0.0 <= self.segformer_min_confidence <= 1.0:
                raise ValueError("backend.segformer.min_confidence must be in [0, 1]")
        if self.max_time_error_sec < 0.0:
            raise ValueError("backend.precomputed.max_time_error_sec is invalid")
        if self.max_rate_hz < 0.0:
            raise ValueError("processing.max_rate_hz must be non-negative")
        if not 0.0 <= self.visualization_alpha <= 1.0:
            raise ValueError("visualization.alpha must be in [0, 1]")
        if self.diagnostics_interval_sec <= 0.0 or self.warning_interval_sec <= 0.0:
            raise ValueError("Diagnostic intervals must be positive")

    def _create_backend(self) -> SemanticBackend:
        if self.backend_type == "precomputed":
            return PrecomputedMaskBackend(
                self.precomputed_directory,
                self.schema,
                manifest_name=self.precomputed_manifest,
                max_time_error_ns=int(round(self.max_time_error_sec * 1e9)),
                default_confidence=self.default_confidence,
                unknown_confidence=self.unknown_confidence,
                cache_size=self.cache_size,
            )
        from semantic_mapping.segformer_tensorrt_backend import (
            SegformerTensorRtBackend,
        )

        return SegformerTensorRtBackend(
            self.segformer_engine,
            self.segformer_model_config,
            self.segformer_preprocessor_config,
            self.segformer_label_mapping,
            self.schema,
            min_confidence=self.segformer_min_confidence,
        )

    def _rgb_callback(self, message: Image) -> None:
        self.received_frames += 1
        now = time.monotonic()
        if self.first_receive_monotonic is None:
            self.first_receive_monotonic = now
        if self.publish_once and self.published_once:
            return
        if self.require_frame_id and not message.header.frame_id:
            self.invalid_frames += 1
            self._warn("Dropping RGB image with an empty frame_id")
            return
        if (
            self.require_frame_id
            and self.camera_frame
            and message.header.frame_id != self.camera_frame
        ):
            self.invalid_frames += 1
            self._warn(
                "Dropping RGB image with unexpected frame_id: "
                f"{message.header.frame_id!r}, expected {self.camera_frame!r}"
            )
            return

        timestamp_ns = stamp_to_nanoseconds(message.header.stamp)
        try:
            self.backend.validate_timestamp(timestamp_ns)
        except SemanticFrameUnavailable as error:
            self.unavailable_frames += 1
            self._warn(f"No timestamp-matched semantic mask: {error}")
            return
        if self._rate_limited(timestamp_ns):
            self.rate_limited_frames += 1
            return
        self.last_attempt_stamp_ns = timestamp_ns

        start = time.monotonic()
        try:
            rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
            semantic_frame = self.backend.infer(rgb, timestamp_ns)
            overlay = blend_semantic_overlay(
                rgb,
                semantic_frame.label_image,
                semantic_frame.confidence_image,
                self.schema,
                self.visualization_alpha,
            )
        except SemanticFrameUnavailable as error:
            self.unavailable_frames += 1
            self._warn(f"No timestamp-matched semantic mask: {error}")
            return
        except (CvBridgeError, ValueError) as error:
            self.invalid_frames += 1
            self._warn(f"Dropping invalid semantic input: {error}")
            return

        label_message = self.bridge.cv2_to_imgmsg(
            semantic_frame.label_image, encoding="mono8"
        )
        confidence_message = self.bridge.cv2_to_imgmsg(
            semantic_frame.confidence_image, encoding="32FC1"
        )
        visualization_message = self.bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        for output in (label_message, confidence_message, visualization_message):
            output.header = message.header
        self.label_publisher.publish(label_message)
        self.confidence_publisher.publish(confidence_message)
        self.visualization_publisher.publish(visualization_message)

        elapsed = time.monotonic() - start
        source_timestamp_ns = semantic_frame.source_timestamp_ns
        source_error_sec = (
            0.0
            if source_timestamp_ns is None
            else abs(source_timestamp_ns - timestamp_ns) * 1e-9
        )
        self.processed_frames += 1
        self.total_inference_sec += elapsed
        self.total_source_time_error_sec += source_error_sec
        self.max_source_time_error_sec = max(
            self.max_source_time_error_sec, source_error_sec
        )
        self.published_once = True

    def _rate_limited(self, timestamp_ns: int) -> bool:
        if self.max_rate_hz <= 0.0 or self.last_attempt_stamp_ns is None:
            return False
        elapsed_ns = timestamp_ns - self.last_attempt_stamp_ns
        return 0 <= elapsed_ns < int(1e9 / self.max_rate_hz)

    def _publish_class_metadata(self) -> None:
        metadata = self.schema.to_metadata()
        metadata["backend"] = self.backend_type
        message = String()
        message.data = json.dumps(metadata, sort_keys=True, separators=(",", ":"))
        self.metadata_publisher.publish(message)

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if (
            self.last_warning_monotonic is None
            or now - self.last_warning_monotonic >= self.warning_interval_sec
        ):
            self.get_logger().warning(message)
            self.last_warning_monotonic = now

    def _log_diagnostics(self) -> None:
        runtime_sec = (
            0.0
            if self.first_receive_monotonic is None
            else time.monotonic() - self.first_receive_monotonic
        )
        input_fps = self.received_frames / runtime_sec if runtime_sec > 0.0 else 0.0
        output_fps = self.processed_frames / runtime_sec if runtime_sec > 0.0 else 0.0
        mean_inference_ms = (
            1000.0 * self.total_inference_sec / self.processed_frames
            if self.processed_frames
            else 0.0
        )
        mean_time_error_ms = (
            1000.0 * self.total_source_time_error_sec / self.processed_frames
            if self.processed_frames
            else 0.0
        )
        self.get_logger().info(
            "Semantic perception diagnostics: "
            f"rgb_fps={input_fps:.2f}, output_fps={output_fps:.2f}, "
            f"processed={self.processed_frames}, unavailable={self.unavailable_frames}, "
            f"invalid={self.invalid_frames}, rate_limited={self.rate_limited_frames}, "
            f"mean_inference_ms={mean_inference_ms:.2f}, "
            f"mean_mask_time_error_ms={mean_time_error_ms:.3f}, "
            f"max_mask_time_error_ms={self.max_source_time_error_sec * 1e3:.3f}"
        )

    def destroy_node(self) -> bool:
        self.backend.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the semantic perception ROS node."""
    rclpy.init(args=args)
    node = SemanticPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
