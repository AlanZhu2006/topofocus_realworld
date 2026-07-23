#!/usr/bin/env python3
"""Republish one ROS Image stream under a verified equivalent frame alias.

TinyNav's stereo keyframe image is expressed in the left-infrared pixel
geometry.  The patched perception publisher labels that image with the
descriptive RealSense frame ``camera_infra1_optical_frame``, while TinyNav's
keyframe depth, CameraInfo, and pose use its historical equivalent alias
``camera``.  The semantic geometry node intentionally requires one identical
frame label for all three inputs.

This deployment bridge changes only the Image header after checking the exact
source frame, dimensions, and encoding.  It never resamples pixels and drops
anything outside the approved contract.
"""
from __future__ import annotations

import argparse
from typing import Sequence


def validate_image_contract(
    *,
    frame_id: str,
    width: int,
    height: int,
    encoding: str,
    expected_frame: str,
    expected_width: int,
    expected_height: int,
    expected_encoding: str,
) -> str | None:
    """Return a rejection reason, or ``None`` when the alias is safe."""
    if frame_id != expected_frame:
        return f"frame_id={frame_id!r}, expected {expected_frame!r}"
    if (width, height) != (expected_width, expected_height):
        return (
            f"dimensions={width}x{height}, expected "
            f"{expected_width}x{expected_height}"
        )
    if encoding != expected_encoding:
        return f"encoding={encoding!r}, expected {expected_encoding!r}"
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--source-frame", required=True)
    parser.add_argument("--target-frame", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--encoding", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.input_topic.startswith("/") or not args.output_topic.startswith("/"):
        raise SystemExit("input and output topics must be absolute")
    if args.input_topic == args.output_topic:
        raise SystemExit("input and output topics must differ")
    if not args.source_frame or not args.target_frame:
        raise SystemExit("source and target frames must not be empty")
    if args.source_frame == args.target_frame:
        raise SystemExit("source and target frames must differ")
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("width and height must be positive")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header

    class ImageFrameAliasNode(Node):
        def __init__(self) -> None:
            super().__init__("focus_image_frame_alias")
            self.received = 0
            self.published = 0
            self.dropped = 0
            self.publisher = self.create_publisher(
                Image, args.output_topic, qos_profile_sensor_data
            )
            self.subscription = self.create_subscription(
                Image,
                args.input_topic,
                self._callback,
                qos_profile_sensor_data,
            )
            self.timer = self.create_timer(5.0, self._diagnostics)
            self.get_logger().info(
                "Strict Image frame alias ready: "
                f"{args.input_topic} [{args.source_frame}] -> "
                f"{args.output_topic} [{args.target_frame}], "
                f"{args.width}x{args.height} {args.encoding}"
            )

        def _callback(self, message: Image) -> None:
            self.received += 1
            reason = validate_image_contract(
                frame_id=message.header.frame_id,
                width=message.width,
                height=message.height,
                encoding=message.encoding,
                expected_frame=args.source_frame,
                expected_width=args.width,
                expected_height=args.height,
                expected_encoding=args.encoding,
            )
            if reason is not None:
                self.dropped += 1
                self.get_logger().error(
                    f"Rejected Image outside approved alias contract: {reason}",
                    throttle_duration_sec=5.0,
                )
                return
            output = Image(
                header=Header(
                    stamp=message.header.stamp,
                    frame_id=args.target_frame,
                ),
                height=message.height,
                width=message.width,
                encoding=message.encoding,
                is_bigendian=message.is_bigendian,
                step=message.step,
                data=message.data,
            )
            self.publisher.publish(output)
            self.published += 1
            if self.published == 1:
                self.get_logger().info(
                    "Published first verified frame alias without pixel resampling"
                )

        def _diagnostics(self) -> None:
            self.get_logger().info(
                "Image frame alias diagnostics: "
                f"received={self.received}, published={self.published}, "
                f"dropped={self.dropped}"
            )

    rclpy.init(args=None)
    node = ImageFrameAliasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
