#!/usr/bin/env python3
"""Run TinyNav BuildMapNode against live ROS topics without its BagPlayer.

This deployment wrapper intentionally has no navigation/control publisher. A
map is finalized only through TinyNav's existing ``/benchmark/stop`` callback;
the wrapper exits after ``BuildMapNode._save_completed`` becomes true. It
refuses an existing output path because TinyNav's scratch database constructor
removes same-named database files.
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from tinynav.core.build_map_node import BuildMapNode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-save-path", type=Path, required=True)
    parser.add_argument("--global-frames-ratio", type=float, default=1.1)
    parser.add_argument("--quiet-timers", action="store_true")
    args = parser.parse_args()

    output = args.map_save_path.expanduser().resolve()
    if output.exists():
        parser.error(f"refusing existing map output path: {output}")
    if args.global_frames_ratio < 1.0:
        parser.error("--global-frames-ratio must be >= 1.0")

    rclpy.init()
    node = None
    executor = SingleThreadedExecutor()
    completed = False
    try:
        node = BuildMapNode(
            str(output),
            verbose_timer=not args.quiet_timers,
            global_frames_ratio=args.global_frames_ratio,
        )
        executor.add_node(node)
        node.get_logger().info(
            "Live BuildMap gate ready; finalize with "
            "`ros2 topic pub --once /benchmark/stop std_msgs/msg/Bool '{data: true}'`"
        )
        while rclpy.ok() and not node._save_completed:
            executor.spin_once(timeout_sec=0.2)
        completed = bool(node._save_completed)
        if completed:
            node.get_logger().info("BuildMap finalized; wrapper exiting cleanly")
            return 0
        logging.error("ROS stopped before /benchmark/data_saved=true; map is not finalized")
        return 3
    except KeyboardInterrupt:
        logging.error("Interrupted before explicit BuildMap save; map is not finalized")
        return 130
    finally:
        if node is not None:
            executor.remove_node(node)
            if completed:
                # TinyNav's override sees _save_completed and skips duplicate work.
                node.destroy_node()
            else:
                # Bypass TinyNav's save-on-destroy override. Saving must happen while
                # the ROS callback/context is healthy via /benchmark/stop.
                Node.destroy_node(node)
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

