#!/usr/bin/env python3
"""Navigation-focused adapter for the archived TinyNav occupancy mapper.

The provenance snapshot remains byte-for-byte unchanged.  This deployment
adapter replaces two runtime-only policies:

* adjacent-pose jump detection is bounded by elapsed source time and the
  platform's physical speed, so a slow callback cannot classify ordinary
  motion as relocalization;
* the authoritative 2-D navigation grid is published without constructing
  and sorting a full 3-D visualization cloud or recounting every sparse voxel
  on every update.

The node has no target, velocity, SDK, or chassis interface.
"""
from __future__ import annotations

import json
from pathlib import Path
import resource
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.base_camera_calibration import load_base_camera_calibration
from focus_hub.geometry import compose_rigid, invert_rigid
from focus_hub.navigation_occupancy import clear_current_footprint
from focus_hub.rate_aware_keyframes import RateAwareKeyframeSelector
from semantic_mapping import occupancy_mapper_node as source_mapper


class DeploymentKeyframeSelector(RateAwareKeyframeSelector):
    """Bind the selector to the guarded robots' physical velocity envelope."""

    def __init__(self, config: object) -> None:
        super().__init__(
            config,
            maximum_translation_speed_mps=0.25,
            translation_margin_m=0.20,
            maximum_dynamic_translation_m=1.50,
            maximum_rotation_speed_degps=30.0,
            rotation_margin_deg=15.0,
            maximum_dynamic_rotation_deg=150.0,
        )


# OccupancyMapperNode resolves this module global when its constructor runs.
# Replacing it here adapts deployment behavior without changing the archived
# semantic_mapping source or its checksummed provenance files.
source_mapper.KeyframeSelector = DeploymentKeyframeSelector


class NavigationOccupancyMapper(source_mapper.OccupancyMapperNode):
    """Publish planner geometry without unbounded visualization-side work."""

    def __init__(self) -> None:
        self._latest_target_T_camera: np.ndarray | None = None
        self._last_cleared_footprint_cells = 0
        self._base_camera_calibration = None
        self._footprint_shape = ""
        self._footprint_front_m = 0.0
        self._footprint_rear_m = 0.0
        self._footprint_half_width_m = 0.0
        self._footprint_radius_m = 0.0
        super().__init__()
        self.declare_parameter("navigation.robot_id", "")
        self.declare_parameter("navigation.camera_frame", "")
        self.declare_parameter("navigation.base_camera_calibration_file", "")
        self.declare_parameter("navigation.footprint_shape", "")
        self.declare_parameter("navigation.footprint_front_m", 0.0)
        self.declare_parameter("navigation.footprint_rear_m", 0.0)
        self.declare_parameter("navigation.footprint_half_width_m", 0.0)
        self.declare_parameter("navigation.footprint_radius_m", 0.0)
        robot_id = str(self.get_parameter("navigation.robot_id").value)
        camera_frame = str(
            self.get_parameter("navigation.camera_frame").value
        )
        calibration_file = str(
            self.get_parameter(
                "navigation.base_camera_calibration_file"
            ).value
        )
        self._footprint_shape = str(
            self.get_parameter("navigation.footprint_shape").value
        )
        self._footprint_front_m = float(
            self.get_parameter("navigation.footprint_front_m").value
        )
        self._footprint_rear_m = float(
            self.get_parameter("navigation.footprint_rear_m").value
        )
        self._footprint_half_width_m = float(
            self.get_parameter("navigation.footprint_half_width_m").value
        )
        self._footprint_radius_m = float(
            self.get_parameter("navigation.footprint_radius_m").value
        )
        if not robot_id or not camera_frame or not calibration_file:
            raise ValueError(
                "navigation occupancy requires robot, camera and measured "
                "base-camera calibration parameters"
            )
        self._base_camera_calibration = load_base_camera_calibration(
            Path(calibration_file),
            expected_robot_id=robot_id,
            expected_camera_frame=camera_frame,
        )
        if self._footprint_shape not in {"rectangle", "circle"}:
            raise ValueError(
                "navigation.footprint_shape must be rectangle or circle"
            )

    def _synchronized_callback(
        self, cloud_message: object, pose_message: object
    ) -> None:
        if (
            cloud_message.header.frame_id == self.target_frame
            and pose_message.header.frame_id == self.target_frame
        ):
            position = pose_message.pose.position
            orientation = pose_message.pose.orientation
            try:
                self._latest_target_T_camera = (
                    source_mapper.make_transform_matrix(
                        [position.x, position.y, position.z],
                        [
                            orientation.x,
                            orientation.y,
                            orientation.z,
                            orientation.w,
                        ],
                    )
                )
            except ValueError:
                pass
        super()._synchronized_callback(cloud_message, pose_message)

    def _publish_map(self, stamp: object) -> None:
        start = time.perf_counter()
        source_bev = source_mapper.project_occupancy_to_bev(
            self.voxel_map, self.bev_config
        )
        bev = source_bev
        self._last_cleared_footprint_cells = 0
        if (
            self._latest_target_T_camera is not None
            and self._base_camera_calibration is not None
        ):
            target_T_base = np.asarray(
                compose_rigid(
                    tuple(self._latest_target_T_camera.reshape(-1)),
                    invert_rigid(self._base_camera_calibration.matrix),
                ),
                dtype=np.float64,
            ).reshape(4, 4)
            bev, self._last_cleared_footprint_cells = (
                clear_current_footprint(
                    source_bev,
                    target_T_base,
                    shape=self._footprint_shape,
                    front_m=self._footprint_front_m,
                    rear_m=self._footprint_rear_m,
                    half_width_m=self._footprint_half_width_m,
                    radius_m=self._footprint_radius_m,
                )
            )
        self.total_bev_sec += time.perf_counter() - start
        self.bev_updates += 1
        self.last_bev_monotonic = time.monotonic()
        # Preserve the sensor-derived map for serialization. The padded,
        # current-footprint-cleared copy exists only on navigation topics.
        self.last_bev = source_bev
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
            source_mapper._image_message(
                bev.occupancy_probability, "32FC1", header
            )
        )
        self.free_probability_publisher.publish(
            source_mapper._image_message(bev.free_probability, "32FC1", header)
        )
        self.explored_publisher.publish(
            source_mapper._image_message(
                bev.explored * np.uint8(255), "mono8", header
            )
        )
        self.height_max_publisher.publish(
            source_mapper._image_message(bev.height_max, "32FC1", header)
        )

        grid = bev.occupancy_grid
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
            "active_voxels": len(self.voxel_map),
            "free_cells": int(np.count_nonzero(grid == 0)),
            "occupied_cells": int(np.count_nonzero(grid > 0)),
            "unknown_cells": int(np.count_nonzero(grid < 0)),
            "processed_keyframes": self.processed_keyframes,
            "ground_estimation_enabled": self.ground_estimation_enabled,
            "ground_estimation_attempts": self.ground_estimation_attempts,
            "ground_estimation_accepted": (
                self.ground_estimator.accepted_updates
            ),
            "ground_estimation_rejected": (
                self.ground_estimator.rejected_updates
            ),
            "ground_estimation_mode": self.ground_estimator.mode,
            "runtime_adapter": "focus-navigation-occupancy-v1",
            "occupied_voxel_visualization": "disabled_unbounded_work",
            "current_footprint_policy": (
                "measured_base_camera_source_footprint_clear"
            ),
            "current_footprint_shape": self._footprint_shape,
            "current_footprint_cleared_cells": (
                self._last_cleared_footprint_cells
            ),
            "source_bev_origin_xy": source_bev.origin_xy.tolist(),
            "source_bev_width": source_bev.width,
            "source_bev_height": source_bev.height,
        }
        self.metadata_publisher.publish(
            String(data=json.dumps(metadata, sort_keys=True))
        )

    def _log_diagnostics(self) -> None:
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
            "Navigation occupancy diagnostics: "
            f"input_hz={input_hz:.2f}, pairs={self.received_pairs}, "
            f"keyframes={self.processed_keyframes}, "
            f"skipped={self.skipped_non_keyframes}, "
            f"dropped={self.dropped_frames}, "
            f"pose_jumps={self.pose_jump_events}, "
            f"rays={self.total_integrated_rays}, "
            f"active_voxels={len(self.voxel_map)}, "
            f"free_updates={self.total_unique_free_updates}, "
            f"occupied_updates={self.total_unique_occupied_updates}, "
            f"integration_ms={integration_ms:.2f}, bev={bev_size}, "
            f"bev_ms={bev_ms:.2f}, ground_z={self.ground_z:.3f}, "
            f"ground={ground_status}, ground_fit={ground_fit}, "
            f"ground_mode={self.ground_estimator.mode}, "
            f"ground_updates={self.ground_estimator.accepted_updates}/"
            f"{self.ground_estimation_attempts}, rss_mb={rss_mb:.1f}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationOccupancyMapper()
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
