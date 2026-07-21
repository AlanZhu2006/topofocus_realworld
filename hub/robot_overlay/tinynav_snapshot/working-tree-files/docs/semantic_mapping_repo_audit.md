# Semantic Mapping Repository Audit

Audit date: 2026-07-17

Scope: phase 0/1 audit for a TinyNav-pose-conditioned RGB-D mapper. This
document records what is supported by repository code, what was measured from
a local bag, and what still requires a live robot check. It does not redefine
the TinyNav SLAM, planning, or control contracts.

## Repository State

- Repository: the local Go2 deployment branch based on `UniflexAI/tinynav`.
- Current commit: `933fce5` (`complete deployment`).
- Branch: `main`, two commits ahead of `origin/main`.
- Separate remote: `go2_tinynav` points to `AlanZhu2006/go2_tinynav`.
- Pre-existing dirty file: `scripts/run_go2_cmd_bridge.sh`. It is unrelated to
  semantic mapping and is not touched by this work.
- Python target: 3.10, ROS 2 Humble, RealSense ROS from the local
  `realsense_ws`, and TinyNav TensorRT/GTSAM dependencies.

## Files Inspected

The audit covered the requested integration surfaces and their direct
dependencies:

- `README.md`, `DEPLOYMENT.md`, `docs/README.md`, and both RViz configs.
- `scripts/tinynav_auto_map.sh`, `scripts/tinynav_auto_nav.sh`.
- `scripts/run_realsense_sensor.sh`, `scripts/run_rosbag_record.sh`.
- `scripts/run_rosbag_build_map.sh`, `scripts/run_navigation.sh`.
- `scripts/tinynav_baseline_build_map.sh`,
  `scripts/tinynav_baseline_nav.sh`.
- `tool/static_occupancy_grid_publisher.py`.
- `tool/map_keyframe_publisher.py`, `tool/current_pose_marker.py`.
- `tool/rviz_goal_to_poi.py`, `tool/validate_tinynav_bag.py`.
- `tool/global_pointcloud_publisher.py` and map conversion utilities.
- `tinynav/core/perception_node.py`, `imu_propagator_node.py`.
- `tinynav/core/build_map_node.py`, `map_node.py`, `planning_node.py`.
- `tinynav/core/math_utils.py` and the Go2 command/control wrappers.

## Existing Runtime Chain

`scripts/tinynav_auto_nav.sh` keeps these processes alive during online
navigation:

1. RealSense driver, unless an existing `/camera/camera` node passes its stream
   checks.
2. `perception_node.py` for stereo depth and visual-inertial odometry.
3. `planning_node.py` for the rolling 3D grid, ESDF, and local trajectory.
4. `map_node.py` for relocalization, saved-map alignment, and global planning.
5. `cmd_vel_control.py` for trajectory following.
6. `go2_cmd_bridge.py` for `/cmd_vel` to Unitree `SportClient.Move()`.
7. Goal, static-map, keyframe, current-pose, and RViz helper nodes.

The original auto-map/auto-nav scripts remain unchanged. Their semantic copies
start the mapper as a separate process/window, so its failure does not tear
down Go2 control:

```text
scripts/tinynav_semantic_auto_map.sh
scripts/tinynav_semantic_auto_nav.sh
```

## Confirmed Sensor Inputs

The following values were checked both in code and in the complete local bag
`$HOME/.local/share/tinynav/rosbags/map_record_20260707_120552` where noted.

| Input | Topic | Type / encoding | Frame | Status |
|---|---|---|---|---|
| RGB | `/camera/camera/color/image_raw` | `sensor_msgs/Image`, `rgb8`, 848x480 | `camera_color_optical_frame` | Confirmed in bag |
| Raw depth | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image`, `16UC1` millimeters, 848x480 | `camera_depth_optical_frame` | Confirmed in bag |
| Aligned depth | `/camera/camera/aligned_depth_to_color/image_raw` | `16UC1`, 848x480 | `camera_color_optical_frame` | Confirmed live at about 30 Hz |
| Aligned depth info | `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | `camera_color_optical_frame` | Confirmed live at about 30 Hz |
| RGB info | `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `camera_color_optical_frame` | Confirmed in bag |
| Infrared left | `/camera/camera/infra1/image_rect_raw` | `mono8`, 848x480 | `camera_infra1_optical_frame` | Confirmed in bag |
| Infrared right | `/camera/camera/infra2/image_rect_raw` | `mono8`, 848x480 | its `CameraInfo` is expressed in the left optical frame | Confirmed in bag |
| IMU | `/camera/camera/imu` | `sensor_msgs/Imu` | RealSense IMU frame | Confirmed in bag |

Measured RGB intrinsics:

```text
fx=605.5551758 fy=605.7315674 cx=418.1565247 cy=255.6905060
```

Measured infrared/depth stereo intrinsics:

```text
fx=428.7664185 fy=428.7664185 cx=426.3483887 cy=243.3571777
```

The different intrinsics and optical frames prove that raw depth cannot be
indexed with RGB pixels. Similar timestamps do not make the images registered.

### Depth alignment status

The original `scripts/run_realsense_sensor.sh` remains unchanged and does not
enable alignment. The isolated `scripts/run_realsense_semantic_sensor.sh`
enables both `align_depth.enable` and `enable_sync`. The installed driver and
D435i were tested live with that entry point.

The installed RealSense driver explicitly publishes these topics when alignment
is enabled:

```text
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
```

The semantic point-cloud node treats an RGB-sized, RGB-optical-frame aligned
depth image as a required input and rejects shape/frame mismatches.

### TinyNav-generated depth

`perception_node.py` also publishes `/slam/depth` as `32FC1` meters with the
left-infrared timestamp. This is neural stereo depth in the left infrared
optical geometry, even though the message frame is the TinyNav alias `camera`.
It is suitable for TinyNav planning, but it is not RGB-aligned depth.

## Time Synchronization

The complete local bag is 118.864 seconds, contains 55,805 messages, and has
3,564 RGB frames plus 3,563 raw depth frames. Nearest RGB-to-depth timestamp
differences were:

```text
minimum: 0.000015259 s
median:  0.000018120 s
maximum: 0.033338070 s
<=1 ms: 3563 / 3564
```

The approximately 33 ms outlier is the first/startup frame. Online semantic
mapping should use approximate RGB/depth/CameraInfo synchronization with a
small bounded slop, then perform the pose lookup at the image timestamp. It
must not use the callback wall-clock time.

## TinyNav Pose Outputs

| Topic / TF | Timestamp | Meaning | Suitability |
|---|---|---|---|
| `/slam/odometry_visual` | exact processed left-infrared image stamp | `T_world_camera`; visual/IMU optimized pose | Good exact pose sample |
| TF `world -> camera` | same image stamp as visual odometry | same `T_world_camera` | Preferred TF pose sample |
| `/slam/odometry` | IMU message stamp | propagated `T_world_camera` | Good high-rate pose buffer input |
| `/slam/keyframe_odom` | exact keyframe image/depth stamp | keyframe `T_world_camera` | Good keyframe-only input |
| `/map/relocalization` | keyframe stamp | successful camera pose in the saved map coordinate system, published with frame `world` | Sparse correction observation |
| TF `world -> map` | node `now`, after successful relocalization | saved-map frame represented in current odometry world | Required map alignment, but timestamp semantics need improvement |
| `/mapping/current_pose_in_map` | node `now` | `inv(T_world_map) * T_world_camera` for a keyframe | Not a primary pose source |

`/mapping/current_pose_in_map` is currently only published from the global-path
code path after a POI is active. Its timestamp is publication time rather than
the camera image time, and its `Odometry` header/child IDs are not a clean
`map -> camera` contract. Semantic mapping must not depend on this topic.

## Frames and TF

Confirmed TinyNav frames:

- `world`: the live TinyNav odometry frame. Gravity initialization intends
  world Z to be up.
- `camera`: a TinyNav alias for the left infrared optical camera coordinates.
  Backprojection in core code uses optical axes: X right, Y down, Z forward.
- `map`: the saved mapping-session coordinate system used during online
  relocalization.
- `camera_link`: the RealSense rig root, not the Unitree body frame.
- `camera_color_optical_frame`: RGB optical frame.
- `camera_depth_optical_frame`: raw depth optical frame.
- `camera_infra1_optical_frame`: left infrared optical frame.

No `odom` frame, `base_link` frame, or calibrated `base_link -> camera_link`
transform is published by this repository. Go2 geometry is instead encoded as
a camera-relative offset in `planning_node.py` (`camera_x=0.2 m`). A real robot
URDF/static transform is still needed before ground height or body clearance
can be considered calibrated.

The existing online TF graph is conceptually:

```text
                  -> camera             (TinyNav dynamic)
world
                  -> map                (map_node dynamic)

camera_link -> camera_*_frame -> camera_*_optical_frame
```

The TinyNav `camera` alias is not physically connected to the RealSense static
tree. For RGB points the mapper must compose:

```text
T_map_color = inverse(T_world_map) * T_world_camera * T_infra1_color
```

`T_infra1_color` comes from the RealSense static TF tree while the equality
between TinyNav `camera` and `camera_infra1_optical_frame` is an explicit,
documented alias. No XYZ axis permutation is performed.

### Recommended map-to-camera acquisition

1. Query the timestamped TinyNav TF pose at the RGB/depth stamp.
2. Query/retain the RealSense static color-to-infrared optical extrinsic.
3. Compose into the requested output frame.
4. Reject the frame if any dynamic transform cannot be resolved within the
   configured timeout/error bound.

Direct `lookup_transform("map", "camera", image_stamp)` is attempted first.
Current `world -> map` publication is sparse and uses `now`, so the implemented
fallback composes the latest map-alignment edge with an exact image-time
`world -> camera` lookup. Only the frame alignment may be latest; the moving
camera pose may not. Failures remain visible and the frame is dropped after a
bounded 2.0 second buffer.

## Existing Occupancy Maps

### Static map

`build_map_node.py` creates a dense 3D grid with hard-coded resolution 0.1 m
and values:

```text
0 unknown
1 free
2 occupied
```

The current completed map `output/map_record_20260707_120552` has:

```text
shape:      115 x 158 x 30
origin:     [-5.5, -6.1, -1.9]
resolution: 0.1 m
```

`tool/static_occupancy_grid_publisher.py` collapses the full Z column with
`max`, publishes `/mapping/static_occupancy_grid` in `map`, and uses transient
local reliable QoS. This projection is useful for alignment checks but is not
height aware.

### Local planning map

`planning_node.py` maintains a rolling 100x100x10 grid at 0.1 m, integrates
`/slam/depth` with pose-synchronized ray casting, and publishes local products
in `world`. This grid decays over time and must not be repurposed as the
persistent semantic map.

## QoS Audit

- RealSense bag image publishers were recorded as reliable/volatile; IMU is
  best-effort/volatile; static TF is reliable/transient-local.
- `perception_node.py` uses best effort for IMU and default depth-10 ROS QoS for
  stereo images and CameraInfo.
- TinyNav pose/depth publishers use default depth-10 QoS; continuous odometry
  publisher depth is 50.
- Map helpers use reliable/transient-local QoS for static map and markers.
- The new RGB-D synchronizer uses sensor-data best effort so it can connect to
  either reliable or best-effort camera publishers. Its point cloud uses
  reliable depth-1 QoS.

## Local Rosbag Assessment

Usable existing bag:

```text
$HOME/.local/share/tinynav/rosbags/map_record_20260707_120552
```

It is valid for replaying TinyNav perception and rebuilding its map. It is not
yet a complete semantic mapping bag because it lacks:

- aligned depth-to-color image and CameraInfo;
- `/tf` dynamic TinyNav poses;
- `/slam/odometry*` pose topics;
- `/mapping/current_pose_in_map`.

Those poses can be regenerated while replaying the raw sensor bag, but the
aligned RealSense depth cannot be reconstructed by merely renaming the raw
topic. A new bag should be recorded after alignment is enabled, preferably
while TinyNav is running so both raw sensor and pose/TF streams are retained.

A new minimal posed RGB-D bag was recorded and replayed successfully:

```text
/run/user/1000/tinynav_semantic_phase1_20260717_080342
```

It is 5.67 seconds / 330 MiB and contains 169 RGB frames, 170 aligned-depth
frames, 171 aligned CameraInfo messages, 18 visual poses, 94 propagated poses,
25 dynamic TF messages, and one static TF message. It lives on tmpfs because
the root filesystem had only about 19 MiB free, so it must be moved to durable
storage before reboot.

## Integration Points

- Sensor boundary: RGB, aligned depth, and RGB/aligned CameraInfo from
  RealSense.
- Pose boundary: timestamped TinyNav TF, with a future odometry-buffer fallback
  using translation interpolation and quaternion SLERP.
- Validation boundary: overlay the semantic mapper occupancy BEV with
  `/mapping/static_occupancy_grid` without importing it as the 3D geometry.
- Runtime boundary: a separate ament-Python package and process; no dependency
  from planning/control back to semantic mapping in phase 1.
- Persistence boundary: future semantic keyframes retain input timestamps,
  intrinsics, pose, labels/confidence, and depth references so a corrected map
  can be rebuilt after loop closure.

## Confirmed vs Uncertain

Confirmed:

- Topic names, encodings, image sizes, and camera intrinsics above.
- TinyNav odometry and TF pose publication uses the source image stamp.
- TinyNav camera-space math follows ROS optical axes.
- Static-map and local-planning resolutions are currently 0.1 m.
- The existing bag is available and repeatable for TinyNav replay.
- No robot `base_link` TF exists in this repository.
- The D435i publishes aligned depth and aligned CameraInfo in the RGB optical
  frame at about 30 Hz with the semantic sensor script.
- Live backprojection publishes about 100.8k points per frame at about 3 Hz on
  the current Jetson workload, with no post-startup alignment drops.
- The minimal posed bag reproduces the point cloud without live RealSense or
  TinyNav perception.

Still uncertain and requiring validation at a mapped location:

- Long-run `map <- camera` behavior through repeated relocalization updates.
- The calibrated Go2 body-to-camera transform and camera mounting height.
- The numerical ground Z in a saved map after arbitrary mapping starts.
- Overlay alignment against the saved TinyNav occupancy grid. The current
  robot location is outside the audited saved map, so relocalization was not
  expected during this test.
