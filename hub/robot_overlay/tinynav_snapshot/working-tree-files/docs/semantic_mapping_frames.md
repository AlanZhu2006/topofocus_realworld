# Semantic Mapping Frame Contract

This document uses `T_A_B` to mean a homogeneous transform that maps a point
from frame B into frame A:

```text
p_A = T_A_B * p_B
```

## Frame Inventory

| Frame | Owner | Axis convention | Notes |
|---|---|---|---|
| `world` | TinyNav perception | intended Z up | Live odometry frame; resets between sessions |
| `map` | TinyNav map node | saved mapping-session coordinates | Available only after successful relocalization online |
| `camera` | TinyNav perception | optical: X right, Y down, Z forward | Alias for left-infrared optical pose |
| `camera_link` | RealSense driver | ROS camera rig/body axes | RealSense root, not Go2 `base_link` |
| `camera_infra1_optical_frame` | RealSense driver | optical | Physical equivalent of TinyNav `camera` coordinates |
| `camera_depth_optical_frame` | RealSense driver | optical | Raw depth image frame |
| `camera_color_optical_frame` | RealSense driver | optical | RGB and aligned-depth projection frame |
| `base_link` | absent | expected X forward, Y left, Z up | Must be calibrated/published in a later deployment step |
| `odom` | absent | n/a | TinyNav calls its live odometry frame `world` |

## Transform Sources

| Transform | Source | Time behavior |
|---|---|---|
| `T_world_camera` | `/slam/odometry_visual` and TF `world -> camera` | source image timestamp |
| high-rate `T_world_camera` | `/slam/odometry` | IMU timestamp |
| `T_world_map` | TF `world -> map`, published by `map_node.py` | publication `now`; updates after relocalization |
| RealSense optical extrinsics | `/tf_static` | static/transient local |
| `T_map_camera` | `inverse(T_world_map) * T_world_camera` | derived at image time |
| `T_infra1_color` | RealSense static TF composition | static |
| `T_map_color` | `T_map_camera * T_infra1_color` | desired RGB-D pose |
| published `T_target_color` | `/semantic_mapping/camera_pose` | same stamp and target frame as the posed point cloud |

## Camera Optical Axes

Backprojection uses the ROS optical-frame convention without manual axis
swaps:

```text
X = (u - cx) * Z / fx   # right
Y = (v - cy) * Z / fy   # down
Z = depth               # forward
```

The result is transformed only by calibrated SE(3) matrices. A hard-coded
`[x,z,y]` or sign flip is prohibited because it would hide a broken TF chain.

## Current TF Graph

TinyNav dynamic tree:

```text
world -> camera
world -> map
```

RealSense static tree, abbreviated:

```text
camera_link -> camera_depth_frame -> camera_depth_optical_frame
camera_link -> camera_color_frame -> camera_color_optical_frame
camera_link -> camera_infra1_frame / aligned-depth frame
```

The two trees are not connected by a published TF. The semantic mapper uses the
documented identity of TinyNav `camera` coordinates and the physical left IR
optical coordinates, then composes the RealSense color extrinsic separately.
This alias must be removed if TinyNav later changes its tracking camera.

## Map to RGB Camera

For online navigation after relocalization:

```text
T_map_color = inverse(T_world_map) * T_world_camera * T_infra1_color
```

The node first requests this full chain at the image stamp. Because TinyNav's
map alignment is published sparsely, it may fall back to:

```text
latest(T_map_world) * image_time(T_world_camera) * T_infra1_color
```

This does not substitute a latest camera pose. The only latest value is the
piecewise-constant saved-map alignment edge.

For initial offline map construction, no saved `map` frame exists yet. The
mapping-session `world` frame is the future saved-map coordinate system, so the
phase-1 node can be configured with `target_frame=world`:

```text
T_world_color = T_world_camera * T_infra1_color
```

The output frame ID must match the transform actually used. A point cloud
computed in `world` must never be relabeled as `map` without applying
`inverse(T_world_map)`.

The geometry front end publishes that exact transform as
`/semantic_mapping/camera_pose`. The Phase-2 occupancy node uses its translation
as the DDA ray origin and the accompanying point cloud as surface endpoints.
This prevents a second TF lookup from selecting a different map-alignment edge.

## RealSense Bag Measurements

The audited bag provides these static facts:

- RGB frame: `camera_color_optical_frame`.
- Raw depth frame: `camera_depth_optical_frame`.
- Infrared left frame: `camera_infra1_optical_frame`.
- Color origin is approximately 14.824 mm from `camera_link` along the rig Y
  direction, with a small calibrated rotation.
- Raw depth and RGB use different intrinsics; they are not pixel aligned.

The current RealSense TF message uses
`camera_aligned_depth_to_infra1_frame -> camera_infra1_optical_frame` rather
than the exact edge name expected by one path in `build_map_node.py`. Static TF
lookups should therefore use graph connectivity, not string-matching a fixed
set of edges.

## Go2 Body Frame Gap

The repository does not publish `base_link`. `planning_node.py` encodes a
camera-to-control-center offset of 0.2 m along camera optical Z, but that value
is planner geometry, not a full calibrated 6-DoF transform.

Before height-aware BEV is accepted on the robot, measure and publish:

```text
base_link -> camera_link
```

The measurement must include camera height, pitch, roll, and yaw. It will be
used to validate gravity direction, estimate ground height, and define Go2's
collision band.

## Frame Validation Procedure

1. Start RealSense, TinyNav perception, and (online) map localization.
2. Confirm `tf2_echo world camera` changes smoothly and its stamp tracks image
   time.
3. After relocalization, confirm `tf2_echo map camera` is available.
4. Confirm aligned depth and RGB have identical size, timestamp proximity,
   intrinsics, and `camera_color_optical_frame` geometry.
5. Display the semantic point cloud with RViz fixed frame `world` offline or
   `map` online.
6. Move the robot forward: static walls must remain fixed rather than follow
   the camera.
7. Overlay `/mapping/static_occupancy_grid`; reject any 90-degree rotation,
   mirror, or translation discontinuity.
8. Check a vertical wall and floor plane before enabling voxel integration.
9. Overlay `/semantic_mapping/occupancy_bev` and
   `/mapping/static_occupancy_grid` when genuine relocalization is available.

## Timestamp Behavior

`world -> map` is currently broadcast with node time rather than the keyframe
image stamp. Exact historical `map <- camera` lookup can therefore fail at
startup or around relocalization. The mapper reports direct lookup failures and
uses the latest map alignment only when enabled; `world -> camera` remains an
image-time lookup. On the tested Jetson, perception published valid
image-stamped poses about 0.4-0.8 seconds after the source images, so the node
buffers RGB-D for 2.0 seconds rather than using `now()`.
