# Semantic Mapping Topic Contract

## Sensor Inputs Used by the Geometry Front End

| Parameter | Default topic | Type | Required condition |
|---|---|---|---|
| `topics.rgb` | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB/BGR 8-bit image |
| `topics.depth` | `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `16UC1` mm or `32FC1` m; RGB pixel geometry |
| `topics.camera_info` | `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics for aligned image |
| TF pose | `/tf`, `/tf_static` | TF2 | timestamped TinyNav pose plus RealSense static extrinsic |

RGB `CameraInfo` may be selected instead of aligned-depth `CameraInfo` because
both should describe the RGB projection after alignment. The node checks image
dimensions and configured frame IDs before publication.

## TinyNav Pose and Map Topics

| Topic | Type | Frame contract | Notes |
|---|---|---|---|
| `/slam/odometry_visual` | `nav_msgs/msg/Odometry` | `world -> camera` | processed image stamp |
| `/slam/odometry` | `nav_msgs/msg/Odometry` | `world -> camera` | high-rate IMU propagation |
| `/slam/keyframe_odom` | `nav_msgs/msg/Odometry` | `world -> camera` | exact keyframe stamp |
| `/slam/depth` | `sensor_msgs/msg/Image` | TinyNav `camera`, infra geometry | stereo depth, meters |
| `/slam/keyframe_depth` | `sensor_msgs/msg/Image` | TinyNav `camera`, infra geometry | exact keyframe stamp |
| `/map/relocalization` | `nav_msgs/msg/Odometry` | camera pose in saved-map coordinates | sparse success observations |
| `/mapping/current_pose_in_map` | `nav_msgs/msg/Odometry` | intended map camera pose | currently POI-gated and stamped at publish time |
| `/mapping/static_occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | `map` in auto-nav | saved map, 0.1 m currently |
| `/planning/occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | `world` | rolling local map |

## Geometry Front-End Outputs

| Topic | Type | QoS | Content |
|---|---|---|---|
| `/semantic_mapping/semantic_pointcloud` | `sensor_msgs/msg/PointCloud2` | reliable, keep-last 1 | map-frame XYZ/RGB points plus source RGB pixel `u`/`v` fields |
| `/semantic_mapping/camera_pose` | `geometry_msgs/msg/PoseStamped` | reliable, keep-last 1 | exact `T_target_camera_color` used for that cloud, with the same timestamp |

The semantic mapper uses `u`/`v` to index the synchronized label and confidence
images exactly. Consumers that only need XYZ/RGB may ignore those extra fields.

## Phase-3 Perception Outputs

| Topic | Type | QoS/content |
|---|---|---|
| `/semantic_mapping/semantic_label_image` | `sensor_msgs/msg/Image` | reliable, volatile `mono8` class IDs |
| `/semantic_mapping/semantic_confidence_image` | `sensor_msgs/msg/Image` | reliable, volatile `32FC1`, finite probability in `[0,1]` |
| `/semantic_mapping/semantic_visualization` | `sensor_msgs/msg/Image` | reliable, volatile `rgb8` confidence-weighted overlay |
| `/semantic_mapping/semantic_class_metadata` | `std_msgs/msg/String` | reliable, transient-local JSON schema/version/colors/dynamic flags |

All three image messages preserve the source RGB header exactly. The
precomputed backend matches its manifest by that timestamp within
`backend.precomputed.max_time_error_sec`; it does not consume a latest mask.
These are per-camera-frame perception products, not the future map-frame
semantic BEV.

Supported Phase-3 backends:

| `backend.type` | Input behavior |
|---|---|
| `precomputed` | Nearest manifest timestamp, 50 ms default bound; unavailable frames are visible drops |
| `segformer_tensorrt` | Runs the local FP16 engine on each rate-selected RGB frame; source time error is zero |

Precomputed timestamp availability is checked before RGB decoding and before
rate limiting. This prevents a sparse manifest from being skipped because an
unrelated RGB frame consumed the rate slot.

## Phase-2 Occupancy Outputs

| Topic | Type | Encoding/content |
|---|---|---|
| `/semantic_mapping/occupied_voxels` | `sensor_msgs/msg/PointCloud2` | XYZ centers plus float32 `occupancy` probability |
| `/semantic_mapping/occupancy_bev` | `nav_msgs/msg/OccupancyGrid` | `-1` unknown, `0` free, `100` occupied |
| `/semantic_mapping/occupancy_probability_bev` | `sensor_msgs/msg/Image` | `32FC1`; NaN where no collision/ground evidence exists |
| `/semantic_mapping/free_probability_bev` | `sensor_msgs/msg/Image` | `32FC1`; separate free-space confidence |
| `/semantic_mapping/explored_bev` | `sensor_msgs/msg/Image` | `mono8`; 0 unknown, 255 observed vertical column |
| `/semantic_mapping/height_max_bev` | `sensor_msgs/msg/Image` | `32FC1`; maximum stable occupied height relative to ground |
| `/semantic_mapping/map_metadata` | `std_msgs/msg/String` | JSON frame, origin, resolution, shape, stamp, and voxel counts |

All map products use reliable, transient-local QoS. The geometry channels are
kept separate: an occupied furniture cell can later receive a semantic label
without replacing its collision state.

## Phase-2 Services

| Service | Type | Behavior |
|---|---|---|
| `/semantic_mapping/save_map` | `std_srvs/srv/Trigger` | Force-save voxels, BEV channels, planner tensor, and metadata to `output.directory` |
| `/semantic_mapping/save_semantic_map` | `std_srvs/srv/Trigger` | Force-save semantic voxels, schema, and current Phase-5 BEV tensor to `output.directory` |

Dirty online maps are also checkpointed every 30 seconds by default. Use
`scripts/stop_tinynav_semantic_nav.sh` for a checkpointed shutdown.

## Phase-4 Map Semantic Outputs

| Topic | Type | Meaning |
|---|---|---|
| `/semantic_mapping/semantic_voxels` | `sensor_msgs/msg/PointCloud2` | confirmed static semantic voxel centers, packed RGB, class ID, confidence, observations |
| `/semantic_mapping/semantic_voxel_markers` | `visualization_msgs/msg/MarkerArray` | throttled class-colored semantic voxels for RViz |
| `/semantic_mapping/semantic_map_metadata` | `std_msgs/msg/String` | transient-local JSON counts, frame, schema, timestamp, and resolutions |

The current semantic map persists as `semantic_metadata.yaml` and
`semantic_voxels.npz` alongside the Phase-2 geometry files.

## Phase-5 Semantic BEV Outputs

| Topic | Type | Meaning |
|---|---|---|
| `/semantic_mapping/semantic_bev` | `sensor_msgs/msg/Image` | transient-local `mono8` winning semantic class ID; 0 is unknown |
| `/semantic_mapping/semantic_bev_confidence` | `sensor_msgs/msg/Image` | transient-local `32FC1` winning class probability; NaN for unknown |
| `/semantic_mapping/semantic_bev_visualization` | `sensor_msgs/msg/Image` | transient-local `rgb8` schema-colored semantic BEV for RViz |
| `/semantic_mapping/semantic_bev_explored` | `sensor_msgs/msg/Image` | transient-local `mono8`; 255 has confirmed semantic evidence |
| `/semantic_mapping/semantic_bev_height_min` | `sensor_msgs/msg/Image` | transient-local `32FC1` height relative to the geometry ground reference |
| `/semantic_mapping/semantic_bev_height_max` | `sensor_msgs/msg/Image` | transient-local `32FC1` height relative to the geometry ground reference |

`semantic_bev_tensor.npz` contains all channels above plus
`semantic_scores[H,W,C]`, `origin_xy`, `resolution`, and `ground_z`. Its grid
is adopted from `/semantic_mapping/occupancy_bev` or the matching saved `metadata.yaml`, so
it aligns exactly with the independent geometry `planner_tensor.npz`.

After an offline map build, the copied auto-map script reprojects saved semantic
voxels using the final occupancy grid. The equivalent manual command is:

```bash
python3 scripts/export_semantic_bev.py <tinynav_map_directory>
```

## QoS Rules

- Camera subscriptions: sensor-data best effort, volatile, bounded queue.
- TF static: TF2 standard transient-local behavior.
- Point cloud: reliable, volatile, keep-last 1. A slow RViz subscriber must not
  create an unbounded queue.
- Per-frame semantic images: reliable, volatile, keep-last 2; RGB input remains
  sensor-data best effort.
- Semantic class metadata: reliable and transient-local.
- Static occupancy/metadata products: reliable and transient-local.
- High-rate diagnostics: best effort where losing an old sample is harmless.

## Synchronization Rules

1. RGB, aligned depth, and CameraInfo are approximate-time synchronized.
2. The RGB timestamp is the integration timestamp.
3. Depth-to-RGB time difference must be below `sync.max_slop_sec`.
4. `world -> camera` TF is queried for that timestamp, never for callback
   wall-clock time.
5. Online `map` output waits until TinyNav publishes a genuine map/world
   alignment; frames received while waiting are not integrated or counted as
   pose failures.
6. Unresolved pose, excessive pose error, shape mismatch, or frame mismatch
   drops the frame and increments a visible diagnostic counter.
7. Online map output may compose the latest sparse map-alignment edge with the
   exact image-time camera pose; it never uses a latest camera pose.
8. The geometry front end publishes the exact pose it used with the same
   timestamp as the point cloud. Phase 2 synchronizes those two messages and
   does not query TF again.
9. Phase-3 precomputed labels are independently matched to the RGB timestamp;
   a match outside the configured error bound increments a visible unavailable
   counter and produces no semantic output.
10. Phase 4 synchronizes the map-frame point cloud, its exact pose, label, and
    confidence by the original RGB stamp. A missing member drops that set; it
    never indexes a latest semantic frame.
11. Phase 5 treats occupancy BEV as static map geometry, not a pose source. A
    transient-local occupancy grid update reprojects semantic voxels onto its
    origin/resolution/shape and carries its saved ground height.

## Required Future Bag Topics

A semantic mapping recording should include at least:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/slam/odometry_visual
/slam/odometry
/slam/keyframe_odom
/tf
/tf_static
```

Raw depth, infrared images, IMU, and TinyNav keyframe depth should also be kept
for diagnosis and deterministic TinyNav pose regeneration.

Recording entry points:

```bash
# Full TinyNav sensor bag plus semantic inputs and any live pose topics
bash scripts/run_semantic_rosbag_record.sh --output <bag_path>

# Small phase-1 replay bag; TinyNav perception must already be running
bash scripts/run_semantic_rosbag_record.sh --minimal --output <bag_path>
```
