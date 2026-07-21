# TinyNav-Pose-Conditioned Semantic Mapping Design

## Objective

Build a single-robot mapper with this strict ownership split:

```text
TinyNav: tracking, relocalization, map-frame pose, navigation
Semantic mapping: posed RGB-D integration, semantics, persistence, BEV
```

TinyNav remains the only authoritative pose source. The semantic mapper never
runs a second SLAM system.

## Layering

Pure algorithm modules must not depend on ROS state:

```text
depth_backprojection
pose interpolation and SE(3) transforms
ray traversal
sparse voxel storage
occupancy and semantic fusion
height-aware BEV projection
serialization
```

ROS wrappers own subscriptions, synchronization, TF lookup, publishers,
parameters, timers, and diagnostics.

## Phase-1 Geometry Path

```text
RGB + aligned depth + aligned CameraInfo
                 |
                 v
depth validation and optical-frame backprojection
                 |
                 v
timestamped TinyNav map-to-camera transform
                 |
                 v
RGB PointCloud2 in map/world frame
```

Depth conversion rules:

- `16UC1` and `mono16`: millimeters to meters.
- `32FC1`: meters.
- Reject zero, NaN, infinity, values below `min_depth_m`, and values above
  `max_depth_m`.
- Sample at configurable stride.
- Optional discontinuity rejection removes samples adjacent to a depth jump
  greater than `edge_threshold_m`.

## Semantic Perception Contract

All perception backends implement one interface:

```text
SemanticFrame
  label_image:      H x W uint8
  confidence_image: H x W float32
  class_names:      class_id -> name
  timestamp_ns:     int
```

Initial backends:

1. `PrecomputedMaskBackend` for deterministic mapper tests.
2. `SegformerTensorRtBackend` for closed-set navigation classes on Jetson.
3. Grounded SAM or later SAM variants as optional mask producers only.

Navigation classes start small: unknown, floor, wall, door, couch, chair,
table, cabinet, other furniture, target, and dynamic object.

### Phase-3 Implementation

Phase 3 implements the backend-neutral contract, versioned class schema,
`PrecomputedMaskBackend`, and `SegformerTensorRtBackend`. A precomputed dataset
has this layout:

```text
precomputed_masks/
  manifest.yaml
  labels/<rgb_timestamp_ns>.npy       # H x W uint8
  confidence/<rgb_timestamp_ns>.npy   # optional float16/float32 on disk
  color_labels/<rgb_timestamp_ns>.png
  visualization/<rgb_timestamp_ns>.png
```

The version-1 manifest records the class-schema version and the source RGB
timestamp for every file. Matching uses nearest timestamp with a configurable
50 ms default limit and deterministic earlier-frame tie breaking. A missing
match drops the RGB frame visibly; the backend never reuses the latest mask.
All paths are contained within the configured directory, NPY object loading is
disabled, and image shape/class/confidence validation occurs before publish.

`semantic_perception_node` publishes the RGB-stamped `mono8` label image,
`32FC1` confidence image, confidence-weighted RGB overlay, and a transient-local
JSON class contract. Confidence is always published as `float32` even when the
offline file uses `float16` to reduce storage.

The closed-set backend uses SegFormer-B0 fine-tuned on ADE20K at 512x512. A
pinned 15.3 MB ONNX export is SHA256-verified, then converted locally to an
Orin/TensorRT-specific FP16 engine. The model produces 150 ADE20K logits at
128x128. A validated name-based YAML mapping collapses 68 relevant source
classes into floor, wall, door, couch, chair, table, cabinet, other furniture,
and dynamic object; all remaining source classes become unknown. Low-confidence
and unknown outputs receive zero semantic confidence. Labels/confidence are
restored to RGB dimensions with nearest-neighbor sampling so they remain a
strict pixel pair.

The backend is selected by parameter, so offline manifests and live TensorRT
use identical ROS outputs. Phase 4 consumes those RGB-stamped products and
writes only valid, static surface observations into its separate semantic voxel
layer.

## Sparse Voxel Representation

Use a hash map keyed by integer voxel coordinates:

```text
(ix, iy, iz) -> SemanticVoxel
```

Indexing uses one fixed origin and floor division:

```text
index = floor((point_map - origin) / voxel_resolution)
```

The geometry and semantic layers use the same integer coordinate convention but
remain independently stored. Geometry voxels retain free/occupied log odds;
semantic voxels retain class evidence only for static observed surfaces. This
lets a planner treat a couch as both occupied and semantically `couch` rather
than choosing one state over the other.

A geometry voxel stores at least:

```text
occupancy_log_odds: float32
semantic_scores: float32[C]
observation_count: uint32
free_observation_count: uint32
occupied_observation_count: uint32
last_seen_timestamp_ns: int64
```

The Phase-4 semantic voxel stores a `float32[C]` score vector, observation
count, and last-seen timestamp. Use 0.05 m initially on desktop and 0.10 m as
the Jetson fallback. Python dictionary storage is acceptable for
correctness-first development. The API must hide storage so a C++ hash map or
block map can replace it later.

## Ray Traversal

Every accepted depth sample creates one camera-to-surface ray. Use
Amanatides-Woo 3D DDA because it visits each crossed voxel exactly once and
handles negative coordinates without slope special cases.

For each ray:

1. Begin at the camera origin in map coordinates.
2. Traverse to `depth - truncation_distance` and apply the free log-odds update
   to intermediate voxels.
3. Apply the occupied update to the endpoint voxel or configured endpoint
   band.
4. Apply semantic scores only to occupied surface voxels.

Required edge cases:

- Zero-length rays return no free cells.
- Camera origin and endpoint in one voxel produce only the endpoint update.
- Endpoints outside configured bounds are rejected before allocation.
- The endpoint must never also receive the free update.
- Dynamic endpoints may carve free space before the object but do not enter the
  persistent occupied/semantic layer.

### Phase-2 Implementation

The scalar Amanatides-Woo traversal is the reference implementation used by
boundary, negative-coordinate, diagonal, and zero-length tests. Frame
integration uses an equivalent NumPy batch state machine: every active ray
advances at the same next-boundary parameter, tied axes advance together, and
the resulting cells are deduplicated once per frame. Occupied endpoints are
removed from the global free set before log-odds updates.

This avoids evidence strength depending on image pixel density and reduced a
6,000-ray Jetson benchmark from 18.3 seconds to 1.88 seconds without changing
the resulting voxel sets.

## Occupancy Fusion

Use clamped log odds:

```text
L_new = clamp(L_old + delta, L_min, L_max)
P_occ = 1 / (1 + exp(-L_new))
```

Initial values:

```text
free update:      -0.40
occupied update:  +0.85
minimum log odds: -4.0
maximum log odds: +4.0
free threshold:    0.30 probability
occupied threshold:0.70 probability
```

Unknown is a state with no observations or probability between thresholds; it
is not equivalent to free.

## Semantic Fusion

For surface voxel V and class c:

```text
score[V,c] += confidence * exp(-depth / depth_decay) * edge_weight
```

Only confirm a label after `min_semantic_observations`. A single contradictory
frame adds evidence but does not overwrite accumulated evidence. Keep geometry
and semantics independent: a couch voxel can be occupied and classed as couch
simultaneously.

Dynamic classes are configured by class ID/name. Their endpoint occupancy and
semantic evidence go to an optional short-lived layer; phase 4 may initially
drop them from the static endpoint integration while preserving free carving
before the endpoint.

### Phase-4 Implementation

`semantic_pointcloud_node` carries the sampled source RGB pixel as `u` and `v`
fields in each map-frame point. `semantic_mapper_node` synchronizes that cloud,
the exact pose used to construct it, the RGB-stamped label image, and the
confidence image. It rejects shape/frame/schema mismatches rather than using a
latest mask or pose.

For a valid endpoint, the initial evidence weight is:

```text
confidence * exp(-range_m / depth_decay_m) * mask_edge_weight
```

The edge term is computed from the label boundary and bottoms out at the
configured `min_edge_weight`. Duplicate pixels in the same voxel are combined
once per input frame, so one high-resolution view cannot dominate multi-view
votes. Unknown, below-confidence, and configured dynamic labels do not update
the persistent semantic map. A label is published only after the configured
minimum observation count and normalized winning-score threshold.

## Keyframes, Relocalization, and Rebuild

Integrate a frame when any condition is met:

```text
translation >= 0.20 m
rotation >= 10 degrees
elapsed time >= 1.0 s
```

Detect pose jumps relative to the preceding accepted pose:

```text
translation > 0.5 m or yaw > 20 degrees
```

After a jump, record an event and pause integration for a configured number of
stable frames.

The long-term rebuild format should retain every integrated semantic keyframe:

```text
timestamp
depth reference or compressed depth
label/confidence reference
camera intrinsics
T_map_camera
configuration/class schema version
```

Phase 4 currently retains the fused map timestamp and applies the pose-jump
gate before integration. It does not yet persist the source RGB-D/keyframe set,
so a full rebuild after a loop-closure correction remains the next robustness
increment rather than an online promise.

## Ground and Height Reference

Assume map Z-up only after the frame validation procedure passes. Estimate
ground Z from a local depth point cloud using a gravity-constrained plane fit,
then filter it over time. Fall back to configured `ground_z` if fitting fails.

Phase 2.5 implements a deterministic local RANSAC fit of `z = ax + by + c`.
Normal tracking candidates are limited to a configurable XY radius and narrow
height band around the current ground estimate. Fits are rejected by absolute
inlier count, inlier ratio, plane tilt, or a 0.15 m candidate jump. Accepted
candidates enter a nine-sample median window prefilled by the saved/configured
ground value, then a slow EMA with a bounded update step. Failed fits never
modify `ground_z`.

A new TinyNav map can have a valid z-up frame while its floor is not at zero.
The mapper therefore begins with a separate bootstrap stage: it searches a
wider vertical band, accepts only near-horizontal planes below the current
camera origin, and requires three candidates within 4 cm before directly
locking `ground_z`. It then switches permanently to the conservative tracking
band above. This uses timestamped `map -> camera` RGB-D observations and does
not require a fixed `base_link -> camera_link` transform.

A measured body-to-camera transform remains useful for independently verifying
the Go2 collision envelope and for deployments that only expose `map -> base`.
It is not a prerequisite for metric RGB-D mapping when TinyNav supplies the
timestamped camera transform.

## Height-Aware BEV

Each XY cell aggregates a vertical voxel column relative to `ground_z`:

```text
ground band:    [-0.10, 0.15] m
collision band:[ 0.10, 0.75] m
semantic band: [ 0.05, 1.50] m
ignore above:   1.80 m for navigation collision
```

Projection rules:

- `explored` is true if any voxel in the relevant column has an observation.
- Free probability aggregates traversed free voxels without treating missing
  voxels as free.
- Occupancy is computed only from stable occupied voxels in the collision
  band.
- Floor evidence in the ground band must not create an obstacle.
- Semantic scores aggregate occupied surface evidence in the semantic band.
- Overhead objects retain optional semantics/height but do not block Go2 when
  they are above the collision band.

Phase 2.5 initializes from configured or saved `ground_z` and updates it only
through the filtered estimator above. Stable surface evidence in the ground
band is treated as traversable support unless the same XY column contains
stable occupied evidence outside the ground band and inside the collision
band. Overhead-only observation marks a column explored but does not claim it
is free.

Internal BEV representation:

```text
occupancy:      H x W float32
free:           H x W float32
explored:       H x W uint8
semantic_scores:H x W x C float32
semantic_label: H x W uint8
height_min:     H x W float32
height_max:     H x W float32
```

Publish navigation occupancy and semantic labels as separate products. A
semantic class never replaces collision state.

### Phase-5 Implementation

Phase 5 implements `semantic_bev_projector.py` and keeps the semantic layer
strictly separate from Phase-2 log-odds occupancy. The semantic mapper consumes
the transient-local `/semantic_mapping/occupancy_bev` only for its 2D grid
geometry: origin, resolution, width, height, and the stored ground height in
`OccupancyGrid.info.origin.z`. It never derives occupancy from a semantic label.

Confirmed semantic voxel score vectors are accumulated in the semantic height
band. Confirmed floor voxels in the ground band are used only when that XY cell
has no wall/object semantic evidence, so floor support cannot overwrite a chair,
table, or wall in the same vertical column. Each output cell contains normalized
class scores, winning label/confidence, semantic-explored state, and the selected
semantic height range.

The mapper publishes `mono8` class IDs, `32FC1` winning confidence, `mono8`
semantic exploration, `32FC1` min/max heights, and an `rgb8` class-color image.
It persists the full `H x W x C` score tensor in `semantic_bev_tensor.npz` beside
the Phase-2 `planner_tensor.npz`; both files share map origin and resolution.
This file split preserves ownership while the downstream planner message format
is still undecided.

## Persistence

Map directory layout:

```text
semantic_map/
  metadata.yaml
  voxels.npz
  occupancy_bev.npy
  planner_tensor.npz
  semantic_metadata.yaml
  semantic_voxels.npz
  semantic_bev.npy
  semantic_confidence_bev.npy
  semantic_explored_bev.npy
  semantic_height_min_bev.npy
  semantic_height_max_bev.npy
  semantic_bev_tensor.npz
```

`metadata.yaml` includes frame ID, timestamps, voxel/BEV resolution, fixed
origin, ground Z, class schema/version, configuration digest, and source robot
ID. Including robot ID and source observations leaves a clean later boundary
for multi-robot map fusion without implementing it now.

Online runs checkpoint dirty maps every 30 seconds when `output.directory` is
set. `/semantic_mapping/save_map` (`std_srvs/srv/Trigger`) forces a checkpoint.
`/semantic_mapping/save_semantic_map` independently persists the Phase-4
semantic layer and its schema.
The copied semantic navigation chain is stopped through
`scripts/stop_tinynav_semantic_nav.sh`, which calls the service before closing
the tmux session.

## Diagnostics

Publish/log rolling values for:

- input RGB/depth rates and synchronized triplet rate;
- pose lookup success/failure and mean/max time error;
- processed keyframes and drop reasons;
- active/free/occupied/semantic voxel counts;
- ray casting, semantic inference, integration, and BEV durations;
- BEV dimensions and update rate;
- process RSS memory.

Pose failures, alignment mismatches, and jump events are warnings with
throttling, never silent skips.

## Performance Plan

Correctness target first:

```text
semantic inference: 2-10 Hz
voxel integration:  2-10 Hz
BEV publication:    1-5 Hz
```

Optimization order:

1. Keyframe filtering and depth stride.
2. Vectorized backprojection and batched endpoint preprocessing.
3. Vectorized voxel-to-BEV column reduction.
4. Numba/C++ 3D DDA while preserving exact tests.
5. Block/hash storage backend.
6. Bounded semantic score representation if class count grows.

The Phase-2.5 NumPy BEV reducer preserved the artificial-column tests and
reduced a live 136k-voxel projection on Jetson from about 3.5 seconds to
0.15-0.20 seconds.

## Test Coverage

- Known-pixel backprojection and depth encoding conversion.
- SE(3) transform and timestamp interpolation with quaternion SLERP.
- Positive/negative/boundary voxel indexing with origin offsets.
- Axis-aligned/diagonal/zero-length DDA traversal.
- Log-odds clamping and free/occupied/unknown classification.
- Semantic schema, invalid IDs, precomputed timestamp matching, time bounds,
  image validation, default/explicit confidence, and overlay blending.
- Consistent/conflicting/confidence-weighted semantic fusion (Phase 4).
- Source-pixel PointCloud2 layout, dynamic/unknown rejection, per-frame voxel
  normalization, semantic map save/load, and ROS four-way synchronization.
- Artificial ground, low obstacle, tall obstacle, overhead object, and semantic
  object BEV columns.
- Ground RANSAC with clutter, slope rejection, temporal median, and bounded
  height updates.
- Save/load round trip and deterministic rebuild from semantic keyframes.
- Offline semantic BEV re-export against a final occupancy grid with a changed
  origin, shape, timestamp, and ground reference.
