# Semantic Mapping Experiments

## 2026-07-17 Repository and Bag Audit

Bag inspected:

```text
$HOME/.local/share/tinynav/rosbags/map_record_20260707_120552
```

Commands:

```bash
ros2 bag info "$HOME/.local/share/tinynav/rosbags/map_record_20260707_120552"
python3 tool/validate_tinynav_bag.py \
  --bag "$HOME/.local/share/tinynav/rosbags/map_record_20260707_120552"

# This is expected to fail for the audited pre-alignment bag.
python3 tool/validate_tinynav_bag.py \
  --bag "$HOME/.local/share/tinynav/rosbags/map_record_20260707_120552" \
  --require-semantic-inputs --require-recorded-pose --skip-timestamp-check
```

Results:

- Duration: 118.864 s.
- Size: 9.5 GiB.
- Messages: 55,805.
- RGB frames: 3,564.
- Raw depth frames: 3,563.
- RGB/depth median timestamp difference: about 18 microseconds.
- One startup RGB frame was about 33 ms from the nearest depth frame.
- No aligned-depth topic was recorded.
- No dynamic TF or TinyNav odometry topic was recorded.

## Calibration Inspection

The first RGB, raw depth, CameraInfo, and static TF messages were deserialized
directly with `rosbag2_py`. Results are recorded in the repository audit and
frame contract. The key finding is that raw depth and RGB share resolution but
not intrinsics or frame, so direct RGB pixel indexing is invalid.

## Existing Map Artifact Inspection

Map inspected:

```text
output/map_record_20260707_120552
```

Results:

- 215 optimized keyframe poses.
- Static occupancy shape 115x158x30 at 0.1 m.
- Origin `[-5.5, -6.1, -1.9]`.
- Occupancy counts: 463,358 unknown; 72,750 free; 8,992 occupied.
- `T_rgb_to_infra1.npy` contains `None` for this map.

An older map, `output/map_record_20260616_071537`, contains a valid 4x4 RGB to
infrared optical transform. The difference tracks a RealSense static TF edge
name mismatch in current `build_map_node.py`, not evidence that RGB and infrared
became co-located.

## Live Aligned RGB-D and Point Cloud

Hardware:

```text
Intel RealSense D435I, serial 344422071135, firmware 5.17.0.10
```

Measured live results:

- RGB rate: about 30 Hz.
- Aligned-depth rate: about 30 Hz.
- Both aligned depth and its CameraInfo use
  `camera_color_optical_frame`, 848x480, with RGB intrinsics.
- TinyNav `world -> camera` retained source image timestamps but arrived about
  0.4-0.8 seconds later under the Jetson inference load.
- A 2.0-second pending buffer produced point clouds at about 3.1 Hz.
- Typical cloud size was 100.8k valid RGB-D points after stride/depth/edge
  filtering.
- A sampled cloud had bounds approximately
  `x[-1.39,0.42], y[0.27,2.11], z[-0.20,0.39]` meters in `world`.
- RViz rendered the metric colored scene; the screenshot is
  `docs/semantic_mapping_phase1_live.png`.

## Posed RGB-D Bag and Offline Replay

Recorded bag:

```text
/run/user/1000/tinynav_semantic_phase1_20260717_080342
```

Validation command:

```bash
python3 tool/validate_tinynav_bag.py \
  --bag /run/user/1000/tinynav_semantic_phase1_20260717_080342 \
  --semantic-only
```

The bag contains 821 messages over 5.67 seconds and passed image timestamp
continuity checks. Offline replay, with the camera and perception stopped,
published a 100.8k-point cloud in `world`. Three leading images were visibly
dropped because recording began before the first dynamic pose sample. This is
a bounded startup condition, not a latest-pose fallback.

The bag is on tmpfs because `/` was 100% full during the test. Copy it to
durable storage before reboot.

## Saved-Map Relocalization Attempt

`map_node.py` was started without planning or control. No `world -> map`
transform was produced because the robot is currently outside the region
covered by `output/latest_map`; this is expected. No identity/static map
transform was fabricated. Online `map` output remains gated on a real TinyNav
relocalization.

## Synthetic Map-Frame Transform Check

To test map-frame composition independently of place recognition, a fixture
published a known `T_world_map` translation `[1, 2, 0]` meters. Two nodes
processed the same replayed RGB-D stamp into `world` and `map`.

```text
stamp:                 1784275425.711662354
points:                100893
mean(p_map-p_world):   [-1, -2, 0] m
maximum absolute error: 1.2e-7 m
```

This validates TF direction and point-cloud frame labeling. It is not reported
as a TinyNav relocalization result.

## Phase-2 DDA Performance

Input: 6,000 deterministic random rays spanning roughly 4x4x4.4 meters at
5 cm voxel resolution on the Jetson.

```text
scalar per-ray Python DDA:       18.321 s
batched NumPy DDA + dict update:  1.878 s
repeat into allocated map:        1.658 s
unique active voxels:           178,094
```

The random-ray case deliberately minimizes shared cells and is more expensive
than an indoor depth image. Batch and scalar DDA outputs are compared as exact
free/occupied voxel sets in the unit tests.

## Phase-2 Posed-Bag Replay

Commands:

```bash
source install/setup.bash
ros2 launch semantic_mapping semantic_mapping_offline.launch.py \
  output_directory:=/tmp/tinynav_phase2_validation_fast

ros2 bag play \
  /run/user/1000/tinynav_semantic_phase1_20260717_080342 \
  --clock --rate 0.5
```

Observed ROS output:

```text
/semantic_mapping/occupancy_bev
  frame: world
  resolution: 0.05
  width: 39
  height: 45
  origin: [-1.45, -0.05, 0.0]
```

Serialized map counts after clean shutdown:

```text
active:     5942
free:       4311
occupied:   1404
uncertain:   227
```

`load_occupancy_voxel_map()` restored the saved map and reproduced these
counts. Leading images were again dropped explicitly because the short bag
starts before its first `world -> camera` TF sample.

## Phase-2.5 Live Saved-Map Run

New map and raw bag:

```text
TinyNav map: output/semantic_map_record_20260717_102052
sensor bag:  $HOME/.local/share/tinynav/rosbags/semantic_map_record_20260717_102052
duration:    65.29 s
bag size:    6.6 GiB
```

The copied navigation chain was run with `--no-go2`. TinyNav initially
published repeated relocalization poses near `[-0.37, -0.03, 0.62]` meters in
the new map. The map-frame geometry chain then built and checkpointed a live
occupancy map. One checkpoint contained 140,651 active voxels and a 94x141 BEV;
the exact live count is not a fixed benchmark.

Phase-2.5 observed the following on this map:

```text
map-alignment gate:       zero integrated frames before relocalization
near-ground candidates:  4,000 per fit after bounded sampling
typical plane inliers:    700-1,000
filtered ground_z:        0.000 m after a full checkpoint interval
BEV projection before:    about 3.5 s at 136k voxels
BEV projection after:     0.15-0.20 s at 136k-140k voxels
online integration:       about 1.1-1.5 accepted pairs/s in the static test
checkpoint period:        30 s
```

Both the automatic timer and `/semantic_mapping/save_map` returned a successful
checkpoint. `scripts/stop_tinynav_semantic_nav.sh` saved the map, stopped the
copied tmux session, and a subsequent `--no-go2` launch loaded the same nested
voxel map. On that restart the static view produced only 31-47 feature matches,
below TinyNav's 50-match relocalization threshold; the semantic mapper correctly
held integration at zero rather than inventing a map transform.

## Phase-3 ROS Perception Smoke Test

A version-1 manifest referenced one 3x4 `uint8` label array and one matching
`float32` confidence array at timestamp `1234567890 ns`. A real
`semantic_perception_node` consumed a synthetic `rgb8` ROS image with that
header and published:

```text
/phase3_smoke/label          mono8,  step 4
/phase3_smoke/confidence     32FC1, step 16
/phase3_smoke/visualization  rgb8,   step 12
/phase3_smoke/class_metadata transient-local JSON
```

The labels read back exactly as `[0,1,1,2,0,4,4,2,6,6,5,2]`. Every image
retained `camera_color_optical_frame` and the exact source stamp. Four frames
were processed with zero unavailable/invalid results; the cached tiny-array
path averaged about 3.2 ms. The smoke node exited with status 0 on SIGINT.

## Phase-3 TensorRT and Real-Bag Run

Model preparation used the pinned full-precision ONNX export of
SegFormer-B0 ADE20K. SHA256 validation passed before TensorRT 10.3 generated a
fixed 1x3x512x512 FP16 engine on Orin:

```text
ONNX size:                 15.3 MB
engine size:               10.1 MB
engine build time:         304 s
engine output:             1 x 150 x 128 x 128 float32 logits
trtexec mean GPU latency:  11.50 ms
trtexec throughput:        86.15 queries/s
```

End-to-end processing includes RGB resize/normalization, TensorRT execution,
150-class softmax/label collapse, 480x848 restoration, NPY serialization, and
two PNG visualizations. The complete 65.29-second bag run at 2 Hz produced:

```text
source RGB messages:  1,912
processed frames:       131
output directory:       207 MB
wall time:               48.802 s
mean processing:         97.202 ms/frame
```

Pixel totals over the complete output were:

```text
unknown             3,459,219
floor              28,689,944
wall               19,322,912
door                  302,968
chair                 320,948
table                 153,656
cabinet                60,766
other_furniture       752,544
dynamic_object        259,283
```

An isolated real ROS TensorRT replay preserved the 480x848 RGB header and ran
at about 2.16 output FPS while TinyNav remained active. Six frames completed
with zero unavailable/invalid inputs and about 179 ms mean callback time under
the concurrent load. The direct offline generator averaged 97.2 ms.

Sparse precomputed playback initially checked rate limiting before timestamp
availability, which could miss manifest frames. Moving the cheap timestamp
check first fixed that ordering. A 50 ms bound was selected after the running
system dropped individual 30 Hz RGB samples: observed accepted errors were
29.2 ms mean and 33.4 ms maximum. Accelerated 5x playback still drops best-
effort RGB messages and is not the correctness benchmark; generation reads the
bag directly and normal deployment runs at source rate.

## Phase-4 Semantic Fusion and Full Auto-Map Validation

Pure-map tests cover consistent and conflicting votes, confidence/range/edge
weighting, unknown and dynamic rejection, confirmation thresholds, and
semantic NPZ/YAML round-trip persistence. A real ROS smoke node received four
map-frame points with matched pose, labels, and confidence. It retained only
floor and wall, published two confirmed semantic voxels and class markers, and
successfully saved then loaded the result through the semantic-map service.

The complete sensor bag was then rebuilt through the copied mapping chain:

```bash
bash scripts/tinynav_semantic_auto_map.sh \
  --from-bag "$HOME/.local/share/tinynav/rosbags/semantic_map_record_20260717_102052" \
  --map-dir output/phase4_validation_map_20260717 \
  --semantic-masks output/latest_semantic_masks \
  --keep-temp
```

TinyNav perception generated the image-time `world -> camera` pose because the
recorded `/tf` contains RealSense internal transforms but no mapping pose. The
copy completed all map stages and saved both geometry and semantic artifacts.

```text
input point sets:         17
accepted semantic frames:  5
non-keyframes skipped:     3
pose jumps detected:       9
input points:         499,805
static accepted:      440,346
unknown rejected:      42,559
low confidence:       16,900
semantic voxel updates: 9,805
active / confirmed: 3,042 / 2,324
floor / wall / furniture: 1,045 / 1,273 / 6
mean semantic fusion: 179.06 ms per accepted keyframe
```

The first five pose-consistent observations produce the displayed static room
surface. Later TinyNav odometry corrections exceeded the configured 0.5 m or
20 degree jump threshold, so fusion paused as required. The map was
checkpointed before those jumps; no shifted geometry was silently fused.

## Phase-4 Copied Auto-Nav Load Validation

The saved map was loaded through the copied navigation entry point without a
Go2 bridge or RViz:

```bash
bash scripts/tinynav_semantic_auto_nav.sh \
  --session tinynav_phase4_nav_validation \
  --map output/phase4_validation_map_20260717 \
  --no-go2 --no-rviz \
  --semantic-masks output/latest_semantic_masks
```

The occupancy mapper loaded 115,776 geometry voxels and the semantic mapper
loaded 3,042 semantic voxels. A transient-local subscriber received width 2,324
from `/semantic_mapping/semantic_voxels`, matching the confirmed count in the
saved metadata. No live integration occurred: the robot is outside the map and
TinyNav relocalization had zero similar embeddings, so the existing map stayed
published while the map-alignment gate rejected new observations. The copied
stop script waits for ROS service discovery and then saved both
`/semantic_mapping/save_map` and `/semantic_mapping/save_semantic_map`
successfully before closing the tmux session.

## Phase-5 Height-Aware Semantic BEV

Phase 5 adds a pure semantic projector that consumes confirmed Phase-4 voxel
score vectors and either an explicit BEV geometry or a Phase-2 occupancy grid.
The semantic output never changes occupancy: it contains independent class
scores, class ID, winning confidence, semantic exploration, and semantic height
range. Unit cases cover ground floor fallback, wall/object evidence in the
semantic band, object-over-floor precedence, overhead exclusion, empty grids,
and nonzero external origins.

The real saved map was loaded in an isolated ROS node. The node read matching
occupancy metadata, immediately published the semantic BEV, and persisted it
through the existing semantic save service:

```text
semantic BEV:           mono8, 87 x 170
semantic visualization: rgb8,  87 x 170
semantic tensor:        float32, 87 x 170 x 11
origin:                 [-6.5, -2.0] m
resolution:             0.05 m
occupancy grid shape:   87 x 170
confirmed cells:        151 wall cells
```

The copied auto-nav was then run with `--no-go2 --no-rviz` against the same map.
It loaded the persisted geometry/semantic layers, published the same BEV topic
dimensions, and checkpointed both services successfully. Because the current
robot view has no relocalization match, map-frame RGB-D integration remained
correctly gated at zero.

The output directory now contains:

```text
semantic_bev.npy
semantic_confidence_bev.npy
semantic_explored_bev.npy
semantic_height_min_bev.npy
semantic_height_max_bev.npy
semantic_bev_tensor.npz
```

The tested map's Phase-2 `ground_z` is `0.0 m`, while its confirmed floor
semantic voxels are near `-0.4 m`. With the deployed geometry ground reference,
only wall cells fall in the Phase-5 semantic band. This is a height-reference
mismatch to resolve through the posed RGB-D ground bootstrap before
collision-certified floor semantics, not a semantic projection fallback to a
guessed height.

## Ground Bootstrap Validation

The historical Phase-4 occupancy map was used as an offline geometry diagnostic.
Its median TinyNav camera position was `[-0.562, 0.423, -0.084] m`. Starting
from `ground_z=0.0 m`, the new bootstrap searched the camera-local wide band and
returned the same near-horizontal plane on three deterministic RANSAC updates:

```text
candidate ground_z:  -0.525 m
plane inliers:       873 / 4,000
camera-plane height: 0.441 m
bootstrap result:    accepted after 3 candidates
```

The resulting ground reference places the historical floor semantic voxels in
the configured ground band. This is an offline diagnostic over saved occupancy
voxels; the next posed bag replay is the runtime validation and will write the
observed ground reference to both geometry and semantic BEV products.

## Ground Bootstrap Full-Bag Replay

The complete 65.29-second posed RGB-D bag was replayed through the copied
auto-map chain with the same deterministic precomputed masks. The startup
bootstrap first collected two pending candidates, then locked the map ground
reference on the third valid local plane fit:

```text
candidate ground_z:       -0.415 m
bootstrap consensus:      -0.413 m
final filtered ground_z:  -0.380 m
bootstrap plane tilt:       1.38 deg
bootstrap inliers:      2,714 / 4,000
```

The saved sparse occupancy map contains 116,304 active voxels (52,968 free,
19,184 occupied, and 44,152 uncertain). It accepted 69 geometry keyframes.
The static semantic layer contains 2,988 active voxels, 2,479 confirmed voxels,
including 1,104 floor voxels, 1,369 wall voxels, and six other-furniture voxels.
Pose-jump gating remained active during later TinyNav corrections.

The final geometry BEV expanded by one column during shutdown. The copied map
script now runs the following no-ROS re-export after both mapper nodes stop, so
saved semantic products are projected against the final geometry metadata:

```bash
python3 scripts/export_semantic_bev.py \
  output/ground_bootstrap_validation_20260717
```

The verified final result is `169 x 155` at `0.05 m`, with matching origin and
`ground_z=-0.380 m` in `metadata.yaml`, `semantic_metadata.yaml`, and
`semantic_bev_tensor.npz`. Its semantic BEV has 682 floor cells, 261 wall
cells, and three other-furniture cells. This command is automatically run by
`scripts/tinynav_semantic_auto_map.sh` whenever semantic voxels were saved.
