# Live map contract

`hub/tools/hub_pipeline_daemon.py` is the incremental RGB-D map used by the
Foxglove dashboard and dry-run decision pipeline. It is separate from
TinyNav's finalized native BuildMap artifact. RedNet remains the default
source-derived semantic baseline. The checksum-pinned
`segformer-ade20k` deployment adapter is available for real-camera pixel
semantics after RedNet's observed chair domain gap. YOLOv10 remains Stage-1
Perception-VLM text evidence in the current clean-map launcher; it is not
painted into the BEV.

## Startup

The daemon does not choose a fixed map extent from one observation. It waits
for three consecutive poses that satisfy all of these defaults:

- adjacent translation no greater than 2.0 m;
- adjacent rotation no greater than 90 degrees;
- adjacent capture interval no greater than 10 s;
- one transform version and one parent frame.

The median startup position centers a 26 m map. A three-frame deterministic
RANSAC consensus estimates the complete ground plane `z = ax + by + c`; the
mapper does not reduce that plane to one scalar height. If no consistent
near-horizontal ground is visible, startup remains blocked. An operator may
explicitly choose `--ground-mode camera-height --camera-height <metres>` only
when that physical height is measured for the current posture.

## Incremental geometry

The live defaults accept a mapping keyframe after 0.20 m translation, 10
degrees rotation, or 5 s. A translation above 2.0 m or rotation above 90
degrees between adjacent observations latches the map as halted; it does not
resume irreversible fusion into a potentially different pose frame. Start a
new output directory/session after investigating the discontinuity.

With RANSAC ground mode, every observation is checked before the keyframe gate
or RedNet inference. A frame with no accepted floor candidate is skipped. A
candidate differing from the startup floor by more than 3 degrees or 8 cm is
also skipped immediately; three consecutive accepted-but-outlying fits latch
the map and require a fresh calibrated session. Any subsequent in-range fit or
observation without a valid floor breaks the consecutive run and clears the
pending streak. This confirmation rule tolerates a single dynamic turning
frame without ever integrating that frame. An accepted frame's own plane
coefficients are used for height classification, which prevents a small
residual floor slope from turning distant carpet into an obstacle.

Obstacle geometry uses one frame-level update per cell:

- traversed free cell: log odds -0.40;
- height-filtered endpoint: log odds +0.85;
- clamp: [-4, 4];
- occupied threshold: probability >0.70;
- minimum supporting keyframes: 2;
- collision band: 0.15–0.75 m above the validated per-frame ground plane.

Explored always preserves the upstream maximum-fusion contract. Semantic
projection uses its own 0.25–1.50 m height band. `--semantic-fusion-mode max`
preserves the upstream semantic maximum. The real-camera adapter may instead
use `multi_view`: one class/cell/keyframe vote, a configurable minimum hit
count, and one winning class per cell. The current clean-map launcher requires
two supporting keyframes and a one-vote winner margin. This prevents one model
frame or a conflicting class from becoming permanent map color.

## Real-camera pixel semantic adapter

`--semantic-backend segformer-ade20k` loads only the pinned local files under
`artifacts/vision/segformer_b0_ade20k_hf`. Run
`bash hub/scripts/prepare_segformer_ade20k.sh` on a new clone; the script pins
Hugging Face revision `489d5cd81a0b59fab9b7ea758d3548ebe99677da` and rejects
any file whose byte size or SHA-256 differs from `manifests/artifacts.json`.
It downloads no simulator data.

The adapter uses the same SegFormer-B0/ADE20K model family, confidence 0.35
gate and nearest-neighbour categorical restoration already validated in
TinyNav's isolated real-camera semantic mapper. Direct ADE20K equivalents are
collapsed to the existing MP3D IDs consumed by `CentralMapper`; broad
floor/wall/door/other-furniture labels remain unknown. Its map output is a
depth-projected pixel silhouette, not a detector bounding box.

This backend is an explicitly recorded deployment adaptation. It does not
change the source Perception -> Judgment -> gate -> Decision cascade, shared
directional memory, sequential multi-agent candidate removal, or target-channel
override. It also does not make semantic accuracy verified: without labelled
real-camera masks, every result remains
`deployment_adapter_model_inference_unverified`.

## Goal-scoped YOLO semantic reinforcement

The upstream HPC project already loads `detect/yolov10m.pt` and calls
Ultralytics YOLO at confidence 0.2, but uses its detections only in the
Perception-VLM prompt. It does not paint YOLO results into the BEV map. The
older Hub deployment experiment is enabled explicitly with `--semantic-yolo`
and is independent of `--glm-url` and `--no-cascade`.

For every accepted mapping keyframe, the daemon retains YOLO class,
confidence and image-space box. By default it reinforces only the current
`--goal-category`; additional categories require repeated
`--semantic-yolo-category` arguments. This allowlist is deliberate: a class
which YOLO happens to emit must not silently become persistent map evidence
for an unrelated navigation goal.

An accepted box must exceed confidence 0.35 and contain at least 25 valid
aligned-depth pixels. Its depth anchor is the median in the central 40% of the
box; only returns within ±0.45 m receive the matching MP3D label.
Using a central robust anchor and a symmetric interval rejects both a small
nearer occluder and the farther wall visible through an open chair. The
existing mapper then applies the transported camera pose, semantic height
band and per-cell evidence threshold. This is sparse depth-grounded box
evidence, not an invented segmentation mask. Detector failure is recorded and
leaves RedNet/geometry running; it never enables motion or changes the
HOLD-only decision policy.

The model path, byte size and SHA-256, configuration, per-category detection
counts, last boxes/evidence and failure count are persisted in both live
status and map summary. All YOLO-derived results are labelled
`model_inference_depth_projected_unverified`: live consistency is not a
ground-truth accuracy benchmark. `start_fresh_dual_maps.sh` intentionally does
not enable this path: a depth-filtered detector box can still spread class
evidence across background surfaces and does not satisfy the requested
pixel-mask visualization.

## Snapshot and fusion

Every new snapshot pair (`central_map.npz` plus `map_summary.json`) contains at
least:

- grid, origin, resolution, compatibility floor height, authoritative
  `floor_plane_coefficients` and floor source;
- frame ID and robot transform version;
- optional shared-frame calibration ID;
- obstacle fusion mode, height band and hit threshold;
- semantic backend provenance, semantic fusion mode/hit gates, optional YOLO
  model checksum and goal-category allowlist;
- ground rejection/drift counters, thresholds and last residuals;
- `map_format_version=focus-hub-central-map-v3`.

Foxglove refuses legacy snapshots without frame/transform metadata unless
`--allow-legacy-maps` is explicitly given for an unverified per-robot view.
Fusion never accepts that override. `--fuse` requires every input map to name
the same non-empty `shared_frame_calibration_id` and frame ID. Once that
contract passes, the relay publishes both `/fused/geometry_map` and
`/fused/semantic_map`. The geometry view is the operator default because it
does not turn an empty or low-quality semantic layer into apparent object
evidence.

A local camera mount outside the robot TF tree must use an explicit calibrated
extrinsic. Cross-robot calibration must not rotate gravity: use
`derive_ground_camera_extrinsic.py` and
`calibrate_gravity_shared_frame_via_board.py` for the Yunji-style board flow.
The older unconstrained SE(3) tool remains valid only when both input pose
frames are independently known to share gravity.

WSJ no longer feeds `/slam/keyframe_image` infrared into semantic inference.
`focus_ros_sender.py --register-rgb-to-depth` synchronizes RealSense
`/camera/camera/color/image_raw` and its CameraInfo with TinyNav
`/slam/keyframe_depth`, CameraInfo and odometry. The observed static
color-to-infra transform reprojects real color onto TinyNav's depth pixel grid,
so pose/intrinsics remain TinyNav-native. D435 color covers about 50.4% of
valid infra-depth pixels in this configuration. Outside that calibrated
overlap, transported depth is explicitly zero; resized color may provide 2-D
model context but can never enter the metric BEV. RGB/depth skew, overlap and
transform provenance are recorded by the sender.

Five observed registered-color frames (sequences 16141–16145) still produced
zero production-thresholded RedNet chair pixels. The pinned SegFormer adapter
produced a chair silhouette in every one of those frames and was therefore
selected for the clean `20260723_rgb_pixel_v2` maps. This comparison has no
pixel ground truth and is recorded in
`audit/PIXEL_SEMANTIC_OVERVIEW_20260723.md`.

Yunji's Odin1 replacement uses `/odin1/image`, `/odin1/cloud_slam` and
`/odin1/odometry`; it does not consume the advertised-but-silent
`/odin1/cloud_raw` or vendor depth completion. The adapter rectifies the
factory FishPoly camera and z-buffers the colored SLAM cloud into aligned
PNG16 depth. Before a fresh board calibration, transport `shared_world` is
defined as the session-local Odin odom only, with a unique transform version
and no shared calibration ID. Such a map must remain a per-robot view and may
not enter `--fuse`. The observed 2026-07-22 cutover passed a fresh board fit and
independently moved-board holdout. That historical result used
`shared-board-odin1-20260722-v1`. The last predecessor 2026-07-24 fused view
used `shared-board-odin1-20260723-v3` and the `rebuild_v12_router025` map pair
listed in [CURRENT_STATUS.md](../../CURRENT_STATUS.md). It is evidence, not a
strict persistent `current` session. Every active session's exact ID must
match on both maps. See
[YUNJI_ODIN1_DEPLOYMENT.md](YUNJI_ODIN1_DEPLOYMENT.md).

Every calibration ID is session-bound, not a permanent camera extrinsic. Old
maps remain frozen audit evidence. A power cycle can preserve mechanical
calibration only when a no-motion pose/origin check proves that robot pose,
mount and placement did not move. The normal new-session path is
[`ONECLICK_SESSION_WORKFLOW.md`](ONECLICK_SESSION_WORKFLOW.md); it creates the
fit/holdout, calibration ID, sequence-bound map contract and map pair
together. The original manual procedure is preserved only as historical
detail in [VLM_LIVE_EXPERIMENT_20260723.md](VLM_LIVE_EXPERIMENT_20260723.md).

## Dashboard interpretation

- `/<name>/geometry_map` remains available in the 3-D evidence panel:
  dark gray/translucent is unknown, light gray/white is explored free space,
  and near-black is current obstacle evidence;
- `/<name>/semantic_map` is a separate combined layer and is hidden by default
  in the checked-in layout. The July 22 chair reinforcement gate produced
  non-zero chair cells on both robots; enable this topic in the existing 3D
  panel to inspect them without importing a new layout;
- `/<name>/map_pose` draws a red current camera XY and a blue relay-lifetime
  camera trail. It also copies every original 5 cm semantic cell as a shallow
  palette-colored block just above the geometry plane (chair is red). This
  makes the complete semantic pixel blob visible without replacing it by a
  bounding box or changing its footprint. A single small class label is
  anchored over the largest connected component, so the color remains
  identifiable without covering the blob. The camera marker is not a
  calibrated body footprint or heading. An unexpired VLM shadow allocation
  is drawn on this same topic as one magenta cylinder labelled
  `SHADOW <frontier> · <goal> · NO MOTION`; it is a display artifact, not a
  planner or robot command;
- `/fused/geometry_map` is the large shared-frame panel by default, with both
  robots' map-pose trails overlaid;
- `/fused/semantic_map` is present but hidden. Enable it only after
  `/fused/status` reports non-zero semantic evidence and the relevant target
  has been independently checked. Geometry occupancy alone does not prove a
  semantic class such as `chair`;
- evidence is max-reduced before categorical color assignment. The previous
  RGBA block average could invent irregular blended colors which were not
  semantic classes;
- `/<name>/semantic_overview` is the example-style 2-D operator image: near
  white unknown, light-gray explored, charcoal obstacle endpoints, exact
  semantic pixel components, compact non-overlapping class callouts, the
  transported camera trajectory and current heading triangle, plus optional
  A–D frontier markers. Tiny missing-depth slits are closed only in this
  display raster; the evidence tensor and VLM inputs are untouched;
- `/fused/semantic_overview` renders the same representation after the strict
  shared-frame fusion contract passes, with both robot trajectories and
  current poses. These three Image topics are the checked-in layout's default
  lower panels; the 3-D Grid panels remain available for evidence debugging.

Foxglove does not update an already imported local layout when this repository
file changes. Re-import `hub/foxglove/dual_robot_dashboard.json` after a layout
revision.

`hub/tools/live_vlm_shadow.py` is the one-round live multi-robot scheduler.
It freezes and hashes both map/RGB/depth inputs, validates the shared-frame
fusion contract, filters operator-rejected semantic classes only in a copied
decision tensor, runs the real Perception -> Judgment -> Decision cascade in
robot order, and removes each allocated frontier before the next call. Its
default behavior rejects a mapping-locked input, an input older than 30 s, or
cross-robot capture skew above 5 s. Even when explicitly used for
blocked/stale-input forensics, it has no `GOAL` publication path: optional
Hub publication is `HOLD`, and `shadow_target.json` is consumed only by this
relay. Targets expire and are rejected on robot, frame, transform, shared
calibration or display-only-authority mismatch. The 2026-07-22 observed run is
recorded in
`audit/LIVE_VLM_SHADOW_20260722.md`.

`hub/tools/live_vlm_scene.py`, normally entered through
`hub/scripts/run_live_vlm_scene.sh`, composes those immutable rounds into one
source-derived episode state. It requires a new fresh synchronized accepted
keyframe from every robot per round and preserves the executable HPC shared
history, frozen sequential candidate copies, decision clock, frontier/history
gate and target-semantic override. It remains shadow-only: `Find_Goal` is
reported as a paused handoff awaiting robot-local planner STOP and independent
target verification; it is never episode completion or navigation success.
The operator procedure is in
`hub/docs/VLM_LIVE_EXPERIMENT_20260723.md`.

A stationary depth camera naturally produces one fan-shaped observed sector.
It cannot yield a complete room outline. Movement tests require an operator at
the robot and remain separate from Hub software deployment.

The relay retains the latest JPEG and republishes it with its original
timestamp for late/reconnected Foxglove subscribers. Status age is based only
on real camera pushes and the actual map file mtime, so this retention does not
claim a stale image is fresh.

Offline parameter replay, RedNet confidence diagnosis, the operator-present
moved-map acceptance gate and reuse of the existing board calibration scripts
are documented in [OFFLINE_MAP_VALIDATION.md](OFFLINE_MAP_VALIDATION.md).
