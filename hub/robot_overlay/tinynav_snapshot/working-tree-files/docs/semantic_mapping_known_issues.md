# Semantic Mapping Known Issues

## Validation Coverage

- Live D435i aligned RGB-D, TinyNav `world -> camera`, point-cloud publication,
  and RViz rendering are validated.
- The older complete 119-second bag still predates aligned-depth publication
  and cannot validate semantic geometry.
- The new 5.67-second posed RGB-D bag is minimal and lives on tmpfs; move it to
  durable storage before reboot if it must be retained.
- Genuine saved-map relocalization and live map-frame overlay were validated
  against the new map. A later restart from a feature-poor static view produced
  only 31-47 matches against TinyNav's required 50; a small camera motion may be
  required to relocalize again.

## Pose and Frame Contract

- TinyNav `camera` is an alias for left-infrared optical coordinates and is not
  connected to the RealSense TF tree.
- `world -> map` is stamped with node time, not the relocalized keyframe stamp.
  Exact historical `map <- camera` lookup can fail around startup/corrections;
  the mapper may use latest map alignment while retaining exact image-time
  camera pose.
- `/mapping/current_pose_in_map` is POI-gated and stamped at publish time, so it
  is not suitable as the primary semantic integration pose.
- There is no `base_link` or calibrated Go2 body-to-camera TF.

## RGB/Depth Alignment

- Existing raw depth is `camera_depth_optical_frame`, `16UC1`, and has different
  intrinsics from RGB.
- The complete local bag cannot be used as if raw depth were aligned depth.
- Full future recordings must include aligned-depth image and CameraInfo.
- Current `build_map_node.py` string-matches an infrared static TF edge that is
  absent in the newest bag; its saved `T_rgb_to_infra1.npy` is therefore `None`.

## Map Semantics

- TinyNav static occupancy uses a non-height-aware max projection for its RViz
  map. It is only an alignment reference for this project.
- Sparse occupancy fusion, DDA free-space carving, height-aware occupancy BEV,
  and NPZ/YAML geometry serialization are implemented in Phase 2.
- Local ground RANSAC, startup bootstrap, and conservative temporal filtering
  are implemented. The mapper does not require a fixed body-to-camera
  transform when TinyNav supplies timestamped `map -> camera` TF. A measured
  `base_link -> camera_link` transform is still needed for independent
  collision-envelope validation before hardware safety certification.
- Ground-band occupied endpoints are treated as traversable support. This
  avoids floor-as-obstacle failure but can intentionally ignore very low
  obstacles below `ground_max_z_relative`.
- The sparse Python dictionary is correctness-first. DDA is bounded to 6,000
  rays per keyframe and BEV column reduction is vectorized; very large
  environments may still require a C++/block-map storage backend.
- Offline construction stores the mapping session as frame `world`. The copied
  semantic auto-nav script explicitly aliases that saved occupancy to `map`
  only when it is nested inside the selected TinyNav map directory. Generic
  launch usage rejects frame mismatches unless override is explicitly enabled.
- Phase 4 implements a separate static 3D semantic voxel layer. Unknown,
  below-confidence, and configured dynamic labels are excluded from that layer;
  dynamic geometry still needs a distinct short-lived occupancy layer rather
  than only semantic filtering.
- Phase 5 now emits a semantic BEV and `semantic_bev_tensor.npz` aligned to the
  Phase-2 occupancy grid. Semantic labels deliberately do not overwrite
  Phase-2 collision occupancy; planners must consume the two channels together.
- The historical Phase-5 artifact has `ground_z=0.0 m` but confirmed floor
  semantic voxels near `z=-0.4 m`, so it retains wall cells but not floor
  cells. It predates startup ground bootstrap and is retained only as a
  historical result. The rebuilt full-bag artifact records `ground_z=-0.380 m`
  and contains floor semantics. Do not manually patch the historical artifact
  for collision decisions.
- A geometry BEV may grow on the final checkpoint. The copied auto-map script
  now reruns `scripts/export_semantic_bev.py` after shutdown so the persisted
  semantic tensor adopts the final occupancy origin, resolution, width, height,
  and ground reference. Direct users of the save services should run the same
  export command after both map services have completed.
- ADE20K-to-navigation collapse is heuristic. `target` has no ADE20K source
  class, and small/thin furniture boundaries are coarse because the model
  logits are 128x128 before nearest-neighbor restoration. Phase 4 must use
  confidence weighting and mask-boundary erosion rather than treating one
  frame as ground truth.
- The TensorRT engine is specific to the local TensorRT/CUDA/Jetson build. Run
  `scripts/prepare_segformer_trt.sh --force-engine` after changing runtime or
  hardware; do not copy the engine across incompatible devices.
- Precomputed matching defaults to 50 ms. The actual source-time error is
  logged; accelerated rosbag playback can drop best-effort RGB samples and is
  not used as the deterministic correctness path.

## Runtime

- TinyNav perception pose publication lag measured about 0.4-0.8 seconds on the
  tested Jetson. The mapper buffers for 2.0 seconds; a sustained lag above that
  threshold causes explicit drops.
- Minimal bag recording started before its first pose sample, so three leading
  RGB-D frames are intentionally dropped during replay.
- Offline Phase-2 replay confirmed 5,942 active voxels and a 39x45 BEV. Live
  saved-map operation has also been checked with roughly 140k voxels and a
  94x141 BEV.
- The 65.29-second Phase-4 full auto-map run saved 3,042 active semantic voxels
  and 2,324 confirmed static labels. Its later TinyNav pose stream had nine
  jump events, so the configured gate stopped additional irreversible fusion
  after five accepted keyframes. This protects the output but means the current
  full-bag semantic coverage is intentionally limited until semantic keyframe
  persistence and rebuild-after-correction are implemented.
- The target-alignment gate intentionally publishes no new point cloud while
  TinyNav relocalization is unavailable. The loaded occupancy/BEV remains
  available through transient-local publishers.
