# G3 single-robot replay driving central RedNet mapping

Date: 2026-07-18 (Asia/Shanghai)

Target: `asus4090`, environment `hub/.venv` (torch 2.8.0+cu128), RedNet
checkpoint `artifacts/checkpoints/rednet_semmap_mp3d_40.pth` (epoch 53).

## Input sample (observed, immutable)

Recorded by the robot itself on 2026-07-17 (TinyNav `build_map_node` "map
record" format, not a rosbag):

```text
wsj:/home/nvidia/twork/tinynav/output/semantic_map_record_20260717_102052
```

copied read-only via rsync to
`data/robot_replays/wsj_semantic_map_record_20260717_102052/` (646 MB), then
converted with `/usr/bin/python3.10 hub/tools/extract_tinynav_record.py`
(system interpreter: the depth store is a Berkeley DB `shelve` that the conda
stack cannot open) into
`data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted/`
(303 raw-pickle depth keyframes, 472 MB, every source and extracted file
SHA-256'd in its `manifest.json`, payload files chmod 444).

Format semantics were verified against TinyNav source on the robot before use:

- 303 keyframes; identical timestamp sets across `depths.db`, `poses.npy`
  and `rgb_images_db/meta.json`; video frame indices monotonic in timestamp
  order (all checked at load time by `TinyNavReplayReader`).
- depth: float32 metres, rectified infra1 frame (`intrinsics.npy`), 0 =
  invalid; sample frame valid range 0.43–1.94 m.
- `poses.npy`: camera-to-world for the infra1 optical frame; TinyNav world is
  gravity-aligned, +z up (confirmed from `generate_occupancy_map`, which
  builds its BEV with `np.max(..., axis=2)`).
- `T_rgb_to_infra1` maps RGB-optical points into infra1-optical
  (`T_world_rgb = pose @ T_rgb_to_infra1`, the same composition TinyNav's own
  `tool/convert_to_nerf_format.py` uses).
- RGB keyframes are H.264 CRF-18 re-encoded (lossy) — inherent to the robot's
  recorder, recorded here as a data limitation.

## Gate command and result

```bash
hub/.venv/bin/python hub/tools/g3_replay.py \
  --record data/robot_replays/wsj_semantic_map_record_20260717_102052 \
  --extracted data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted \
  --output data/robot_replays/g3_output_wsj_20260717_102052 --runs 2
```

Result: **PASS**, exit 0.

- 303/303 keyframes fused per run; map extent auto-sized to 18 m (trajectory
  bounding box + sensor range), 360×360 cells at the upstream 5 cm resolution,
  17 channels (obstacle, explored, 15 HM3D categories); floor height estimated
  deterministically at −0.403 m (camera ≈0.4 m above floor, consistent with a
  Go2-mounted RealSense).
- Determinism: two full independent runs produced byte-identical fused maps,
  SHA-256 `9d3a5fa1c7336e57…` both times (`g3_run_manifest.json` keeps the full
  hashes, per-run stats and the input-manifest hash). RedNet inference repeated
  on the same frame is bit-identical on this GPU/stack.
- Runtime ≈25 s per full 303-frame replay (RedNet + splat + fusion).
- Unit tests: `hub/.venv/bin/python -m pytest hub/tests -q` → 16 passed
  (11 pre-existing + 5 new replay/mapper tests, including synthetic-geometry
  placement and mismatched-keyframe rejection).

## Independent geometry cross-check (observed)

The same record contains TinyNav's own occupancy output
(`occupancy_grid.npy`, 0.1 m raycast voxel grid), produced by a completely
different geometry pipeline (raycasting with ground filtering vs. our
height-band splatting). Comparing in world coordinates:

- 86.8 % of our obstacle cells lie within 10 cm of a TinyNav occupied cell;
- 86.4 % of our explored area lies inside TinyNav's known (free|occupied) area;
- raw obstacle IoU 0.543 at 10 cm (different resolutions/definitions bound
  this below 1 even for perfect data);
- visual overlay `crosscheck_vs_tinynav.png` shows the two maps coincide with
  no mirroring, rotation or scale error.

This validates pose/intrinsics/extrinsics interpretation, which is the G3
safety-relevant claim. It does not validate semantic quality.

## Recorded deviations and limitations

- Upstream's mapper integrates Habitat 2-D dead-reckoned poses via
  grid_sample warps; the hub mapper transforms points directly with the SE(3)
  SLAM poses. Thresholds (map/exp/cat), 5 cm resolution, height-band obstacle
  definition, 15-category `mp_categories_mapping` and element-wise max fusion
  are kept source-derived.
- Upstream gates six semantic channels with a second Grounded-SAM model; the
  replay mapper uses RedNet alone (mapping_only scope).
- RedNet is trained on MP3D indoor scans; on this real corridor-like scene
  only `plant` (109 cells) crossed the 0.5 threshold. Semantic richness on
  real scenes is a known open item for later gates, not a G3 criterion.
- Poses come from TinyNav SLAM (loop-closed), not independent ground truth.
- Single robot, mapping only: no transport, no dual-map fusion, no commands.

## Scope

G3 proves: a real robot's recorded synchronized RGB-D + pose stream, in its
native format, drives the central RedNet semantic mapper to a reproducible,
hash-stable world-frame map that geometrically agrees with the robot's own
independent occupancy pipeline. It is not evidence of live transport (sender),
two-robot shared-frame fusion (G4) or hardware-in-loop safety (G5).
