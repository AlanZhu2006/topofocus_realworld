# Offline map diagnostics and operator moved-run gate

This workflow separates work that is safe to do without a robot operator from
the one gate that requires physical motion. None of the tools in this document
publishes a robot target or changes `allow_goal`.

## 1. Foxglove interpretation

The relay publishes two map views per robot:

- `/<name>/geometry_map`: default dashboard view. Gray is unknown, white is
  observed free space, and black is current geometric obstacle evidence.
- `/<name>/semantic_map`: geometry plus RedNet categories. It is retained for
  diagnosis but hidden by default while the real-camera semantic gate is open.
- `/<name>/map_pose`: red current camera XY and a blue trail sampled since the
  relay process started. It is a camera marker, not a fabricated body pose.

Evidence is reduced before colors are assigned. The old RGBA block average
could blend unrelated category colors into irregular purple/yellow patches;
the new path always emits an exact legend/category color.

Foxglove does not update an already-imported layout when the repository JSON
changes. Re-import `hub/foxglove/dual_robot_dashboard.json` after deployment.

## 2. Bounded geometry parameter replay

This runs only against append-only spooled RGB-D observations. It uses the live
startup-pose gate, three-frame RANSAC ground estimate and keyframe selector, but
zeroes semantic predictions so every profile sees identical geometry inputs.

```bash
hub/.venv/bin/python hub/tools/analyze_live_map_sweep.py \
  --spool hub/runtime/spool \
  --robot-id robot-0 \
  --start-after-sequence 6217 \
  --max-observations 220 \
  --max-keyframes 30 \
  --output hub/runtime/analysis/wsj-map-sweep
```

The default profiles compare the deployed live policy, three-hit persistence,
a 0.60 m obstacle-band upper edge, and legacy irreversible maximum fusion.
The JSON records every source observation path and checksum plus coverage,
obstacle density, connected components, thin cells and checkpoint history.

These numbers cannot select an "accurate" profile by themselves: there is no
surveyed cell-level ground truth. A cleaner-looking profile remains unverified
until the controlled moved run below.

## 3. RedNet live-spool diagnosis

Use a bounded range; an unbounded live spool is intentionally rejected.

```bash
hub/.venv/bin/python hub/tools/analyze_rednet_domain_gap.py \
  --spool hub/runtime/spool \
  --robot-id robot-0 \
  --start-after-sequence 6502 \
  --max-frames 10 \
  --output hub/runtime/analysis/wsj-rednet
```

The report separates raw MP3D-40 argmax from the production output after the
upstream fixed 0.8 confidence threshold. It also writes RGB, raw-class,
thresholded-class and confidence images with hashes. Do not lower the online
threshold merely to make colors appear: a labelled real-camera validation set
is required before raw low-confidence regions can be treated as objects.

## 4. Operator-present moved-run gate

Do not perform this section remotely. An operator must be beside the robot,
the local emergency stop must be available, posture must remain fixed, and Hub
health must still report `goal_output_enabled=false`. Movement is performed
only through the already-approved local/manual interface; no Hub tool below
contains a motion command.

1. Confirm the live map is not halted and copy a stable snapshot directory to
   an ignored checkpoint location. For WSJ, for example:

   ```bash
   mkdir -p hub/runtime/moved_gate
   cp -a hub/runtime/map_out_wsj_live_v2_20260722 \
     hub/runtime/moved_gate/wsj-before
   ```

2. Record the baseline `last_observation_sequence`. Have the operator move at
   low speed along a short, collision-free path with at least 0.5 m cumulative
   XY displacement and multiple overlapping views. Do not change standing/
   lying posture during the run.
3. Wait for a new map snapshot, stop manual motion, then copy the final state:

   ```bash
   cp -a hub/runtime/map_out_wsj_live_v2_20260722 \
     hub/runtime/moved_gate/wsj-after
   ```

4. Run the read-only gate:

   ```bash
   hub/.venv/bin/python hub/tools/validate_moved_map_run.py \
     --before hub/runtime/moved_gate/wsj-before \
     --after hub/runtime/moved_gate/wsj-after \
     --spool hub/runtime/spool \
     --robot-id robot-0 \
     --output hub/runtime/moved_gate/wsj-report.json
   ```

The default gate requires a stable map contract, no pose-jump latch, at least
three new integrated keyframes, 0.5 m accepted path, bounded adjacent pose
steps, changed cells, at least 25 newly explored cells, and obstacle/explored
ratio no greater than 0.50. Passing proves continuity and bounded map behavior,
not metric SLAM accuracy or autonomous-navigation readiness.

## 5. Reuse the board, but preserve gravity

Reuse the physical board and PnP detector; do not reuse an old session's
numeric matrix. There are two cases.

If both published camera poses and mounts are independently verified against a
gravity-aligned TF tree, the original two-step flow remains available:
`calibrate_camera_offset_via_board.py` followed by
`calibrate_shared_frame.py`. Check the resulting transform's +Z tilt before
accepting it.

For a local camera outside TF (the Yunji D455 case), first derive its mount
rotation from several archived floor views spanning materially different base
headings. Use an explicit nominal artifact so its provenance is not hidden:

```bash
hub/.venv/bin/python hub/tools/derive_ground_camera_extrinsic.py \
  --spool hub/runtime/spool \
  --robot-id robot-1 \
  --sequence <heading-a-frame-1> \
  --sequence <heading-a-frame-2> \
  --sequence <heading-b-frame-1> \
  --sequence <heading-b-frame-2> \
  --nominal-extrinsic hub/config/calibration/<nominal-mount>.json \
  --camera-model d455 \
  --camera-frame d455_color_optical_frame \
  --output hub/runtime/calibration/<corrected-mount>.json
```

The tool preserves translation and corrects only the observed orientation. It
refuses output if any selected frame's residual floor tilt exceeds its gate.
Do not use multiple near-duplicate frames from one heading as evidence that the
mount will remain correct after yaw.

Then reuse the existing `find_board_pose()` implementation through the
gravity-constrained board tool. Give it the old mount/transform that were
already baked into each recorded Yunji pose so it can reconstruct odom/base
before applying the corrected mount:

```bash
hub/.venv/bin/python hub/tools/calibrate_gravity_shared_frame_via_board.py \
  --spool hub/runtime/spool \
  --reference-robot robot-0 \
  --other-robot robot-1 \
  --reference-sequence <fit-wsj-sequence> \
  --other-sequence <fit-yunji-sequence> \
  --old-other-extrinsic hub/config/calibration/<nominal-mount>.json \
  --corrected-other-extrinsic hub/runtime/calibration/<corrected-mount>.json \
  --holdout-reference-sequence <moved-board-wsj-sequence> \
  --holdout-other-sequence <moved-board-yunji-sequence> \
  --holdout-other-recorded-shared-transform <transform-applied-to-holdout>.json \
  --rows 7 --cols 10 --spacing-m 0.04 \
  --transform-version <new-session-transform-version> \
  --calibration-id <new-shared-calibration-id> \
  --output hub/runtime/calibration/<new-session>.json
```

The fit aligns the board origin with the closest yaw-only rotation; its shared
transform cannot rotate gravity. The moved-board holdout must independently
pass centre, normal and sync-skew gates. Apply the corrected mount and shared
transform together, bind both fresh map daemons to the same calibration ID,
and start each map strictly after its pre-change sequence.

Keep `--fuse` off until the holdout passes and both v3 maps report no ground
drift latch. A camera mount, posture, SLAM-origin or sender transform change
always requires a new map session even if a previous board artifact exists.
