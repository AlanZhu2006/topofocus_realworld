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

## 5. Reuse the existing board calibration

Reuse the existing tools; do not reuse an old session's matrix. First compute
the synchronized board-relative camera offset with
`calibrate_camera_offset_via_board.py`, using each camera's current intrinsics:

```bash
hub/.venv/bin/python hub/tools/calibrate_camera_offset_via_board.py \
  --reference-image <wsj-board-image> \
  --other-image <yunji-board-image> \
  --rows 7 --cols 10 --spacing-m 0.04 \
  --reference-fx <fx> --reference-fy <fy> \
  --reference-cx <cx> --reference-cy <cy> \
  --other-fx <fx> --other-fy <fy> \
  --other-cx <cx> --other-cy <cy> \
  --output hub/runtime/calibration/current-board-offset.json
```

Immediately after the synchronized capture, turn that offset and the latest
matching poses into the versioned shared-frame file:

```bash
hub/.venv/bin/python hub/tools/calibrate_shared_frame.py \
  --spool hub/runtime/spool \
  --reference-robot robot-0 \
  --other-robot robot-1 \
  --offset-file hub/runtime/calibration/current-board-offset.json \
  --max-sync-skew-s 1.0 \
  --transform-version <new-session-transform-version> \
  --calibration-id <new-shared-calibration-id> \
  --output hub/runtime/calibration/<new-session>.json
```

The output now records `shared_frame_calibration_id` and hashes of both source
metadata files and the board offset. Apply the transform only to the "other"
sender, bind both fresh map daemons to the same calibration ID, and start each
map strictly after its pre-calibration sequence. Keep `--fuse` off until a
shared physical landmark independently overlays with a recorded residual and
the calibration's camera-frame assumptions have been checked for both mounts.

The July 21 board-calibration scripts and procedure are reusable. Its numeric
matrix is not evidence for the current WSJ v3/Yunji sessions after sender,
camera mount or SLAM-origin changes.
