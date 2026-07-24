# WSJ calibration sensor-epoch recovery — 2026-07-24

## Observed failure

The `20260725-lab01` moved-board calibration itself passed, but the subsequent
read-only BuildMap startup stopped at:

```text
TinyNav data-plane verification failed:
timed out waiting for fresh ROS messages: occupancy
```

Observed robot-side evidence:

- `/semantic_mapping/occupancy_bev` had one live publisher, but the occupancy
  mapper reported `input_hz=0`, `pairs=0`, and `keyframes=0`.
- the semantic point-cloud node repeatedly reported that it was waiting for
  synchronized RGB-D input;
- TinyNav perception repeatedly rejected stereo frames with
  `IMU watermark did not catch up`;
- raw RealSense IMU remained healthy at approximately 199–200 Hz;
- Hub policy remained `GOAL=false` for both robots and no Go2 bridge existed.

The board artifact is therefore not the failed component. Its independent
holdout passed with 0.02686 m center residual, 2.6245 degree normal residual,
0.1453 s synchronization skew, 0.1177 m board translation, and 12.1564 degree
board rotation.

## Source-derived diagnosis

Camera and TinyNav perception have separate process lifetimes. A camera restart
can leave the perception process holding the preceding IMU timestamp epoch.
RGB then remains live while processed depth and every downstream occupancy
product stop. The previous launcher checked only RGB and consequently detected
the fault only at the final occupancy verifier.

With Hub goals disabled, navigation paused, the v2 receiver removed, and no
chassis bridge present, perception was respawned while the complete BuildMap
compute load remained active. Fresh messages then returned on:

- `/slam/depth` at sensor stamp `1784891913.271535400`;
- `/slam/keyframe_odom` at `1784891915.005663086`;
- `/semantic_mapping/occupancy_bev` at `1784891918.609084229`.

This is observed evidence that coordinated perception recovery fixes the fault
under the final compute load; it is not a claim that raw IMU transport failed.

## Implemented fix

- `start_wsj_calibration_observation.sh` may recover dead or stale camera and
  perception panes only before any board frame defines the tracking epoch.
- A camera recovery always forces a perception recovery.
- Calibration now requires fresh RGB, processed depth, keyframe depth,
  keyframe odometry, and both camera-info streams, followed by a second
  processed-depth/keyframe-pose check after a short soak.
- `start_wsj_buildmap_v2.sh` no longer silently respawns camera or perception
  after calibration. A stale calibrated stream fails immediately with an
  instruction to create a new board-calibration session, because restarting it
  would change the tracking origin.

No TinyNav source or dependency tree was modified.

## Follow-up: asymmetric-rate holdout selection

The next `20260725-lab02` run passed sensor recovery and its initial board fit,
then timed out selecting the moved-board holdout. Both robots had continued to
upload and both latest images visibly contained the full board. The failure
was a second, independent local selection bug:

- WSJ keyframes arrived at approximately 0.27 Hz;
- Yunji observations arrived at approximately 10 Hz;
- the selector examined the latest 12 frames from each robot, representing
  roughly 44 seconds for WSJ but only 1.2 seconds for Yunji;
- a valid Yunji frame could therefore be discarded before the matching slow
  WSJ keyframe was tested.

Replaying the immutable spool with a complete candidate window found a valid
pair immediately: WSJ sequence `22677`, Yunji sequence `209000`, synchronization
skew 0.201624 s, with the 10x7 board detected in both images.

`select_live_board_pair.py` now loads inexpensive timestamp metadata over the
freshness window, pairs observations by capture time first, and runs the board
detector only on synchronized candidates. A regression test covers the
0.27-Hz/10-Hz asymmetric-rate case.

## Verification and provenance

- both edited shell launchers pass `bash -n`;
- the complete local `hub/tests` suite passes;
- `git diff --check` passes;
- physical validation used observation-only ROS topics and issued no motion
  target.

| Artifact | Size | SHA-256 | Classification |
|---|---:|---|---|
| `hub/robot_overlay/start_wsj_calibration_observation.sh` | 9,303 B | `e5a4f8737ae65c8b4899d82c30e9aafa44921d9fa841485a47bc64659dfd47e3` | source-derived and locally tested |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 10,585 B | `2f66c0a2d43eafffb99e08673d8e852b692514bf47225fae876537238e7c0aa1` | source-derived and locally tested |
| `hub/tools/select_live_board_pair.py` | 16,862 B | `4ec419f861e3e67816224c73ddf0263c10e0dd68b4e7de23aa21c52d91da31a6` | source-derived, locally tested, and replay-validated |
| `hub/runtime/calibration_sessions/20260725-lab01/shared_frame.json` | 5,832 B | `32134c4d205492bb18234cf4c112ea9b2229716c36e2e2e170efa698140b159b` | observed passed board artifact; incomplete session |
