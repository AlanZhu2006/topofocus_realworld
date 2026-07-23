# 2026-07-23 live VLM experiment quick start

> **Historical procedure.** This page records how the July 23 session was
> brought up. Its `v1` placeholders and statements that v2 is unverified are
> superseded by [CURRENT_STATUS.md](../../CURRENT_STATUS.md). The current
> no-motion/live entry point is `hub/scripts/realworld_oneclick.sh`; do not
> rerun these dated commands against the 2026-07-24 maps.

## What can run immediately

The first on-site experiment remains a real-sensor, real-map, real-GLM
two-robot shadow run. It produces source-shaped allocations and displays them
in Foxglove while publishing no physical goal. After that passes, the shortest
physical continuation is now documented in
[V2_PHYSICAL_QUICKSTART.md](V2_PHYSICAL_QUICKSTART.md).

At the time these steps were written, the runtime was still mapping-only. That
is no longer the current implementation state: the versioned receivers and
both planner paths have since been exercised under supervision, while the
safe resting state still uses `allow_goal=false`. No attempt has met the
official success definition.

Both robots start new odometry origins after their power cycles. The July 22
transform and calibration IDs are invalid for this session.

## 1. Start WSJ observation and upload

With an operator beside the robot, and before starting any navigation:

```bash
ssh wsj
cd /home/nvidia/focus_sender
bash verify_go2.sh --hardware \
  --tinynav-root /home/nvidia/focus_sender/tinynav_imu_fix_worktree_20260721
bash start_wsj_mapping_session.sh \
  --env /home/nvidia/focus_sender/go2_20260723.env \
  --transform-version wsj-tinynav-depth-20260723-session-v1
```

The command refuses known planner/control processes and reports the first Hub
sequence advance before returning. It starts only D435i, patched perception,
the TinyNav-native mapping sender and Foxglove preview.

## 2. Start Yunji/Odin in raw calibration mode

Power Yunji and Odin, keep the chassis stationary, and stop the old calibrated
sender so it cannot reuse the July 22 transform. Use non-login Bash, as the
login environment loads the incompatible MVS `libusb`:

```bash
sudo systemctl stop focus-yunji-odin1-sender.service
sudo systemctl start focus-yunji-odin1-driver.service

tmux new-session -d -s focus_odin1_raw_20260723 \
  "/bin/bash -c 'cd /home/nyu/focus_sender_odin1 && \
   set -a && source focus-odin1.env && set +a && \
   unset FOCUS_ODIN1_SHARED_TRANSFORM_FILE && \
   source /opt/ros/humble/setup.bash && \
   source /home/nyu/odin_ws/install/setup.bash && \
   exec python3 -u odin1_sender.py \
     --calibration-file odin1_O1-P070100205_factory_20260722.json \
     --transform-version yunji-odin1-raw-20260723-v1 \
     --rate-hz 1 \
     --metrics-out runtime/odin1_raw_20260723_metrics.json'"
```

This sender reads WATER health but contains no `/api/move` call.

## 3. Recalibrate the shared frame

Place the existing 7x10 circle board where both cameras see it. Record one
synchronized pair, move only the board, then record a second independent
holdout pair. Select pairs with no more than 250 ms capture skew.

```bash
hub/.venv/bin/python hub/tools/calibrate_gravity_shared_frame_via_board.py \
  --spool hub/runtime/spool \
  --reference-robot robot-0 --other-robot robot-1 \
  --reference-sequence <wsj-fit-sequence> \
  --other-sequence <odin-fit-sequence> \
  --holdout-reference-sequence <wsj-holdout-sequence> \
  --holdout-other-sequence <odin-holdout-sequence> \
  --other-pose-is-camera \
  --transform-version yunji-odin1-board-20260723-v1 \
  --calibration-id shared-board-odin1-20260723-v1 \
  --output hub/runtime/calibration/yunji_odin1_board_20260723_v1.json
```

Do not continue unless both the fit and moved-board holdout pass. Copy the
result to Yunji, stop the raw tmux session, and restart `odin1_sender.py` with
both of these explicit arguments:

```text
--transform-version yunji-odin1-board-20260723-v1
--shared-frame-transform-file /home/nyu/focus_sender_odin1/yunji_odin1_board_20260723_v1.json
```

The WSJ sender remains the reference frame and does not need the other-robot
transform file.

## 4. Start completely fresh maps

Remove the board from the navigation view, note the latest sequence for each
robot, and create new map directories:

```bash
bash hub/scripts/start_fresh_dual_maps.sh \
  --session-tag 20260723_v1 \
  --calibration-id shared-board-odin1-20260723-v1 \
  --wsj-transform wsj-tinynav-depth-20260723-session-v1 \
  --yunji-transform yunji-odin1-board-20260723-v1 \
  --wsj-start-after <latest-wsj-sequence> \
  --yunji-start-after <latest-yunji-sequence>
```

Wait for both `live_status.json` files to exist and report:

```text
mapping_blocked_reason = null
shared_frame_calibration_id = shared-board-odin1-20260723-v1
semantic_yolo.failures = 0
```

The startup plane requires three stable floor-bearing frames. A wall/table-only
view is not enough.

## 5. Switch the existing Foxglove connection

Restart the relay on the unchanged 8765/8766 ports with:

```text
robot-0:wsj:hub/runtime/map_out_wsj_20260723_v1
robot-1:yunji:hub/runtime/map_out_yunji_20260723_v1
```

Keep `--fuse`. No new layout is required because robot names and topics are
unchanged. Confirm camera/map freshness and the new calibration ID in status;
do not judge freshness from a retained JPEG alone.

## 6. Run one real VLM smoke round

This single wrapper checks that Hub GOAL is disabled, GLM responds, maps are
fresh/unblocked, source captures are within 30 seconds and 5 seconds of each
other, and the exact new calibration ID matches. It has no forensic override:

```bash
bash hub/scripts/run_live_vlm_shadow.sh \
  --wsj-map hub/runtime/map_out_wsj_20260723_v1 \
  --yunji-map hub/runtime/map_out_yunji_20260723_v1 \
  --calibration-id shared-board-odin1-20260723-v1 \
  --goal-category chair
```

Expected output is one distinct frontier per robot, two HTTP-202 `HOLD`
records, and magenta `SHADOW ... NO MOTION` markers that expire after ten
minutes. The complete manifest freezes/hashes all inputs and VLM outputs.

This command is a one-round plumbing check, not a completed HPC scene.

## 7. Run one continuous source-derived shadow scene

After the smoke round passes, start a new uniquely named scene. The supported
HM3D ObjectNav targets are the six from the original HPC task: `chair`,
`bed`, `plant`, `toilet`, `tv`, and `sofa`.

```bash
bash hub/scripts/run_live_vlm_scene.sh \
  --wsj-map hub/runtime/map_out_wsj_20260723_v1 \
  --yunji-map hub/runtime/map_out_yunji_20260723_v1 \
  --calibration-id shared-board-odin1-20260723-v1 \
  --scene-id chair_20260723_01 \
  --goal-category chair
```

The runner waits for a newer accepted YOLO keyframe from both robots before
every round. It preserves one shared history across both agents, freezes the
agent-0 history candidates for sequential allocation, uses the exact source
decision clock `0, 24, 49, ..., 499`, and keeps all 15 source semantic classes
visible to the VLM. Each round still publishes only expiring `HOLD` plus a
Foxglove display marker.

A shadow scene can end in only these meaningful states:

- `paused_shadow_target_found_awaiting_robot_local_planner_stop`: the
  source `Find_Goal` rule saw any positive target-semantic evidence and chose
  its largest connected component. This is model-derived, unverified target
  evidence, not arrival, episode termination, or navigation success. Source
  still needs local planner STOP; HM3D `multi_Total_SR` also requires the GT
  agent to find the target, whose real-world analogue must be independent
  target verification.
- `complete_shadow_max_steps_without_target`: all 21 source decision
  opportunities through logical step 499 completed without target evidence.
- `aborted_fail_closed` / `aborted_internal_fail_closed`: calibration,
  freshness, synchronization, mapping, Hub policy, model, or internal state
  violated its contract.

`--max-rounds` may intentionally stop earlier for a bounded test, producing
`complete_requested_shadow_rounds`. The logical step count is a source-derived
shadow clock; it does not claim the robots executed Habitat's 500 discrete
actions. The only deliberate behavioral safety deviation is that source's
no-frontier random map point becomes HOLD. Do not add a confidence/multi-frame
success rule or call a History VLM: neither exists in the executable HPC path.

## Stop/rollback

- WSJ: `bash /home/nvidia/focus_sender/stop_go2_observation.sh --session focus_wsj_mapping_20260723`.
- Yunji: stop the current Odin sender tmux/service; do not kill the driver with
  `SIGKILL`.
- Hub maps: send `Ctrl-C` to both windows in `shared_maps_20260723_v1`.
- Continuous scene: send `Ctrl-C`; it records `stopped_by_operator` and does
  not convert the partial scene into success.
- Runtime maps and audit artifacts are preserved; do not delete or append a new
  transform epoch to them.
