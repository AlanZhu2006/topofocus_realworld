# WSJ BuildMap smoke, RealSense depth failure isolation, and TinyNav-native mapping recovery — 2026-07-21

## Scope and safety boundary

This run continued the WSJ mapping investigation without starting a planner,
`cmd_vel`, Unitree sport/motion control, or any other actuator path. The robot
was kept stationary for the bounded validation. Hub `/healthz` reported
`goal_output_enabled.robot-0=false`; every accepted observation had
`mapping_only=true`.

The original TinyNav checkout and its launcher were not modified. Perception
changes remained isolated in
`/home/nvidia/focus_sender/tinynav_imu_fix_worktree_20260721`; deployment-only
changes were made under this workspace's `hub/` package and copied to a new
remote filename.

## TinyNav BuildMap static smoke

`tinynav/core/build_map_node.py` was verified identical to the original source
(45,365 B, SHA-256
`4fcac8a38e870981e202bdcf133ab1bad3bbe58b0fa46c5950fba17e1eaae82e`).
It was run against the live stationary robot in an isolated tmux pane and
output directory
`/home/nvidia/focus_sender/buildmap_live_smoke_20260721_2semdr`.

Observed:

- BuildMap ran for roughly 90 seconds, accumulated 24 poses, created online
  static loop constraints, and completed Ceres solves.
- Its process used roughly 1.25–1.31 GiB RSS. Total Jetson RAM use rose from
  about 4.17 GiB to 4.94 GiB; GPU utilization was bursty and reached 99% once.
- During the run, perception observed an IMU sample gap of
  `0.310459852 s` and correctly emitted
  `optimizer_status=skipped_imu_invalid`. BuildMap was stopped before any
  physical movement.
- Graceful SIGINT stopped BuildMap in about three seconds. TinyNav's shutdown
  order invalidated the `rclpy` context before the save path finished, so this
  run did not produce `poses.npy` or an occupancy-grid export. The partial
  artifacts were preserved; this is a failed static gate, not a saved map.

Preserved partial artifacts include `depths.db` (39,432,192 B), `features.db`
(12,890,112 B), `embeddings.db` (110,592 B),
`mapping_continuous_odom.npy` (383,819 B), `tf_messages.npy` (3,767 B), and
`build_map_live.log` (5,096 B).

## Perception recovery repair

The previous fail-closed path rejected the tentative current frame but retained
the historical bad interval in the keyframe queue. One gap therefore poisoned
all future optimizations. Commit
`29f26bc058886ff450f02cdc0d6e9977e1c57010` keeps only the newest sensor frame
as a timestamp/image anchor after rejection, freezes the last trusted pose,
zeros velocity conservatively, and still publishes no pose for the rejected
frame.

Remote dependency-free tests passed (`7 passed` with third-party pytest plugin
autoload disabled). In the recovered live session, repeated invalid intervals
were rejected individually and later valid intervals returned to
`optimizer_status=ok`; the historical interval no longer grew without bound.

Important launch finding: running perception with `python -m` stalled the first
full pass in this environment. The original script entry point
`python -u tinynav/core/perception_node.py`, with the worktree first on
`PYTHONPATH`, completed the cold pass and was retained.

## RealSense depth-stream failure and isolation

The Hub sender initially produced no new observations because its four-way
synchronizer saw current color/keyframe pose but a stale
`/camera/camera/aligned_depth_to_color/image_raw` stamp. Both raw and aligned
hardware depth were 0 Hz, while color, infrared, and IMU remained live.

Direct camera evidence:

- RealSense ROS `4.58.1`, librealsense `2.58.1`, firmware `5.17.0.10`;
  D435I serial `344422071135`, USB 3.2 at physical path `2-2.2`.
- The source log recorded UVC streamer watchdogs on endpoint 132 at
  `11:19:43 UTC` and endpoint 130 at `11:20:32 UTC`.
- A full ROS restart and a dynamic depth disable/enable did not recover the
  depth stream; the latter blocked while reopening the Depth Module.
- Native librealsense tests, with ROS stopped, disproved a broken camera or
  insufficient aggregate bandwidth:
  - depth only: 30/30 frames in 1.386 s;
  - depth + both infrared streams: 30/30 for all three in 1.515 s;
  - depth + both infrared + color: 30/30 for all four in 2.411 s;
  - motion only: 978 gyro and 507 accel samples in 5 s;
  - all video + motion: 141 frames for each video stream, 1,009 gyro and
    514 accel samples in 5 s.

ROS parameter bisection then showed:

| ROS configuration | Observed result |
|---|---|
| depth only, `align=false`, `enable_sync=false` | live depth; roughly 18–26 Hz during CLI sampling |
| depth + dual infrared + color, no IMU, `align=false`, `enable_sync=false` | live raw depth near 29 Hz |
| all video + IMU, `align=false`, `enable_sync=false` | live raw depth near 29 Hz |
| all video + IMU, `align=true`, `enable_sync=false` | endpoint 130 watchdog reproduced; depth stopped |
| original all-stream profile, `align=true`, `enable_sync=true` | endpoint watchdog reproduced; depth stopped |

At the time of this bisection, the D435i device-level
`power/control` was later found to have reverted to `auto`, while
`usbfs_memory_mb=1000` and the upstream Genesys hub remained `on`. Therefore
the evidence establishes `align_depth` as a repeatable trigger under that
runtime state, but does **not** prove that the align implementation alone is
the root cause when autosuspend is correctly pinned. The current session uses
both mitigations: no align/hardware-depth dependency and `power/control=on`.

The persistent udev asset
`hub/robot_overlay/99-realsense-usb-power.rules` now matches
`add|bind|change`, not only `add`, so a later driver bind/rebind reapplies
`power/control=on`. It was installed at
`/etc/udev/rules.d/99-realsense-usb-power.rules`; the prior rule is recoverable
as `99-realsense-usb-power.rules.bak-20260721-1154`.

## TinyNav-native RGB-D transport

The stable camera runtime now enables dual infrared, gyro/accel, and color
(for the independent Foxglove preview), while disabling hardware depth,
`align_depth`, and `enable_sync`. Initial measured rates were:

- infrared 1: 29.7 Hz;
- infrared 2: 30.0 Hz;
- unified `/camera/camera/imu`: 197.8 Hz;
- color preview source: 23.5 Hz.

TinyNav's own synchronized mapping tuple was used instead of RealSense aligned
depth:

- `/slam/keyframe_image`: 848×480 `mono8`;
- `/slam/keyframe_depth`: 848×480 `32FC1` metres;
- `/slam/camera_info`: 848×480 intrinsics;
- `/slam/keyframe_odom`: child frame `camera`.

`hub/robot_overlay/focus_ros_sender.py` now accepts both the unchanged default
`16UC1` millimetre path and the opt-in `32FC1` metre path. The latter uses the
same rounding/clipping semantics as the Hub's existing png16 encoder;
NaN/Inf/negative values become zero. The wire contract is unchanged:
`depth_encoding=png16`, `depth_scale_m=0.001`.

Verification:

- local sender tests: `18 passed`;
- full local Hub suite: `130 passed`;
- remote Python syntax check: passed;
- bounded live run, sequences 4363–4372: 10/10 accepted, zero retries,
  mean upload 477.6 ms, mean pose skew 0.0 ms;
- all ten frames used transform
  `wsj-tinynav-depth-20260721-live-v3`, `mapping_only=true`, child frame
  `camera`, and `DEGRADED / slam_optimizer_imu_valid;covariance_unavailable`;
- decoded depth: 848×480 uint16 for all ten frames, mean valid-range pixel
  ratio 0.8359, median depth range 0.716–0.722 m, p95 range 2.433–2.538 m;
- bounded map checkpoint: 10 frames, sequences 4363–4372, 5,674 explored
  cells and 1,411 obstacle cells.

After the bounded gate passed, the sender resumed continuously from sequence
4373. At the 19:52 local checkpoint the map had processed 29 frames through
sequence 4391, with the camera still stationary near `(0.0015, 0.0002)` m.

## Controlled moved-robot gate

The operator manually moved the robot while all Hub goal output remained
disabled. No planner, velocity, sport/motion-control, or actuator process was
started by this test. The movement is bounded by spooled observations
5012–5018:

- baseline capture: `2026-07-21T20:39:28.652944+08:00`, camera XY
  `(0.0466, -0.8837)` m;
- final moving capture: `2026-07-21T20:39:34.925910+08:00`, camera XY
  `(-0.0161, 0.5978)` m;
- TinyNav-reported XY path length: 1.5110 m; net XY displacement: 1.4829 m;
- displacement components: `(-0.0627, +1.4816, -0.0032)` m in the current
  TinyNav world frame;
- all intermediate positions progressed continuously. The largest adjacent
  keyframe displacement was 0.5871 m over 1.068 s (about 0.55 m/s), rather
  than a zero-time pose jump;
- the following 20-second stationary sample changed by only about 0.98 mm in
  XY, so there was no continuing post-stop drift at that checkpoint;
- sender acknowledgements remained `accepted`, pose skew remained 0.0 ms,
  and observations/maps continued through the movement without a sequence
  reset.

This passes the continuity and transport gate. It does **not** establish
metric pose accuracy: no independently measured start/end distance was
recorded for this motion, so the reported 1.4829 m has no physical ground
truth comparison yet.

Post-run `/slam/data` at `process_cnt=6550` reported
`optimizer_status=ok`, four valid current IMU intervals with maximum sample
gaps near 5 ms, zero rejected/overwritten IMU messages, and zero available
pose covariance. The cumulative invalid-interval reanchor count had reached
220, so IMU scheduling gaps remain a real intermittent condition even though
the recovery path returned to a valid current optimization.

## 2D-map quality finding from the moved run

The moved run confirms that the confusing black fan is a mapper-accumulation
problem, not a failure to receive motion. Before movement, the live checkpoint
at sequence 5011 had 10,179 explored and 9,602 obstacle cells. After movement
and the short stationary tail, sequence 5043 had 10,555 explored and 10,297
obstacle cells. Thus the map changed, but it was already almost saturated as
obstacle.

The mechanism is source-derived from `hub/src/focus_hub/central_mapping.py`:
every valid depth endpoint marks a cell explored, one endpoint in the configured
height band is sufficient to mark it obstacle (`map_pred_threshold=1`), and
temporal fusion is an irreversible element-wise maximum. The continuous sender
also integrates near-duplicate stationary keyframes. A deterministic local
recalculation of channels 0–1 from the preserved spool showed the obstacle /
explored ratio rising while the robot was still stationary:

| Last sequence | Frames | Obstacle | Explored | Obstacle / explored |
|---:|---:|---:|---:|---:|
| 4372 | 10 | 1,411 | 5,674 | 24.9% |
| 4400 | 38 | 1,771 | 5,813 | 30.5% |
| 4500 | 138 | 3,010 | 6,089 | 49.4% |
| 4700 | 338 | 7,549 | 9,009 | 83.8% |

The live sequence-5118 checkpoint reached 10,321 obstacle cells out of 10,575
explored cells (97.6%). Accordingly, the present map is useful as evidence
that RGB-D projection and pose transport are active, but it fails the
navigable occupancy-map quality gate. More physical movement should wait for
motion/keyframe gating and non-irreversible occupancy evidence handling, then
use a clean map session.

## Foxglove state

Foxglove now reads WSJ from
`hub/runtime/map_out_wsj_tinynav_depth_20260721` and Yunji from its unchanged
`map_out_yunji_board_calib_20260721` directory. Cross-robot fusion is disabled:
the fresh WSJ v3 origin has not been re-calibrated against Yunji, so fusing the
two grids would be geometrically false. Individual WSJ/Yunji map topics and the
camera previews remain available. Relay `/healthz` returned both robot names.

The camera preview had one relay-restart disconnect and recovered automatically;
its later checkpoints resumed successful pushes.

## Provenance

| Artifact | Size | SHA-256 | Status |
|---|---:|---|---|
| Remote original RealSense launcher | 1,273 B | `ac48656ed5f2cd0dac680bc2511fc01fbfe834b84c4ded3ba97b9c634a579896` | observed, unchanged |
| Remote patched perception node | 40,286 B | `3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3` | observed/live-tested |
| Remote perception health helper | 2,564 B | `291a2f06dc4fe0c1dd6c9846d00e9a69f37fede8797f6e5731e32b46853a9ddb` | source-derived/tested |
| Remote perception health tests | 2,272 B | `777d8f92f143ef6815be550b52ca432b232488db3aef028d950d0bde4879cdd8` | tested |
| Local and deployed TinyNav-depth sender | 37,212 B | `6195a575ad132660c088a3c981c39f59515a843a5ca660d8950908f6349e7a70` | tested/live-tested |
| Sender tests | 9,149 B | `a2a89d24d9bd782d6b702e78f2e363df027081c5c2bb0cad99377f0e6d498561` | tested |
| Bounded ten-frame sender metrics | 4,872 B | `b047b107aa5b73825948fd0ad0fd114f0ea0d12951f57cda577e0d2801aaa205` | observed |
| Persistent udev rule | 721 B | `908883197f0c3f79fa3d7d9801f3834a5e4b92f4d88600196cd512a9ed05ad90` | installed/bind-tested |
| Moved-run baseline metadata, `hub/runtime/spool/robot-0/00000000000000005012/metadata.json` | 2,194 B | `6358ff791782b5f3d257f8acb2845f30be8b6d99ffe77e260c6b148b90b58a41` | observed |
| Moved-run final metadata, `hub/runtime/spool/robot-0/00000000000000005018/metadata.json` | 2,194 B | `e8be16af24d3e93ec6418f9e2eb91ecf6515537f4cd6058e838384a42ae0a4c9` | observed |

The native librealsense smoke binaries were temporary diagnostics only. Their
observed hashes were
`f82e49dbd997101a1668292dc54130ebcd5ef423e35f150f4800107d73b003c4`
and
`a3b0ab915b8aba58d8d856083915009f3ab8e087541eb56efe4ef4605fa03c29`;
they were removed after the results above were recorded.

## Remaining gates

- IMU sample gaps still occur. At `process_cnt=6550`, perception was `ok` with
  current valid intervals, zero rejected/overwritten IMU messages, and
  220 historical invalid-interval reanchors. The recovery bug is fixed; the
  upstream scheduling/transport jitter is reduced but not eliminated.
- TinyNav still publishes zero pose covariance, so localization remains
  deliberately `DEGRADED`; command-capable navigation remains blocked.
- The controlled moved-robot path passed continuity/transport, but absolute
  scale remains unverified until the physical displacement is measured.
- The current 2D map fails the occupancy-quality gate because repeated static
  frames plus irreversible max fusion saturate explored cells as obstacles.
- The new WSJ v3 origin is not calibrated to Yunji; fused visualization and
  joint navigation must remain disabled until a fresh shared-landmark
  calibration is applied.
- The original TinyNav launcher still enables hardware depth, align, sync, and
  `initial_reset=true`. The current stable tmux command is a runtime deployment
  override; reusing the original launcher would reintroduce the unverified
  path and must be treated as a separate test.
