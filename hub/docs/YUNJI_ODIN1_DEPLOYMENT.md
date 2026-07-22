# Yunji Odin1 deployment

This is the current sensor-replacement path for Yunji. It replaces the local
RealSense mapping input with Odin1 serial `O1-P070100205`; the previous D455
code and calibration remain available only as a rollback lane.

## Current state and remaining gate

As of 2026-07-22, the read-only Odin observation path and main Foxglove map
have been cut over. Autonomous navigation remains gated:

- the native driver publishes image, SLAM cloud and odometry at about 10.3 Hz
  and reports SLAM tracking;
- a fresh WSJ/Odin board fit and an independently moved-board holdout passed;
- tracked artifact `config/calibration/yunji_odin1_board_20260722_v1.json` is
  5,544 bytes, SHA-256
  `9e340a882df936e005902de29bb6e54c0a76da6e41c7bda26a040a0ce1421519`;
- the active sender resumed at sequence 159949 under transform
  `yunji-odin1-board-20260722-v1` and calibration ID
  `shared-board-odin1-20260722-v1`, with exact image/cloud/odometry device
  stamps and `TRACKING` localization;
- fresh WSJ and Yunji maps use separate new directories and the same explicit
  calibration ID. Their initial three-frame floor estimates differed by about
  2.4 cm, with no ground rejection, pose jump or mapping block at cutover;
- an operator-present low-speed run added 85 integrated keyframes over a
  1.193 m accepted path, changed 1,737 cells and explored 1,574 new cells;
  every moved-map continuity/quality check passed;
- the existing relay ports and `/wsj/*`, `/yunji/*`, `/fused/*` topic names
  were retained. `/yunji/camera` is pushed continuously through a
  loopback-bound reverse tunnel, so no new Foxglove layout is required;
- the driver and sender units are active but deliberately disabled for boot;
- no planner, autonomous GOAL or Hub motion output was invoked. The later
  moved-map gate used only an explicitly armed operator keyboard session over
  WATER `/api/joy_control` with the robot-local watchdog and stop authority.

This validates observation cutover and controlled manual map continuity, not
surveyed metric accuracy, semantic accuracy or autonomous navigation. Keep
GOAL output disabled until those separate physical gates pass.

## Observed device contract

The source deployment record is
`/home/nyu/workspace/tinynav/yunji-water-robot/docs/odin1_deployment.md`
(4,441 bytes, SHA-256
`f187ff1b4905415c8a5f1cf84537bf3b6ea5a920f665a606ede0a1e915c4528b`).

| Item | Observed value |
| --- | --- |
| USB | `2207:0019`, USB 3.2 / 5 Gbit/s |
| firmware | SoC 0.13.1, SLAM 0.12.1 |
| driver | v0.13.0, commit `13aa528b1da581e2168ac858f8b144f0b4438a7a` |
| mode | `custom_map_mode: 1` |
| RGB | `/odin1/image`, 1600×1296 `bgr8` |
| mapping cloud | `/odin1/cloud_slam`, colored XYZRGB in `odom`, about 10.3 Hz |
| pose | `/odin1/odometry`, `odom -> odin1_base_link`, about 10.3 Hz |

`/odin1/cloud_raw` is advertised but produced no messages in the deployed mode,
so it is not a fallback. The vendor depth-completion node is disabled and its
sparse nearest-neighbour result is not used. `odin1_sender.py` instead:

1. rectifies FishPoly RGB into a zero-skew pinhole image;
2. composes the factory `T_imu_camera` with live `T_odom_imu`;
3. transforms `/odin1/cloud_slam` back to the camera frame;
4. creates aligned PNG16 depth with a nearest-point z-buffer.

Device stamps count from Odin boot and are used only for local synchronization.
The Hub capture time is the NTP-synchronized nyush-nuc wall-clock receipt time.
The default 20 ms synchronization gate rejects an adjacent approximately 97 ms
Odin cycle, and consumed cloud stamps must increase strictly.

## Reconstruct the driver

Do not edit the TopoFocus `source/` or `dependencies/` snapshots. Rebuild the
external driver at its exact commit and apply the captured deployment patch:

```bash
git clone https://github.com/manifoldsdk/odin_ros_driver.git
cd odin_ros_driver
git checkout 13aa528b1da581e2168ac858f8b144f0b4438a7a
git apply --check /path/to/topofocus_realworld/hub/robot_overlay/odin1_snapshot/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch
git apply /path/to/topofocus_realworld/hub/robot_overlay/odin1_snapshot/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch
```

The patch is required for firmware 0.13.1: it starts streams in mode 0 before
switching to mode 1. Without it, a cold start can leave RGB/DTOF/IMU rates at
zero while the driver misleadingly reports readiness.

The factory calibration is serial-specific. For this device, both
`/home/nyu/odin_ws/src/odin_ros_driver/config/calib.yaml` and
`/home/nyu/odin_ws/calibration/O1-P070100205.calib.yaml` must have SHA-256
`c8cbd48bd8f8b08b8f174f557faf48649ee1101a3dfe0daf82ceae3832d7c23d`.
Run the read-only verifier after building:

```bash
bash hub/robot_overlay/verify_odin1.sh
bash hub/robot_overlay/verify_odin1.sh --hardware  # driver must already run
```

## Install the deployment overlay

Copy these tracked files to `/home/nyu/focus_sender_odin1/`:

- `robot_overlay/odin1_sender.py` and its shared `yunji_sender.py` helper;
- `robot_overlay/odin1_driver_headless.launch.py`;
- `config/calibration/odin1_O1-P070100205_factory_20260722.json`;
- `config/calibration/yunji_odin1_board_20260722_v1.json`;
- `robot_overlay/odin1_snapshot/` and `robot_overlay/verify_odin1.sh`.

Create a mode-0600 environment file from
`robot_overlay/config/odin1.env.example`. Put the real robot token only in that
ignored remote file. The current example binds the matching transform version
and shared artifact together. For a deliberately isolated pre-calibration
session, use a new local-only transform version and leave
`FOCUS_ODIN1_SHARED_TRANSFORM_FILE` empty; never combine one of those values
with the current calibrated value.

The two tracked service units deliberately contain no motion process. Install
them only after reviewing their absolute paths:

```bash
sudo install -m 0644 hub/robot_overlay/systemd/focus-yunji-odin1-driver.service /etc/systemd/system/
sudo install -m 0644 hub/robot_overlay/systemd/focus-yunji-odin1-sender.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Both units deliberately use a non-login `/bin/bash -c`. On the observed Yunji
NUC, a login shell prepends `/opt/MVS/lib/64` and `/opt/MVS/lib/32`; that loads
the MVS copy of `libusb-1.0.so.0`, which lacks
`libusb_interrupt_event_handler`, instead of the compatible Ubuntu library.
Changing either unit back to `bash -lc` makes `host_sdk_sample` exit with a
symbol-lookup error before any Odin topic is published.

The driver unit is headless; use the vendor `start_odin1_rviz.sh` instead when
interactive RViz is wanted. Never run both driver launchers concurrently, and
stop the vendor launcher with its documented SIGINT script rather than
`kill -9`.

Before enabling a continuous sender, run a bounded read-only check:

```bash
source /opt/ros/humble/setup.bash
source /home/nyu/odin_ws/install/setup.bash
cd /home/nyu/focus_sender_odin1
python3 -u odin1_sender.py \
  --calibration-file odin1_O1-P070100205_factory_20260722.json \
  --dry-run --max-frames 10 --rate-hz 0 \
  --evidence-dir runtime/dryrun \
  --metrics-out runtime/dryrun/metrics.json
```

Every accepted tuple should have unique increasing cloud stamps and image/cloud
plus odometry/cloud skew below 20 ms. A dry run reads WATER health but cannot
upload observations and has no motion endpoint.

The continuous service also pushes the rectified RGB preview when
`FOCUS_ODIN1_CAMERA_PREVIEW_URL` is set. The tracked example uses only
`http://127.0.0.1:18766/camera/yunji`; it reuses the robot token unless a
separate `FOCUS_CAMERA_PREVIEW_TOKEN` is supplied. Do not change this to a
non-loopback URL merely to bypass the SSH tunnel.

## Pre-calibration local map and wire-frame convention

Transport v1 requires the wire name `shared_world`. Before cross-robot
calibration, this session defines that name as `yunji_odin1_odom`; this is the
same explicitly documented aspirational convention used by
`calibrate_shared_frame.py`. It is safe for a single-robot map only because:

- the transform version is unique to Odin;
- no `shared_frame_calibration_id` is assigned;
- a new map daemon uses a new output directory and an exact expected transform;
- Foxglove fusion refuses maps without one identical, non-empty calibration ID.

Never append Odin observations to a D455 map directory. Start after the last
old sequence and bind the exact new transform version. A camera view must show
enough actual floor for three-frame ground consensus; a stable tabletop is not
a substitute for a measured floor or camera height.

## Fresh shared-board calibration

The old D455 board artifact is invalid for Odin because both the physical
camera and Odin odometry origin changed. Reuse the existing board detector and
gravity-preserving solver with its direct-camera-pose mode:

```bash
hub/.venv/bin/python hub/tools/calibrate_gravity_shared_frame_via_board.py \
  --spool hub/runtime/spool \
  --reference-robot robot-0 --other-robot robot-1 \
  --reference-sequence <wsj-board-sequence> \
  --other-sequence <odin-board-sequence> \
  --holdout-reference-sequence <wsj-moved-board-sequence> \
  --holdout-other-sequence <odin-moved-board-sequence> \
  --other-pose-is-camera \
  --transform-version yunji-odin1-board-<date>-v1 \
  --calibration-id shared-board-odin1-<date>-v1 \
  --output hub/runtime/calibration/yunji_odin1_board_<date>_v1.json
```

Both sequence pairs must be synchronized and the moved-board pair must be an
independent holdout. After it passes, set the environment transform version
and file path together, restart the sender, start another fresh map directory,
and give both map daemons the same new calibration ID. Only then may the main
relay use `--fuse`.

### Observed 2026-07-22 result

The deployed symmetric 7×10 circle board needed a detector fallback because
the Odin image contained many carpet/IR blobs at 4× enlargement. The tool now
tries the original detector first, then 2× and `CALIB_CB_CLUSTERING` fallbacks.
It also canonicalizes the symmetric-grid endpoint and uses the physical grid
center as the landmark origin, preventing a valid detection from alternating
by 180° between frames.

The source-derived yaw-only transform used these observed spool inputs:

| Role | WSJ sequence | Odin sequence | Sync skew | Center residual | Normal residual |
| --- | ---: | ---: | ---: | ---: | ---: |
| fit | 13234 | 159827 | 72.98 ms | 0 | 0.705° |
| independently moved-board holdout | 13568 | 159929 | 59.70 ms | 1.15 cm | 0.447° |

The holdout passed the 5 cm, 3° and 250 ms gates; the shared transform tilt is
exactly 0° by construction. The JSON records absolute source paths, byte sizes
and SHA-256 values for all four observed metadata/RGB pairs, marks the rigid
alignment as source-derived, and records that no robot interface or command
was used.

Fresh runtime maps were then created without modifying the old D455 maps:

- WSJ: `runtime/map_out_wsj_odin1_board_v1_20260722`, after sequence 13736,
  first integrated sequence 13737;
- Yunji: `runtime/map_out_yunji_odin1_board_v1_20260722`, after sequence
  159948, first integrated sequence 159949.

Both initialized with `shared-board-odin1-20260722-v1`, had
`mapping_blocked_reason=null`, zero pose jumps and zero rejected ground frames
at the recorded cutover. Runtime map files remain ignored; these values are an
observed deployment record, not packaged replay data.

The existing Foxglove layout does not need new topic names when the accepted
Odin map replaces Yunji under the same relay label. It does need a reconnect if
the relay process/port changes.
