# Yunji Odin1 deployment

This is the current sensor-replacement path for Yunji. It replaces the local
RealSense mapping input with Odin1 serial `O1-P070100205`; the previous D455
code and calibration remain available only as a rollback lane.

## Current gate

As of 2026-07-22, sensor ingest is implemented and real-machine verified, but
the main Foxglove/Yunji map has **not** been cut over:

- the native driver is publishing image, SLAM cloud and odometry at about
  10.3 Hz and reports SLAM tracking;
- ten consecutive adapter frames had exact image/cloud/odometry device stamps;
- Hub sequences 159816–159818 were accepted once each under transform version
  `yunji-odin1-local-odom-20260722-v1`;
- no D455 shared transform was reused and no shared-frame calibration ID was
  asserted;
- a separate trial map was stopped because the camera saw a nearby table and
  wall but no real floor. Its three-frame plane fit was therefore not accepted
  as a physical floor validation;
- no planner, velocity command, WATER move endpoint or other robot actuation
  was invoked.

The Odin driver may remain available for RViz inspection. Do not enable the
continuous sender or replace the main Foxglove relay until the floor-view and
fresh-board gates below pass.

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
- `robot_overlay/odin1_snapshot/` and `robot_overlay/verify_odin1.sh`.

Create a mode-0600 environment file from
`robot_overlay/config/odin1.env.example`. Put the real robot token only in that
ignored remote file. For a local-only session, leave
`FOCUS_ODIN1_SHARED_TRANSFORM_FILE` empty.

The two tracked service units deliberately contain no motion process. Install
them only after reviewing their absolute paths:

```bash
sudo install -m 0644 hub/robot_overlay/systemd/focus-yunji-odin1-driver.service /etc/systemd/system/
sudo install -m 0644 hub/robot_overlay/systemd/focus-yunji-odin1-sender.service /etc/systemd/system/
sudo systemctl daemon-reload
```

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

## Local map and wire-frame convention

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

The existing Foxglove layout does not need new topic names when the accepted
Odin map replaces Yunji under the same relay label. It does need a reconnect if
the relay process/port changes.
