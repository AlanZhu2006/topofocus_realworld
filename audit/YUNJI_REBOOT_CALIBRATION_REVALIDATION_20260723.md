# Yunji reboot calibration revalidation — 2026-07-23

## Scope and safety

The operator reported that Yunji was powered off and back on without changing
its physical position.  This check determines whether the existing
`shared_world_T_yunji_odom` matrix can be reused after the Odin odometry epoch
reset.  Only the Odin sensor/SLAM driver and read-only observation/v2 receiver
were started.  No WATER move/cancel request or other robot command was issued.

## Inputs

| Input | Size | SHA-256 | Status |
|---|---:|---|---|
| `hub/runtime/calibration/yunji_odin1_board_20260723_v2.json` | 5569 B | `b01b2ca489fb69d0c4f97f51858b4488604075bc1b4ee33c7718adcb43318f55` | previously observed board calibration |
| `hub/runtime/calibration/yunji_odin1_base_camera_20260723_operator.json` | 1027 B | `ae0498f36bc4e1d8866e387bac48ece9eb29c01ca7069d8d3c6eee32e44037fb` | operator-measured mount; mount was not independently remeasured |
| `hub/config/calibration/odin1_O1-P070100205_factory_20260722.json` | 3597 B | `ba0811b52950730d65981556b13b703eb036b1ed6e85302628d402c459fe6de6` | observed factory calibration/source-derived `T_imu_camera` |

The remote copies of the first two artifacts were observed after reboot at:

- `/home/nyu/focus_sender_odin1/yunji_odin1_board_20260723_v2.json`
  (`5569` B)
- `/home/nyu/.local/state/topofocus/calibration/yunji_odin1_base_camera_20260723_operator.json`
  (`1027` B)

## Post-reboot observation

- Host boot time: `2026-07-23 19:14:37 +08:00`
- Odin driver active time: `2026-07-23 19:22:30 +08:00`
- Sensor USB ID: `2207:0019`
- Required live topics observed:
  `/odin1/image`, `/odin1/cloud_slam`, `/odin1/odometry`
- Odom sample stamp: `498.133455349` seconds on the Odin boot clock
- `T_odom_imu` translation:
  `[0.000388, -0.000184, 0.000378]` m
- `T_odom_imu` quaternion xyzw:
  `[0.001408, 0.03179, 0.00004, 0.999493]`

The observed odometry sample was composed with the factory
`T_imu_camera`.  The resulting current `T_odom_camera` was compared with
`corrected_other_camera_pose_at_sync` preserved in the board calibration:

- translation delta: `0.003096202` m
- rotation delta: `0.225344994` degrees

Both are comfortably below the original board-calibration holdout thresholds:

- maximum translation residual: `0.05` m
- maximum rotation residual: `3.0` degrees

As a second continuity check, the largest recent spool time gap corresponds to
the power cycle:

- sequence: `179400 -> 179401`
- capture gap: `719.476` seconds
- shared camera position jump: `0.019875` m
- shared camera rotation jump: `2.068670` degrees

This also remains within the original `0.05 m / 3 degree` thresholds.

## Decision

The existing matrix, transform version
`yunji-odin1-board-20260723-v2`, and shared calibration ID
`shared-board-odin1-20260723-v2` are reused for this stationary reboot.  This
is an observed numerical revalidation, not an assumption based only on the
operator report.  A new board calibration is still required if the chassis
position/heading or camera mount changes, or if a later reboot exceeds either
threshold.

After revalidation, the read-only stack resumed successfully.  Hub observation
sequence `179457` was fresh, command-capable metadata was present, localization
reported `TRACKING`, safety reported `READY`, and Hub GOAL policy remained
disabled.

## Provenance classification

- Observed: reboot/service state, USB device, ROS topics, odometry sample,
  fresh Hub observations, and pre/post-reboot spool poses.
- Source-derived: rigid composition and pose-delta calculations.
- Operator-reported: the chassis was not physically moved during power-off.
- Unverified: independent board-image validation after this reboot and any
  physical navigation command.
