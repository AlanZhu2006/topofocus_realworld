# WSJ DDS recovery PID and persistent-process fix — 2026-07-27

## Outcome

The ordered WSJ publisher recovery initially failed before the camera restart
because its `pgrep | head` selected the tmux Bash wrapper PID `17702` instead
of the persistent Python sender PID `17753`.  The recovery now accepts exactly
one real Python process whose `/proc/<pid>/cmdline` identifies
`focus_ros_sender.py --runtime-command-contract-file`.  The corrected recovery
preserved PID `17753`, restarted camera then perception, observed a new
synchronized tuple, and wrote tracking epoch
`wsj-camera-perception-20260727T120755_175`.  Navigation was paused and no
receiver or chassis bridge existed.

The sender process identity no longer changes merely because an unrelated Git
commit changes.  Immutable sender code and DDS arguments determine whether the
participant must be replaced.  The process-launch commit remains recorded
separately from the current session deployment commit in the hot-loaded
runtime contract.  A real sender-code or DDS-argument change still replaces
the process and requires the ordered publisher recovery.

## Stationary re-anchor

The operator confirmed that WSJ remained stationary and was not moved or
rotated.  Five observations immediately before the publisher restart
(`38025`–`38029`) and five after it (`38052`–`38056`) were checked.  The
result passed:

- pre-epoch maximum translation: `0.000427399 m`;
- pre-epoch maximum rotation: `0.0274315°`;
- post-epoch maximum translation: `0.000417769 m`;
- post-epoch maximum rotation: `0.0333524°`;
- anchor translation residual: `0.0 m`;
- gravity-preserving orientation residual: `0.0140454°`;
- robot commands issued: `false`.

| Classification | Path | Size | SHA-256 |
|---|---|---:|---|
| observed/source-derived input calibration | `hub/runtime/calibration_sessions/scene02-plant-20260727-1957-yunjireanchor1-groundfix1/shared_frame.json` | 14,690 B | `84be1028c14143d859b5d9ead0892a65259cd3a5cc4b5ffc85724cb5c3050915` |
| observed/source-derived validated dual re-anchor | `hub/runtime/calibration_sessions/scene02-plant-20260727-2012-dualreanchor-groundfix2/shared_frame.json` | 15,492 B | `85d333d9b252aa27fa4d5dfa741b1e44b9e9b561442e53a7fc6d882b3757e92b` |
| implemented ordered-recovery guard | `hub/robot_overlay/recover_wsj_publishers_after_sender.sh` | 9,386 B | `03138a49b39f946b5b546f1de10c0b33852c295ed8cfad4897609245a931e2dd` |
| implemented persistent sender contract | `hub/robot_overlay/start_wsj_command_observation.sh` | 22,254 B | `9df4638f3caa833af552875eb242c70397f88fa2d62061bf9ad22312dc81f387` |
| local regression tests | `hub/tests/test_realworld_operator_scripts.py` | 22,866 B | `b22cff58ac4a87b44c60ad0a8b67b0e2155d8aa637cfe6f55e2557d13cfe8b18` |

The re-anchor artifact itself records the source paths, sizes, checksums and
capture timestamps for every pre/post RGB-D observation.

## Verification

- `bash -n` passed for both modified launchers.
- `61` focused operator, sender-health and stationary-reanchor tests passed.
- `git diff --check` passed.
- Hub remained on the generated debug policy with GOAL output disabled.
