# Yunji Odin1 boot recovery — 2026-07-24

## Observed failure

Calibration attempt `20260725-lab04` passed the local repository check, both
remote release-manifest checks, Hub startup, and WSJ startup. Yunji then
returned `Odin driver is not active`.

Read-only inspection of the existing Yunji SSH/tmux session established:

- NUC boot time: `2026-07-24 19:53:15 +08:00`;
- `focus-yunji-odin1-driver.service`: loaded, disabled, inactive, result
  `success`, and no journal records from the current boot;
- Odin1 remained physically enumerated on USB as `2207:0019`;
- the prior-boot journal showed a successful driver and stream start.

This is observed evidence of a missing post-reboot service start, not evidence
of an Odin hardware failure or driver crash.

## Recovery and verification

The read-only driver unit was enabled and started. It became active with PID
`5259` at `2026-07-24 20:03:47 +08:00`. The deployed
`verify_odin1.sh --hardware` then passed all three required live topics
(`/odin1/image`, `/odin1/cloud_slam`, `/odin1/odometry`) and the pinned Odin
source/calibration checks. No WATER or robot-motion command was issued.

The source-derived deployment fix adds a calibration-only recovery helper.
Before a new board fit it verifies USB presence, enables/starts only the Odin
sensor/SLAM unit when necessary, waits for the unit, and runs the bounded
hardware verifier. The calibrated navigation launcher retains its existing
fail-closed behavior: it does not restart Odin after a shared transform has
been computed.

Incomplete calibration directories are now moved intact beneath
`hub/runtime/calibration_sessions/failed/` on retry. A directory containing
`shared_frame.json` is still protected from replacement. The board selector
also performs one final attempt after its nominal deadline so that a slow WSJ
keyframe arriving at the timeout boundary is not discarded.

## Artifact provenance

| Path | Size | SHA-256 | Classification |
|---|---:|---|---|
| `hub/robot_overlay/prepare_yunji_odin1_calibration_driver.sh` | 1,565 B | `e197daa96b8cc0339a996e7c919fef80afebc00ba212957c84f474a0164a8232` | source-derived and locally tested |
| `hub/robot_overlay/start_yunji_calibration_observation.sh` | 3,126 B | `7c8e6bebd130c6deb98a15cbfeb923df7343cc3a711f712d6024d921fa5691ed` | source-derived and locally tested |
| `hub/scripts/calibrate_realworld_session.sh` | 24,709 B | `0c75a60c7491ee91f52962f6b890a9c37ef3b924bd8e8c3e24800bac63523f1a` | source-derived and locally tested |

The full Hub test suite passed after these changes. No file under `source/` or
`dependencies/` was modified.
