# WSJ command-observation startup gate — 2026-07-27

## Outcome

Scene 02 calibration `scene02-plant-20260727-151151` passed its independent
moved-board holdout and deployed the checked shared transform.  The subsequent
strict read-only debug launcher falsely failed because its WSJ sender timeout
still assumed that a stationary TinyNav keyframe tuple would arrive within
15 seconds.

The second sender was healthy.  It accepted its first Hub upload about
17 seconds after the launcher created it and continued from sequence 29,935
through at least 29,948 without another restart.  The local launcher default
now allows 30 seconds for that first end-to-end sequence advance.  The check
still returns immediately when a sequence arrives and still permits only one
bounded restart of the read-only sender.

This change has not yet been synchronized to either robot and no runtime was
refreshed while implementing it.

## Observed evidence and provenance

| Classification | Path | Size | SHA-256 | Observation |
|---|---|---:|---|---|
| observed calibration plus source-derived alignment | `hub/runtime/calibration_sessions/scene02-plant-20260727-151151/shared_frame.json` | 5,983 B | `de46d72acbc40cbc38d3aff5e24c9efe37722c45f65d861e0292606c08defc44` | passed; independent board translation `0.3738725035 m`, center residual `0.0057745789 m`, normal residual `1.5135400416°`, sync skew `0.012885861 s`, no robot command |
| source-derived session contract | `hub/runtime/sessions/scene02-plant-20260727-151151/session.json` | 3,405 B | `430d158686b3cb40cdf2dcb7fe184cb905d0411525e99e539256e936bb3e4eb3` | calibration-bound session at Git commit `dd9029db95821ab4bf50c1322c8e6367134a7383`; strict debug was not marked passed |
| observed remote log | `/home/nvidia/.local/state/topofocus/wsj-command-observation-20260727T071716_587118071.log` | 727 B | `f2c543ad2830e61ebc200eee1a0ddbba0e64310e79936d3da51b377eb1b6339f` | first 15-second attempt initialized the synchronized sender but produced no upload before the launcher restarted it |
| observed remote log at `2026-07-27 07:18:56 UTC` | `/home/nvidia/.local/state/topofocus/wsj-command-observation-20260727T071732_989084231.log` | 1,385 B at observation time | `5ce2cdea78009ab077835e2ff345f025e473a582d9e739fa7abdc78a9aa7d0c2` at observation time | second sender started at sequence 29,935, locked registration, then uploaded 29,935–29,937; the live log may grow after this recorded prefix |
| observed accepted Hub input | `hub/runtime/spool/robot-0/00000000000000029935/metadata.json` | 2,577 B | `f9ed85ec581e83f542e0f5db32229f2c81334ad86bf37e340c5a931d7e660378` | first accepted tuple from the second sender |
| observed accepted Hub RGB | `hub/runtime/spool/robot-0/00000000000000029935/rgb.jpg` | 116,496 B | `8b2bdf5be7aa40fe59b65ab2fe0c264942292c3adbfe97e5836fe34f3b51af97` | image paired with sequence 29,935 |
| observed accepted Hub depth | `hub/runtime/spool/robot-0/00000000000000029935/depth.png` | 115,361 B | `9ee95b64fd2da4f8d4915145d5d594198b9037a92f094580d91eee292f3a18c1` | depth paired with sequence 29,935 |

The second tmux window was created at
`2026-07-27T07:17:32.989084231Z`.  Its ROS node reported ready at
`07:17:35.875Z`, locked the observed RealSense registration at
`07:17:49.671Z`, and uploaded sequence 29,935 at `07:17:50.027Z`.
Therefore the end-to-end first upload required about 17.0 seconds from window
creation, while the old gate expired after 15 seconds.  Read-only topic
inspection at `07:18:46Z` also found live publishers for continuous
`/slam/depth` and `/slam/odometry_visual`; no keyframe message appeared during
either bounded eight-second rate sample, consistent with stationary,
content-selected keyframes.

## Repair and safety boundary

- `hub/robot_overlay/start_wsj_command_observation.sh` changes only the
  default first-sequence upper bound from 15 to 30 seconds.
- A healthy sender returns as soon as the Hub sequence advances, so the
  nominal path does not sleep for the full bound.
- Camera, perception, mapping, planner, receiver, controller and Go2 bridge
  behavior are unchanged.
- The Hub remained fail-closed and no receiver, bridge, GOAL or robot motion
  was started by the diagnosis.

## Local verification

- `bash -n hub/robot_overlay/start_wsj_command_observation.sh`
- `python -m pytest -q hub/tests/test_realworld_operator_scripts.py`:
  26 passed
- `python -m pytest -q hub/tests`: 500 passed
- `bash -n` passed for every shell launcher under `hub/robot_overlay` and
  `hub/scripts`
- `git diff --check`

Physical/runtime verification remains pending until the operator returns both
robots to their start positions and explicitly permits the next refresh.
