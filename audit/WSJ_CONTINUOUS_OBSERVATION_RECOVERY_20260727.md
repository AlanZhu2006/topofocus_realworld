# WSJ continuous formal-observation recovery — 2026-07-27

## Outcome

The formal WSJ sender was coupled to the motion/content-selected
`/slam/keyframe_depth` and `/slam/keyframe_odom` streams. After the sender was
restarted while WSJ was stationary, no new keyframe was created, so restarting
the read-only sender could never recover the missing tuple. The raw camera,
continuous `/slam/depth`, `/slam/camera_info` and
`/slam/odometry_visual` streams remained live; this was not a chassis,
network, calibration or base-controller fault.

The sender now synchronizes the continuous depth, depth intrinsics and visual
odometry tuple. The independent color stream and its intrinsics are cached
separately. A tuple is accepted only when the latest color timestamp is within
`0.05 s` of the depth timestamp, after which the existing calibrated
RGB-to-depth reprojection and `0.38` coverage gate still apply. The launcher
retains one bounded read-only restart and a `15 s` sequence-advance gate.

This change is locally verified and awaits byte-exact robot synchronization
and a read-only hardware probe.

## Preserved Scene 02 calibration

The workstation restarted unexpectedly after calibration. The artifact
survived and all three independently observed copies were byte-identical:

| Classification | Path | Size | SHA-256 |
|---|---|---:|---|
| observed/source-derived passed calibration | `hub/runtime/calibration_sessions/scene02-plant-20260727-151151/shared_frame.json` | 5,983 B | `de46d72acbc40cbc38d3aff5e24c9efe37722c45f65d861e0292606c08defc44` |
| observed WSJ persisted copy | `/home/nvidia/.local/state/topofocus/calibration/scene02-plant-20260727-151151_shared_frame.json` | 5,983 B | `de46d72acbc40cbc38d3aff5e24c9efe37722c45f65d861e0292606c08defc44` |
| observed Yunji persisted copy | `/home/nyu/.local/state/topofocus/calibration/scene02-plant-20260727-151151_shared_frame.json` | 5,983 B | `de46d72acbc40cbc38d3aff5e24c9efe37722c45f65d861e0292606c08defc44` |

Calibration ID:
`shared-board-odin1-scene02-plant-20260727-151151-v1`.
Its independent moved-board holdout passed with `0.3738725035 m` board
translation, `0.0057745789 m` center residual, `1.5135400416°` normal
residual and `0.012885861 s` pair skew.

The operator plans one Yunji power cycle. The board artifact must not be
overwritten or recomputed. If Yunji is not moved or rotated and its camera
mount is unchanged during power-off, the next tracking epoch is handed into
this saved board frame by the existing stationary re-anchor validation.
Failure of that continuity gate must stop the experiment rather than silently
replace or misuse the calibration.

## Changed artifacts

| Classification | Path | Size | SHA-256 |
|---|---|---:|---|
| locally implemented | `hub/robot_overlay/focus_ros_sender.py` | 58,602 B | `245abd26bda2fb5bbbd62348467a3bc8342440bbccba5e4bbe35920fa2f4a92b` |
| locally implemented | `hub/robot_overlay/start_wsj_command_observation.sh` | 14,329 B | `c3174d435e6b548693afacaf3b6126ae15eb8c9edba777ac000a189d92edf8c0` |
| local regression tests | `hub/tests/test_focus_ros_sender_health.py` | 14,046 B | `a2691b5344b06f66213615f01aaadab3518041f063c586554ae3f8d2075bb4e6` |
| local launcher tests | `hub/tests/test_realworld_operator_scripts.py` | 17,973 B | `826045e6a3886495749778650e8860a51f82a6ff62964eaae9ef5832be1b49a0` |

## Verification and safety boundary

- Python AST parsing and launcher `bash -n` passed.
- Focused sender/launcher/receiver tests passed.
- The complete Hub suite passed: `502` tests, with one pre-existing Starlette
  deprecation warning.
- The cached-color tests accept a `20 ms` skew and reject a `51 ms` skew.
- Hub was restored with the generated debug robot policy; goal output is
  disabled.
- Calibration checks and implementation issued no robot command. Physical
  runtime verification and motion remain pending.
