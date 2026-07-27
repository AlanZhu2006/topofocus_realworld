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
odometry tuple. The independent color stream keeps a bounded 90-frame history,
and its intrinsics are cached separately. A tuple is accepted only when the
nearest color timestamp is within `0.05 s` of the depth timestamp, after which
the existing calibrated RGB-to-depth reprojection and `0.38` coverage gate
still apply. The launcher retains one bounded read-only restart and a `15 s`
sequence-advance gate.

The first deployed candidate used only the latest color message. Its read-only
hardware probe produced no upload because TinyNav depth processing trails the
raw 30 Hz color callback; the previously observed latest-message skew was
about `0.97 s`. Both bounded attempts recorded zero accepted tuples. This
additional evidence caused the bounded nearest-timestamp history above; that
candidate is locally verified and awaits its byte-exact hardware probe.

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
| locally implemented | `hub/robot_overlay/focus_ros_sender.py` | 59,670 B | `48eac0778a70cbd04d3f6ac24c4d933bbc8a490e77b9fc71d8f661a05c90348c` |
| locally implemented | `hub/robot_overlay/start_wsj_command_observation.sh` | 14,606 B | `125097430bbaaf7f6cc1918c34ce5b70537bac457107cac423038732c2a8ab41` |
| local regression tests | `hub/tests/test_focus_ros_sender_health.py` | 14,693 B | `c5014a227279c531d1387d3fd30bb05b71d5578196a933beb39d8720189e33ba` |
| local launcher tests | `hub/tests/test_realworld_operator_scripts.py` | 18,087 B | `2863cad1552e4d5c2ac151d09806a08958f0209ba5065c5187035958d5225fd8` |

## Verification and safety boundary

- Python AST parsing and launcher `bash -n` passed.
- Focused sender/launcher/receiver tests passed.
- The complete Hub suite passed: `502` tests, with one pre-existing Starlette
  deprecation warning.
- The cached-color tests select an older `20 ms` match despite a newer
  non-matching color message, and reject a nearest `51 ms` skew.
- Hub was restored with the generated debug robot policy; goal output is
  disabled.
- Calibration checks and implementation issued no robot command. Physical
  runtime verification and motion remain pending.
