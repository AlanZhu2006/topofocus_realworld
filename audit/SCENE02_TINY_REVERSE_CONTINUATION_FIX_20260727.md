# Scene 02 tiny-reverse continuation fix — 2026-07-27

## Scope and classification

This record covers supervised episode
`official-plant-20260727-2055-dualmotionfix1`.  Robot poses, controller
events and robot-local journal messages are **observed**.  The causal
reconstruction is **source-derived**.  The corrective code is regression
tested but remains **physically unverified** until the next supervised run.

No file under `source/` or `dependencies/` was changed.

## Preserved evidence

| Artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260727-2052-dualmotionfix1_live_scene02-plant_20260727_205711_327698141/episode_report.json` | 10,018 B | `a317295e386b8607a7c56bfbd55c2c87705b5d000d792e2b01ee67b02421ed28` | source-derived report over observed feedback |
| `hub/runtime/oneclick_scene02-plant-20260727-2052-dualmotionfix1_live_scene02-plant_20260727_205711_327698141/controller_events.jsonl` | 29,520 B | `0e28fe2d9593433f48a57525a39afdab92b5b59aaaec8f1db982664762716c23` | source-derived Hub controller log |
| `hub/runtime/oneclick_scene02-plant-20260727-2052-dualmotionfix1_live_scene02-plant_20260727_205711_327698141/round_02_step_049/cross_round_progress_guard.json` | 835 B | `f11c164ceb30fe5e9ab49710cd83d85c19afe9d491fcf0d27456b4e1062420c2` | source-derived fail-closed progress result |
| Yunji systemd journal query for `focus-yunji-tinynav-controller-v1.service`, 20:57:34–20:59:19 CST | size/checksum unverified at source | unverified | observed remote controller journal |

The runtime directory is intentionally not committed to Git.  The episode
ended `failed_cross_round_no_progress_holding`; WSJ travelled `3.524316 m`,
Yunji travelled `0.220826 m`, and both terminal events recorded
`velocity_zero_confirmed=true`.

## Cause

The previous path-frame fix worked: Yunji's logged rotate-first heading error
converged from `-123.3°` through `-98.8°` to `-85.9°`, while the reverse
component changed from `-0.200 m` to `-0.028 m`.  At the next replan it crossed
the controller's tiny-reverse deadband at `-0.0007 m`.

The tiny-reverse branch correctly prohibited translation, but it also reset
the active rotate-first state and zeroed angular velocity.  The controller
therefore remained at the approximately 90-degree transition instead of
finishing the already-authorized turn.  This was not a footprint, WATER,
receiver, route-conflict or DDS failure.

## Corrective behavior

An existing rotate-first recovery now crosses that deadband only when all of
the following remain true:

- rotate-first was already active and explicitly enabled;
- navigation is not paused;
- the segment is negative but below the meaningful `0.02 m` reverse threshold;
- the stable route heading error is still at least `35°`.

If pinned output is momentarily zero in that state, the existing latched turn
direction continues at the configured `0.10 rad/s` minimum.  This cannot start
a new recovery.  The original 12-second deadline continues from the initial
turn, and the pinned stale-pose/path, depth, pause and acceleration guards keep
final authority.

| Implemented artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/robot_overlay/yunji_tinynav_cmd_vel_control.py` | 27,884 B | `69930b21479b5606eefb594d03a3b63682e059dd7c8e46507fb21b80b2c349d8` | implemented robot-local bounded continuation |
| `hub/tests/test_yunji_tinynav_cmd_vel_control.py` | 8,193 B | `48b5e10e5a0dd15196850944ac61414cdd7b6568b22447705ed5096d119e4921` | local regression tests |

## Verification

- Focused controller, router and receiver tests passed.
- The complete `hub/tests` suite passed.
- Repository Python, JSON and YAML verification, `compileall`, and
  `git diff --check` passed.
- No physical command was issued while implementing or validating the fix.
