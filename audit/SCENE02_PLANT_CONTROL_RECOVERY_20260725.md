# Scene 02 plant control recovery — 2026-07-25

## Scope and safety state

The operator ended physical testing before this repair. Yunji was subsequently
powered off; WSJ remained powered on. No post-experiment robot motion command
was issued while implementing or validating this change.

The repair changes deployment code only under `hub/`. The pinned TinyNav source
and all `source/` and `dependencies/` content remain immutable.

## Evidence

| Classification | Source path | Size | SHA-256 | Relevant observation |
|---|---|---:|---|---|
| source-derived episode report | `hub/runtime/oneclick_scene02-plant-20260725-201949-yunjireanchor1-forwardseed1_live_scene02-plant_20260725_220646_417804110/episode_report.json` | 9,970 B | `30059b8f731f64a05c965f48b6af51e453e5ec4c4ffb982480c6ade7ad62d7b8` | Yunji travelled 1.8296317858491367 m before the coordinated run closed in HOLD |
| source-derived controller event log | `hub/runtime/oneclick_scene02-plant-20260725-201949-yunjireanchor1-forwardseed1_live_scene02-plant_20260725_220646_417804110/controller_events.jsonl` | 13,840 B | `4ac516d9435934d0684b77f36dbbfc200a13b443d14e621656d3c589dae556b0` | Round 1 found the semantic target, then robot-1 reported `LOCAL_PATH_REVERSE_REQUIRED` |
| observed robot-local receiver log | `/home/nyu/.local/state/topofocus/yunji-v2-tinynav-live-20260725T140603Z.jsonl` | unverified after Yunji shutdown | unverified after Yunji shutdown | The router waypoint remained ahead while one replanned controller lookahead moved behind |
| source-derived pinned controller | local repository `/home/asus/Research/pengyue/go2_tinynav_mono_sim`, object `5705bb61dafb407594970ab2bc85c63fc71e0a24:tinynav/platforms/cmd_vel_control.py` | 15,083 B | `40519ebb1c9845e0a112f55f0a1ef5790280153ebaf198ff5122103e1372c50b` | Negative forward motion is already clipped to zero and a behind-target heading produces an in-place yaw request |

The robot-local telemetry captured before shutdown showed:

- Forward state: local pose `(1.4154, 0.6582, 0.5558)`, router waypoint
  `(1.725, 0.625)`, controller lookahead `(1.5017, 0.6883)`.
- Next replan: first trajectory point `(1.4629, 0.6898)`, lookahead
  `(1.2754, 0.6203)`, despite the unchanged forward router waypoint.
- The Focus wrapper classified that approximately `-0.200 m` controller
  segment as reverse-required and zeroed it before the pinned controller's
  rotate-first behavior could execute.

Separately, the strict debug startup observed a live WSJ sender process whose
five-topic approximate-time callback did not advance the Hub sequence.
Restarting only that read-only sender advanced the observed sequence from
29,833 to 29,835. This terminal observation is observed evidence; its original
remote log size and checksum were not copied before the SSH session ended.

## Root cause and repaired contract

The high-level target, shared-frame calibration and online A* router were not
the cause. A TinyNav local replan transiently selected a lookahead behind
Yunji. The deployment wrapper rejected that geometry immediately, even though
the pinned controller already converts it to zero translation plus yaw.

Yunji now opts into this bounded policy:

1. A meaningful behind lookahead can never produce negative linear velocity.
2. Linear velocity is held at exactly zero while yaw is capped at
   `0.35 rad/s`.
3. The first turn direction is latched so a near-180-degree heading cannot
   alternate left/right on successive replans.
4. Pinned pause, stale-pose, stale-path, relocalization, depth and arrival
   guards keep final command authority.
5. If a forward segment does not return within 12 seconds, the existing
   `LOCAL_PATH_REVERSE_REQUIRED` rejection and guarded zero command apply.
6. This behavior is explicit in the Yunji launcher; WSJ does not opt in.

The WSJ observation launcher now requires a real Hub sequence advance. If a
new or existing sender process does not advance within 15 seconds, it restarts
only the read-only sender once and waits one more bounded interval. Camera,
perception, tracking and every command component remain untouched. A second
failure is terminal.

## Offline validation

- `bash -n hub/robot_overlay/*.sh hub/scripts/*.sh`
- `python -m compileall -q hub/robot_overlay/yunji_tinynav_cmd_vel_control.py`
- `python -m pytest -q hub/tests`: 493 tests passed
- `git diff --check`

No claim of post-repair physical validation is made; Yunji was powered off
before the code repair.
