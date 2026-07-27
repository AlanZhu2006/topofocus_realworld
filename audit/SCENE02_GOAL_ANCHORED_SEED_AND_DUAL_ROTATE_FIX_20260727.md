# Scene 02 goal-anchored seed and dual rotate-first repair — 2026-07-27

## Scope and safety state

The supervised plant run stopped fail-closed with both robots in `HOLDING` and
`velocity_zero_confirmed=true`. The one-click cleanup then removed both command
paths and restarted the Hub with GOAL publication disabled. No robot motion was
issued while diagnosing or implementing this repair.

Only deployment code under `hub/` changed. `source/` and `dependencies/` remain
immutable.

## Provenance

| Classification | Source path | Size | SHA-256 | Relevant evidence |
|---|---|---:|---|---|
| source-derived episode report | `hub/runtime/oneclick_scene02-plant-20260727-2103-deadbandfix1_live_scene02-plant_20260727_210856_804439598/episode_report.json` | 10,006 B | `dc7858cfd00eb80fff869183fc4109b19793236a8952e6dd398151d0ddecc584` | outcome `failed_cross_round_no_progress_holding`; WSJ and Yunji terminal velocity zero confirmed |
| source-derived controller log | `hub/runtime/oneclick_scene02-plant-20260727-2103-deadbandfix1_live_scene02-plant_20260727_210856_804439598/controller_events.jsonl` | 56,719 B | `105f965375ed7793095ce5a8c39486630c5e3ffecf30139937829c8dfe6ea3a4` | both robots crossed the two-interval, 0.05 m displacement guard |
| source-derived round guard | `hub/runtime/oneclick_scene02-plant-20260727-2103-deadbandfix1_live_scene02-plant_20260727_210856_804439598/round_05_step_124/cross_round_progress_guard.json` | 835 B | `c0a23d8b7a8d7d3440103425b413c534fb049bb23d673b8c583c83092f106b19` | WSJ displacement 0.001242 m; Yunji displacement 0.029578 m |
| source-derived accepted command batch | `hub/runtime/oneclick_scene02-plant-20260727-2103-deadbandfix1_live_scene02-plant_20260727_210856_804439598/round_05_step_124/initial_batch.json` | 4,887 B | `950e01dbb5aa87a37c3f540f7a57c76ad39b54c3d168c959eba851ceefc17c42` | both robots received non-null frontier goals |
| observed WSJ receiver log | `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260727T130842Z.jsonl` | 169,770 B | `ba4854385d97aee5818059188f8d6ecafbce6df6b8e63ce3bef696f9f51f7deb` | local goal remained about 3.0 m away while the controller command stayed zero |
| observed pinned TinyNav source on WSJ | `/tinynav/tinynav/platforms/cmd_vel_control.py` | 10,478 B | `ea67c986934232b6ae42ffaca239dce21e3136efa7f133defb3037addde5350d` | controller derives motion only from the first two Path poses |
| immutable Focus source | `source/Focus_realworld/main.py` | 103,808 B | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` | shared A/B/C/D frontiers may be selected behind a robot; the simulator local policy owns turning |
| immutable Focus prompt | `source/Focus_realworld/src/SystemPrompt.py` | 22,350 B | `10ac3c18a4bd5438298fdd76972efd362e686608f267700bc56dd8747a1e45f1` | direction is a preference, not a hard frontier filter |

The WSJ receiver telemetry recorded:

- local base pose `(-3.134, 1.335, -3.072 rad)`;
- local goal `(-0.110, 1.322)`, about 3.0 m away;
- first Path point near `(-3.304, 1.283)`;
- second Path point near `(-3.115, 1.296)`;
- `raw_cmd=[0.0, 0.0]`.

The first point was a clearance seed in the robot's current forward direction,
but the next A* point immediately returned toward the fixed goal. Because the
pinned controller consumes only those two Path poses, it saw a reverse segment
even though the measured base-to-seed segment was forward.

Yunji's system journal was observed through the existing SSH/tmux session and
showed repeated `forward_component=-0.200 m` with a stable heading error at
approximately `+/-180 deg`. That terminal output was not exported before the
Yunji SSH session ended, so its size and checksum are unverified.

## Root cause and repair

The previous repair preferred a clearance seed using the robot's current yaw.
For a fixed goal behind the robot, every replan moved the seed as the base
turned. The first A* segment could therefore remain behind indefinitely.
Yunji repeatedly rotated; WSJ had not opted into the same bounded rotate-first
policy and held zero velocity.

The repaired contract is:

1. The bounded footprint escape remains unchanged: unknown and occupied cells
   outside the measured start-footprint override are still forbidden.
2. The seed heading is anchored to the fixed robot-local goal bearing, not the
   changing robot yaw.
3. WSJ and Yunji both explicitly opt into the same rotate-first controller:
   zero linear velocity, maximum `0.35 rad/s` yaw, latched turn direction and a
   12 s deadline.
4. Both receivers retain final authority. An unresolved reverse segment is
   rejected and held at zero after the deadline.
5. Hub leases, route-conflict guards, local depth/pose/path freshness guards
   and physical bridge limits are unchanged.

## Offline validation

- 532 tests passed with `hub/.venv/bin/python -m pytest -q hub/tests`.
- Repository verification passed: 357 Python files plus 21 JSON and 75 YAML
  files parsed; immutable-source manifests verified.
- `python -m compileall`, shell syntax checks and `git diff --check` passed.
- A regression fixture covers a self-occupied start footprint with a goal
  directly behind the robot and proves that the seed/path remains anchored to
  the goal side.

Post-repair physical behavior is not yet verified. A new operator confirmation
is required before another live GOAL may be published.
