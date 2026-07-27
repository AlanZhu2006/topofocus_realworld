# Scene 02 dual-robot frontier guard correction — 2026-07-27

## Scope and classification

This record covers the engineering episode `scene02-plant-run01` under
session `scene02-plant-20260727-185353`.

- Robot motion, frozen poses, maps and feedback are **observed**.
- The causal reconstruction and five-round replay are **source-derived** from
  the preserved artifacts below.
- The correction is regression-tested and replayed without robot contact, but
  remains **physically unverified** until a new supervised episode.

No file under immutable `source/` or `dependencies/` changed.

## Preserved evidence

All relative paths are below
`hub/runtime/oneclick_scene02-plant-20260727-185353_live_scene02-plant_20260727_191033_171215778/`.

| Artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `episode_report.json` | 10,176 B | `467158012a539080e8a22646d74bd3e6024a02adf2ab428793c35e22c0d9707d` | source-derived report over observed feedback |
| `round_00_step_000/frontier_clearance_guard.json` | 7,559 B | `b4070a845948815568ae383385eea5d87442f8f5e01fc2d04b41bdfc6ccf37cb` | source-derived original guard result |
| `round_00_step_000/accepted/wsj/central_map.npz` | 24,685 B | `095dddf2c903c7c73c4307b2df543dc9a62e652a79879a724cf392532cb733d9` | observed-input-derived frozen WSJ map |
| `round_04_step_099/frontier_clearance_guard.json` | 8,408 B | `7fdaf4a465a175ff95c2192c50cb54a5180606eb96cb7aad3148930ecd135680` | source-derived original guard result |
| `round_04_step_099/vlm_candidate_batch.json` | 4,477 B | `5ee599ef29efe38b07db7cd84011aef8957da5d94652bfb8dede4ed7b25c0478` | unmodified source/VLM candidate |
| `round_04_step_099/shadow/shadow_manifest.json` | 20,071 B | `477cb4912632ff0d21caebb26f9616a135f1eb915a968f769ba46dc342ce940c` | frozen source-derived VLM manifest |
| `round_04_step_099/accepted/wsj/central_map.npz` | 24,686 B | `c6965090da9e193f652d0ba978a4ad36363010dc18badd37bb158fc00f8e9e10` | observed-input-derived frozen WSJ map |
| `round_04_step_099/accepted/yunji/central_map.npz` | 26,366 B | `34a0071783d3e53494aec37a8f3c654c80d82e46c6aab2a977d76279c40a0971` | observed-input-derived frozen Yunji map |

## Observed causes

WSJ's nearest known-free seed was about `0.377 m` from its frozen shared
pose. The central guard incorrectly reused WSJ's `0.35 m` footprint clearance
as the maximum start-seed snap distance. This made its reachable cell count
zero in every round and withheld every WSJ goal. The robot was not physically
classified as blocked by its local planner because no goal reached that layer.

Yunji moved about `5.06 m`. In round 4 its source-ranked fallback frontier `A`
had no `0.34 m`-clear cell inside the unchanged `0.50 m` arrival disk. The
nearest clear cell in any known-free component was `0.725430906 m` away, but
the closest cell in Yunji's own four-connected footprint-clear component was
`1.202081528 m` from the source target. The old guard therefore held Yunji
instead of issuing a safe partial-progress waypoint.

## Corrective behavior

1. Start-seed snapping is independent of footprint clearance for both robots:
   WSJ uses its existing bounded `0.75 m` router value and Yunji uses `1.0 m`.
2. When an arrival disk is too tight, the execution adapter may retain the
   same source frontier while projecting only to the closest cell in the
   robot's start-connected, footprint-clear component.
3. A projection requires at least `0.10 m` predicted robot travel and
   `0.25 m` progress toward the original source target. If either is absent,
   the guard still holds the robot. The robot-local planner retains final
   authority to stop or reject.
4. The original VLM candidate batch remains unchanged and the guard report
   records the source target, execution target, projection mode and progress.

## Read-only replay and verification

Replaying all five frozen rounds produced a central `GOAL` for both robots in
every round. The conservative straight-corridor separations were respectively
`1.158`, `2.281`, `3.881`, `4.619` and `4.551 m`, all above the configured
`0.9 m` concurrency threshold.

- 11 focused frontier-clearance tests passed.
- 54 frontier, route-conflict, source-episode and operator-script tests passed.
- The complete Hub suite passed: 519 tests.
- Ruff, Python compilation and `git diff --check` passed.
