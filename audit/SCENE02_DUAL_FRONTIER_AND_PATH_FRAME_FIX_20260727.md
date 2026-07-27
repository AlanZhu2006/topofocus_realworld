# Scene 02 dual-frontier allocation and path-frame fix — 2026-07-27

## Scope and classification

This record explains episode `official-plant-20260727-2027-ddsfix1`.
Robot poses, path lengths, navigation states and frozen maps are **observed**.
The causal reconstruction and frozen-map replays are **source-derived**.  The
fix is regression-tested and replayed against every frozen round, but remains
**physically unverified** until the next supervised episode.

No file under `source/` or `dependencies/` was changed.

## Preserved evidence

| Artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260727-2014-ddsfix1_live_scene02-plant_20260727_202747_783522889/episode_report.json` | 9,975 B | `9f2988b0aee76e9989fb60f0dbc93f66a4df64334789fabc4b7c6dbe5b244417` | source-derived report over observed robot feedback |
| `hub/runtime/oneclick_scene02-plant-20260727-2014-ddsfix1_live_scene02-plant_20260727_202747_783522889/controller_events.jsonl` | 78,051 B | `a809839b27e499ea340a112e9b1d67ad32958625bb9b13f1c5c93569f1413315` | source-derived Hub controller log |
| `hub/runtime/oneclick_scene02-plant-20260727-2014-ddsfix1_live_scene02-plant_20260727_202747_783522889/round_00_step_000/freeze_result.json` | 8,082 B | `e6ffcc7c74c9c1fa0c7f35c2747dd913e7a2ccc6d97410d19b2b2c2ca9612f84` | frozen observed dual-robot input identity |
| `hub/runtime/oneclick_scene02-plant-20260727-2014-ddsfix1_live_scene02-plant_20260727_202747_783522889/round_00_step_000/frontier_clearance_guard.json` | 12,775 B | `eafee53153a2c0f3b5d9aa72f27ed8d594b2cdf39a091082873f87528d4ced08` | source-derived pre-fix guard result |

The runtime directory is intentionally not committed to Git; the table
preserves its exact workspace path, size and checksum.

## Cause

The earlier footprint guard was active.  Two separate contract defects
produced the observed behavior:

1. Both selected frontier centroids lay on the expected known/unknown
   boundary.  Bounded projection nevertheless required the centroid itself to
   be reachable known-free, and used full endpoint footprint clearance for
   the entire reachability graph.  The fallback allocator was then greedy:
   WSJ consumed frontier `A`, although it could also use `C`, leaving Yunji
   without its only safe fallback.
2. The TinyNav goal router published every `Path` pose with an identity
   quaternion.  The pinned controller interprets path translation in the
   first pose's orientation, so a shared-world negative-X segment continued
   to look like reverse motion after Yunji turned.  Rotate-first recovery
   therefore timed out and replanned repeatedly instead of converging.

The terminal HOLD was fail-closed: WSJ's reported path length was
`0.954105844 m`, Yunji's was `5.321431355 m`, and both final events recorded
`velocity_zero_confirmed=true`.

## Corrective implementation

- Frontier centroids may remain unknown boundary points.  Projection searches
  a robot-router-matched known-free graph (`0.05 m` WSJ, `0.30 m` Yunji), but
  still requires the endpoint to pass the full measured footprint clearance
  (`0.35 m` WSJ, `0.34 m` Yunji).
- Fallback selection now uses deterministic maximum-cardinality matching
  before source rank, preventing a flexible robot from consuming a
  constrained robot's only safe frontier.
- Every freshly published TinyNav route carries the current measured odometry
  orientation, so controller-relative forward and heading calculations use
  the live robot frame.
- The path-clearance values and projection provenance are explicit in the
  episode manifest and per-round guard artifact.  Robot-local planning, depth
  stop and chassis authority are unchanged.

| Implemented artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/v2_frontier_clearance.py` | 39,159 B | `1f2d97987476f0499e4a5218ad5ccfeeb621d3356f46af4e829bcb8d6f1b9362` | implemented Hub safety/allocation guard |
| `hub/tools/run_v2_source_episode.py` | 84,343 B | `55d7dabf9e285ce1f8395cfe28eb2e23722697be64d545376b35572635270258` | implemented source-episode contract |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 41,488 B | `1997ea354383da0401fc948c5ae80c8ca7d24fd934ac5ac76d5451d576612532` | implemented robot-local route-frame fix |
| `hub/tests/test_v2_frontier_clearance.py` | 23,224 B | `cff0025c1c1f2045efb7a69cf7e4f51fcf78a9eba1ebe16350682025adb16cc3` | local regression tests |
| `hub/tests/test_tinynav_buildmap_goal_router.py` | 17,872 B | `dfa0498308a6eaf46d06158a136ae42997f3cdd4c86cadf3a3867cb0ef36bff2` | local structural regression tests |

## Verification

- All ten frozen rounds from the failed episode were replayed offline.  Every
  round produced `GOAL,GOAL` with no blocked robot.  Round 0 deterministically
  assigned WSJ to `C` and Yunji to `A`.
- In round 0, WSJ's projected endpoint was
  `(-0.13992628564431264, 5.171371078671054)` with `0.738640 m` projection
  distance and `2.675900 m` source progress.  Yunji retained source frontier
  `A` at `(-5.8323181180729815, 3.9067791303873136)`.
- The complete `hub/tests` suite passed; repository Python, JSON and YAML
  verification passed; `compileall`, launcher `bash -n` checks and
  `git diff --check` passed.
- No physical command was issued during diagnosis, replay or verification.
