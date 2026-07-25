# One-click reliability and latency audit — 2026-07-25

## Scope and safety boundary

This pass covers the persistent calibration, debug, supervised-live, cleanup
and evidence-sealing chain after the Scene 02 plant experiments. Physical
testing had ended before the pass began. No GOAL, velocity, ROS command or
robot-side motion process was started during implementation or verification.

No file under `source/` or `dependencies/` changed, and no simulator asset was
downloaded. The robot launchers remain fail-closed: Hub targets are versioned
and expiring, while each robot-local receiver, planner, watchdog and bridge
retains final stop/reject authority.

Evidence classifications in this record are:

- **observed**: inspected or executed in the local workspace;
- **source-derived**: established from code and regression checks but not yet
  timed on the physical robots after this repair;
- **operator-reported**: supplied by the onsite operator but not reproduced by
  the workstation;
- **unverified**: requires the next stationary/read-only or supervised run.

The baseline inspected was Git object
`18fd3133ea0b1199ceb2e89f9ee43d3b81db2655`.

## Entry-point and failure-path audit

The audited chain was:

1. `calibrate_realworld_session.sh` for raw observation, board fit/holdout,
   checked deployment, calibrated debug, map/session creation and optional
   strict debug;
2. `realworld_oneclick.sh` for exact-session resolution, SSH/release checks,
   tracking continuity, maps, GLM, Dashboard, robot debug/live launch and
   fail-closed cleanup;
3. `focus_hub_up.sh` and `start_fresh_dual_maps.sh` for the clean Hub epoch and
   immutable map boundary;
4. `start_wsj_buildmap_v2.sh`,
   `start_wsj_command_observation.sh` and `start_yunji_v2.sh` for robot-local
   data-plane, sender, receiver and command-route gates;
5. `run_v2_source_episode.py` for bounded live execution and automatic terminal
   evidence sealing.

The existing tracking-epoch, map-generation, exclusive command-route,
per-robot rejection, route-conflict, semantic-arrival, forward recovery seed
and bounded rotate-first guards remain present. The repair does not shorten or
bypass any of those gates.

| Audited failure | Repair | Classification |
| --- | --- | --- |
| A saved SSH pane could be dead, making a valid one-click invocation fail immediately or wait on an unusable shell. | Both entry points now respawn only the exact pre-existing SSH pane, verify its original start command and require a unique remote-shell witness. A pane that dies during the probe fails immediately. | source-derived |
| WSJ and Yunji release checks and several read-only startup stages waited serially. | Independent dual-host checksum work, calibration deployment/raw/final startup, and local read-only services now begin together and still require every result. Child shells cannot inherit the parent cleanup trap. | source-derived |
| Matching files on disk did not prove that a long-lived observation sender or Yunji systemd core had loaded the same code. | The full 40-character Git object is bound to WSJ sender tmux state and Yunji transient-unit environments. An old or unmarked sender reloads once; a matching sender remains warm. | source-derived |
| A sender process could exist while its Hub sequence remained frozen. | Both senders must demonstrate a new Hub sequence. Each launcher may restart only its read-only sender once, then fails closed. Yunji's check overlaps non-actuating core startup. | source-derived |
| WSJ ran three serial lightweight ROS witnesses and then a verifier subscribing to the same CameraInfo/odometry data. | The duplicate witnesses were removed. The remaining complete verifier still checks both CameraInfo topics, visual odometry, geometry, occupancy and router state without a full-resolution Image subscriber. | source-derived |
| Transitioning out of calibration did not explicitly stop Yunji's calibration observation unit. | The normal Yunji launcher now stops that unit before creating the command-capable observation path. | source-derived |
| One hanging HTTP attempt could outlive the Hub launcher's outer deadline. | Each Hub readiness request now has a two-second transport bound. | source-derived |
| Live startup checked the clean observation epoch and heartbeat readiness in two sequential loops. | One 25-second gate now requires both post-epoch observations and heartbeat-backed `ready_for_goal=true` for both robots. | source-derived |
| Runtime code could change locally after a session identity was resolved. | One-click now requires committed, clean `hub/`, `source/` and `dependencies/` content before any remote operation. | source-derived |

## Latency result

The structural critical path changed as follows. These are code-path bounds,
not a post-repair physical stopwatch claim.

| Phase | Previous dependency | Current dependency |
| --- | --- | --- |
| dual release verification | `T_wsj + T_yunji` | `max(T_wsj, T_yunji)` |
| calibration raw start, calibration deployment and calibrated debug start | each pair serial | each pair concurrent, all results mandatory |
| full read-only recovery after maps | GLM, dual robots and Dashboard serial | `max(T_glm, T_robots, T_dashboard)` |
| WSJ sensor witnesses | three separate 15 s bounds plus the complete verifier | one complete 35 s verifier |
| dead SSH failure | up to 30 s probe | 15 s maximum and immediate dead-pane detection |
| post-arm live readiness failure bound | clean epoch up to 90 s, then heartbeat up to 60 s | one combined 25 s bound |
| Hub HTTP attempt | no per-request bound | 2 s per request |

Every run now prints `ONECLICK_TIMING phase=... elapsed_s=...`, so the next
robot run will separate release, maps, GLM, Dashboard, robot launch, live arm
and clean-epoch readiness rather than reporting one opaque startup delay.

There is one intentional remaining per-round cost. Across 23 complete
`round_inputs_frozen` → `frontier_clearance_guard_evaluated` intervals in eight
preserved physical controller logs under `hub/runtime/oneclick_*`, the observed
range was `16.534117–18.446793 s` and the median was `16.960390 s`. The two
robot decisions cannot be made independent without changing the transported
source cascade: the second allocation depends on the first allocation and
shared memory. This pass therefore leaves the source `0,24,49,...,499` protocol
unchanged.

Post-repair physical startup time is **unverified**. The first following run
should archive the emitted `ONECLICK_TIMING` lines before considering further
timeout changes.

## Local verification and provenance

Observed local result:

- `bash hub/scripts/verify_repository.sh --tests` passed;
- all 357 repository Python candidates parsed;
- all repository shell candidates passed `bash -n`;
- all 21 JSON and 75 YAML candidates parsed;
- immutable source and TinyNav snapshot manifests passed;
- whitespace, forbidden runtime/secret path, high-confidence credential and
  oversized-file checks passed;
- all 498 Hub tests passed, with one pre-existing Starlette deprecation
  warning;
- `python -m compileall -q` passed for Hub source, tools, robot overlay and
  tests;
- `git diff --check` passed.

The modified deployment artifacts are:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/scripts/realworld_oneclick.sh` | 40,458 | `5e884d62c6ba59cc77b59d4d31acbeadc1e57f047e1672a9f526c7d215d6bbd4` | locally implemented |
| `hub/scripts/calibrate_realworld_session.sh` | 30,573 | `5978d8af42aa0ddc05e285ee8ef80dd6bf8464476844d6c967a861d519912104` | locally implemented |
| `hub/scripts/focus_hub_up.sh` | 8,214 | `1fc36e4c85186e6fb99c100d74cf6ceb95945c9ec6fa6297c50404bde90a4fa1` | locally implemented |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 20,454 | `370dd81b398fab1f6237de134d509460ef2120fe5acec0ce98b5fdceaede04ef` | locally implemented |
| `hub/robot_overlay/start_wsj_command_observation.sh` | 13,952 | `63bc36e8cc74c7b5dee4c326c1e72e29d0ac9ff874542e6a47887d0f096e5e47` | locally implemented |
| `hub/robot_overlay/start_yunji_v2.sh` | 20,090 | `dfcb23fc92100cc07814016a3016cab3178ed2a0bd251e8ff0838617227782ab` | locally implemented |

The runtime-log latency values above are observed timestamps from preserved
local evidence. The dependency reduction and robot behavior are
source-derived until the next physical run.

## Yunji synchronization boundary

The operator reported that Yunji was currently synchronizable. From this
workstation, however, the existing `nyush-nuc` SSH alias still resolved to
`10.209.85.41` and timed out. The workstation was observed at
`10.208.2.249/23`; a read-only sweep of that directly connected `/23` did not
find the recorded Yunji onboard-NIC MAC `48:21:0B:6E:1F:BD`, and the two exact
existing Yunji SSH/tmux panes remained disconnected.

No alternate host was guessed and no new persistent transport was created.
The implementation was published to `origin/main` as
`aa4881a9046b30c06c2de7b8f7f81ce0bb6facda`
(`Harden and accelerate one-click startup`). After publication, the exact
existing `focus_yunji_tunnel_20260722:sensor-audit` pane was respawned once
with its saved SSH/reverse-tunnel command; it exited with SSH status 255
without producing a remote-shell witness.

Git publication and robot-disk synchronization are therefore separate:
robot synchronization remains unverified until the existing Yunji pane can
reach the operator-confirmed address and the extracted bytes pass the release
manifest.
