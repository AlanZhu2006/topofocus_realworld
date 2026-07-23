# Source-derived continuous VLM scene runner — 2026-07-23

## Result

The Hub now has a bounded, persistent, two-robot **shadow** scene runner that
follows the executable HPC control-flow semantics needed between VLM rounds.
It does not contain a GOAL publication mode and does not claim physical
navigation success. On source `Find_Goal`, this shadow runner pauses as
`paused_shadow_target_found_awaiting_robot_local_planner_stop`; the
source-equivalent `stop && found_goal` evidence remains unavailable until a
robot-local planner and operator-present HIL gate exist. HM3D
`multi_Total_SR` also requires the GT agent's target evidence, so a real-world
success result additionally needs independent target confirmation.

`source/` and `dependencies/` were not modified. All deployment changes are
under `hub/`, as required.

## Executable source contract

These behaviors are **source-derived**, confirmed by direct reading of the
immutable local HPC snapshot:

- `main.py` owns one `history_nodes/history_count/history_states/history_score`
  set outside the agent loop, shared by all agents for the episode.
- Decisions occur at logical action steps `0, 24, 49, ..., 499`, from
  `num_local_steps=25` and `MAX_EPISODE_STEPS=500` (21 opportunities).
- Each agent executes Perception, registers/increments its current shared
  history node, includes that updated node list in Judgment, and only then
  writes `2 * Perception + Judgment` into the directional state.
- The frontier gate is exactly `FN_PR[0] >= 0.5 or l_step <= 125`.
- Gate-pass invokes the frontier Decision VLM. Gate-fail does **not** invoke
  the History prompt present in `SystemPrompt.py`; executable `main.py` uses
  the first maximum of frozen `history_score_copy`.
- Agent 0 freezes frontier/history candidate copies. Later agents consume
  those copies sequentially; later shared-memory updates do not rewrite the
  frozen scores.
- With no frontier, Perception still updates history using
  `2 * Perception_PR[0]` before source selects a random map point.
- In `vlm_agents.py`, any positive target-semantic channel activates
  `Find_Goal`; the largest 8-connected component becomes the full local goal
  mask. TV first receives the source 7x7 dilation. HM3D ObjectNav targets are
  limited to chair, bed, plant, toilet, TV and sofa.
- STOP is emitted only when the local planner returns `stop` while
  `found_goal == 1`. Any STOP makes the shared Habitat task inactive; HM3D
  `multi_Total_SR` is then counted only when both predicted and GT agents have
  `Find_Goal`.

The earlier statement in `audit/VLM_DECISION_CASCADE_20260720.md` that a
separate History VLM branch remained unported was incorrect and is now
explicitly corrected there.

## Hub implementation

- `hub/src/focus_hub/source_episode.py`: exact logical clock, target set,
  largest target component/TV dilation, frozen-history selection and strict
  persistent state schema.
- `hub/src/focus_hub/directional_memory.py`: two-phase visit registration and
  score application, shared across agents, including the original asymmetric
  0°/360° slice arithmetic.
- `hub/src/focus_hub/vlm_decision.py`: Perception -> history registration ->
  Judgment -> frontier/history selection, plus source no-frontier history
  behavior.
- `hub/tools/live_vlm_shadow.py`: one immutable round, semantic target mask
  artifacts, sequential allocation and persistent-state transaction.
- `hub/tools/live_vlm_scene.py`: waits for a new fresh synchronized accepted
  keyframe from every robot, runs at most 21 source-derived rounds, and records
  one terminal scene manifest/event log.
- `hub/scripts/run_live_vlm_scene.sh`: fail-closed field entry point with no
  stale/block override and no GOAL option.

Every accepted round now locks both each robot's source sequence and the
SHA-256 of `central_map.npz` before launching the child process. A live update
between readiness inspection and input freezing aborts the round without
advancing episode state.

All 15 source semantic categories remain visible to the VLM decision map;
only the six original HM3D ObjectNav categories are accepted as navigation
targets.

## Deliberate real-robot safety deviations

These are explicit deployment adaptations, not claims of byte-for-byte
Habitat execution:

1. Source's no-frontier random map goal is suppressed to HOLD. Synthesizing a
   random physical target is unsafe.
2. Per-stage model faults are recorded and fail to a non-command shadow
   result instead of crashing a robot process.
3. Habitat's discrete action count is represented by a labelled logical
   shadow clock; it does not claim physical actions occurred.
4. The dashboard receives a centroid only for display. The saved full
   connected-component mask is authoritative source evidence; neither is a
   robot command.
5. Real heading is approximated from the calibrated camera pose and remains
   unverified against a robot-body heading ground truth.

## Observed real-GLM stateful forensic round

At run ID `shadow-20260723-000934-4e83dcab`, the real loopback GLM-4V service
processed one stateful logical step-0 round over preserved July 22 maps:

- WSJ sequence 15147: Perception Yes 0.05998, Judgment Yes 0.86840, frontier
  D selected; accumulated chair map produced a 245-cell largest component.
- Yunji sequence 165893: Perception Yes 0.74572, Judgment Yes 0.90997,
  frontier B selected; accumulated chair map produced a 103-cell component.
- Both semantic components overrode the exploration proposals, matching
  `vlm_agents.py`; terminal status was
  `target_found_awaiting_robot_local_planner_stop`.
- Persistent state advanced exactly once: round 0 -> 1, next logical step
  0 -> 24, and two shared history nodes/scores were saved.
- Zero cascade errors, zero Hub publications, zero Foxglove target writes,
  `robot_commands_sent=false`, and no GOAL path.
- GLM inference and state transaction took 16.044 s. The service was then
  stopped and loopback port 31511 was confirmed released.

This is **observed forensic evidence only**, not a fresh scene pass: WSJ was
locked for ground-plane drift; inputs were 6810.661 s / 6653.400 s old; their
capture skew was 157.261 s. The explicit stale/block overrides were used only
because no command or display publication was enabled. WSJ's current YOLO
frame did not detect chair, so its target component came from accumulated map
evidence; it remains model-derived and unverified.

The sequence/map transaction-lock change was added after this observed model
round and is source-derived plus unit-tested; it has not yet been exercised
against a fresh moving live stream.

Ignored runtime evidence:

| Artifact | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/runtime/vlm_source_stateful_forensic_round_20260722/shadow_manifest.json` | 21,477 | `c782bfc76248eb08d2eab6067a846d8e1c52210bafcc106b507e7a477ab3e9b9` | observed |
| `hub/runtime/vlm_source_stateful_forensic_round_20260722/scene_state_after.json` | 12,917 | `9265371cbf7cf62b358f63171ca4510c6e738090687a1d88df511f0a94c3eab5` | observed/source-derived |
| `hub/runtime/vlm_source_stateful_forensic_round_20260722/source_goal_masks/wsj_chair.png` | 1,113 | `2857fd97b295496a1f59c4ebfda8b06b204886ac91a404103a74d63c607cde37` | model-derived, unverified |
| `hub/runtime/vlm_source_stateful_forensic_round_20260722/source_goal_masks/yunji_chair.png` | 1,035 | `dd09731a80c2561dac4bb1ac40ff844788bc774d982b2bfd351cb7734d646dea` | model-derived, unverified |

## Provenance

Immutable authoritative source:

| Path | Bytes | SHA-256 |
| --- | ---: | --- |
| `source/Focus_realworld/main.py` | 103,808 | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` |
| `source/Focus_realworld/agents/vlm_agents.py` | 46,500 | `992f0174d50b6959d538a418c224907156f784ffd4b35b5ef67c02da3461bee0` |
| `source/Focus_realworld/arguments.py` | 14,140 | `66dc9a94459215d9a51d97bf8f195fd486759d7f34529c60e2a57999665a61d3` |
| `source/Focus_realworld/constants.py` | 28,432 | `6217a75db7e012602b70d6f5c76265cf90ff8d365a6176e5ce293fad5aafd106` |
| `source/Focus_realworld/tasks/multi_objectnav_hm3d.yaml` | 1,386 | `b4dd539bd886cd6b17c794b04fceda705577c08c684965e30ba46066c5f0c498` |
| `dependencies/habitat-lab/habitat/core/embodied_task.py` | 13,673 | `4801fa5f1e81016a5d590d0ecda1f027c7589be71ff102ac148c9ac9986f47ec` |
| `dependencies/habitat-lab/habitat/tasks/nav/nav.py` | 47,881 | `97ddb4534b2307d6728098bb6581a40272569102ed24cc17833983115aebb2e2` |

Current source-derived adapters:

| Path | Bytes | SHA-256 |
| --- | ---: | --- |
| `hub/src/focus_hub/directional_memory.py` | 9,274 | `f07124e78fa77f1f8a135920505c2c089b255eaf8cda4f510a62e6a0e40e7bf1` |
| `hub/src/focus_hub/source_episode.py` | 16,567 | `e5bed073f936cf05a0e884dd0107b537ce3940f316b5f2701f8cda18b8aa3a06` |
| `hub/src/focus_hub/vlm_decision.py` | 13,199 | `2a8a10cad814a766b81205b95ad5e1c3f1a7bcd9cbb93953a254c8b71b0ff2fa` |
| `hub/tools/live_vlm_shadow.py` | 42,409 | `5b1c95d4f35779002026989c32ab053513735f2b6ab51e87609fe1b5859cdf56` |
| `hub/tools/live_vlm_scene.py` | 23,719 | `7bb4e3a98a27e125b0bb8cd79c8637c1bc6efe145bb2a4d58b7c36f59fc34aaa` |
| `hub/scripts/run_live_vlm_shadow.sh` | 4,091 | `630b5d66f2932d3ae94a353be1cdbb5a79fb7e69e6f42daeb1cf42e232ca19aa` |
| `hub/scripts/run_live_vlm_scene.sh` | 3,452 | `de66592ae12f466f168a842559b9d3831aa516365062d0ee9498af1c00ca783d` |

## Verification and remaining gate

- `bash hub/scripts/verify_repository.sh --tests`: passed, 224 tests.
- Targeted source-scene regression set: 30 passed.
- Ruff, Python byte compilation, Bash syntax, help output and
  `git diff --check`: passed.
- `git status --short source dependencies`: empty.
- Local GLM port 31511: released after the forensic run.

Still **unverified**: a fresh same-session dual-map run, body-heading accuracy,
continuous multiple-round behavior under real motion, robot-local planner
target-mask consumption, source-equivalent STOP, and G5 hardware-in-the-loop
rejection, plus independent target confirmation analogous to source GT.
Therefore this work completes the shadow handoff orchestration, not a source
episode or autonomous real-world ObjectNav.
