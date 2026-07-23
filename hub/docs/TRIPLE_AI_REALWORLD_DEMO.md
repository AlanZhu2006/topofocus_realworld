# Triple-AI supervised real-world demo protocol

> Current status: the 4 × 5 schema, scorer and evidence rules are implemented,
> but no valid official episode has completed. All `official-run01` retries
> are engineering attempts and are excluded. See
> [CURRENT_STATUS.md](../../CURRENT_STATUS.md).

## Scope

The minimum reportable experiment is four physical layouts with five fresh
episodes per layout (20 episodes total).  The robots run autonomously while
an operator remains beside them with stop/takeover authority.  A takeover is
always permitted, but that episode is labelled `operator_intervention` and
scores zero in autonomous SR/SPL.

This protocol does not require production deployment hardening.  It still
retains the repository's non-negotiable boundary: the Hub may issue only a
versioned, expiring high-level target, while each robot keeps local planning,
rejection and final STOP authority.

## Historical-image preflight

The tracked manifest
`hub/config/experiments/triple_ai_demo_image_preflight_v1.json` selects four
previously observed synchronized RGB-D pairs without copying private runtime
images into Git:

| Case | Replay target | WSJ/Yunji sequence | Capture skew | Intended check |
|---|---|---:|---:|---|
| `chair_shared_view` | chair | 9388 / 147885 | 0.188 s | chair visible in both views |
| `plant_single_robot_view` | plant | 10738 / 158529 | 0.027 s | plant visible only to WSJ |
| `chair_with_table_distractor` | chair | 9739 / 150697 | 0.745 s | chair versus table-dominated distractor |
| `chair_odin_pair` | chair | 15147 / 165736 | 0.156 s | latest pair using Yunji Odin1 |

The first three cases retain historical Yunji D455 imagery.  Only the last
case contains Odin1 data, so this set tests image/VLM behavior and is not a
current-sensor acceptance result.  Some old wire messages also carried
`water_bottle` or `chair` as their goal; the replay target override is
recorded explicitly rather than presented as the original command.

Verify all 24 metadata/RGB/depth files against recorded sizes, SHA-256 hashes
and wire identities:

```bash
hub/.venv/bin/python hub/tools/run_demo_image_preflight.py \
  --output hub/runtime/triple_ai_demo_image_preflight_verify.json
```

With the local GLM server listening only on loopback, run source Perception
VLM five times for each of the two robot images in all four cases (40 calls):

```bash
FOCUS_GLM_PORT=31511 bash hub/scripts/run_glm_offline.sh

hub/.venv/bin/python hub/tools/run_demo_image_preflight.py \
  --run-perception \
  --output hub/runtime/triple_ai_demo_image_preflight_perception.json
```

The tool runs the upstream-compatible YOLO confidence policy once per image
and records every GLM probability and latency.  It contains no Hub client,
decision publisher or robot command path.

Static images cannot produce navigation SR or SPL: they contain no newly
executed path, planner STOP, arrival in a surveyed goal region or independent
terminal verification.  Every preflight output therefore contains
`official_navigation_metrics_eligible: false`.

## Official episode definition

For each of the four physical layouts:

1. Fix the target category, target pose, obstacle layout and a surveyed valid
   stopping/observation region.  Use only the six source goals: chair, bed,
   plant, toilet, tv or sofa.
2. Pre-register five pairs of robot start poses.  Physical robots need not
   share one pose; record this real-world adaptation.
3. Before every episode, clear both maps, the shared directional memory, VLM
   history and episode state.  Reusing a completed map changes the task into
   known-map navigation and is not comparable.
4. Start both sensing/mapping pipelines.  The robots may execute high-level
   moves alternately for the first demo; simultaneous motion is not required.
5. End on a verified local-planner STOP, a fixed timeout, wrong target,
   operator intervention, collision, system failure or explicit abort.

Autonomous success for robot `i` requires all of:

- the episode terminates normally as `completed`;
- its local planner reports STOP;
- its final pose is in the pre-surveyed valid goal region;
- an independent terminal image/operator annotation verifies the target;
- no operator navigation intervention occurred.

Episode SR is one when either robot succeeds.  This is the real-world analogue
of the source's any-agent success aggregation while keeping `Find_Goal`
separate from navigation completion.

## Metrics

For each robot, record actual odometry path length `P`, surveyed shortest
collision-free path `L`, and the start/stop positions.  The scorer reports:

- standard SPL: `S * L / max(L, P)`;
- exact source-compatible SPL numerator: start-to-stop Euclidean displacement,
  divided by `P` and clipped to one;
- episode multi-SPL: the maximum per-robot value, matching
  `source/Focus_realworld/main.py`;
- per-scene and overall SR, mean SPL, population standard deviation and
  termination counts.

The result file uses schema `focus-realworld-demo-results-v1`.  Each episode
contains two robot records with start/stop coordinates, actual path, surveyed
shortest path and the three terminal facts above.  Each robot record also
references the trajectory log, shortest-path survey source and (when target
verification is true) terminal image/annotation by workspace-relative path,
byte size, SHA-256 and observed/source-derived classification.  The scorer
verifies every referenced byte before emitting metrics.  Score a complete
4 x 5 record with:

```bash
hub/.venv/bin/python hub/tools/score_realworld_demo.py \
  --records hub/runtime/triple_ai_demo_results.json \
  --output hub/runtime/triple_ai_demo_metrics.json
```

During collection, `--allow-incomplete` produces a progress report but keeps
the missing-scene/trial errors visible.  A partial report is never labelled
complete.

## What remains before physical collection

The image preflight, metric accounting, v2 target/feedback transport and
persistent-session operator flow are implemented. Before physical collection,
run the one-command board calibration for a new session, require its strict
no-motion debug result, and choose a target outside both arrival radii. One
bounded operator-authorized episode must end with local STOP plus independent
target/goal-region verification.

Immediately convert the episode report into an auditable trial:

```bash
hub/.venv/bin/python hub/tools/record_realworld_trial.py \
  --episode-report <episode_report.json> \
  --results hub/runtime/triple_ai_demo_results.json \
  --experiment-id <experiment-id> \
  --trial-index <1-5> \
  --termination completed \
  --robot-0-shortest-m <surveyed-metres> \
  --robot-0-shortest-evidence <survey-file> \
  --robot-0-reached-goal-region <yes-or-no> \
  --robot-0-target-verified <yes-or-no> \
  --robot-1-shortest-m <surveyed-metres> \
  --robot-1-shortest-evidence <survey-file> \
  --robot-1-reached-goal-region <yes-or-no> \
  --robot-1-target-verified <yes-or-no>
```

Add each successful robot's `--robot-*-terminal-evidence` file. The adjacent
metrics stays `incomplete` until all four scenes × five unique trials exist.
Only complete, independently evidenced trials are eligible for SR/SPL. The
full operator sequence is
[`ONECLICK_SESSION_WORKFLOW.md`](ONECLICK_SESSION_WORKFLOW.md).
