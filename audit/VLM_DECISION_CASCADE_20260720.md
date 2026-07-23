# VLM decision cascade: full port, render-flip bug fix, live validation — 2026-07-20

> **2026-07-23 executable-source correction.** This audit originally said
> the gate-fail branch called a separate History Decision VLM and was not
> ported. A new direct audit of the immutable executable
> `source/Focus_realworld/main.py` shows the opposite: although
> `SystemPrompt.py` defines a history prompt, `main.py` assigns
> `Final_PR = history_score_copy` and takes the first maximum. The current
> Hub implementation now follows that executed branch. Historical test counts
> and results below remain dated evidence; the corrected current status is in
> `audit/SOURCE_DERIVED_VLM_SCENE_RUNNER_20260723.md`.

## Background

The hub's VLM frontier-choice logic (`choose_frontier_glm`, pre-existing) was a
single simplified GLM-4V call over an annotated BEV, not upstream's real
decision architecture. Prompted by a direct question ("这个focus realword的vlm端
具体是怎么做的"), reading `source/Focus_realworld/main.py` (both the local
read-only copy and the authoritative original on `ssh alantorch:/scratch/jl9356/
Focus_realworld`) surfaced that upstream actually runs a 3-stage cascade per
agent per step:

1. **Perception VLM** — Yes/No, "is this scene worth exploring for `{target}}`",
   given the raw RGB frame plus real YOLOv10 object-detection results.
2. **Judgment VLM ("FN")** — Yes/No, "explore a new frontier or revisit a
   historical point", given a rendered semantic map (frontier points, history
   points, robot pose+heading).
3. **Directional-memory update** — the combined Perception+Judgment score gets
   written with the source's exact 39-bucket slice arithmetic into a
   360-bucket per-location array shared by all agents
   (`history_nodes`/`history_count`/`history_states`/`history_score`), keyed to
   the nearest known position within 25 cells.
4. **Gate** — `FN_PR[0] >= 0.5 or l_step <= 125`; only if true does the
   Decision VLM actually run.
5. **Selection** — if the gate passes, Decision VLM picks a lettered frontier
   using the decision-first prompt patch. If the gate fails, executable
   `main.py` selects the first argmax from its frozen shared-history scores;
   it does not make a History VLM call.

Per the standing HPC-fidelity directive, this was ported in full rather than
left as the simplified version.

## What was confirmed on the authoritative HPC source (`alantorch`)

- `run_cmd.txt`/`arguments.py`: real baseline experiments use `--yolo yolov10`
  (the default, not a placeholder), `--yolo_weights` defaulting to
  `detect/yolov10m`, called as standard `ultralytics.YOLO(weights)(source=rgb,
  conf=0.2)`. FastV/PruMerge vision-token pruning are env-gated ablations, off
  by default, confirmed unused in the real baseline runs.
- `patch_frontier_prompt`'s decision-first patch applies unconditionally
  (not run-mode-gated).
- The VLM server (`CogVLM2/basic_demo/glm4_openai_api_demo_1gpu.py`, upstream's
  own code) is OpenAI-compatible with a non-standard `return_string_probabilities`
  field: each candidate string must map to exactly one token id (asserted), the
  softmax is taken over only the first generated token, sliced to the
  candidate token ids, renormalized. `temperature=0`, `max_tokens=1`.

## What was ported

- `hub/src/focus_hub/vlm_prompts.py` (new): verbatim system/template prompt
  text for all three stages, `perception_weight_decision` (the Bayesian-style
  renormalization of raw logits against the emitted text label — matches
  upstream's `Perception_PR`/`FN_PR` computation exactly, including its
  "Neither" fallback), `extract_scene_objects` (ported from `Objects_Extract`:
  per-category-channel contour extraction, `cv2.findContours` + `approxPolyDP`
  at 5% arc-length epsilon), `patch_frontier_prompt` (verbatim "Step 4" patch),
  `parse_frontier_decision` (diagnostics/logging only — the real decision uses
  logit scores, not this regex parse).
- `hub/src/focus_hub/directional_memory.py` (new): `DirectionalMemory`,
  ported faithfully including a genuine upstream inconsistency in the
  first-visit vs. revisit branches' wraparound arithmetic near 0°/360°.
  The asymmetric source slice bounds are reproduced exactly, not normalized.
  The four lists are episode-global in `main.py`, so the continuous adapter
  uses one shared instance for all agents.
- `hub/src/focus_hub/yolo_detector.py` (new): thin `YoloDetector` wrapper
  around `ultralytics.YOLO`, matching upstream's exact call shape. Uses the
  already-present `artifacts/vision/yolov10m.pt` (from the original G0
  transfer) — a real, non-placeholder detector, not a stand-in.
- `hub/src/focus_hub/frontiers.py`: added `render_semantic_decision_map`
  (ported from `Decision_Generation_Vis`) alongside the pre-existing
  `render_annotated_bev`, and **fixed a render-orientation bug in both** (see
  below).
- `hub/src/focus_hub/vlm_decision.py`: rewritten around `_call_glm` (shared
  request/parse mechanics), `choose_scene_worth_exploring_glm`,
  `choose_explore_or_revisit_glm`, upgraded `choose_frontier_glm` (now uses
  the decision-first-patched prompt), and `run_decision_cascade` (full
  orchestration, matches the `main.py` block from the two VLM calls through
  the `angle_score`/gate check).
- `hub/tools/hub_pipeline_daemon.py`: wired the cascade into the live daemon
  loop (YOLO detection, directional memory, both map renders, `pre_goal_point`
  tracked across cycles matching upstream's `pre_goal_points[j]`), with a
  `--no-cascade` escape hatch back to the old single-call path.

**Corrected on 2026-07-23:** the gate-fail branch is now ported as the
executable source implements it: freeze `history_nodes_copy` and
`history_score_copy` after agent 0, select the first maximum, and remove a
selected copied entry only when multiple candidates exist. No extra History
VLM call is invented. The same correction also preserves source ordering:
Perception, register/increment the current shared-history node, construct the
Judgment prompt, then apply the directional score. With no frontier,
Perception still updates history using `2 * Perception_PR[0]`; only the
source's unsafe random physical goal is suppressed by the deployment layer.

Other labelled, non-fidelity-breaking substitutions: `heading_deg` for real
robots is derived from `T_shared_camera`'s own +Z column (approximate — real
robots, especially wsj, don't carry Habitat's clean simulated-agent heading
state); the semantic-category background palette is a deterministic HSV
sweep, not upstream's hand-picked array (colors aren't prompt-load-bearing;
only frontier=black/history=green/pose=red/prev-goal=blue are, and those are
exact); the pose arrow is a simple triangle, not upstream's Habitat-utils
contour helper.

## Bug found and fixed: rendered VLM images were upside-down

While visually verifying the new `render_semantic_decision_map`, found that
both it and the pre-existing `render_annotated_bev` built the canvas with row
0 at the top (standard image indexing), drew **all** markers and letter
glyphs, and only then called `np.flipud()` on the finished canvas to achieve
the documented "row 0 at bottom (world +y up)" convention. Flipping an
already-rendered canvas mirrors the text glyphs vertically along with
everything else — confirmed visually (an "A" rendered as an inverted-V, and
lowercase history letters looking like different letters).

**This means every GLM-4V frontier/decision call made anywhere in this
project's history (including prior E2E, soak, and G4-pseudo-dual runs) was
shown a garbled, vertically-mirrored image of the letters it was asked to
reason about.** The frontier/history/pose *positions* were still geometrically
correct (only text glyphs are visibly asymmetric under a flip); what was
wrong is glyph legibility, i.e. the model was being shown corrupted labels,
not swapped positions.

Fixed in both functions by flipping the background image **before** any
drawing, and changing the row→pixel-y formula from `(row+0.5)*scale` to
`(h-1-row+0.5)*scale`, then returning the canvas directly (no flip at
return). Verified:

- Visually, via saved PNGs read back through the Read tool — letters upright
  and correctly positioned in both functions, before/after comparison.
- Via two new regression tests (`hub/tests/test_frontiers.py`):
  `test_render_annotated_bev_row_ordering_not_mirrored`,
  `test_render_annotated_bev_frontier_letter_upright` (checks ink-density
  asymmetry of the Hershey-Simplex "A" glyph — more ink in the lower half
  than the upper, which a vertically-flipped glyph would invert),
  `test_render_semantic_decision_map_history_row_ordering_not_mirrored`.

## Test coverage added

- `hub/tests/test_vlm_prompts.py` — prompt construction, `patch_frontier_prompt`,
  `perception_weight_decision` (including the zero-total fallback-to-label
  branch), `parse_frontier_decision` (decision-first, letter-anywhere,
  fallback), `extract_scene_objects` (real blob vs. sub-threshold noise).
- `hub/tests/test_directional_memory.py` — first-visit node creation, nearby
  revisit merging, far visit creating a new node, wedge wraparound at 0°,
  nearest-of-multiple-nodes selection.
- `hub/tests/test_frontiers.py` — the render-flip regression tests above,
  plus existing frontier-extraction coverage.
- `hub/tests/test_vlm_decision.py` (new this pass) — `_call_glm`-mocked tests
  for all three stage functions (including the no-scores/neutral-fallback and
  no-frontiers-raises paths) and `run_decision_cascade`: gate-passes-runs-
  decision, gate-fails-skips-decision (only 2 of 3 GLM calls made), early-
  episode step forces the gate open even with a low judgment score, zero-
  frontiers short-circuits before any GLM call, and a per-stage exception
  (perception/judgment/decision all raising) is recorded in `result.errors`
  and defaulted to neutral rather than propagated — one bad VLM call cannot
  kill a decision cycle.
- Full suite: 104 passed (was 93 before this session's additions; +11 from
  `test_vlm_decision.py`, the render-fix regression tests and directional-
  memory/prompt tests were added earlier this session and already counted in
  the 93).

The preceding paragraph is the original 2026-07-20 result. As of 2026-07-23,
the no-frontier regression instead requires one Perception call plus the
source history update, and new tests cover current-node prompt ordering,
frozen-history first-argmax, persistent shared state, exact decision steps,
largest target components, and TV's source 7x7 dilation.

## Live end-to-end validation against real GLM-4V-9B inference

Started the real offline server (`scripts/run_glm_offline.sh`, port 31511,
GLM-4V-9B, 4-bit, FastV/PruMerge confirmed disabled per startup log) against
the locally-cached HF weights (`artifacts/models/hf_cache`, 26 GB, no network
access). Ran a dedicated smoke script (not part of the pytest suite — a real
GPU inference call, minutes not milliseconds) through one full decision cycle:

- Input: `data/robot_replays/rednet_domain_gap_20260719/samples/frame0258_rgb.png`
  (a real wsj recording frame with a visible chair, from the earlier RedNet
  domain-gap dataset), a synthetic 60x60 grid with an explored region and a
  `chair`-category blob, and two synthetic frontiers.
- Real `YoloDetector` on the real frame: `{'person': 0.904, 'chair': 0.877,
  'tv': 0.529}`.
- `extract_scene_objects` produced a real polygon string for the chair blob.
- Full cascade result: Perception `P(worth exploring)=0.951`; Judgment
  `P(explore new frontier)=0.984`; gate passed (`judgment_pr_yes=0.984>=0.5`);
  Decision VLM chose frontier `B` with `P(B)=0.818` vs. `P(A)=0.182`; zero
  errors; `DirectionalMemory` recorded one node at the robot's position.
- The rendered decision map (`smoke_decision_map.png`, saved and visually
  inspected) shows upright "A"/"B" letters at their correct positions and a
  correctly-oriented red heading arrow — confirms the render-flip fix holds
  under the actual images sent to the real model, not just in synthetic unit
  tests.

Server cleanly stopped after the test; GPU memory confirmed back to its
pre-test baseline (988 MiB, same as before startup).

## Follow-up, same day: live validation against a real-built map (not a synthetic grid)

The smoke test above used a real recorded frame but a *synthetic* 60x60 grid
and two synthetic frontiers — it validated the GLM round-trip but not the
map/frontier-extraction plumbing feeding the cascade. Closed that gap with a
new script, `hub/tools/live_validate_vlm_cascade.py`: replays 60 real
keyframes from `data/robot_replays/wsj_semantic_map_record_20260717_102052`
through the real `CentralMapper`/`RedNetSegmenter` (same code path as G3),
extracts real frontiers from the resulting map (4 found: A/B/C/D, sizes
28-45 cells), derives real `robot_rc`/`heading_deg` from the last real pose,
runs real YOLO on the last real RGB frame, and drives `run_decision_cascade`
against the same live GLM-4V-9B server end to end.

Result: Perception `P(worth exploring)=0.002` (real YOLO found no detections
on that particular frame, and the model correctly said "No" for target
`chair`); Judgment `P(explore new frontier)=0.975`; gate passed (both by
score and by `step<=125`); Decision VLM chose frontier `D` with `P(D)=0.470`
(highest of the four, vs. 0.153/0.285/0.093 for A/B/C) — a real, non-uniform
distribution, not the flat 0.5/0.5 seen while debugging the endpoint URL.
Zero errors. This is different scene content from the earlier smoke test (no
chair visible in this particular frame, hence Perception correctly says
"No") — the two runs are consistent with each other, not contradictory: the
cascade responds differently to genuinely different real inputs, which is
the expected behavior of a real (not stubbed) pipeline.

This closes the "no live test against a real moving robot's data stream" gap
below — the map, frontiers, pose, and detections driving this run were all
produced by the real pipeline, not hand-built.

Server cleanly stopped afterward (confirmed via `pkill` + process-list
check); GPU memory confirmed back to baseline (991 MiB). Scratch runtime dir
(`hub/runtime/glm/`) removed after the run.

## What remains open

- `heading_deg_from_pose`'s approximation has not been validated against any
  ground truth (no robot currently reports a clean "forward" heading the way
  Habitat's simulated agent state does).
- The 2026-07-23 continuous runner is still shadow-only. A model-derived
  `Find_Goal` cannot become physical navigation success until a robot-local
  planner produces the source-equivalent `stop && found_goal` result under
  operator-present HIL safety validation.
