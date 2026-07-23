# Triple-AI historical-image preflight — 2026-07-23

## Result

Four historical two-robot image cases were selected for a five-repetition
Perception-VLM preflight.  All 24 referenced metadata/RGB/depth files passed
size, SHA-256 and wire-identity verification.  The set contains 2,226,951
bytes of observed local runtime input, with cross-robot capture skew between
0.027 s and 0.745 s.

The local loopback GLM completed all 40 sequential agent-image calls in
101.617 s with zero errors.  Mean call latency was 2.504 s (minimum 2.466 s,
maximum 3.096 s).  Every image produced identical source-weighted
Perception probabilities across its five repeats (population standard
deviation 0).

| Case / target | Robot, sequence | Source-compatible YOLO input | Mean Perception `P_yes` | Five-repeat std |
|---|---|---|---:|---:|
| chair shared view | WSJ 9388 | chair 0.9105, tv 0.2100 | 0.868398 | 0 |
| chair shared view | Yunji 147885 | chair 0.9334 | 0.999578 | 0 |
| plant single view | WSJ 10738 | potted plant 0.6395 | 0.890752 | 0 |
| plant single view | Yunji 158529 | no detections | 0.001846 | 0 |
| chair + table distractor | WSJ 9739 | chair 0.9344 | 0.999578 | 0 |
| chair + table distractor | Yunji 150697 | clock 0.5457 | 0.131602 | 0 |
| chair Odin pair | WSJ 15147 | suitcase 0.2800, tv 0.2001 | 0.059978 | 0 |
| chair Odin pair | Yunji/Odin1 165736 | chair 0.9390 | 0.999116 | 0 |

The asymmetric cases behave coherently: a plant visible only to WSJ is
strongly preferred only on WSJ; a chair visible only to Yunji/Odin1 is
strongly preferred only on Yunji.  The table-dominated view was deliberately
used as a distractor and was never promoted to an unsupported ObjectNav
target.

## Evidence boundary

This is observed historical-image and model-derived Perception evidence.  It
is not a full VLM decision cascade, new mapping evidence, physical navigation,
SR or SPL.  Static inputs have no newly executed path, robot-local planner
STOP, surveyed goal-region arrival or independent terminal verification.
Both runtime reports explicitly contain
`official_navigation_metrics_eligible: false`.

The tool has no Hub client or decision publication path.  During this run it
recorded zero Hub publications and `robot_commands_sent: false`.  The GLM
listened only on `127.0.0.1:31511` and was stopped after the run; the port and
GPU allocation were released.

## Provenance

Observed/model-derived runtime evidence (ignored by Git, retained locally):

| Path | Bytes | SHA-256 | Classification |
|---|---:|---|---|
| `hub/runtime/triple_ai_demo_image_preflight_verify_20260723.json` | 7,318 | `0e915ee58ad77156a5e36e27b4654f9a7973f4926fec16e8e8429a05534a3ff4` | observed byte/identity validation |
| `hub/runtime/triple_ai_demo_image_preflight_perception_20260723.json` | 23,564 | `d3d16058597de23275abdeda0c57e8d34bc94efa0dded96522acd51beca8c8ed` | observed/model-derived inference |
| `artifacts/vision/yolov10m.pt` | 33,643,667 | `6dc78f7a88591cec1e8716b8f5c7e3aefa9206684f025d202be34439ccb329a0` | source artifact model weights |

Tracked experiment implementation at the time of the run:

| Path | Bytes | SHA-256 |
|---|---:|---|
| `hub/config/experiments/triple_ai_demo_image_preflight_v1.json` | 11,327 | `57d81c87cd607a883bc62563280fc7def0b3aa6abe6a4b22a4c6931a1641d232` |
| `hub/src/focus_hub/demo_image_preflight.py` | 10,009 | `ab59ef9539077d52d518d57865dda8bcc0154ec60793b5a1304db10303cacb30` |
| `hub/src/focus_hub/realworld_eval.py` | 13,217 | `65738c3ed1a5c62a1313c16fca1f722cf0452c923d3e44797fba63eea29f2576` |
| `hub/tools/run_demo_image_preflight.py` | 8,107 | `6f31a8dd459676601b78ff8aac623f64a46fdbfb66f5bb7304abf7acf197c2b6` |
| `hub/tools/score_realworld_demo.py` | 2,593 | `d28613dd0fe2cfdd06a413fa003b704a2a27fd2b605c8c041a40e2de74e20364` |
| `hub/docs/TRIPLE_AI_REALWORLD_DEMO.md` | 6,007 | `e675225cbff5c7c2f37b0162ce4d70332bbd916404119d086ea6bc6799c9c92a` |

Immutable source references used to preserve the original prompt, success
aggregation and source-compatible SPL arithmetic:

| Path | Bytes | SHA-256 |
|---|---:|---|
| `source/Focus_realworld/main.py` | 103,808 | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` |
| `source/Focus_realworld/src/SystemPrompt.py` | 22,350 | `10ac3c18a4bd5438298fdd76972efd362e686608f267700bc56dd8747a1e45f1` |
| `source/Focus_realworld/agents/vlm_agents.py` | 46,500 | `992f0174d50b6959d538a418c224907156f784ffd4b35b5ef67c02da3461bee0` |

## Verification

- Historical input validation: 4 cases, 24 artifacts, all passed.
- Perception preflight: 40/40 calls passed, zero inference errors.
- New focused unit tests: 8 passed.
- Full repository verification: 232 tests passed.
- `source/` and `dependencies/`: no working-tree changes.
