# Executable-source behavior parity

This note defines what “source-compatible” means for the real-world Hub. The
reference is the executable default path under `source/Focus_realworld/`, not
comments, paper-level descriptions, or inactive command-line modes. The Hub
still emits only versioned, expiring high-level targets; each robot retains
final stop/reject authority.

## Reviewed immutable source

The following inputs were observed locally on 2026-07-29. Runtime shadow
manifests recompute and preserve these identities under
`focus-source-behavior-contract-v1`; both shadow generation and batch
construction fail closed if a size/hash changes or if the new contract loses
one of its required behavior, semantic or A–D binding records.

| Source path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `source/Focus_realworld/main.py` | 103808 | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` | observed immutable authoritative source |
| `source/Focus_realworld/agents/vlm_agents.py` | 46500 | `992f0174d50b6959d538a418c224907156f784ffd4b35b5ef67c02da3461bee0` | observed immutable authoritative source |
| `source/Focus_realworld/arguments.py` | 14140 | `66dc9a94459215d9a51d97bf8f195fd486759d7f34529c60e2a57999665a61d3` | observed immutable authoritative source |
| `source/Focus_realworld/constants.py` | 28432 | `6217a75db7e012602b70d6f5c76265cf90ff8d365a6176e5ce293fad5aafd106` | observed immutable authoritative source |
| `source/Focus_realworld/src/SystemPrompt.py` | 22350 | `10ac3c18a4bd5438298fdd76972efd362e686608f267700bc56dd8747a1e45f1` | observed immutable authoritative source |
| `source/Focus_realworld/src/frontier_parser.py` | 10618 | `7add79b0f8110cf11468c8e8d1d11127d46b84c8427bf5721e7f5a3ed995bcb7` | observed immutable authoritative source |
| `source/Focus_realworld/src/vlm.py` | 13244 | `ac0503b2f311c924a794f9c2cf678684e43b7a3797f6dc20a8d9e55a7dc713e8` | observed immutable authoritative source |
| `source/Focus_realworld/utils/semantic_prediction.py` | 8684 | `e7c2591235f69ef03917bce813516b53805ab8dbb529fa16b9d9fce7affec95d` | observed immutable authoritative source |
| `source/Focus_realworld/utils/visualization.py` | 4381 | `8a989f5ffcab28dbc1a2d000ed5cd144b434a36b7becde32d4d6b556a1a6e582` | observed immutable authoritative source |

The available RedNet backbone artifact is
`artifacts/checkpoints/rednet_semmap_mp3d_40.pth`, 656550984 bytes,
SHA-256
`f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995`.
The source-referenced Detectron2 artifact is
`artifacts/checkpoints/detectron2_mask_rcnn_R_50_FPN_3x_model_final_f10217.pkl`,
177841981 bytes, SHA-256
`9a737e290372f1f70994ebcbd89d8004dbb3ae30a605fd915a190fa4a782dd66`.
Both identities are checked before the exact pixel backend starts. Artifact
and toolchain provenance is pinned in
`hub/config/source_semantic_stack.json`.

## Active default behavior

| Mechanism | Executable source behavior | Hub implementation and status |
| --- | --- | --- |
| Decision cadence | VLM rounds occur at logical steps `0,24,49,...,499`. | Persistent source clock in `live_vlm_shadow.py`; exact source-derived cadence. |
| Shared map | Per-agent obstacle, explored and semantic channels are fused with elementwise maximum. | Frozen synchronized maps use the same channel-wise maximum after frame/calibration validation. |
| Frontier geometry | Close explored mask with `5x5`; keep the largest explored contour; subtract a `3x3` obstacle dilation; label with 8-connectivity; accept area greater than four cells; sort by area; keep at most A–D. The executed loop also skips the first scan-ordered component. | `frontiers.py` reproduces these operations, including the observable skipped-component behavior. Only rectangular-grid coordinate scaling is generalized for real maps. |
| VLM cascade | Perception, Judgment, the source frontier/history gate, then Decision. | The same three-stage prompts, gate order, and error fail-closed contract are retained. |
| VLM image transport | `src/vlm.py` serializes source arrays losslessly as PNG bytes under its historical `data:image/jpeg` URI. Camera input is RGB; the already-BGR semantic decision array is passed to PIL unchanged. | The Hub now reproduces the same stage-specific array and lossless byte contract. It no longer JPEG-compresses small A–D/history glyphs or semantic text. |
| VLM generation request | Source requests model `cogvlm2` with temperature/top-p `0.8/0.8` and reads first-step candidate scores plus generated text. Its generic client allows up to 2048 output tokens. | The Hub uses the same model and sampling parameters, but caps output at the first label token because every active prompt consumes only that token and the first-step score vector. This removes unused generation latency without changing the score step used by the algorithm. |
| A–D meaning | Candidates come from one shared frontier dictionary and normally an assigned entry is removed before the next robot. The one-frontier branch instead reuses that point for every robot. In the executed later-agent branch, the prompt also re-enumerates the remaining dictionary while the image is rendered from the original dictionary, so a letter can refer to different coordinates. | The Hub preserves one shared stable A–D component ID across image, prompt, score token, selection and transport, then removes the assigned ID. This explicitly corrects the source’s later-agent image/prompt mismatch. It also HOLDs a later robot when only the already-assigned frontier remains, rather than commanding both physical robots to converge on one point. |
| History branch | If the gate selects history, the first argmax directional-history score chooses the shared historical node. Beyond 26 nodes, the source image switches to uppercase labels but its Judgment prompt continues incrementing Unicode after `z`, creating another image/prompt mismatch. | Directional history and its first-max score order are retained. The Hub keeps one stable `a-z`, then `A-Z` label between Judgment image and prompt, explicitly correcting that source ambiguity; a stale or spatially blocked history candidate is rejected before physical publication. |
| Decision image | Fixed source palette, semantic layers, frontier edge, A–D, green history nodes, red pose arrow, blue previous goal, visited paths and a 480-pixel canvas. Before prompt construction, source also flips semantic polygon rows into the rendered 480px coordinate system. | `render_semantic_decision_map` reproduces these layers, and semantic polygon prompt coordinates are bound to the same rendered pixels. New maps store the observed base trajectory inside the same atomic NPZ generation, so a later `live_status.json` update cannot leak future path points into the frozen VLM image; legacy replay records its weaker temporal alignment explicitly. |
| Semantic target override | A detected goal category overrides exploration for that robot through the largest connected target component. The 7x7 TV dilation is applied only to the local planner target mask, not to the VLM decision-map rendering. | The same per-robot override and TV planner-mask rule are applied before shared frontier allocation; the renderer keeps the undilated semantic component as the source does. |
| Goal continuity | Retain the previous frontier while the robot remains at least 25 source cells from that previous goal. | `v2_goal_continuity.py` measures current-pose-to-previous-goal distance. At 0.05 m/cell the boundary is 1.25 m. The formal runner locks this value instead of accepting a source-incompatible override. Continuity memory stores the lineage’s pre-projection source frontier; a robot-specific clearance projection remains a one-round execution point and cannot drift the next source decision. |
| Stationary replan | Independently, movement of at most 2.5 source cells between decision boundaries triggers a new goal. | `v2_source_replan.py` handles this separately and in source order. At 0.05 m/cell the threshold is 0.125 m and one qualifying interval triggers replanning; both values are locked at the formal entry point. |
| Collision evidence | Less than 0.10 m forward progress marks cells ahead in the agent-local collision map and changes the next FMM query. | Robot-local planners retain their obstacle authority. An explicit local rejection is additionally carried across Hub rounds as robot-, pose-, target- and direction-specific high-level failure memory. |
| Failed-candidate fallback | Source replans over its updated traversible map while retaining VLM score ordering. | The Hub rejects only a matching failed approach, then tries remaining candidates in the original Decision or history score order; each still passes the current robot’s frozen-map reachability and footprint-clearance check. |
| Per-agent frontier availability | Even with shared frontiers present, the executable branch checks whether that agent’s own map has a frontier before its VLM decision; its no-frontier fallback is an unconstrained random map cell. | The current per-robot frozen-map reachability/footprint guard is the physical equivalent of that local availability test. The VLM candidate evidence is still preserved, but an agent with no locally reachable source-ranked candidate HOLDs instead of receiving the source’s unsafe random point. |
| Previous-goal/failure pose | Source has the current simulator pose synchronously. | The Hub uses a live shared-base pose captured no earlier than the rejection event when available. A frozen round-start proxy is explicitly labelled source-derived rather than claimed as an observed failure pose. |
| Target arrival | A source semantic STOP can redirect/stop the group through shared terminal state. | A robot-local semantic arrival atomically holds both physical robots and preserves terminal evidence. This is a conservative physical safety adaptation. |

## Mechanisms checked but intentionally selection-neutral

`arguments.py` defaults `enable_pruning` to false. Although `run_mode` defaults
to the string `pruned`, the parser does not translate that string into
`enable_pruning`; the executable Decision branch checks only the Boolean flag.
Therefore the reviewed default selection path is unpruned.

- Room segmentation and room semantics are computed for visualization,
  analysis and active-patch bookkeeping, but are not supplied to the Decision
  VLM in the active default branch.
- Attention degree-of-difference is recorded after selection and does not
  change the selected candidate.
- Active patches update possible future pruning state and do not affect the
  current unpruned decision.

The Hub records this execution profile in every new shadow manifest. Replay
validation rejects a malformed profile instead of treating these inactive
mechanisms as hidden selection inputs.

## Semantic pixel-model contract

The executable source pixel path is not RedNet alone. It first predicts
MP3D-40 classes with RedNet, then applies Detectron2 Mask R-CNN COCO masks:
chair, sofa and bed clear unsupported RedNet pixels; plant, toilet and TV add
Mask R-CNN pixels.

The Hub now contains a compatibility build of Detectron2 0.6 from pinned
upstream commit `b4a4a3bd136852dae5fb1de37978dee412653e31` and the exact
source-referenced Mask R-CNN weight. The
`source_rednet_detectron2_hm3d15` backend retains a multi-hot HM3D tensor and
reproduces the six source statements in their original order, including their
literal `== 0` and `== 1` instance-count tests. It also keeps the source
Mask R-CNN input at original camera resolution rather than silently applying
Detectron2's `DefaultPredictor` resize. New sessions default to this backend
with source maximum semantic fusion. YOLO detections remain Perception-VLM
evidence only and do not mutate semantic map pixels. Thus:

- navigation/VLM control flow is source-derived as described above;
- the default new-session pixel composition is the executable source
  RedNet + Detectron2 path;
- SegFormer is an explicit deployment adapter, not falsely labelled as the
  executable source pixel model;
- RedNet without the Mask R-CNN override is labelled `rednet_backbone_only`;
- both robots must use one identical pixel backend, temporal fusion mode,
  HM3D channel order and YOLO map-mutation policy in a fused round;
- the manifest states
  `source_maskrcnn_override_available_in_hub: true` only for the exact
  composite backend. Historical SegFormer/RedNet manifests remain replayable
  and retain `false`.

This establishes source-code pixel-pipeline equivalence, not real-camera
semantic accuracy: outputs remain `model_inference_unverified` until evaluated
against labelled real-camera masks. Only the source-referenced model and
compatibility toolchain artifacts were downloaded; no HM3D data, simulator
scene, overlay or SIF was fetched.

## Physical execution adaptations

These checks are downstream of the preserved raw VLM candidate and do not
rewrite its score:

1. per-robot known-free reachability and footprint-clear frontier approach;
2. bounded approach projection only to a source-ranked candidate’s reachable
   arrival disk;
3. shared-route conflict serialization;
4. expiring leases, local freshness/health gates and final robot stop/reject
   authority;
5. platform-specific local navigation: TinyNav on WSJ and WATER-backed
   high-level execution on Yunji.

The simulator’s discrete camera tilt/turn actions and exact FMM motor action
loop are not portable robot commands. Their relevant decision evidence is
preserved, while physical velocity authority remains local.

Two simulator shortcuts are deliberately not copied into physical authority.
When no frontier exists, the source samples an unconstrained random map cell;
the same random fallback also applies when an individual agent has no local
frontier. The Hub instead remains HOLD unless a source-ranked history/frontier
candidate passes that robot’s physical guards. The source also forces visited
cells traversible and can clear its local map after repeated
replan/collision counts; visited paths remain visible to the VLM, but neither
rule may erase a real robot’s current obstacle evidence.

## Regression evidence

- Unit tests cover exact frontier component ordering, source rendering,
  A–D binding, semantic-contract uniformity, 25-cell continuity, the separate
  2.5-cell stationary rule, failure-pose provenance, rejected-approach memory,
  source-ranked fallback and per-robot clearance.
- A read-only replay of Scene 03 round 7→8 uses frozen runtime evidence from
  `hub/runtime/oneclick_scene03-plant-long-20260729-215035-startmap1_live_scene03-plant-long_20260729_215643_421780621`.
  The old controller ended the episode after counting Yunji’s explicit
  `LOCAL_PLANNER_PATH_STALE` HOLD as ordinary cross-round stagnation. With
  the corrected rules, that interrupted interval is excluded, Yunji bypasses
  stale-goal continuity, and the failed approach is remembered. Round 8
  already supplies Yunji a different VLM frontier at
  `(5.1464 m, 7.8028 m)`, so failure memory correctly does not reject it and
  the round continues. Re-running the exact source frontier extractor on the
  same frozen map yields its nearest corresponding component at
  `(5.0464 m, 7.8028 m)`; the 0.10 m difference comes from replacing the old
  simplified extractor, not from robot motion.

The replay inputs are preserved as observed frozen-runtime evidence:

| Path relative to the run directory | Bytes | SHA-256 |
| --- | ---: | --- |
| `scene_manifest.json` | 60062 | `97af81c8d0d609b33ce3c0a7e7b5c7925e229d6e96c356e22578ca851a50d425` |
| `round_07_step_174/initial_batch.json` | 4758 | `1def30fb4af53f3c4e005ce1d4cb09253c739434304cf535c853bf692dc06dbb` |
| `round_07_step_174/frontier_clearance_guard.json` | 8219 | `af32a8f8149f64fef637c0012bcded13b638c0f408a8bde61019c0d595246ca6` |
| `round_08_step_199/vlm_candidate_batch.json` | 4554 | `50237b2abab6fe963ccdd7b86d202303e1087e280ae9580d40f1ddb6015ea10e` |
| `round_08_step_199/shadow/fused_decision_map.npz` | 26159 | `37625b40a83c26de2836b75312f88032edb4f41dc511520378b4b77712c1b402` |
| `round_08_step_199/accepted/wsj/live_status.json` | 6926 | `9206f55888fcf1292c214703a3219bcafe5c453b36a2d5930e208260ca832f07` |
| `round_08_step_199/accepted/yunji/live_status.json` | 8567 | `6f19df26d28ba9b79e79fd82f9ff859aab25ece8b28c193f379378d98fda02ee` |

This replay is observed frozen-runtime evidence plus source-derived offline
execution. It is not a claim of a new physical success; a future live run
still requires a fresh operator motion authorization.
