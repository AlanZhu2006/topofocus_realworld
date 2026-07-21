# RedNet domain-gap diagnostic on a real wsj recording

Date: 2026-07-19. Runs the exact same `RedNetSegmenter` used in production
(`hub/src/focus_hub/central_mapping.py`) over every keyframe of a real
recorded session and reports per-class statistics plus visual samples.
Tool: `hub/tools/analyze_rednet_domain_gap.py`. Mapping only — nothing here
talks to a robot.

## What this is and isn't

There is no ground-truth semantic annotation for this recording (or for any
real robot recording in this workspace), so this is **not** an IoU/accuracy
benchmark. What it is: a statistical + visual plausibility check against a
real, un-curated indoor scene, which is exactly the kind of input this
project's central mapper has to work with in deployment.

## Data

`data/robot_replays/wsj_semantic_map_record_20260717_102052` — the same
303-keyframe recording used for G3, 848×480 RGB-D. Full output:
`data/robot_replays/rednet_domain_gap_20260719/`.

## The 15 categories actually used for the semantic BEV map (`MP_CATEGORIES_MAPPING`)

| category | mp3d class id | frames present (of 303) | mean area when present |
| --- | --- | --- | --- |
| chair | 4 | **0** | — |
| sofa | 11 | **0** | — |
| plant | 15 | 21 (6.9%) | 2.8% |
| bed | 12 | 2 (0.7%) | 0.9% |
| toilet | 19 | **0** | — |
| tv | 23 | **0** | — |
| bathtub | 26 | **0** | — |
| shower | 24 | **0** | — |
| fireplace | 28 | **0** | — |
| appliances | 38 | **0** | — |
| towel | 21 | **0** | — |
| sink | 16 | 4 (1.3%) | 1.3% |
| chest_of_drawers | 14 | 1 (0.3%) | 0.05% |
| table | 6 | 2 (0.7%) | 1.2% |
| stairs | 16 | 4 (1.3%) | 1.3% |

Note: `stairs` and `sink` share the same raw class id (16) in upstream's own
`mp_categories_mapping` (`source/Focus_realworld/constants.py`) — verified
this is an upstream characteristic, not something introduced here; not
"fixed" locally per the standing HPC-fidelity rule (upstream is
authoritative even where it looks like it might be an upstream bug).

**9 of the 15 categories never fired once across 303 real frames**
(corrected from an earlier miscounted "10" — see "Correction" below). The
ones that did fire only appear in a small fraction of frames with small
area when present.

## Visual confirmation this is a real failure, not just an empty scene

The raw 40-class output is dominated by just two classes (`id 1`: 72.6% of
all pixels across the session; `id 3`: 9.9%) — no authoritative full 1–40
class-name list exists anywhere in this workspace (`mp3d_category_id` in
`constants.py` is a different, non-matching partial mapping — checked and
explicitly not trusted for naming), so these two are reported by numeric id
only rather than guessed names. But three sample frames, picked at evenly
spaced points in the session and inspected directly (not inferred from the
histogram alone), settle the question:

- **Frame 129**: RGB clearly shows a wheeled office chair in the foreground
  and multiple folding chairs stacked against the wall. The semantic
  overlay for this exact frame is **one single uniform color across the
  entire image** — the chairs get no distinct region at all.
- **Frame 258**: RGB clearly shows a green/gray office chair with a
  person's legs next to it, and server rack equipment. The overlay again
  shows no region distinguishing the chair (or the person, or the
  equipment) from the floor/wall — the only visible boundary in the overlay
  tracks the floor/wall geometric edge, not any object outline.
- **Frame 0**: a large white planter with visible plant leaves; the overlay
  boundary again follows the floor edge, not the planter's silhouette.

Across all three, the dominant "orange" region's shape closely tracks each
frame's floor/near-ground geometry, not any object boundary — the model is
behaving like a coarse floor-vs-not-floor segmenter on this footage, not an
object-category recognizer, for the categories that matter to this
project's ObjectNav task.

## Correction (same day, after user pushback)

The original version of this doc said "10 of 15 categories never fired,
including chair and sofa — arguably the two most common indoor ObjectNav
targets" and recommended YOLO reinforcement "chair/sofa most urgently."
Both parts were wrong in ways worth recording plainly rather than quietly
editing away:

1. **Arithmetic error**: it's 9, not 10 (re-verified directly against the
   summary JSON: `chair, sofa, toilet, tv, bathtub, shower, fireplace,
   appliances, towel` = 9 categories at zero).
2. **Overstated scope**: the user pointed out this specific recording
   likely just doesn't contain a sofa at all, and asked to double-check
   before treating it as a RedNet failure. Re-inspecting all 8 saved sample
   frames (not just the 3 originally picked) specifically hunting for a
   sofa: **there isn't one in any of them.** This is an office/corridor
   environment (carpet, wood-panel walls, fire extinguisher, stacked
   books, office chairs) — not a plausible setting for a sofa, toilet,
   bathtub, shower, fireplace, appliances, or towel either. Those 7
   categories reading zero is much more likely a correct true negative
   (nothing there to detect) than a demonstrated RedNet failure — this doc
   never actually visually confirmed those objects were present and
   missed; it only confirmed chair was.
3. **What actually holds up**: re-scanning all 8 sample frames (not just
   the 3 originally checked) for chairs specifically, they appear clearly
   in **4 of 8** (frames 0086, 0129, 0172, 0258 — spread across different
   points in the 303-frame sequence, not one fluke frame), zero detected
   in any. The chair finding is solid. The sofa/other-6-category framing
   was not equally supported and has been removed from the conclusion
   below.

## Conclusion

This is a real, visually-confirmed domain gap **for the `chair` category
specifically** — not a statistical artifact or an artifact of an
object-sparse recording (4 of 8 inspected sample frames contain
unambiguous, prominent chairs that all get zero detection, spread across
the session rather than one fluke frame). RedNet here was trained on MP3D
(Matterport-scan-derived synthetic renders); this real Jetson+RealSense
footage differs in enough ways (camera/lens characteristics, real-world
lighting and materials, chair styles, image compression) that the model's
practical recognition rate for at least this one target category looks
close to zero on this session. The other 8 zero-detection categories are
plausible true negatives for this specific office/corridor environment,
not separately confirmed failures — see "Correction" above.

**Recommendation: YOLO reinforcement is worth building for `chair`**,
where the evidence is solid. Extending the recommendation to sofa or the
other categories would need either a recording that plausibly contains
them, or ground-truth annotation — neither exists here. Scope note: this
only evaluates RedNet's *object-category* channels: the `obstacle`/
`explored` channels (height-banded geometry, not learned category
recognition) are unaffected and were separately, favorably cross-checked
against TinyNav's own raycast occupancy in G3
(`audit/G3_LOCAL_REPLAY_MAPPING.md`, ~87% agreement) — this domain-gap
finding does not call that into question.

## What this does and does not prove

Proves: RedNet's category recognition, as actually wired into this
project's central mapper, fails to detect visually obvious instances of at
least one of its target categories (chair) in real footage from this
robot's actual camera — a genuine, visually-confirmed finding, not
speculation.

Does not prove: the exact cause (could be camera/lens/exposure
characteristics, real-world material appearance, image compression, or a
combination — not isolated here), that YOLO would actually fix it (not
built or tested), or that this generalizes to Yunji's camera (different
sensor, not tested here), or a quantitative accuracy number (no ground
truth exists to compute one).
