# Pixel semantic overview and clean-map cutover

Status time: 2026-07-23 16:20 CST

## Scope and safety

This audit records the operator-only 2-D overview requested from
`example.png`, the real-camera semantic backend comparison, and the cutover to
fresh map directories. No receiver, planner, `GOAL`, velocity command, or
robot-control topic was started. Hub `/healthz` reported
`goal_output_enabled=false` for both robots before the map launch.

`source/` and `dependencies/` were not modified. The source decision cascade
is unchanged. SegFormer is explicitly a Hub deployment adapter for real-camera
pixel masks; its output is model inference without labelled real-world ground
truth.

## Visual reference

Observed workspace reference:

| Path | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `example.png` | 10,736 | `ecdc053d57ddf23dbd5fc80cc3f5692c96b381cd74245f16da75aa7ebddb5360` | observed user-supplied reference |

The implemented overview combines light geometry, semantic pixel components,
compact labels, transported camera position/heading, accumulated trajectory,
and optional lettered frontiers. It is a read-only raster; it does not replace
or mutate the map/VLM tensor.

Representative observed render:

| Path | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/runtime/analysis/semantic_overview_20260723/fused_rgb_pixel_v2_display3.png` | 38,903 | `389fd5e0de9858badb53deed278302494ac7470cfef704d70876bdac33f114f5` | observed local render from live map snapshots |

`hub/runtime/` is intentionally ignored by Git; the table preserves identity
without pretending the generated image is a source artifact.

## WSJ input correction

The previous map daemon had accumulated infrared RGB plus goal-scoped
YOLO-box/depth-slab evidence. Maximum semantic fusion made those old
chair/plant regions irreversible. That directory remains preserved as
historical evidence and was not reused.

The WSJ sender now synchronizes real RealSense color with TinyNav keyframe
depth/pose and reprojects color onto the TinyNav infra/depth grid using the
observed static camera transform. Only calibrated overlap keeps non-zero
depth. This preserves the existing TinyNav pose and shared-board calibration
while preventing unregistered fallback color from entering 3-D.

## Observed RedNet comparison

The exact production RedNet checkpoint was run over registered-color WSJ
sequences 16141–16145:

- checkpoint:
  `artifacts/checkpoints/rednet_semmap_mp3d_40.pth`,
  656,550,984 bytes,
  SHA-256
  `f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995`;
- valid 0.3–5.0 m depth fraction: `0.44036409`;
- production confidence gate: `0.8`;
- production-thresholded chair pixels: zero in all five frames;
- raw low-confidence plant area fraction: `0.01261842`;
- production-thresholded plant pixels: zero.

Report:
`hub/runtime/analysis/rednet_wsj_registered_rgb_20260723_v1/domain_gap_summary.json`,
21,547 bytes, SHA-256
`41d5ea1ac25ac51abd51e8b7db219cd83a75f5a274273c9c097d34a380643b3c`.
This is an observed model diagnostic without semantic ground truth.

## SegFormer adapter provenance and observation

Pinned source:
`nvidia/segformer-b0-finetuned-ade-512-512`,
revision `489d5cd81a0b59fab9b7ea758d3548ebe99677da`.
Exact file identities are in `manifests/artifacts.json`; the preparation
script verifies them before use.

Using confidence `0.35` and the previously validated native-logit argmax /
nearest-neighbour restoration, every one of WSJ sequences 16141–16145
contained a thresholded chair silhouette. Per-frame chair pixel counts were:
`12630, 10869, 10958, 11131, 8104`.

The live map uses `semantic_fusion_mode=multi_view`,
`semantic_min_hits=2`, and `semantic_winner_margin_hits=1`. Each class may add
at most one vote per map cell per accepted keyframe. This is why a single
frame can no longer paint a permanent semantic region.

## Clean map sessions

- WSJ:
  `hub/runtime/map_out_wsj_20260723_rgb_pixel_v2`,
  start-after sequence `16350`,
  transform `wsj-tinynav-depth-20260723-calib-v1`;
- Yunji:
  `hub/runtime/map_out_yunji_20260723_rgb_pixel_v2`,
  backfilled only sequences `170701–170790`,
  transform `yunji-odin1-board-20260723-v1`;
- shared calibration:
  `shared-board-odin1-20260723-v1`;
- map format:
  `focus-hub-central-map-v3`;
- both maps reported `mapping_blocked_reason=null`.

Yunji's live upload stopped at 15:56:59 CST, so the bounded 90-frame backfill
was used rather than fabricating freshness. It spans 2.40 m of observed
trajectory and uses one transform/frame contract throughout.

One apparent Yunji plant was checked against source sequence 170776. The
thresholded pixels coincide with a real potted plant visible beyond the chair:

| Path | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/runtime/analysis/semantic_overview_20260723/yunji_segformer_plant_peak.jpg` | 199,436 | `0ed0de63e0c712e2f6dbdd19d325fba1e4f1fe506c1fc764e3e09fbf4a071864` | observed model overlay, no GT |

## Foxglove cutover

The relay at the unchanged production addresses `8765/8766` now reads the two
clean map directories. A protocol client observed advertisements for:

- `/wsj/semantic_overview`;
- `/yunji/semantic_overview`;
- `/fused/semantic_overview`;
- the existing camera, geometry, semantic Grid, pose, status and `/tf` topics.

After subscription, observed non-empty protobuf payload sizes were 24,208,
31,967 and 30,805 bytes respectively. The WSJ camera topic also delivered a
current message. Yunji camera delivered no message within six seconds,
consistent with its stopped input; no stale frame was synthesized.

The old map directories remain recoverable. Their daemons and old relay were
stopped only after the new relay passed health and message subscription checks.

## Interpretation limits

- A stationary depth camera naturally creates a fan-shaped partial map and
  samples only currently visible object surfaces. A complete room-like outline
  and a non-trivial WSJ trajectory require physical movement.
- Charcoal cells are height-filtered depth endpoints. Source-pixel inspection
  showed the current WSJ endpoints on real desk/chair/table surfaces, not on
  the carpet. Thin missing-depth seams are closed only in the operator raster.
- The displayed triangle is the transported camera pose and optical-forward
  heading, not an independently calibrated Go2 body footprint.
- Pixel semantics are substantially more appropriate than detector boxes for
  this view, but their quantitative accuracy remains unverified until a
  labelled real-camera validation set exists.
