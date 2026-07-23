# Semantic overview and Foxglove re-audit — 2026-07-24

Status time: 2026-07-24 05:17 CST

## Outcome and safety boundary

The requested `example.png`-style operator view is implemented for WSJ,
Yunji and strict shared-frame fusion. The current production relay was
replaced and protocol-level subscriptions received all three non-empty PNG
topics. No planner, receiver, WATER API, ROS goal, Hub `GOAL` or velocity
command was invoked.

Both local Hub health endpoints were observed with
`goal_output_enabled=false` for both robots before relay replacement.
`source/` and `dependencies/` have no diff. The old v12 map daemons were not
restarted or reprocessed because their physical session is historical and
Yunji has since left that placement. After the final snapshots were frozen,
both daemons handled SIGINT, wrote `shutdown` records and were removed from
tmux; the read-only relay continues to display their preserved files.

## Root causes found

### The production relay had not loaded the renderer

The process owning ports 8765/8766 was PID 2708861, started at 01:16:19 CST.
The semantic-overview implementation on disk was newer than that process.
Its old `/healthz` response exposed only a generic status and robot-name list,
so an open port could be mistaken for current code. This directly explains
why a source change or layout update did not change the visible old image.

The relay now computes one import-time SHA-256 over
`hub/tools/foxglove_relay.py` and
`hub/src/focus_hub/map_visualization.py`. Health exposes that loaded value,
and one-click requires it to match the current checkout. It also requires
both robot overview images and the fused overview to be generated before
declaring Foxglove ready.

### Stationary interval frames defeated the earlier multi-view gate

The existing real-camera adapter required two semantic hits, but the keyframe
selector accepts a frame every five seconds even when translation and rotation
remain below threshold. Each such `reason=interval` frame previously counted
as another semantic view. Repeated inference from one frozen image could
therefore confirm a chair/plant error and make the earlier two-view fix appear
ineffective.

`multi_view` now counts semantic votes only for first/translation/rotation
keyframes. An interval-only keyframe still refreshes explored/obstacle
geometry, but does not increment semantic hits. Dedicated tests prove that
the first frame votes and the stationary interval refresh does not.

This is a source-derived repair and locally tested. It is not retroactively
applied to the old v12 tensor and remains physically unverified until the next
fresh calibrated map.

### The displayed pose was the camera, not the robot base

Historical `live_status.json` carried only camera XY/heading/trajectory.
Future map daemons now derive
`shared_T_base = shared_T_camera × inverse(base_T_camera)` from the measured
mount already present in the transport contract. Foxglove and the VLM
frontier location prefer this calibrated base pose and +X heading. Old
snapshots remain explicitly labelled `historical_camera_pose_fallback`; no
body pose is fabricated for them. A pose-jump latch also resets the displayed
trajectory to the current point, preventing a false line between incompatible
coordinate frames.

### File rewrite time was being mistaken for map freshness

The old daemon rewrote the same `central_map.npz` every snapshot interval even
with no new observations. During this audit, both map files changed mtime
again after five seconds while `frames_total`, `observations_seen` and
`live_status.json` remained unchanged. A file-age display could therefore
make frozen content look fresh.

New daemons persist a map only after a newly handled observation, changed
integration count or changed mapping latch. They record the last handled
sequence and the last actually integrated capture timestamp. Foxglove reports
input-capture and map-content age from those timestamps; file mtime is write
provenance only.

## Current semantic evidence: what is and is not a false positive

The frozen source images used by the current v12 map show a real chair and a
real potted plant in both robot views:

| Input | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/runtime/spool/robot-0/00000000000000021765/rgb.jpg` | 117,239 | `1604fc6c704e472b136b5b7a4099ca8d1da0292debba2d689735436cd8cc5fa1` | observed RGB; chair and plant visually present |
| `hub/runtime/spool/robot-1/00000000000000200514/rgb.jpg` | 153,329 | `05422b8c5ebdcd955c7adf8feae5218a9107fdc9a59f06484ea8ac6cf10df2f0` | observed RGB; chair and plant visually present |

The current map statistics are:

| Map | Geometry | Semantic cells/components | Status |
| --- | --- | --- | --- |
| WSJ v12 | 5,249 explored; 1,024 obstacle | chair 109/1; plant 59/1 | frozen SegFormer map inference, no ground truth |
| Yunji v12 | 10,367 explored; 1,000 obstacle | chair 61/7; plant 19/5 | frozen SegFormer map inference, no ground truth |

The last WSJ YOLO frame reported chair 0.510 and potted plant 0.471. The last
Yunji frame reported chair 0.871, potted plant 0.860 and two implausible
`airplane` boxes over the lower robot/image region. However,
`semantic_yolo.map_reinforcement_enabled=false` and `last_evidence=[]` on both
robots. Therefore YOLO detections are Stage-1 text evidence only and did not
paint these map pixels. The colored BEV is produced by the SegFormer
deployment adapter plus depth/pose projection.

A visible plant region is therefore not proven false merely because plant was
not the requested goal. Conversely, the images do not constitute labelled
pixel ground truth: class accuracy and metric projection accuracy remain
unverified. The old v12 evidence is preserved rather than cosmetically erased.

## Reproducible operator raster

Reference identity:

| Path | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `media/image/example.png` | 10,736 | `ecdc053d57ddf23dbd5fc80cc3f5692c96b381cd74245f16da75aa7ebddb5360` | observed user-supplied visual reference |

The new read-only exporter froze the v12 map/status inputs, rendered the same
code path used by Foxglove and wrote a provenance manifest:

| Output | Bytes | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/runtime/analysis/semantic_overview_20260724_reaudit_v4/wsj_semantic_overview.png` | 30,543 | `9bcd32dd7d9edaeffc286fd4d4d84f0ac7b32366acca061b44cde689d64802b7` | source/model-derived operator view |
| `hub/runtime/analysis/semantic_overview_20260724_reaudit_v4/yunji_semantic_overview.png` | 31,685 | `1ed6988ef6d3da3aadd695f0fb1f1f86a4caf4526b33703cd5a77b3dbfd2315c` | source/model-derived operator view |
| `hub/runtime/analysis/semantic_overview_20260724_reaudit_v4/fused_semantic_overview.png` | 44,497 | `2559e29326f3f20f4b91ea945de128a59bd4590db245372ebf5cc7822aa2a852` | strict-contract fused operator view |
| `hub/runtime/analysis/semantic_overview_20260724_reaudit_v4/manifest.json` | 5,593 | `dd7aa450f8e0d097a46dbb749ab7f90f52a9280f26feb08ef62400c3595a7ec7` | observed checksum/provenance manifest |

The renderer assigns exact colors after categorical evidence reduction, fills
the complete accepted semantic pixel component, labels its largest component,
draws trajectories and heading triangles, and lays out robot names, frontiers
and semantic callouts without text overlap. Display-only closing of very small
missing-depth seams does not modify the map tensor or VLM input.

The v4 directory also contains checksum-verified copies of each exact
`central_map.npz` and `live_status.json` under `inputs/<robot>/`; later changes
to a source map path cannot invalidate reproducibility. `hub/runtime/` is
intentionally ignored by Git. These identities preserve the evidence without
misrepresenting runtime outputs as source files.

## Live Foxglove verification

The managed production relay session is
`foxglove_relay_20260724_rebuild_v12_router025`, reading the two historical
v12 directories at ports 8765/8766. After the final source reload, health
reported:

- contract `focus-semantic-overview-v2`;
- loaded relay/renderer source SHA-256
  `954ecb205d39bbae4cec601748d6a0cafae490a0ac238e2680260a1709bc8632`;
- both per-robot semantic overviews ready;
- fused overview ready with calibration
  `shared-board-odin1-20260723-v3`;
- WSJ map counts 5,249/1,024/168
  explored/obstacle/semantic;
- Yunji map counts 10,367/1,000/80.

The same health response reported WSJ/Yunji map-content ages of approximately
11,496/11,508 seconds while their rewritten snapshot-file ages were only
approximately 1.1/3.3 seconds. This is direct observed evidence for separating
content freshness from file mtime; the legacy status correctly names its
source `legacy_last_input_capture_time_fallback`.

A raw client used the required `foxglove.sdk.v1` protocol, observed all three
channel advertisements, subscribed, decoded the protobuf
`foxglove.CompressedImage` messages and validated PNG signatures, dimensions,
non-white pixels and non-zero pixel variance. This proves that the server sent
real images rather than merely advertising topics.

| Live topic | PNG bytes | Dimensions | SHA-256 |
| --- | ---: | --- | --- |
| `/wsj/semantic_overview` | 17,827 | 420 × 688 | `df241f5eba0355aaa737f72ff07e373729358ae6fe09e89fc8b45d584acf0733` |
| `/yunji/semantic_overview` | 22,328 | 692 × 744 | `d1eaaad00e4f98a9a26469c9e636926468870ffaabf8fce5e368ff521980bbb5` |
| `/fused/semantic_overview` | 27,136 | 764 × 740 | `4d3e352f55336eb74b7c152daa2e623a66a628576fd74dd34e5e15fea46edad4` |

Fresh relay startup intentionally had no retained camera JPEG for either
robot. Camera panels remain blank until a real preview publisher pushes a new
frame; the relay does not synthesize a stale camera image. This does not affect
the three map overviews.

## Verification

- complete `bash hub/scripts/verify_repository.sh --tests`: passed;
- Python AST, repository JSON/YAML, shell syntax and whitespace gates: passed;
- focused mapper/pipeline/renderer/relay/export/layout tests: passed;
- `source/` and `dependencies/` diff: empty;
- physical robot commands issued: false.

## Remaining physical gate

The current v12 maps are useful only as historical visualization evidence.
The next onsite run must create one new persistent session and fresh maps.
That run is the first opportunity to observe all of the following together:

1. interval-only semantic refreshes do not increase semantic-vote counters;
2. displayed/tracked pose is calibrated `base_link`, not camera fallback;
3. snapshot/input/map-content ages advance only with real observations;
4. chair/plant masks remain consistent from distinct moved views;
5. pixel accuracy is assessed against deliberately placed, photographed
   objects rather than map appearance alone.

An already imported Foxglove layout will not update itself. Re-import
`hub/foxglove/dual_robot_dashboard.json` once to receive the revised panel
sizes; the topic names and production URL remain unchanged.
