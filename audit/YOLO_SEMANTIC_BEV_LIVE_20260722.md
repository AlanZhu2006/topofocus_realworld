# Goal-scoped YOLO semantic BEV live gate — 2026-07-22

## Outcome and safety scope

The existing HPC YOLOv10 detector is now connected to the real Hub semantic
BEV through aligned depth. Fresh WSJ and Yunji/Odin maps both produced
persistent `chair` cells from their current live images. The production
Foxglove relay was switched to those fresh directories on its unchanged
8765/8766 ports; the already-open remote client reconnected.

The first operator check exposed a display-specific gap: WSJ's 10 raw chair
cells became only two 15 cm output cells after the relay's factor-3 evidence
downsampling, effectively invisible at a 26 m full-map zoom. The relay now
copies every original 5 cm semantic cell as a shallow palette-colored block on
the already-visible per-robot `map_pose` topic. Chair is red, and one small
`chair` label is anchored over its largest connected component. There is no
enclosing box, so the visible footprint is the actual BEV evidence rather than
a display approximation. This changes presentation only; the Grid evidence
remains unchanged and the existing Foxglove layout needs no new topic.

This work sent no planner, velocity, Unitree sport/motion, Yunji chassis or
actuator command. Both daemons still use `--no-cascade`; the Hub decision path
remains HOLD-only. `source/` and `dependencies/` were not modified.

## Upstream provenance versus deployment extension

The detector is not a substitute model introduced by the Hub:

- upstream `source/Focus_realworld/arguments.py` defaults to
  `--yolo yolov10 --yolo_weights detect/yolov10m`;
- upstream `source/Focus_realworld/main.py` loads
  `YOLO(args.yolo_weights)` and calls it with `conf=0.2`;
- the local transferred model is 33,643,667 B with SHA-256
  `6dc78f7a88591cec1e8716b8f5c7e3aefa9206684f025d202be34439ccb329a0`.

What is new is the real-robot deployment connection. Upstream sends YOLO's
class/confidence dictionary to the Perception VLM but only RedNet contributes
semantic-map pixels. `hub/src/focus_hub/semantic_yolo.py` retains boxes, uses
the median of the box's central 40% as an aligned-depth anchor, writes only the
current goal category within a symmetric depth cluster into the RedNet label
image, and leaves the existing world/height projection to `CentralMapper`.

The default live gate is intentionally chair-only. On the same WSJ image YOLO
also emitted `tv` around 0.45–0.51 for the equipment/board region; it was
recorded but not fused because it was not the navigation goal. Sink is not
supported by this reinforcement because upstream assigns MP3D id 16 to both
sink and stairs. These restrictions prevent unrelated model guesses from
becoming persistent BEV evidence.

## Why chair disappeared before

The RedNet domain-gap audit found zero chair predictions over 303 real WSJ
frames even though visually inspected samples contained prominent chairs.
The previous table and plant map cells were sparse RedNet outputs, not proof
that the real-time chair pipeline was healthy; the plant overlay did not
reliably follow the planter silhouette. The current RedNet-only daemons also
ran with `--no-cascade`, so YOLO was not loaded at all. Even enabling the old
cascade would only have supplied VLM text and would not have populated BEV
semantic channels.

## Initial live observed gate

Checkpoint copies are preserved under
`hub/runtime/semantic_yolo_gate_20260722_2129/` (runtime, Git-ignored). All
semantic results below are model-derived and unverified against ground-truth
labels; the source images were visually inspected and do contain chairs.

| Robot | Fresh start-after | Frames | YOLO evidence frames | Failures | Chair cells | Obstacle / explored |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| WSJ | 14411 | 29 | 29 | 0 | 9 | 1,660 / 5,105 |
| Yunji/Odin | 162817 | 37 | 37 | 0 | 131 | 323 / 3,566 |

The WSJ checkpoint's last chair was confidence 0.444 at sequence 14462; the
Yunji checkpoint's was 0.906 at sequence 163018. Both used confidence >=0.35,
the box's 10th-percentile aligned depth plus a 0.45 m foreground margin, and
the mapper's independent 0.25–1.50 m semantic height band. The two maps had
the same `shared_world` frame and `shared-board-odin1-20260722-v1`, so fused
publication remained contract-valid.

That initial method established that the upstream detector could reach the
real BEV, but it was superseded after the two robot views produced visibly
different chair locations.

## Cross-view depth correction and current v2 gate

Visual inspection confirmed that WSJ and Yunji/Odin were viewing the same
white chair. WSJ's chair box also contained a nearer vertical pole. The
initial 10th-percentile/foreground rule selected that pole, placing WSJ's
chair around `(-1.13, 0.80)` m while Odin placed it around
`(-1.55, 1.59)` m: approximately 0.9 m apart. The independently moved-board
calibration holdout residual was only 0.0114777 m, so calibration error was
not large enough to explain the discrepancy.

An observed eight-frame-per-robot offline sweep compared depth anchors without
changing either robot. Median depth from the central 40% of the detection box
with a symmetric ±0.45 m interval gave the smallest mean paired projection
gap, 0.048 m. This rejects the small nearer pole and the farther wall visible
through an open chair. It is still depth-grounded box evidence rather than a
true image segmentation mask.

Fresh v2 maps started after WSJ sequence 14783 and Odin sequence 164264:

- `runtime/map_out_wsj_yolo_depthcluster_v2_20260722`;
- `runtime/map_out_yunji_yolo_depthcluster_v2_20260722`.

The preserved 21:55 gate under
`hub/runtime/semantic_yolo_depthcluster_gate_20260722_2155/` contained 37 WSJ
frames/187 chair cells and 45 Odin frames/61 chair cells, with zero YOLO
failures and no mapping block. Their chair-cell centroids were
`(-1.4540, 1.8245)` m and `(-1.5267, 1.7932)` m respectively: 0.0791 m apart,
with overlapping XY extents. This is an observed cross-view consistency check,
not a surveyed object-position accuracy measurement.

| Artifact | Size (B) | SHA-256 | Status |
| --- | ---: | --- | --- |
| v2 WSJ `central_map.npz` | 23,248 | `8d0fe3a0cdcadafbb98e5c5c9b45d8a7706e4c88fdd8f9965b51cb572f863042` | model/source-derived checkpoint |
| v2 WSJ `map_summary.json` | 4,023 | `f80a8fae50a5d516295a18845438c56c6309c74a2d78c604ee37f785cbb8405c` | observed runtime checkpoint |
| WSJ sequence 14849 metadata | 2,196 | `ff0ac918edc6b7e89f1bb146103ef088ae86f6c16596ad0ff1fc411a40a8bde0` | observed spool input |
| WSJ sequence 14849 RGB | 92,419 | `88c7aa2ccdb949f21c53ac5e413114b7e5de7e7b5549a71af7539aebe8e62dc3` | observed spool input |
| WSJ sequence 14849 depth | 216,914 | `1275773e097a2b01421efd3536c1a90aa8eeee62009ec6f069c0ff738e460457` | observed spool input |
| v2 Odin `central_map.npz` | 22,344 | `08f01018852f8ca177f562b10183f5a3cdc5b1271850170b209370f4654a2dbe` | model/source-derived checkpoint |
| v2 Odin `map_summary.json` | 3,716 | `f8cd70da5867338c79293e0ba0d37115d8f6122c5c1aae43d0719ba1bb3d6325` | observed runtime checkpoint |
| Odin sequence 164515 metadata | 2,882 | `8bce1bb67e3f35c2571798f11e5616bbc1dcb68b574e34f63492cd69dc24f391` | observed spool input |
| Odin sequence 164515 RGB | 153,778 | `16d3096f005093bad96b678ae65dba8170529fb0096f40317351672ca0d30cd3` | observed spool input |
| Odin sequence 164515 depth | 226,784 | `8902e9c668a57b67624f30de816e9f7f74205daa7dff16f902da70585574ba84` | observed spool input |

## WSJ geometry finding

The old accumulated WSJ map had reached roughly 37% obstacle/explored cells.
A bounded replay of the same first 60 accepted frames measured 29.73% with the
0.15 m lower collision band and 27.10% with the source-default 0.25 m floor
clearance. The fresh WSJ daemon therefore uses 0.25–0.75 m; the old directory
was preserved instead of attempting to erase max-persistent semantic history.

The operator made the Go2 lie down during the preceding session. The observed
camera-Z span was 0.218 m, while per-frame fitted floor height stayed within
the drift gate; this is consistent with the pose stream tracking a real body
height change rather than the floor moving. The fresh map began after the
robot was down.

A current depth classification overlay contained 365,876 valid returns:
70,077 in the new 0.25–0.75 m collision band and 15,903 in the excluded
0.15–0.25 m slice. Visual inspection showed the carpet itself was not painted
as an obstacle; accepted pixels corresponded mainly to the back wall,
equipment, chairs and robot geometry. The remaining fan/ray appearance is
therefore expected from a stationary single depth viewpoint in this cluttered
scene, not evidence by itself of floor misclassification. A surveyed floor
plan is still unavailable, so this is not an occupancy-accuracy score.

## Exact checkpoint provenance

| Artifact | Size (B) | SHA-256 | Status |
| --- | ---: | --- | --- |
| WSJ checkpoint `map_summary.json` | 4,138 | `36e252dfa093c2b5140f850b11862ce2de64302e78b8c75c8c5b8249fd88a67f` | observed runtime snapshot |
| WSJ checkpoint `central_map.npz` | 23,105 | `1815d16ef95b3e323f03dd436f4e5e4acc8acf64392c1ba812d69e60caebce1b` | model/source-derived map |
| WSJ source sequence 14462 RGB | 92,228 | `7d9075b8359423566ea0768b86f56f4bd092178df2f25235fa5dea3c14ab5d93` | observed spool input |
| WSJ source sequence 14462 depth | 218,994 | `01d548206f150f2e33bed103d8e162de48b25ebaa20ee68b8830e2493ad7590d` | observed spool input |
| Yunji checkpoint `map_summary.json` | 3,848 | `4b263b588c6b60ffe38d06af1d67c002ced0834f4c9a4f0ca6b912f211165787` | observed runtime snapshot |
| Yunji checkpoint `central_map.npz` | 22,382 | `11d4c0c59b288ee49be3fbc21b1d55fa7ca6f7801845922ee1866aa14a5bc8a6` | model/source-derived map |
| Yunji source sequence 163018 RGB | 153,668 | `de464600efa49f507ac191036831b7a3bf6f15111f400da88870942c4b55688d` | observed spool input |
| Yunji source sequence 163018 depth | 226,964 | `25ad41e7b440dbaba8148ce0503b80ad83000b4aef679814b0f767ccf2010127` | observed spool input |

The full Hub test suite passed after the change (203 tests). Runtime daemons
and the relay remained active with `mapping_blocked_reason=null`, zero YOLO
failures, and a live remote WebSocket connection at the recorded checkpoint.

## Limits and next physical gate

YOLO boxes plus foreground depth are not instance masks, and repeated model
agreement is not labelled accuracy. Chair is the only reinforced goal in this
live run. Table or plant should be enabled only for a dedicated goal trial
with the object visibly placed and its resulting BEV location checked. A
complete room map still requires operator-present movement; a stationary
camera can only produce a fan-shaped observed sector.
