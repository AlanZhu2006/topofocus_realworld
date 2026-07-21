# Live Foxglove/map recovery — 2026-07-22

## Scope and safety

This pass diagnosed and replaced only Hub-side mapping and visualization
processes. It sent no planner, velocity, Unitree sport/motion, or other robot
command. Immediately before the runtime switch, Hub `/healthz` reported
`goal_output_enabled=false` for both `robot-0` and `robot-1`.

`source/` and `dependencies/` were not modified. New deployment logic is under
`hub/`. Existing map outputs were retained; the repaired daemons use new output
directories.

## Blank-panel diagnosis

Two independent conditions had looked like one dashboard failure:

1. `/yunji/camera` was not blank at the relay. A direct Foxglove subscription
   received valid 640x480 JPEG messages. The blank Image panel was a client
   subscription/render state after WebSocket reconnects. The relay now retains
   the latest compressed message and republishes it with its original timestamp
   every two seconds. This gives a late/reconnected panel an image without
   pretending that a retained image is fresh.
2. The old Yunji map really was initialized in the wrong place. Sequence 26028
   had camera XY `(3.9440, 17.2719)`; sequence 26029 moved to
   `(-0.3770, 0.3744)`, a 17.441231 m discontinuity after 757.166071 s while
   keeping the same transform-version label. Since the old map fixed its 26 m
   extent from the first observation, almost all later observations landed
   outside it.

The new startup gate requires three temporally and spatially continuous poses
before choosing the map extent. Replaying the exact failure rejects sequence
26028 and initializes from 26029–26031 at median XYZ
`[-0.377004, 0.374397, 0.435]`.

## Black-ray diagnosis and repair

The black cells are obstacle decisions, not unknown cells. The previous
free-space ray fill only changed channel 1 (explored); channel 0 used one-hit,
irreversible maximum fusion. On WSJ sequence 5527, one frame alone produced
1,809 obstacle cells out of 4,224 explored cells (42.83%). After 30 accepted
frames, legacy maximum fusion reached 2,696 / 4,776 (56.45%).

Three fixes are now used together by the live daemon:

- pose/keyframe gating: integrate after 0.20 m translation, 10 degrees
  rotation, or 5 s, instead of every near-duplicate observation;
- reversible obstacle evidence: one free or occupied update per XY cell per
  accepted frame, log-odds updates `-0.40/+0.85`, two occupied keyframes before
  display, and later free rays can clear a cell;
- three-frame deterministic RANSAC ground consensus and a planner collision
  band of 0.15–0.75 m, source-derived from the preserved TinyNav occupancy
  implementation. Semantic evidence retains its separate 0.25–1.50 m band.

The ground correction was material. The old WSJ daemon assumed a fixed 0.4 m
camera height and used floor `z=1.0696`. The first live depth frames consistently
fit a near-horizontal ground plane at about `z=1.2721`, implying a current
camera height near 0.197 m. With reversible evidence but the wrong floor, the
30-frame obstacle/explored ratio was still 44.43%; using the RANSAC ground and
0.15–0.75 m band reduced it to 29.42%. The archived native BuildMap stationary
snapshot is about 27.1%, so the repaired result is in the same observed range.

This does not promise a clean room outline while the robot is stationary. Both
the native BuildMap stationary artifact and the Hub map remain fan-shaped from
one viewpoint, with some thin endpoint structures. Overlapping observations
from a controlled moved path are still required for a room-scale map; no such
motion was requested or performed in this pass.

## Runtime switch and observed result

The old map daemons were stopped gracefully after their final save. The repaired
daemons run in tmux session `live_map_v2_20260722`, starting strictly after:

- WSJ sequence 6217, output
  `hub/runtime/map_out_wsj_live_v2_20260722`;
- Yunji sequence 44562, output
  `hub/runtime/map_out_yunji_live_v2_20260722`.

Initial live snapshots:

| Robot | Stable startup sequences | RANSAC floor z | Obstacle | Explored | Ratio | Pose jumps |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| WSJ | 6218–6220 | 1.270823 m | 1,177 | 4,493 | 26.20% | 0 |
| Yunji | 44563–44565 | -0.045785 m | 898 | 4,086 | 21.98% | 0 |

Later checkpoints remained bounded: WSJ 1,394 / 4,792 (29.09%) after 16
integrated keyframes; Yunji 951 / 4,239 (22.44%) after 32 integrated keyframes.
Yunji skipped 297 of 329 near-duplicate observations at that checkpoint.

The port-8765 relay now reads these directories and does not use `--fuse`.
A real 8 s protocol subscription observed:

| Topic | Messages | Payload bytes |
| --- | ---: | ---: |
| `/wsj/camera` | 43 | 2,787,134 |
| `/yunji/camera` | 73 | 5,749,780 |
| `/wsj/semantic_map` | 2 | 239,744 |
| `/yunji/semantic_map` | 2 | 239,744 |

The existing remote Foxglove client reconnected successfully. The revised
layout shows two independent maps and removes the unverified fused panel.

## Frame/fusion contract

New incremental snapshots record `frame_id`, `transform_version`, optional
`shared_frame_calibration_id`, floor source, obstacle policy, and a map format
version. The relay rejects missing frame metadata by default. Cross-robot
fusion additionally requires the same non-empty
`shared_frame_calibration_id`; a common string `shared_world` alone is not
treated as evidence of calibration.

Both current live maps deliberately have
`shared_frame_calibration_id=null`. The WSJ v3 session has not been physically
recalibrated against the current Yunji session, so fused visualization remains
disabled.

## Remaining semantic limitation

All 15 semantic channels in both new live snapshots are currently zero. The
dashboard topics retain the historical `/semantic_map` name, but these current
snapshots should be interpreted as geometry/exploration maps. The previously
documented RedNet real-camera domain gap remains unresolved; no semantic colors
were invented to hide it.

## Verification

- focused pose, ground, occupancy, snapshot and pipeline tests: passed;
- complete Hub suite: **154 passed**;
- Python compile/import help gates: passed;
- dashboard JSON parse: passed;
- live Hub health: both goal outputs disabled;
- live map summaries: no pose jump latch;
- live Foxglove protocol: both cameras and both maps delivered messages.

## Provenance

Implementation provenance is embedded in the new modules' docstrings. The
pose/keyframe and ground policies are source-derived from the immutable
experimental snapshot under
`hub/robot_overlay/tinynav_snapshot/working-tree-files/semantic_mapping/`.
Runtime counts, coordinates, message counts, and ratios above are observed.
The claim that controlled motion will form a useful room map remains
unverified until an operator-present moved gate is run.

| Artifact | Size (bytes) | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/pose_gate.py` | 8,871 | `ef4135b2f75bd153cc05b92ca73fa6119fd65d178571123dd3fb298113b6b073` | implemented, tested, live-used |
| `hub/src/focus_hub/ground_plane.py` | 9,992 | `b8158eb1e80b80173f114d7a22ad4313554dc4e33c546f4f3f05580d5d95c0b3` | implemented, tested, live-used |
| `hub/src/focus_hub/map_snapshot.py` | 5,014 | `548bbbc8cb679b2b106617cb26bfbe42c25a8d7d6aaf4346f12b26307f4b5aba` | implemented, tested, live-used |
| `hub/src/focus_hub/central_mapping.py` | 17,323 | `78fd5ae798b24a6eb14fdd2543e78bcf8ef2119199f60e1057124da2d2c8b4b3` | modified, tested, live-used |
| `hub/src/focus_hub/pipeline.py` | 11,718 | `33a69e6fed5748fda1d9b17e49a5fddb0ca2cc405dccd33cd5b44cc5cf2647c4` | modified, tested, live-used |
| `hub/tools/hub_pipeline_daemon.py` | 28,879 | `dc920df921987d024eec3eb1033af7ba372202fddbe0c9362fc037b20c507e5d` | modified, tested, live-used |
| `hub/tools/foxglove_relay.py` | 25,745 | `5f1769e65c0d72a54c5784e634674ed04fa05f618efaf5518f4f4d20beee4587` | modified, tested, live-used |
| `hub/foxglove/dual_robot_dashboard.json` | 4,130 | `0789a7b33d8e27717cb1dc4d96bee0706ee89c63cde67cb0befe3b8a8a46fccd` | modified, parse-tested; re-import required for new layout |
| archived `keyframe_selector.py` | 5,530 | `ba5158ff035a5580f46878d62bd60941c8cc0b9572648ef2527e3db253a04806` | immutable source-derived input |
| archived `ground_estimator.py` | 16,863 | `ec4b7209bc1cf089d411af2c59376ba9d5425072ea6308390f42709c6dbd72b8` | immutable source-derived input |
| archived `occupancy_voxel_map.py` | 14,248 | `858090fad7e2da4cbcb2c974730cdfcb5dcb16537c9c7aba14a059d286b6a72c` | immutable source-derived input |
| archived `bev_projector.py` | 7,875 | `0e0d8af9daf47326c3d93bafd984c9c2263293daac717765ff9f0d284b2351b0` | immutable source-derived input |
