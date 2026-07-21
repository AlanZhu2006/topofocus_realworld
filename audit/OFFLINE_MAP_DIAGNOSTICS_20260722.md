# Offline map diagnostics, visualization contract and moved-run gate — 2026-07-22

## Scope and safety

This pass used only Hub-side code, append-only spooled observations and copied
map snapshots. It sent no planner, velocity, Unitree sport/motion or Yunji
chassis command and did not change `allow_goal`. `source/` and `dependencies/`
remain immutable.

Results below are labelled **observed** when produced from current real spool
data, **source-derived** when implementing an existing mapper/calibration
contract, and **unverified** when an operator-present physical gate is still
required.

## Foxglove color diagnosis and repair

The relay previously assigned RGBA colors at full resolution and then block-
averaged those bytes for network downsampling. That operation can create a
purple/yellow intermediate which is neither endpoint category. It also mixed
geometry and semantic interpretation in one visible topic.

The new path max-reduces evidence channels first and assigns one exact color
afterwards. It publishes `/<name>/geometry_map` separately and makes it the
dashboard default: gray unknown, white observed free, black obstacle. The old
`/<name>/semantic_map` remains available but hidden by default. A SceneUpdate
topic adds a red current camera XY and blue relay-lifetime camera trail; no
heading/body footprint is fabricated from the camera-only contract. Status
logs now include obstacle/explored counts, semantic-cell count and a repeated
legend.

Direct inspection of the generated WSJ/Yunji geometry images confirmed only
the exact gray/white/black palette. The fan shape remains visible, as expected
from the stationary single-view geometry; the visualization fix does not hide
it.

The new relay was first started on loopback-only ports 8767/8768. A real
Foxglove WebSocket subscription received both geometry maps, both semantic
maps, both pose scenes, both status logs and the legend in eight seconds. The
production relay was then switched on 8765/8766. A second eight-second
subscription observed 43 WSJ camera messages, 72 Yunji camera messages, one
geometry and one semantic map per robot, four pose/status messages per robot,
and `/tf`. The 30-second legend channel was advertised but did not fall inside
that second short window; it had delivered in the candidate test. Relay health
returned both robots and the existing remote Foxglove client reconnected from
`10.208.98.223`. This switch restarted no map daemon and sent no robot command.

## Real bounded geometry sweep

Tool: `hub/tools/analyze_live_map_sweep.py`. It reused the live stable-start,
three-frame RANSAC-ground and keyframe policies, used the same accepted depth/
pose frames for every profile, and deliberately zeroed semantics.

| Robot | Source observations | Accepted keyframes | Accepted XY path | Ground z |
| --- | ---: | ---: | ---: | ---: |
| WSJ | 45, sequences 6218–6262 | 30 | 0.005826 m | 1.270823 m |
| Yunji | 302, sequences 44563–44864 | 30 | 0.000776 m | -0.045785 m |

The tiny path lengths prove these are stationary stability comparisons, not a
moved-room-map validation.

| Robot/profile | Obstacle | Explored | Obstacle/explored | Thin obstacle cells |
| --- | ---: | ---: | ---: | ---: |
| WSJ live default | 1,399 | 4,816 | 29.05% | 37 |
| WSJ three-hit persistence | 1,376 | 4,816 | 28.57% | 37 |
| WSJ upper band 0.60 m | 1,281 | 4,816 | 26.60% | 45 |
| WSJ legacy irreversible max | 1,792 | 4,816 | 37.21% | 27 |
| Yunji live default | 948 | 4,237 | 22.37% | 26 |
| Yunji three-hit persistence | 947 | 4,237 | 22.35% | 26 |
| Yunji upper band 0.60 m | 814 | 4,237 | 19.21% | 25 |
| Yunji legacy irreversible max | 1,236 | 4,237 | 29.17% | 36 |

**Observed conclusion:** reversible live fusion is materially less saturated
than legacy max on both current real streams. Three vs. two hits changes
little in these stationary windows. A 0.60 m top band lowers density further,
but no surveyed map exists to establish that it is more accurate, so the live
0.75 m band was not changed.

Input provenance is embedded per observation in each JSON. Aggregate source-
manifest hashes are
`88ca5c9c89206c7519618997bfabddde8c93a19d6ac4273350de2223969012ed`
(WSJ) and
`b1c9ef43e36e82c489611fb0f30d5f9951366a9728dc2d8b838aa90b85421bde`
(Yunji).

## Live-spool RedNet confidence diagnosis

The existing domain-gap tool now also accepts a bounded Hub spool range and
reports raw argmax separately from the exact production output after RedNet's
source-derived fixed 0.8 confidence replacement. A one-frame regression check
confirmed the diagnostic's thresholded array is byte-identical to
`RedNetSegmenter.segment()`.

| Robot/range | Valid depth | Confidence >= 0.8 | Target-15 raw argmax | Target-15 production |
| --- | ---: | ---: | ---: | ---: |
| WSJ 6503–6512 | 89.82% | 15.29% | 2.7762% | 0.000295% |
| Yunji 48122–48131 | 78.68% | 14.63% | 6.0733% | 0% |

The WSJ sample RGB visibly contains chairs/furniture. The raw visualization
segments broad regions, while the production-thresholded output collapses to
coarse dominant classes without a reliable target-object region. This
supports a combined real-camera domain-gap/confidence-threshold diagnosis, not
a broken depth-input diagnosis.

No ground-truth labels exist, so lowering 0.8 is not authorized by these
statistics. Raw low-confidence colors must not be presented as valid semantic
detections. Geometry remains useful independently.

At the post-deployment live checkpoint, the accumulated WSJ snapshot still
had 0 semantic cells and Yunji had only 10, versus 5,076/4,676 explored cells.
That sparse evidence does not change the decision to hide semantics by default.

## Moved-run gate

`hub/tools/validate_moved_map_run.py` compares copied before/after map
directories and matching spool metadata. It fails closed on frame/transform/
calibration/extent changes, pose jumps, too little accepted motion, too few
new keyframes, no/newly-insufficient coverage, or excessive obstacle density.
It records every input file hash and never talks to a robot.

A synthetic 0.8 m test trajectory passed with 4 accepted keyframes, 30 newly
explored cells, 3 new obstacles, 5 cleared obstacles and no failed checks.
That validates the tool mechanics only. The real operator-present moved gate
is **unverified** and remains required.

## Existing calibration path reused

No new calibration math was introduced. The existing
`calibrate_camera_offset_via_board.py` -> `calibrate_shared_frame.py` flow is
retained. The latter now adds `shared_frame_calibration_id` and provenance
hashes for the two selected observations and optional board-offset file while
preserving the sender-consumed matrix field. A synthetic synchronized CLI run
confirmed the extended output remains loadable.

Only the scripts are reusable. The July 21 numeric board matrix does not apply
automatically after the WSJ v3/Yunji sender/session changes. Fusion remains
off until a fresh capture and independent common-landmark residual check.

## Result artifacts

Runtime analysis output is intentionally Git-ignored; these exact observed
artifacts remain on the current Hub host:

| Artifact | Size | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/runtime/analysis/live_map_sweep_wsj_20260722/map_parameter_sweep.json` | 78,895 B | `80c9fd792b59e0d3845d7669c86327a19cbb84e3727e53adb489a83d0005ab42` | observed |
| `hub/runtime/analysis/live_map_sweep_yunji_20260722/map_parameter_sweep.json` | 382,335 B | `0250d1863f7354b626ceef7a9b9b1e33dcf2f18833f93890663db2d90ff02ca3` | observed |
| `hub/runtime/analysis/rednet_spool_wsj_20260722_v3/domain_gap_summary.json` | 29,488 B | `0e7980c37921f6e59e1ae65849ae85c5509244706b0ef7cb33279831850a5536` | observed |
| `hub/runtime/analysis/rednet_spool_yunji_20260722_v3/domain_gap_summary.json` | 29,608 B | `b1649e922b89772eb0ff79ef3c79d05604f14277e7af0537beb8aa183ee1b920` | observed |

Each RedNet JSON additionally hashes all 32 generated sample images and its
checkpoint/source observations. The full operating procedure is
`hub/docs/OFFLINE_MAP_VALIDATION.md`.

Tracked implementation provenance:

| Artifact | Size | SHA-256 | Status |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/map_visualization.py` | 3,942 B | `e83608a4fcca2dd5eabb91849883ead9dbc2d0d8b7b779013e5a443e55181dfe` | implemented, tested, live-used |
| `hub/src/focus_hub/map_quality.py` | 4,816 B | `d81f5e5267f74ac009f86f29bd20895949eb3ca158229d5a372397e08972ecd7` | implemented, tested |
| `hub/tools/analyze_live_map_sweep.py` | 16,117 B | `09b84e3cbc23fcfb5add795245464a33fd0d67cb232e70e2465873df4258d025` | implemented, real-data-used |
| `hub/tools/analyze_rednet_domain_gap.py` | 17,386 B | `3dd87afd2efb83d603ab08b5b741c05387024c6f4d767d09cf43748464a79c10` | modified, real-data-used |
| `hub/tools/validate_moved_map_run.py` | 14,359 B | `362ef7b2168ced874f8ec2047526feaa9d99138e8a79cf8bf94efde806436d14` | implemented, synthetic-tested; real gate pending |
| `hub/tools/calibrate_shared_frame.py` | 7,948 B | `510a841eef787f7db382b0d31a23bad9cb2b7f431684e533580e781f460d285b` | modified, synthetic-tested |
| `hub/tools/foxglove_relay.py` | 30,261 B | `592cbdfce0a0880f184dda1c749b147e827e13b85b27a2b77a1ef054cceca27c` | modified, tested, live-used |
| `hub/foxglove/dual_robot_dashboard.json` | 4,328 B | `b4d2f774de091fb80f32e4020497ca8a0b159ebc3860db2fd3a8d015c304ab1b` | modified, parse-tested; re-import required |
