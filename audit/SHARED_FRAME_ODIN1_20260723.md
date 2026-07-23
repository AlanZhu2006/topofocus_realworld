# Fresh WSJ/Odin1 shared-frame calibration — 2026-07-23

## Outcome

A fresh, session-bound gravity-preserving shared-frame calibration was
generated for the current WSJ TinyNav and Yunji Odin1 odometry epochs.

- Reference robot: `robot-0` / WSJ
- Other robot: `robot-1` / Yunji
- WSJ transform version: `wsj-tinynav-depth-20260723-calib-v1`
- Yunji raw transform version: `yunji-odin1-raw-20260723-v1`
- Yunji calibrated transform version: `yunji-odin1-board-20260723-v1`
- Shared calibration ID: `shared-board-odin1-20260723-v1`
- Artifact:
  `hub/runtime/calibration/yunji_odin1_board_20260723_v1.json`
- Artifact size/SHA-256:
  `5521` bytes /
  `53565b6ebde459141f072def3454a6200f250d0ff90bb8a0a7362c9a087c08cf`

The 7×10 symmetric circle grid used 40 mm center spacing. Both robots remained
stationary; the operator moved only the board between the fit and independent
holdout captures. No planner, command receiver, velocity bridge, WATER move
endpoint, or robot motion command was used.

## Observed calibration and holdout

| Role | WSJ sequence | Yunji sequence | Capture skew | Center residual | Normal residual | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| fit | 15198 | 166801 | 51.207 ms | 0 m by construction | 0.364158° | pass |
| moved-board holdout | 15292 | 167149 | 127.958 ms | 0.002817 m | 0.368622° | pass |

The gates were 250 ms maximum capture skew, 0.05 m maximum holdout center
residual, and 3° maximum holdout normal residual. The source-derived transform
has:

- translation `[-0.914588023, 1.286240419, -0.035169698]` m;
- yaw `105.378325839°`;
- gravity tilt `0°`;
- rotation determinant `1.0`;
- maximum rotation orthogonality error `0.0`.

The fit/holdout images and poses are observed robot data. Circle-grid PnP,
the yaw-only rigid alignment, and the residuals are source-derived results.

## Input provenance

All paths below are local Hub spool paths. The calibration JSON also embeds
their absolute paths, sizes, hashes, sequence numbers, capture times, and
transform versions.

| Role | File | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| fit WSJ | `hub/runtime/spool/robot-0/00000000000000015198/metadata.json` | 2205 | `a7c63da5c360f90c1846121ba948b5d47efb6c1bd95706a7e9e5d046b39164f8` |
| fit WSJ | `hub/runtime/spool/robot-0/00000000000000015198/rgb.jpg` | 91468 | `a198065d39fbc027176c9eda9e5de2dfbf0274dfec92dba561bd7451cc3a60bf` |
| fit Yunji | `hub/runtime/spool/robot-1/00000000000000166801/metadata.json` | 2872 | `dabbd401267b85e5aa2c73b54f74159a8ccca6b45d161b59eb4eeab9d9d6a017` |
| fit Yunji | `hub/runtime/spool/robot-1/00000000000000166801/rgb.jpg` | 145718 | `a957167ede3143843a7dbc23958c2ece80f37c7903f6d848f9932ed715277967` |
| holdout WSJ | `hub/runtime/spool/robot-0/00000000000000015292/metadata.json` | 2203 | `bd5bb47f16c3cf89d8a63a04c9fdba85e1a9fc40b37f9cf7d2c280e5e072c765` |
| holdout WSJ | `hub/runtime/spool/robot-0/00000000000000015292/rgb.jpg` | 92142 | `aff2dad599a819877d970dae2e3458b0708748d42c25b3e894b3222dba4480a4` |
| holdout Yunji | `hub/runtime/spool/robot-1/00000000000000167149/metadata.json` | 2868 | `001d729dd9d455c90fa0947a797d4fe072db936e3c83ce73b815d1f749e2cabb` |
| holdout Yunji | `hub/runtime/spool/robot-1/00000000000000167149/rgb.jpg` | 147483 | `ee4da6b98ca66efe6e2fd37e39c721c049d893778f7cd03c939f03272b8138d1` |

## Deployment and independent online check

The artifact was copied without modification to:

`/home/nyu/topofocus_realworld/runtime/calibration/yunji_odin1_board_20260723_v1.json`

The remote size and SHA-256 matched the local artifact. The raw transient
service was stopped and replaced with the observation-only service:

`focus-yunji-mapping-calibrated-20260723-v1.service`

The sender logged
`loaded shared-frame calibration shared-board-odin1-20260723-v1`, resumed at
sequence `167343`, and the Hub accepted new observations carrying transform
version `yunji-odin1-board-20260723-v1`.

An independent post-deployment synchronized pair proved that the transform was
actually applied by the live sender:

| WSJ sequence | Yunji sequence | Capture skew | Shared board center residual | Board normal residual |
| ---: | ---: | ---: | ---: | ---: |
| 15354 | 167415 | 115.362 ms | 0.003534 m | 0.312473° |

The online validation sources were:

| Robot | File | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| WSJ | `hub/runtime/spool/robot-0/00000000000000015354/metadata.json` | 2203 | `91b5f93e202fc06df8b9fa5974ff9a3453d56d26e3a5736e0c2e312b428dfd4f` |
| WSJ | `hub/runtime/spool/robot-0/00000000000000015354/rgb.jpg` | 91730 | `02c77efd02870784a390ddeedbcafcac0a3fc6d981922c2bb7ff8157cefe58a5` |
| Yunji | `hub/runtime/spool/robot-1/00000000000000167415/metadata.json` | 2875 | `9054bd17b5828e24d184ef9aca322f5daf737ecf3f4b5a9458d4e7ea01bed4fc` |
| Yunji | `hub/runtime/spool/robot-1/00000000000000167415/rgb.jpg` | 147647 | `c87dada1051e8321b8749ae9b14a14fc1e8eb3b77242f719f235beae64d1aca1` |

## Deployment fixes made during the run

`hub/robot_overlay/start_wsj_mapping_session.sh` now runs
`focus_ros_sender.py` with `TINYNAV_PYTHON`, because the synchronized sender's
Pydantic dependency is installed in the TinyNav virtual environment rather
than WSJ's system Python. The local and deployed WSJ copy is `5313` bytes with
SHA-256
`bf92651d3acd6d5cab5a702abd3a4304669491d4e58c487616614c69d80e30cf`.

`hub/robot_overlay/run_yunji_mapping_observation.sh` now accepts one explicit
`--shared-frame-transform-file`. It still removes inherited transform state,
and `odin1_sender.py` rejects a file whose transform version or calibration ID
does not match. The local and deployed Yunji copy is `3204` bytes with SHA-256
`c09e0ef26344db9d5c0502afbd5f4657079d37564297d321dc31fcc8bc02f916`.

Both scripts passed `bash -n`; the Odin sender/calibration test module passed
all 15 tests.

## Fresh maps and Foxglove switch

The operator removed the board before the new map boundary. Automatic
circle-grid detection was negative in both boundary-check images, and visual
inspection confirmed that the board was absent.

Fresh map daemons started after these exact sequence boundaries:

- WSJ: `15464`;
- Yunji: `167858`.

They write only to:

- `hub/runtime/map_out_wsj_20260723_v1`;
- `hub/runtime/map_out_yunji_20260723_v1`.

Both live status files reported `mapping_blocked_reason=null`, the expected
per-robot transform version, the common calibration ID
`shared-board-odin1-20260723-v1`, and zero semantic-YOLO failures. The strict
fusion loader observed `frame_id=shared_world`, resolution `0.05 m`, matching
calibration IDs, and map tensors shaped `[17, 520, 520]`.

At `2026-07-23T15:11:24+08:00`, the mutable live artifacts had:

| Robot/file | Bytes | SHA-256 | Observed state |
| --- | ---: | --- | --- |
| WSJ `central_map.npz` | 22788 | `38b6dcfaead79e74ae0acdd146c9c1dcfff6032395e90f1e60f22a785984d4c6` | 28 integrated / 48 observed |
| WSJ `live_status.json` | 1921 | `86df79d51487c0ce0dca90b62743817ea0de2b26654d3dc7f815e30920ce15ca` | unblocked |
| Yunji `central_map.npz` | 22719 | `d95db4af23f54a524b2accc086078273dd1846152bc514e9e0cbd72d2d33f5da` | 35 integrated / 198 observed |
| Yunji `live_status.json` | 2100 | `c020f97f34b209aea3db7a95cb742015e71664cb73b5f801a8143a75bb3faae0` | unblocked |

These hashes identify one observed instant; the files are intentionally
mutable while mapping continues.

The old July 22 relay was stopped and replaced on the unchanged
`8765`/`8766` ports by `foxglove_relay_20260723_v1`, reading only the two new
directories above. A real Foxglove WebSocket subscription received both
per-robot camera, geometry map, semantic map and status channels plus
`/fused/geometry_map`, `/fused/semantic_map` and `/fused/status`. The existing
external client reconnected on the same address.

`hub/scripts/start_fresh_dual_maps.sh` was corrected to default to the current
observation Hub at loopback `127.0.0.1:8188`, accept only an explicit
loopback-only override, and passed syntax/help/negative-URL checks. Its size is
`5160` bytes and SHA-256 is
`864a40e0052086099309f77b8c0c9f9ed23bef7dfb483800b0c6df108228be46`.

The remaining operational step is scene coverage: the robots were stationary
during startup, so the fresh map initially covers only their current views.
Map expansion now requires an operator-supervised mapping walk; it does not
require another shared-frame calibration unless a camera mount or odometry
epoch changes.
