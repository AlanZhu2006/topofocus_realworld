# Shared-frame v2 calibration, fusion, and chair observation — 2026-07-22

## Outcome

This report separates observed results, source-derived calculations, and open
claims:

- **Observed:** the session-specific board calibration
  `shared-board-20260722-153114-v2` passed an independent moved-board holdout.
- **Observed:** the read-only Foxglove relay publishes both per-robot maps and
  `/fused/geometry_map`, `/fused/semantic_map`, and `/fused/status` from two
  live maps carrying that exact calibration ID.
- **Observed/source-derived:** the chair placed by the operator is visible in
  both RGB streams and produces aligned geometry-obstacle evidence.
- **Observed:** neither map contains a `chair` semantic cell. The 27 fused
  semantic cells are `plant` output, so they are not accepted as correct chair
  recognition.
- **Unverified/not claimed:** distinct multi-robot decisions, autonomous
  navigation, and G5 hardware safety remain outside this result. G4 therefore
  remains open even though its calibrated read-only fusion portion now works.

No robot command was sent during calibration, validation, visualization, or
the chair check.

## Why v2 replaced v1

The v1 holdout failed after WSJ was physically moved: its board-center
residual reached 0.114398 m. Fusion stayed disabled. V2 was fitted from a new
synchronized pair after that motion and was not accepted merely from its
algebraic closure.

## Calibration fit

| Item | WSJ / robot-0 | Yunji / robot-1 |
|---|---:|---:|
| Fit observation sequence | 9025 | 145021 |
| Reprojection RMS | 0.131129 px | 0.194597 px |
| Board distance | 2.177233 m | 1.711797 m |
| Applied transform version | `wsj-tinynav-depth-20260722-session-v1` | `yunji-d455-shared-board-20260722-153114-v2` |

The fit pair had 0.082596842 s capture skew. Its source-derived camera-to-
camera translation norm is 1.281629 m. The exact fit artifact records board
geometry, camera intrinsics, source image/metadata paths, sizes, checksums, and
the derived SE(3) result.

## Independent moved-board holdout

The robots were held stationary and only the board/case was moved. The
holdout pair (WSJ 9214, Yunji 146524) was not used to fit v2.

| Check | Observed value | Gate | Result |
|---|---:|---:|---|
| Capture skew | 0.189751967 s | <= 0.25 s | pass |
| Board displacement from fit, WSJ | 0.249272 m | >= 0.20 m | pass |
| Board displacement from fit, Yunji | 0.252381 m | >= 0.20 m | pass |
| Shared board-center residual | 0.005234 m | <= 0.05 m | pass |
| Board-normal residual | 0.679948 deg | <= 3.0 deg | pass |
| WSJ camera pose change | 0.008356 m / 0.116829 deg | <= 0.05 m / 2 deg | pass |
| Yunji camera pose change | 0.003759 m / 0.004415 deg | <= 0.05 m / 2 deg | pass |

This validates the shared frame for the current pose/SLAM session. A SLAM
reset, relocalization discontinuity, camera-extrinsic change, or robot
repositioning outside the tracked pose contract requires a fresh map session
and renewed validation.

## Live maps and Foxglove fusion

Both incremental maps use frame `shared_world`, resolution 0.05 m, and
calibration ID `shared-board-20260722-153114-v2`. Their independent extents
are aligned into a `(17, 532, 543)` union grid with origin
`(-13.872576956743174, -13.644637135133426)`.

A raw `foxglove.sdk.v1` client subscribed to the running port-8765 relay. In a
7-second observed window it received 36 WSJ camera messages, 65 Yunji camera
messages, one map from each robot, one fused geometry map, one fused semantic
map, and one fused status message. The decoded status was:

```text
fused 2 robots: shape=(17, 532, 543), origin=(-13.872576956743174, -13.644637135133426), calibration=shared-board-20260722-153114-v2, explored=8794, obstacles=2175, semantic_evidence=27
```

The dashboard now makes `/fused/geometry_map` the large default panel, overlays
both camera trails, and leaves `/fused/semantic_map` hidden. An already-imported
Foxglove layout does not update automatically; re-import
`hub/foxglove/dual_robot_dashboard.json`.

## Chair observation

The operator reported placing a chair. Archived RGB observation 9388 shows it
at the left/front of WSJ, and observation 147892 shows it centrally from
Yunji. Manually selected image regions were projected with each archived depth
image, intrinsics, and calibrated camera pose. This ROI selection is an
assistant-observed input; the 3-D/grid statistics are source-derived.

| Derived check | WSJ | Yunji |
|---|---:|---:|
| Unique 5 cm obstacle-band endpoint cells | 578 | 197 |
| Endpoints already occupied in the map | 568 | 174 |
| Endpoints explored in the map | 578 | 197 |
| `chair` semantic cells in the ROI | 0 | 0 |
| `plant` semantic cells in the ROI | 0 | 0 |

The two projected regions share 91 calibrated 5 cm world cells. This is strong
geometry evidence from both views, but it is not an object-classification
metric. In the exact archived fused snapshot, `chair=0` and `plant=27`.
Therefore the correct result is **chair detected as geometry, not detected as
chair semantics**.

The immutable evidence bundle for this check is under
`hub/runtime/calibration/board_20260722_153114_cst_v2/chair_detection_seq9388_147892/`.
It contains copied RGB/depth/metadata, copied map snapshots, a source-derived
fused geometry rendering, and `evidence.json` with every source path, byte
size, SHA-256, classification, and calculation result.

## Safety and verification

- Live Hub `/healthz` reported `goal_output_enabled=false` for both robots.
- Both observations are `mapping_only=true`.
- Relay fusion is read-only and has no control dependency.
- `python -m json.tool` accepted the layout and runtime evidence JSON.
- `py_compile` accepted `hub/tools/foxglove_relay.py`.
- The complete test suite passed: **170 tests**.

## Primary provenance

| Artifact | Size | SHA-256 | Classification |
|---|---:|---|---|
| `hub/runtime/calibration/board_20260722_153114_cst_v2/validation_and_provenance.json` | 4,759 B | `314c543bd8985948e05c38b66bc70e572c12cdede5d9326b2d82e6172ce72945` | observed inputs + source-derived fit |
| `hub/runtime/calibration/board_20260722_153114_cst_v2/shared_frame.json` | 3,506 B | `ae5c178d7c90dd7d46d997cb0581a2649664b0be7ff41ee8959e0c17fc1442c3` | source-derived transform |
| `hub/runtime/calibration/board_20260722_153114_cst_v2/holdout_validation_seq9214_146524.json` | 8,615 B | `11f62187d72f6bca3267a85ad5fd064407ae938bc92f80e6eaebc5ab8d369741` | observed independent holdout |
| `hub/runtime/calibration/board_20260722_153114_cst_v2/chair_detection_seq9388_147892/evidence.json` | 7,908 B | `95d97af3094c18ffa7eaddc98175dd1efac49e69b577b2fd688ec8b1626697d0` | observed copies + source-derived check |
| `hub/tools/foxglove_relay.py` | 31,328 B | `9ec48bc60ad7e11c1f6add343c24ff50de4be0efb0eced6214c995c6961c9d85` | implemented, tested, live-observed |
| `hub/foxglove/dual_robot_dashboard.json` | 5,417 B | `0ab664c254241af3e13265c184c05bdadab21c989b2ed0c619d57951bcdffcb5` | implemented, JSON-validated |

Runtime calibration/evidence artifacts are intentionally not treated as
checked-in source. The tracked audit records their exact paths and hashes;
deployment copies must preserve those hashes.
