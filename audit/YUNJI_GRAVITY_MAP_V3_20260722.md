# Yunji gravity-preserving map v3 — 2026-07-22

## Outcome and scope

The Yunji false-floor-obstacle failure is fixed in the live, read-only map
path. The fix is deployed as a new transform/map session; no v2 map was
rewritten or mixed with v3 observations.

- **Observed:** a visibly clear near-carpet ROI that was previously 45.98%
  inside the obstacle height band is now 0 / 41,264 pixels (0%).
- **Observed:** the new Yunji sender is healthy with zero service restarts and
  uploads accepted `mapping_only=true` observations at the new transform
  version.
- **Observed:** both live v3 maps have zero pose jumps, zero ground-candidate
  rejections, zero ground-drift events and no mapping latch in the frozen
  acceptance snapshot.
- **Observed:** the Foxglove v3 relay delivers both cameras, both geometry maps,
  fused geometry and fused status under the new calibration ID.
- **Not fixed:** RedNet still produces no valid table/chair semantic cells.
  Geometry correction does not change the network's raw class output.

No planner, velocity, WATER move API, Unitree sport API or other robot command
was used. Hub `/healthz` reported `goal_output_enabled=false` for both robots
before and after the cutover.

## Root cause

Two independent errors compounded:

1. The local D455 mount is outside the chassis TF tree. Its sender rotation was
   based on an operator report that the mount was level, not an observed
   transform. Equal-weight floor normals from nine archived frames show that
   this nominal rotation needs a 4.073860 degree correction.
2. Board calibration v2 solved an unconstrained SE(3) transform. Its rotation
   tilted Yunji odom +Z by 5.3953 degrees. Combined with the wrong mount, the
   errors partially cancelled at the calibration heading and changed after the
   base yawed approximately 67 degrees.

The deterministic floor fit therefore changed from about 2.2–2.6 degrees at
the original heading to about 7.6–7.9 degrees after the turn. The old mapper
then discarded the fitted plane slopes and retained only one scalar
`floor_z_m`. On Yunji sequence 150697:

- 50,977 / 141,687 valid endpoints entered the 0.15–0.75 m obstacle band;
- the clear near-floor ROI was 13,278 / 28,875 (45.98%) false obstacle;
- the clear mid-floor ROI was 13,233 / 13,263 (99.77%) false obstacle.

This explains why TinyNav alone appeared more stable: its normal geometry
path did not compose this unmeasured local-camera mount with a board-derived
shared transform that rotates gravity, and it uses floor/free-space evidence
rather than one shared scalar height.

## Implemented contract

### Explicit camera mount

`yunji_sender.py` now accepts `--camera-extrinsic-file`, validates a finite
proper rigid matrix and verifies the declared RealSense model. The old D455
constant remains the fallback; it is preserved explicitly in
`hub/config/calibration/yunji_d455_mount_nominal_20260721.json` with its
unverified provenance.

`derive_ground_camera_extrinsic.py` fits each archived floor, converts its
normal to the camera optical frame and applies the minimum rotation that maps
the equal-weight mean to base +Z. It preserves translation because floor
normals do not independently identify the base-link origin.

### Gravity-preserving shared frame

`compute_gravity_preserving_alignment()` returns the closest yaw-only
alignment between two poses of the same landmark while matching the landmark
origin exactly. `calibrate_gravity_shared_frame_via_board.py` reuses the
existing symmetric-circle-grid PnP routine, corrects the other robot's camera
mount, aligns the board rather than camera centres and optionally runs the
independent moved-board holdout.

The deployed transform has exactly zero roll/pitch gravity tilt. This avoids
the yaw-dependent error by construction.

### Plane-relative mapping and fail-closed guard

The map keeps the complete plane `z = ax + by + c`. Each accepted frame is
classified relative to its own validated floor plane rather than one scalar
z. Before RedNet or irreversible semantic fusion:

- an untrustworthy/no-floor candidate skips that frame;
- a floor-normal change above 3 degrees or local height change above 8 cm
  skips that frame immediately;
- three consecutive accepted-but-outlying floor fits latch the map and require
  a fresh calibrated session; an in-range fit or a no-floor frame clears the
  pending streak;
- accepted per-frame coefficients drive obstacle and semantic height bands.

Snapshots are now `focus-hub-central-map-v3` and contain
`floor_plane_coefficients`, ground guard counters and last residuals. v2
snapshots remain readable but are never mixed into the new live map.

## Offline calibration and holdout

The extrinsic used nine existing observations spanning the original and
turned headings: 145299–145301, 147892 and 150695–150699. It did not select a
single favourable frame.

| Check | Result | Gate |
|---|---:|---:|
| Mount correction | 4.073860 deg | recorded, not a pass threshold |
| Maximum per-frame residual floor tilt | 0.783710 deg | <= 2.0 deg |
| Shared-transform gravity tilt | 0.0 deg | 0 deg |
| Calibration-board centre residual | numerical zero | exact origin fit |
| Calibration-board normal residual | 2.657449 deg | <= 3.0 deg |
| Moved-board holdout centre residual | 0.005459 m | <= 0.05 m |
| Moved-board holdout normal residual | 2.379940 deg | <= 3.0 deg |

An additional replay reconstructed odom/base poses under the new extrinsic and
transform. Across original, turned and latest 152663–152665 frames, the maximum
startup-relative plane change was 1.346 degrees / 0.0271 m, below the live
3-degree / 0.08-m guard. The replay pipeline processed both headings without a
ground rejection or latch.

## Live cutover and verification

The old Yunji observation service stopped cleanly after sequence 155792. The
new read-only service is:

```text
focus-yunji-gravity-v3-20260722.service
```

It runs `/home/nyu/focus_sender_yunji/yunji_sender_gravity_v3.py`, the tracked
extrinsic and tracked gravity-board transform. Preview credentials are passed
through service environment variables, not the new command line. At the
verification checkpoint it was active/running with `NRestarts=0`, localization
`TRACKING`, and accepted uploads around 19–26 ms. Temporary staged transfer
files and shell token variables were removed after start.

New Hub runtime sessions and outputs:

| Component | Runtime identity |
|---|---|
| Map tmux | `shared_maps_gravity_v3_20260722` |
| WSJ map | `hub/runtime/map_out_wsj_gravity_v3_20260722`, after sequence 10357 |
| Yunji map | `hub/runtime/map_out_yunji_gravity_v3_20260722`, after sequence 155792 |
| Relay tmux | `foxglove_relay_gravity_v3_20260722` |
| Foxglove | WebSocket 8765, preview HTTP 8766, fused read-only view |

Frozen live snapshot:

| Robot | Keyframes | Obstacles | Explored | Pose jumps | Ground rejected/drift | Blocked |
|---|---:|---:|---:|---:|---:|---|
| WSJ | 54 | 1,419 | 6,359 | 0 | 0 / 0 | no |
| Yunji | 60 | 2,281 | 16,981 | 0 | 0 / 0 | no |

On live Yunji sequence 156178, a deliberately conservative clear-carpet check
found 0 / 41,264 near pixels and 669 / 16,887 (3.96%) mid-range pixels inside
the collision band. The remaining mid-range pixels were concentrated at the
wall/baseboard and furniture boundaries; they were not globally thresholded
away.

A later moving observation exposed an over-sensitive first version of the
guard without corrupting the map. Sequence 157681 produced an accepted floor
fit with 3.291710 degrees startup-normal residual and 0.056917 m local-height
residual, so the original one-frame policy skipped it and latched. Read-only
replay showed 2.338 degrees on 157680 and recovery to 2.100 degrees on 157682;
the RGB sequence records a rapid turn from the table toward the wall. The
guard was therefore changed to skip every outlier immediately but latch only
after three consecutive observation frames with accepted outlying fits. An
in-range fit or a frame without a valid floor breaks the run and resets the
pending streak. This preserves the no-bad-frame-integration property while
tolerating the observed transient.

Observed source provenance for 157681 under
`hub/runtime/spool/robot-1/00000000000000157681/`:

- `metadata.json`: 2,313 bytes,
  SHA-256 `3f6442c6cf8878c086c485f375bcb2db582092c8c36731de5513b587629b138d`;
- `rgb.jpg`: 67,225 bytes,
  SHA-256 `d8f5f0e32fe0a319b826b8508d18cdb9ad79eb6e1635db1217c60179fe3be5e4`;
- `depth.png`: 191,825 bytes,
  SHA-256 `08400860a66d53d85e66f806d2ced1d5f6c8585c12f9dea68513a2cb2bb999ac`.

A 12-second raw `foxglove.sdk.v1` client received 62 WSJ camera messages, 109
Yunji camera messages, three geometry maps from each robot, two fused geometry
maps and two fused status messages. The decoded status was:

```text
fused 2 robots: shape=(17, 529, 540), origin=(-13.710814429520987, -13.653714333609205), calibration=shared-board-gravity-20260722-v3, explored=13752, obstacles=2381, semantic_evidence=0
```

The existing Foxglove layout/topic names are unchanged, so no new layout is
required; reconnecting the existing layout to port 8765 is sufficient.

## Semantic result remains separate

RedNet produced zero raw `table` pixels on sequence 150697 before thresholding
or map projection, so calibration could not have caused that class failure.
Both frozen v3 maps contain zero semantic cells.

The already-present YOLOv10m provides useful but insufficient evidence:

- current sequence 156178: `chair=0.840748` on the visible left chair;
- the visible right table region was weakly labelled `bed=0.211165`;
- the earlier head-on table frame 150697 returned only `clock=0.545705`.

Detection boxes are therefore not painted into the semantic grid. Doing so
would label background pixels and turn a known table misclassification into
persistent false map evidence. A real-camera labelled validation set and a
mask-capable/open-vocabulary semantic backend remain the next semantic gate.

## WSJ plant follow-up

The plant trial separates a map-session failure from a model failure. WSJ
sequence 10738 clearly contains the plant, but its uploaded three-channel
image is byte-for-byte grayscale (`B == G == R` for 100% of pixels) with the
RealSense infrared projector pattern. This is expected from the selected
TinyNav synchronized tuple: `/slam/keyframe_image` is `848x480 mono8`, not the
independent color preview.

On exact sequence 10738, RedNet proposed MP3D class 15 (`plant`) for 19,855
raw pixels (4.877899% of the frame), all below its fixed production confidence
threshold of 0.8. YOLOv10m independently detected `potted plant=0.639502`.
This close frame alone would therefore produce no production plant mask, but
other accepted views did: sequences 10712, 10714 and 10736 respectively had
12,212, 6,554 and 9,067 production plant pixels.

Observed source provenance under
`hub/runtime/spool/robot-0/00000000000000010738/`:

- `metadata.json`: 2,199 bytes,
  SHA-256 `42635f5f8fd1f96b4998f4bf418db748f0b3f5d8c20be4cb42dd6079988599a7`;
- `rgb.jpg`: 94,504 bytes,
  SHA-256 `4573154366c66051aef9750c0b4e54150ea93a73a3b3ecd071ce60b402c4f9e6`;
- `depth.png`: 161,887 bytes,
  SHA-256 `7de88f3bd24f580b2afbced34fe6d86fe0676b614d9c3434894c1f0685cdd4d6`.

The old WSJ map never saw those useful frames. It correctly halted while the
robot moved into the close view: accepted ground fits at 10700--10702 were
0.305--0.884 m away from the old floor, and three consecutive outliers proved
this was not the single-frame Yunji transient. The later pose stabilized with
a new local floor near -0.629 m versus the old -0.255 m. Old and new evidence
was not mixed; a fresh map began after sequence 10711 with startup frames
10712--10714.

The frozen fresh-session snapshot processed 154 keyframes / 286 observations,
with zero pose jumps, ground rejections, drift frames or latch events. It
contains 77 non-zero plant cells: 76 are above 0.5 and 73 equal 1.0. The live
Foxglove relay was moved to `map_out_wsj_gravity_v3b_20260722` without changing
topics or layout.

At the final 17:50 checkpoint the still-running WSJ session had processed 179
keyframes / 334 observations and retained 78 plant cells: 77 were above 0.5
and 75 equal 1.0. It remained unblocked with zero pose jumps, rejected-ground
frames, drift frames and drift events. This live count is observed but mutable;
the 154-keyframe bundle below is the frozen reproducible result.

A bounded fallback was also tested, but not deployed because the ordinary
production mapper succeeded. Requiring both YOLO `potted plant >= 0.5` and a
RedNet raw class-15 pixel inside the YOLO box at confidence >= 0.6 retained
3,693 pixels on 10738 and zero on no-plant sequences 10640 and 10680. This is a
safer next option than painting a whole detection box, but still needs a
labelled multi-scene false-positive gate before persistent fusion.

The HPC source is therefore not a turnkey physical-robot deployment, but that
is not equivalent to saying its models can never recognize real objects.
`source/Focus_realworld/main.py` is a Habitat/HM3D evaluation driver and has no
ROS/RealSense transport, sensor-time synchronization, camera extrinsic
calibration, odometry health gate, or robot-side stop authority. The Hub adds
those contracts. The plant result proves RedNet/YOLO can produce useful real
evidence; the earlier table/chair misses prove the simulator-trained RedNet and
fixed threshold still have a substantial real-camera domain gap.

The source's own `running_inference.md` makes the boundary explicit: a real
robot user must supply their own sensor stream. It does not provide that
driver. The inspected immutable source artifacts were:

| Artifact | Size (bytes) | SHA-256 | Classification |
|---|---:|---|---|
| `source/Focus_realworld/main.py` | 103,808 | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` | source-observed Habitat driver |
| `source/Focus_realworld/README.md` | 3,088 | `d7e3eed94e1da1e5d8c95d196c79b3aa2f6839b00e0bdafe312766a8fdd01c00` | source-observed simulator setup |
| `source/Focus_realworld/running_inference.md` | 3,379 | `c295cc0d4b97dc7d6532a6a949a22764a1a801e241d7f110febe44a2a14123d7` | source-observed deployment note |

## Final v3b relay checkpoint

The consecutive-drift implementation was deployed in new, non-mixed runtime
directories after preserving the original v3 outputs:

| Robot | Runtime directory | Keyframes / observations | Explored / obstacles | Ground rejected / drift frames / events | Blocked |
|---|---|---:|---:|---:|---|
| WSJ | `hub/runtime/map_out_wsj_gravity_v3b_20260722` | 179 / 334 | 925 / 210 | 0 / 0 / 0 | no |
| Yunji | `hub/runtime/map_out_yunji_gravity_v3b_20260722` | 203 / 4,023 | 22,411 / 2,612 | 2,109 / 15 / 0 | no |

The Yunji replay reached the newest available spool sequence 159815 without a
pose jump or drift latch. Its long tail lacked an accepted floor candidate, so
those frames were fail-closed and not fused; `last_sequence=157692` is the last
accepted keyframe, not a claim that later observations were integrated.

The production relay on the unchanged ports 8765/8766 now reads both v3b
directories. A 12-second raw protocol subscription received three geometry
maps, three semantic maps and six status records from each robot, plus one of
each fused map/status topic. It received 61 retained WSJ camera messages and
zero Yunji camera messages. The latter is not a map failure: Yunji's newest
spooled input was already ten minutes older than the final checkpoint and no
new preview was being pushed. The existing loopback-only reverse tunnel was
restored in its original tmux session; no second SSH session or robot command
was created.

## Verification

- focused ground/mapping/pipeline/calibration/sender tests: passed;
- complete Hub suite after live cutover: 180 collected and passed through
  `hub/scripts/verify_repository.sh --tests`;
- Ruff checks on changed Python: passed;
- exact sender/config hashes matched on nyush-nuc before start;
- map fusion contract: common `shared_world`, 0.05 m and
  `shared-board-gravity-20260722-v3`;
- protocol-level Foxglove subscription: passed;
- `source/` and `dependencies/`: unchanged.

## Primary provenance

| Artifact | Size (bytes) | SHA-256 | Classification |
|---|---:|---|---|
| `hub/config/calibration/yunji_d455_mount_nominal_20260721.json` | 748 | `640cb6908ee664e66f5e1aef2b77b4f485ce79cb8274125ff9c5808f276a69f6` | operator-reported, unverified nominal |
| `hub/config/calibration/yunji_d455_ground_extrinsic_20260722.json` | 13,885 | `f047c4bb166a7a8d659dd59be174cbdbdb56eb702d5f79e214e01359a776b1b8` | observed depth + source-derived rotation |
| `hub/config/calibration/shared_board_gravity_20260722_v3.json` | 6,050 | `f940d0c07b08e5beeda6271db70d1af5c1ff3399177521ce171fa05a9009d4ae` | observed board frames + source-derived yaw transform |
| `hub/robot_overlay/yunji_sender.py` | 65,603 | `2fa24b29d2ec49a052bea1b915e2d7741ade097e4e8e57f33de7004b9778b922` | implemented, tested, live-used |
| `hub/tools/derive_ground_camera_extrinsic.py` | 10,140 | `dc44208f98f72f2a0bbb375de9c3f894bde2354180e114da1f5a5fe6e6ae10e0` | implemented, offline-used |
| `hub/tools/calibrate_gravity_shared_frame_via_board.py` | 13,914 | `cf79f5ead8910b1448324a504786d71af45444139d790a03e1cab86d14c927f9` | implemented, holdout-used |
| `hub/runtime/analysis/yunji_geometry_seq150697_20260722/geometry_diagnostic.json` | 4,354 | `665c1286320ab319bf81caaf87108e3025944f712a83e3628daccf2d4b37e038` | observed inputs + source-derived old-failure diagnosis |
| `hub/runtime/analysis/table_semantic_yunji_seq150697_20260722/domain_gap_summary.json` | 12,868 | `daa83b232d7af18624c9a459dde070631c1bad2553424e65e2902a10fedbeb3f` | observed RedNet output before map projection |
| `hub/runtime/analysis/gravity_v3_live_acceptance_20260722/acceptance.json` | 5,637 | `0d54c855e215e76e71f63210ac9a3f70d252046c13006b96b7979518d4223aa8` | frozen live acceptance manifest |
| `hub/runtime/analysis/wsj_plant_semantic_seq10738_20260722/domain_gap_summary.json` | 12,767 | `71aec3d777ae9ce74e9ec7ceee71de3b0d509b4c55c1d9a06b3c2f8098a1366a` | observed RedNet output before map projection |
| `hub/runtime/analysis/wsj_plant_live_acceptance_20260722/central_map.npz` | 21,237 | `b1bca80906c1344fe27867ce5ee1017f9e6efa6bd239ebac6a94aef4af51e3e8` | frozen observed v3b map with plant evidence |
| `hub/runtime/analysis/wsj_plant_live_acceptance_20260722/map_summary.json` | 1,395 | `b5112bdbe2e48e3eb39c6c91e44f116550e2c330f929dbed36a563fb01f65d25` | frozen observed v3b summary |
| `hub/runtime/analysis/wsj_plant_live_acceptance_20260722/live_status.json` | 1,028 | `77d1851d77407ef1f6dd4775c778981ccc8d37f5907862a0949161017d95e35d` | frozen observed v3b live status |

The full extrinsic artifact records every selected metadata/depth source path,
size and SHA-256. The frozen acceptance bundle records its copied map, status,
RGB-D and metadata hashes. Runtime evidence remains Git-ignored; this audit and
the deployable calibration artifacts are tracked.
