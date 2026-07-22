# Yunji Odin1 integration — 2026-07-22

## Outcome

Odin1 serial `O1-P070100205` now has a tested TopoFocus observation adapter, a
reproducible deployment overlay, a fresh WSJ/Odin shared-frame calibration and
a live Foxglove/map cutover. Real RGB, colored SLAM cloud and odometry are
converted to the existing aligned RGB-D Hub contract. The sender is running
continuously under a versioned shared transform and also pushes the rectified
RGB preview to `/yunji/camera` through a loopback-only reverse tunnel.

The early bounded trial described below contained a nearby tabletop and wall
but no visible floor; it was correctly rejected as physical floor validation.
After the sensor view was corrected, a fresh fit plus independently moved-board
holdout passed, and completely new WSJ/Odin map directories passed their
startup ground gates. The old D455 transform and map data were not reused.

No robot command was sent. The Odin driver, sender and verifier have no planner,
velocity publisher or WATER motion endpoint. Hub health reported
`goal_output_enabled=false` for both robots.

## Source provenance

The user identified the authoritative record under Yunji's TinyNav checkout:

| Artifact | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `/home/nyu/workspace/tinynav/yunji-water-robot/docs/odin1_deployment.md` | 4,441 | `f187ff1b4905415c8a5f1cf84537bf3b6ea5a920f665a606ede0a1e915c4528b` | observed remote document |
| `/home/nyu/odin_ws/calibration/O1-P070100205.calib.yaml` | device file | `c8cbd48bd8f8b08b8f174f557faf48649ee1101a3dfe0daf82ceae3832d7c23d` | observed device calibration |
| `config/control_command.yaml` | 5,288 | `c9a0c3466d8526cc290ddd24a31dd8670bb988b8e8a9e1356c625da0dc8ac5ef` | observed runtime configuration |
| `src/host_sdk_sample.cpp` | 107,243 | `edddec679c13f0e7af3940238faf227aa6282a8e14797f4f0d2899f00110ac85` | observed patched driver source |
| `src/yaml_parser.cpp` | 11,400 | `826594ab4397e223b6ed0b05e0a585538bea19155902f2a609741ce349f08024` | observed patched driver source |
| `hub/config/calibration/yunji_odin1_board_20260722_v1.json` | 5,544 | `9e340a882df936e005902de29bb6e54c0a76da6e41c7bda26a040a0ce1421519` | observed spool inputs plus source-derived rigid alignment |

The driver remote is
`https://github.com/manifoldsdk/odin_ros_driver.git`, tag `v0.13.0`, commit
`13aa528b1da581e2168ac858f8b144f0b4438a7a`. Its four-file dirty-tree delta was
captured exactly at
`hub/robot_overlay/odin1_snapshot/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch`:
10,346 bytes, SHA-256
`2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806`.
The temporary remote transfer copy was removed after checksum agreement.

The patch is necessary on the observed firmware. A direct mode-1 cold start
can leave RGB/DTOF/IMU rates at zero and only emit a roughly 0.5 Hz status
heartbeat; the patched path boots mode 0, starts streams, then activates mode
1. The patch also records the mode-1 config and RViz decay display. This is an
observed live deployment delta, not an upstream release claim.

## Observed ROS and clock contract

The existing documented launcher was started and reported ready on VNC `:2`.
The live contract was then inspected without changing robot state:

- `/odin1/image`: 1600×1296 `bgr8`, reliable;
- `/odin1/cloud_slam`: about 30.6k XYZRGB points/frame, frame `odom`, about
  10.3 Hz;
- `/odin1/odometry`: parent `odom`, child `odin1_base_link`, about 10.3 Hz;
- `/odin1/cloud_raw`: advertised but no message in two bounded checks;
- `/odin1/depth_img_competetion`: absent because `senddepth: 0`.

Odin message stamps were around hundreds/thousands of seconds after device
boot while the host epoch was around 1.7847e9 seconds. They are therefore not
UTC. The adapter uses them only to pair local ROS messages and uses
nyush-nuc's NTP-synchronized receipt time for the Hub capture timestamp.

The factory FishPoly intrinsics and camera/lidar transform came from the exact
serial calibration. The driver source supplies lidar-to-IMU translation
`[-0.02663, 0.03447, 0.02174]` m. Reproducing the driver's
`T_imu_camera = T_imu_lidar @ inverse(T_lidar_camera)` yielded the tracked
matrix in
`hub/config/calibration/odin1_O1-P070100205_factory_20260722.json`.

## Adapter implementation and corrections

`hub/robot_overlay/odin1_sender.py` is a read-only ROS 2 source that preserves
the Hub's replayable RGB-D payload:

- FishPoly rectification produces an 800×648 zero-skew pinhole image;
- live `T_odom_imu` and the factory internal extrinsic produce camera pose;
- the colored odom cloud is transformed into camera coordinates and nearest
  depth wins in a true z-buffer;
- a radius-one splat preserves sparse lidar coverage without inventing the
  vendor completion result;
- WATER TCP is used only for robot information/status and heartbeat health;
- the old D435/D455 transform version is rejected explicitly.

The first three-frame dry run exposed adjacent-cycle pairing on two frames:
image/cloud skew was approximately 97.44 and 97.48 ms. The deployed driver
does produce exact matching stamps; a 20 ms gate then gave ten consecutive
0 ms pairs. The default was reduced from 150 to 20 ms.

A second high-rate check exposed replay of an older cloud still present in the
deque. Selection now requires strictly increasing cloud device stamps. The
final ten-frame live check had ten unique increasing stamps and 0 ms skew for
both image/cloud and odometry/cloud.

Across the final check, approximately 24.0k of 30.6k points projected into the
camera. Median absolute RGB mismatch between the cloud's own color and the
rectified image sample was 13–14 intensity values, supporting the internal
extrinsic/projection path. The approximately 0.53 m median depth matched the
observed close wall/table scene; it was not treated as a sensor range fault.

## Hub wire contract and bounded acceptance

An initial upload deliberately used `parent_frame=yunji_odin1_odom`. Transport
v1 rejected it because its model requires `parent_frame=shared_world`. That
rejection also revealed an API bug: Pydantic's embedded `ValueError` context
was not JSON serializable, turning the intended 422 into a 500. The API now
omits that context and has a regression test for a serializable 422.

The existing v1 convention, already documented by
`calibrate_shared_frame.py`, defines wire `shared_world` as the robot's local
odom until calibration. The final Odin metadata follows that convention while
retaining all fail-closed distinctions:

- `transform_version=yunji-odin1-local-odom-20260722-v1`;
- session definition `shared_world := yunji_odin1_odom`;
- `shared_frame_calibration_id=null` at the map layer;
- `mapping_only=true` and `base_T_camera=null`;
- cross-robot fusion disabled.

Those values describe the pre-calibration bounded lane only. It ends at
sequence 159948; calibrated observations start at 159949 under
`yunji-odin1-board-20260722-v1`. No pre- and post-calibration observation is
written into the same map directory.

The production Hub accepted these exact observations once each:

| Sequence | File | Bytes | SHA-256 |
| ---: | --- | ---: | --- |
| 159816 | `metadata.json` | 2,886 | `1b894c578b2925a76f868a69b9ee3c0a96fa1417313a4aabe1dee1cf9472fad1` |
| 159816 | `rgb.jpg` | 123,464 | `5d838f3db3b258d27a605c4eccdd657b0ed49dd60fe86c6e76de080238325592` |
| 159816 | `depth.png` | 168,307 | `2bf5cc5d87cc51f091965e2aa5d3a7f1544faaf6f55bacf3b6964777fa26a641` |
| 159817 | `metadata.json` | 2,884 | `1f7b1696a1919929867880cd4a1db6b2e064a2b55064efd952db9d614a46bfa2` |
| 159817 | `rgb.jpg` | 123,895 | `5facdfb8f72f8cd551a75d37d47a38518e449e467d2b945bd00653753ab96832` |
| 159817 | `depth.png` | 168,591 | `fd017d1120e4063766c6df4a929fcecf6ed9048c04bfd00a04afa14dc00c56e5` |
| 159818 | `metadata.json` | 2,886 | `a534652d8cd432b08c11f7e1aca56cb307c0320ff29b9559e4c4077f1cc0a26c` |
| 159818 | `rgb.jpg` | 123,102 | `f26c7ac6febb4b97c4bb527847709c7070dccfa11b450b80606e927a472c8ec0` |
| 159818 | `depth.png` | 167,663 | `6d04cbd96d0eab09d3fa102c54a0ce9c478214c33e87d8860c44b42ceefd6945` |

The existing Yunji D455 daemon remained bound to
`yunji-d455-gravity-board-20260722-v3`, so its snapshot retained last sequence
157692 and did not integrate the Odin frames.

## Isolated map gate

The bounded frames were replayed into a new ignored directory,
`hub/runtime/map_out_yunji_odin1_local_20260722`, after sequence 159815 and
with the exact Odin transform version. The daemon initialized from sequences
159816–159818 with median plane
`z = 0.00417 x + 0.17799 y - 0.22644` (about 10.10° tilt). Individual
candidates ranged from 9.54° to 11.37°. The first processed frame differed
from the median by 3.0256°, so the live ground guard skipped it.

Visual evidence explains why this is not a valid floor test: the RGB image is
almost entirely a wall, power outlet and tabletop. No physical floor is
visible. The plane's camera-relative height was only about 0.21–0.25 m, which
is consistent with a tabletop below the sensor and not evidence of Yunji's
floor. The candidate daemon was stopped cleanly and no relay was switched to
this map.

## Fresh board calibration and live cutover

The existing 7×10 symmetric-circle board detector initially failed on Odin
because carpet/projector structure produced many blobs at 4× enlargement. The
updated detector preserves the old path first, then tries 2× and
`CALIB_CB_CLUSTERING` fallbacks. Grid endpoints are canonicalized to prevent a
symmetric-board detection from alternating by 180°, and the physical grid
center is the landmark origin.

The source-derived gravity-preserving transform used these observed frames:

| Role | WSJ sequence | Odin sequence | Sync skew | Center residual | Normal residual |
| --- | ---: | ---: | ---: | ---: | ---: |
| fit | 13234 | 159827 | 72.976634 ms | 0 m | 0.7053266° |
| independently moved-board holdout | 13568 | 159929 | 59.702478 ms | 0.0114777 m | 0.4472508° |

The holdout passed limits of 0.05 m, 3° and 0.25 s. The generated transform is
yaw-only, so its tilt is 0° and it cannot rotate gravity. Its tracked artifact
records source paths, sizes/checksums, observed/source-derived status and
`robot_commands_issued=false`.

The sender loaded calibration ID `shared-board-odin1-20260722-v1`, resumed at
sequence 159949, and produced accepted `TRACKING` observations with exact
image/cloud/odometry device stamps. Fresh maps were initialized as follows:

| Robot | Runtime directory | Start-after | First integrated | Transform |
| --- | --- | ---: | ---: | --- |
| WSJ | `map_out_wsj_odin1_board_v1_20260722` | 13736 | 13737 | `wsj-tinynav-depth-20260722-session-v1` |
| Yunji | `map_out_yunji_odin1_board_v1_20260722` | 159948 | 159949 | `yunji-odin1-board-20260722-v1` |

At relay cutover, both maps had the same explicit calibration ID, zero pose
jumps, zero rejected ground frames and no mapping block. Their consensus
`floor_z_m` values were -1.15162 m and -1.12792 m, a 2.37 cm difference. These
runtime maps are ignored and are reported as observed state, not packaged
replay artifacts. The main relay retained ports 8765/8766, robot labels and
topic names, so the existing Foxglove layout remains valid.

## Deployment state and remaining gates

Tracked deployment additions include a headless driver launch, read-only
verifier, environment example and two systemd units. They intentionally omit
RViz, planners and controllers and set `SendSIGKILL=no` so USB shutdown retains
the vendor's SIGINT-only rule.

Both `verify_odin1.sh` and `verify_odin1.sh --hardware` passed on nyush-nuc.
The latter received one live message from each required topic. ROS 2 also
parsed the headless launch with `--show-args` without starting a second driver;
remote `systemd-analyze verify` reported no error in either staged Odin unit
(only unrelated host warnings for netplan/snapd). The units are installed and
active, but remain disabled for boot so startup stays an explicit operator
decision.

Local/remote deployment hashes matched exactly:

| File under `/home/nyu/focus_sender_odin1` | Bytes | SHA-256 |
| --- | ---: | --- |
| `odin1_sender.py` | 31,952 | `2eacc915655f8f615283341f953452a4896963591a6616b14768b4a905fc50ab` |
| `yunji_sender.py` | 65,704 | `4455b70106aefcb9b2415f79e2dd63e448ea7010a5baf6b0abf342a305c87242` |
| `odin1_driver_headless.launch.py` | 1,284 | `3de7f44d318a214b8809dd47ba739d8ce549ebc8d99e9f373f2a4fca9812afe0` |
| `verify_odin1.sh` | 2,847 | `44a8dcc929c9ba5d11b50a3b09fd773c735293e11bd9bd13b43448ef5ef5c7ac` |
| `odin1_O1-P070100205_factory_20260722.json` | 3,597 | `ba0811b52950730d65981556b13b703eb036b1ed6e85302628d402c459fe6de6` |
| `yunji_odin1_board_20260722_v1.json` | 5,544 | `9e340a882df936e005902de29bb6e54c0a76da6e41c7bda26a040a0ce1421519` |
| `systemd/focus-yunji-odin1-driver.service` | 901 | `ba5f06d7e367135715bff2b09dc24052b5aa4528857f4dbe2fa272ca957c1719` |
| `systemd/focus-yunji-odin1-sender.service` | 1,050 | `70cc8a737a7da1d53ad4cb69fb9453628b35f9034275a8be64ccdcf24a497b94` |

At handoff:

- the headless Odin driver and continuous read-only sender are active;
- the new WSJ and Yunji map daemons are active and the old WSJ blocked daemon
  was stopped cleanly without deleting its map;
- the main relay reads the two new map directories and publishes fused maps;
- `/yunji/camera` is live through the loopback-only tunnel;
- both Odin units are disabled for boot;
- the real mode-0600 robot token exists only on nyush-nuc, outside Git.

Remaining physical follow-up is a controlled-motion map test, labelled semantic
target validation, longer soak and the existing disconnect/timeout/e-stop/HIL
gates. None of this observation work authorizes GOAL output or autonomous
navigation. A sensor remount or odometry-origin reset invalidates this artifact
and requires another fit plus independently moved-board holdout.
