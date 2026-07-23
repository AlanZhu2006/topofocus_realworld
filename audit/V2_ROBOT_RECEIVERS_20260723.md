# V2 dual-robot receiver implementation audit (2026-07-23)

## Outcome and classification

The versioned dual-robot high-level target path is implemented through the
robot-local TinyNav and WATER planner boundaries. A complete synthetic
Hub-to-adapter run was observed locally with concurrent `GOAL` decisions for
`robot-0` and `robot-1`. The run used no external network and sent no robot
command. Physical motion, arrival accuracy, lease cancellation latency on the
real devices, and SR/SPL remain **unverified**.

The authority split is source-derived from the project design and implemented
as follows:

- the Hub publishes an atomic pair of versioned, expiring, shared-frame
  high-level targets;
- WSJ converts its target to a TinyNav POI. TinyNav retains planning and
  velocity generation, while a robot-local gate forwards raw `/cmd_vel` only
  to a distinct guarded topic;
- Yunji converts its target to a WATER saved-map point, checks it with WATER's
  accessible-point query, and calls WATER `/api/move`; WATER retains planning,
  avoidance and low-level control;
- each robot renews and terminates its own lease independently. Local health,
  transform, reachability, disconnect, expiry, HOLD and STOP checks fail
  closed.

The WATER endpoint and status semantics are **source-derived**, not physically
observed in this work session, from the vendor WATER software API manual v1.8.7:

`https://bbs.realman-robotics.cn/uploads/20240510/bb301c61dc206155dd8e0ae6fa79e2bb.pdf`

No copy of that PDF remains in the workspace or `/tmp`.

## Locally observed verification

At `2026-07-23T02:39:11+08:00`:

- `bash hub/scripts/verify_repository.sh --tests` passed;
- 275 Hub tests were collected and passed;
- `git diff --check` passed;
- `hub/tools/run_v2_fullstack_dryrun.py` completed with
  `concurrent_goal_robot_ids=["robot-0","robot-1"]`,
  `external_network_used=false`, and `robot_commands_sent=false`;
- the local deployment policy file still had `allow_goal=false` and
  `transform_version="UNSET"` for both robots.

The synthetic report is ignored runtime evidence and is not a physical result:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/v2_fullstack_dryrun_20260723_receiver_final/report.json` | 5,699 | `1732400762a106557901b6b0f0a03ffa0d786b770fdf56cb3f5060452c2f07df` | locally observed synthetic, no motion |
| `hub/config/robots.json` | 255 | `6f5e0172fde4be8cfe452c266f06ba80120b46e9f239683818bc604fe1cb757d` | locally observed policy, ignored runtime config |
| `hub/scripts/verify_repository.sh` | 3,068 | `28c71eb8bc859ea0c51e4a27bde3c456a3da165c8d9cff3ba1c8741de6b2f5cf` | implemented verification entry point |

## Implementation provenance

These hashes identify the reviewed local implementation at the time of this
audit. They are implementation artifacts, not proof of physical behavior.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/transport_v2.py` | 14,761 | `91deda8bd4e8e7ea55d186d7c42416f57c5a9012eae3f44b9ab7840a749ea425` | implemented protocol |
| `hub/src/focus_hub/v2_registry.py` | 15,124 | `859fcd0f2ed4a565e3b78997785ee062770a3d0e8150bea2797febd3276e51bd` | implemented atomic registry |
| `hub/src/focus_hub/v2_scene_batch.py` | 23,475 | `2b143d3a3100c491f831486cc1536153609e76fa4b2d04bc9cc033d198f614bc` | source-derived batch builder |
| `hub/src/focus_hub/v2_episode_control.py` | 2,888 | `043e2bbddc1590ed8303e86f40ab75b95bb3363412d9bdb202724b4ccf99c8be` | implemented independent lease control |
| `hub/src/focus_hub/v2_goal_adapter.py` | 13,794 | `5c238be50be8d59cf2723ae1feb64925971be8c931723525ea2495add2fde1e0` | implemented robot-local target adapter |
| `hub/src/focus_hub/v2_robot_runtime.py` | 13,897 | `7920dfeeba9f098cc21b84667bae619c5eeb07bf4885dd45dd8662e6ab7311f3` | implemented receiver runtime |
| `hub/src/focus_hub/robot_map_alignment.py` | 8,708 | `3fa7c0759d0153e2b7c38ba7417cb735391476231925bad335f2c644b5f33d2d` | implemented alignment gate |
| `hub/src/focus_hub/base_camera_calibration.py` | 3,969 | `924d6f29c7b872e4d5038cf1c625c745a322d041c4a4d1d7f631663e5fc0c81c` | implemented measured-mount loader |
| `hub/robot_overlay/v2_wsj_receiver.py` | 36,467 | `585ab917a338cb892f3735d59887da2cbbf5386c2101cdaa9753e96407f8797c` | implemented, physical motion unverified |
| `hub/robot_overlay/v2_yunji_receiver.py` | 34,270 | `858924d6e6cf3ddf6b336664da1dd7121a33a838913943e16bac48ec21bc5542` | implemented, physical motion unverified |
| `hub/robot_overlay/focus_ros_sender.py` | 39,285 | `6f3da0156eda3b80ed4c527f67255955681aa58b29c9a08db4f7d907b691102e` | implemented command-capable metadata gate |
| `hub/robot_overlay/odin1_sender.py` | 33,816 | `9fb1057ffdfeed8e86bcd60f32fb52c3767b4082a1de4147e8b45c5db8147766` | implemented command-capable metadata gate |
| `hub/tools/build_v2_decision_batch.py` | 2,805 | `42424f2503da467f4c09813f8938997f4514f532f7c5c092208a2576710d4549` | implemented batch CLI |
| `hub/tools/run_v2_supervised_episode.py` | 12,181 | `acd485d74277d9c7efa578ae157a996ebb0f96cd80596ebe6f4cb6b923ef338c` | implemented supervised controller |
| `hub/tools/run_v2_fullstack_dryrun.py` | 18,221 | `6869e3ba106e54f5f9afdc89eba336c3b6cb465bdfd89ef32089a461d9226364` | implemented no-motion verifier |
| `hub/tools/record_base_camera_calibration.py` | 4,330 | `2bebbe3c5a9ae730aa31980bb9ac52645305fe1d7d14c6d284a258d0fba497cc` | implemented measurement recorder |
| `hub/docs/V2_PHYSICAL_QUICKSTART.md` | 8,867 | `5bceacbac54b7801953119daa97d950f168656bacf99568115b1e1823a7b2520` | implementation-derived operator procedure |

## Remaining physical gate

The shortest remaining path is deliberately small: record both measured
`base_link -> camera` transforms, create a fresh shared-board calibration after
both current odometry origins are live, then run one operator-supervised short
dual crawl. Do not enable `allow_goal` before both read-only receivers produce
valid alignment artifacts and both robot-local planner chains are ready. After
the crawl, return `allow_goal` to false. Foxglove styling, detector tuning and
large-scale 4-by-5 evaluation are lower priority until this first physical
chain is observed.

## 2026-07-23 late-session WSJ online-router correction

At approximately `2026-07-23T23:26+08:00`, one explicitly supervised live
attempt was **observed**. Both expiring high-level goals were published. WSJ
rejected its goal as `UNREACHABLE`; Yunji briefly reported `NAVIGATING`; the
episode controller then published HOLD for both robots. Both receivers reported
zero-velocity confirmation. The stack was subsequently returned to debug mode
with Hub `GOAL=false`, WSJ's Go2 bridge absent, Yunji's live receiver inactive,
and Yunji's debug receiver active. This was a fail-closed test, not a successful
navigation episode and not an SR/SPL result.

Read-only ROS inspection identified two independent causes:

- `/slam/odometry` is the optical-camera pose, but the receiver and online
  router had treated its XY position as the robot base;
- the deployment wrapper required the final VLM frontier to be fully reachable
  in the current known-free map and added 20 cm of clearance before TinyNav's
  own collision-aware local planner could run.

The measured WSJ mount artifact is remotely observed at
`/home/nvidia/.local/state/topofocus/calibration/wsj_tinynav_camera_base_20260723_operator.json`,
1,029 bytes, SHA-256
`ef108af55cbd8b84b0c65ad3c9441f92afd1c444056b62ca729a2b00b202214f`,
classified as `operator_measured_physical_mount`. The corrected receiver
records this path, size, checksum, matrix, and classification in each new map
alignment artifact.

The deployed correction derives `tracking_T_base =
tracking_T_camera @ inverse(base_T_camera)`, uses a one-cell/5 cm coarse
reachability gate, and permits only `FRONTIER_POINT` targets to become a
receding-horizon route ending at the closest improving known-free cell.
`SEMANTIC_REGION` targets remain strictly reachable; unknown and occupied cells
remain blocked; TinyNav's unchanged local planner/controller retains final
motion authority.

The following implementation artifacts were checksum-verified after deployment
through the existing WSJ SSH/tmux session:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/v2_goal_adapter.py` | 14,052 | `5ad5cff246a13a112d4f98d6ba932c1ed777402ae3b16e6e8b146e9db9d3c998` | implemented; verified on WSJ and Yunji |
| `hub/robot_overlay/v2_wsj_receiver.py` | 47,781 | `7be3a1b7584ca6be5df53002dc416280225ffa8c150c1ab99e2b8a4ca005ebba` | implemented; verified on WSJ |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 29,726 | `9df856937c3e21d3622fa1901804720b2a5bbbb04cd325b96c4b0b0667923b32` | implemented; verified on WSJ |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 7,791 | `ed67753fee656951cbdda252d96ca0f9e17b7cb27c4246e8d6b5925d25de1e11` | implemented; verified on WSJ |
| `hub/robot_overlay/start_tinynav_buildmap_online_nav.sh` | 6,082 | `0c3b8ac26e693c252bc24c79e4ed6fb4c636ed3667422051493fc51e6b4f407f` | implemented; verified on WSJ |
| `hub/scripts/realworld_oneclick.sh` | 13,279 | `4f24690d15885a748563ce3d6fbc10df18202b4de83f5f4878221786ad0ae266` | implemented locally; fail-closed cleanup added |

The post-deployment, no-motion ROS preflight **observed** the corrected base in
free cell `[50,57]` (occupancy `0`), a 2,377-cell reachable component with one
clearance cell, and a 2.116 m known-free partial route for the previously
rejected frontier. No unknown or occupied cell was traversed by the route.
The real-VLM debug one-click run completed with Hub HOLD only:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_debug_debug-chair-after-wsj-base-fix_20260723_234616/shadow/shadow_manifest.json` | 17,228 | `5f873fca66f3103fbb41fb85b5cdd23119a0780abcc58ba982e4475a34630540` | observed real-VLM shadow, no motion |

The remaining physical gate is now a new operator-confirmed short dual motion
attempt. Formal success and SPL remain unverified until terminal evidence is
recorded and scored.

## 2026-07-24 self-occupied-start correction

An operator-supervised live attempt beginning at approximately
`2026-07-24T00:04+08:00` was observed and failed closed. WSJ initially accepted
the real-VLM `chair` semantic region and reported `NAVIGATING`; Yunji
independently accepted frontier `B` and WATER started the first 0.45 m
receding-horizon segment. WSJ then reported `NO_KNOWN_FREE_PATH`, rejected the
renewed lease, and the controller published HOLD to both robots. The automatic
cleanup removed WSJ's Go2 bridge, cancelled Yunji's WATER move, restored both
debug receivers, and restarted Hub with `GOAL=false`. This is a failed
engineering attempt, not an SR/SPL result.

Read-only inspection of the live WSJ occupancy and measured base pose observed:

- base XY `[1.6257840559, 1.6101101828]`, cell `[136,66]`, occupancy `100`;
- every cell in the base-centered 3-by-3 neighborhood was also `100`;
- the nearest genuinely free cell with one-cell clearance was only 0.1351 m
  away at cell `[133,66]`;
- the unmodified coarse reachability gate consequently returned zero cells.

The immutable HPC source provides the provenance for the correction:
`source/Focus_realworld/agents/vlm_agents.py` constructs the traversible map
from occupancy, restores its dilated `visited_vis`, and explicitly marks the
3-by-3 current-pose neighborhood traversible before invoking FMM. The
deployment overlay now implements the same intent without mutating the
published map: only a 0.18 m disk centered on the measured physical base may
bridge a self-occupied start to a seed that is still genuinely known-free with
the configured clearance. Occupied and unknown cells outside that measured
footprint remain blocked. The online receiver delegates final online
frontier/semantic feasibility to this router; TinyNav's unchanged local
planner/controller and guarded Go2 bridge retain final authority.

The no-bridge physical-map preflight observed
`ONLINE_PARTIAL_PATH_READY`, a 0.137 m start snap, a 0.600 m known-free route,
and waypoint `[2.125,1.575]`. It ran while `/nav/paused=true` and with no
`go2-bridge` window. A subsequent real-VLM debug one-click completed
`DEBUG_FULLSTACK_READY` with Hub `GOAL=false`, read-only receivers, and no WSJ
Go2 bridge. The targeted local regression suite passed 42 tests.

Superseding implementation and evidence provenance:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/v2_robot_runtime.py` | 20,133 | `377e0317e1bc05a6238dc5ef085e4184919427812e0894fdd062c9c069e018f6` | implemented; checksum-verified on WSJ and Yunji |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 33,690 | `47eac6f8ab9bd0fcf64a453e0b7951b10b5ce98aad63ef5b113c4615d3eb669b` | implemented; checksum-verified on WSJ |
| `hub/robot_overlay/v2_wsj_receiver.py` | 51,586 | `391396d1060bdff8d9c2c66866805e39061be5836006f17ef13a3624ba8db62e` | implemented; checksum-verified on WSJ |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260723T160419Z.jsonl` | 5,550 | `f224caf2bf26da60197d65829581d385791f17678258fb618599a3f2bbc04013` | remotely observed live WSJ event log |
| `/home/nyu/.local/state/topofocus/yunji-v2-live-20260723T160424Z.jsonl` | 5,805 | `557f10871135aa06d467907e748460ea730020cf91a05d2730127339e006c012` | remotely observed live Yunji event log |
| `hub/runtime/oneclick_live_scene01-chair_20260724_000431/episode/episode_report.json` | 9,763 | `e3f993a91b596097d78de3bc17ae02c8e2612fb75f805b21f4b466df74c557a7` | observed failed supervised episode |
| `hub/runtime/oneclick_debug_debug-chair-startfootprint_20260724_001906/shadow/shadow_manifest.json` | 20,047 | `73a5a3bbc765b02c6df1d717575bfbbae42497184ed199c514453abb7c6d753d` | observed real-VLM shadow, no motion |

Formal target arrival, success, and SPL remain unverified. A fresh exact
operator confirmation is required before the next motion attempt.

## 2026-07-24 fresh-map WSJ chair shadow

A new, non-actuating dual-map session
`shared_maps_20260724_rebuild_v10_chair` was started after sequence 21,122 for
WSJ and 195,266 for Yunji. Both new map daemons reported
`mapping_blocked_reason=null`; the older v9 mapping processes were stopped
without deleting their artifacts. Foxglove port 8765 was switched to the new
v10 map directories.

On observed WSJ source sequence 21,133, real YOLOv10 detected `chair` at
0.929105 confidence. The source-derived VLM cascade returned
`Perception_PR=[0.960887,0.039113]` and
`Judgment_PR=[0.890752,0.109248]`. The target-semantic override replaced the
frontier with a 39-cell chair region at shared-world display centroid
`(0.666894,-0.220388)` m. Hub published HOLD only; both receivers remained
read-only and WSJ had no Go2 bridge. The debug run used the explicit forensic
skew allowance because its two frozen camera captures differed by 7.367 s;
this does not qualify as a strict synchronized live-motion preflight.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_debug_rebuild-v10-wsj-chair_20260724_003227/shadow/shadow_manifest.json` | 19,768 | `08aee6f3f4285cfb13d20b90fa6286cbdd0b51af9e4930924749bfa86ca584ed` | observed real-model shadow, no motion |
| `hub/runtime/oneclick_debug_rebuild-v10-wsj-chair_20260724_003227/shadow/source_goal_masks/wsj_chair.png` | 1,021 | `c40ef31448bfb7fa18c96412ea43cf0bf899751afb9b0ec1e2c480a8a0b6e62a` | source-derived projected chair mask |

## 2026-07-24 WSJ odometry-callback isolation

One new exact dual-robot operator authorization was consumed by the
`scene01-chair/run02` attempt at approximately `2026-07-24T00:35+08:00`.
Strict input synchronization passed with WSJ sequence 21,145, Yunji sequence
195,566, 1.606 s capture skew, and input ages of 4.678 s and 6.283 s. The
real-model cascade selected the 74-cell WSJ `chair` semantic region at shared
centroid `(0.566894,-0.370388)` m and Yunji frontier B at
`(4.066894,-1.720388)` m.

WSJ accepted the local chair goal and reported `NAVIGATING` three times, then
reported `HOLD/ODOMETRY_STALE` about 2.96 s after POI publication. The
controller immediately issued dual HOLD. Observed displacement before cleanup
was approximately 0.0057 m for WSJ and 0.4155 m for Yunji. Automatic cleanup
removed the WSJ Go2 bridge, restored both debug receivers, cancelled Yunji's
WATER move, and restarted Hub with both goal outputs disabled. This was a
failed engineering attempt, not an SR/SPL result.

The cause classification is deliberately conservative:

- **observed:** the router's one-second callback-age gate emitted
  `ODOMETRY_STALE`;
- **observed after the attempt, not contemporaneous proof:** an independent
  12-second `/slam/odometry` rate check measured approximately 13--22 Hz with a
  maximum reported gap of 0.296 s;
- **source-derived risk:** the router previously ran A* replanning and odometry
  callbacks in one executor group, so a long callback could age its own
  receipt timestamp even while DDS continued delivering odometry;
- **unverified:** whether the failed live interval also contained a genuine
  one-second publisher gap.

The deployment overlay now assigns odometry/occupancy callbacks and
goal/replanning callbacks to separate mutually-exclusive callback groups on a
two-thread ROS executor. Sensor state is copied under a short lock before
planning. The one-second stale threshold is unchanged. The receiver's
independent health gate, expiring Hub lease, `/nav/paused`, guarded command
topic, and robot-local stop authority are unchanged. Router status now records
`odom_age_s` and `plan_duration_s`.

The final no-bridge regression ran for six seconds with `/nav/paused=true`,
Hub goal output disabled, and no `go2-bridge`. It observed 11
`NAVIGATING` messages, zero HOLD messages, maximum odometry age 0.081 s,
maximum planning duration 0.041 s, and a 0.55 m known-free partial route.
The synthetic POI was then invalidated and the router returned to HOLD. The
targeted local suite passed 37 tests.

Superseding implementation and evidence provenance:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 36,442 | `3af0f1c2b6cb79ae2a8662e049ac7b135171a0c8ddfa84c87c286c3a3193158c` | implemented locally; checksum-verified on WSJ |
| `hub/tests/test_tinynav_buildmap_goal_router.py` | 10,430 | `c49656e5a8238316f69f37e52ce83b0d30c3dd386059adea6c98f6e8a3aa1b5e` | locally observed regression source |
| `hub/runtime/oneclick_live_scene01-chair_20260724_003508/episode/episode_report.json` | 9,641 | `5a73fbe778001249e3058da71353c3a4f170392fe9f6ec60561f04a9ca837c53` | observed failed supervised episode |
| `hub/runtime/oneclick_live_scene01-chair_20260724_003508/shadow/shadow_manifest.json` | 19,504 | `7858c6664f07d1e05be2a0fdfee07dddf26d03a8c6aab00fd6fc6a794666f117` | observed real-model live input/decision manifest |
| `hub/runtime/oneclick_live_scene01-chair_20260724_003508/shadow/source_goal_masks/wsj_chair.png` | 1,045 | `4fb493eddab3cdcd9ac2369e235e152ff7ef904df85a22f6d871eea0657021d3` | source-derived projected chair mask |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260723T163457Z.jsonl` | 4,164 | `1066ebef4ed7fbee032a2eb89a5f99c091296e33fbfeb7e0fc25c82df774d8fd` | remotely observed WSJ live event log |
| `/home/nyu/.local/state/topofocus/yunji-v2-live-20260723T163501Z.jsonl` | 4,244 | `2a39fec55c073f8ccba01b4e472fc25e4dbf9744f4c47d2a26abfc947fffbf06` | remotely observed Yunji live event log |

WSJ's root filesystem was observed at zero user-available bytes during
deployment. Only generated, recoverable caches were removed: two deployment
`__pycache__` directories (64 KiB and 92 KiB as observed by `du`), the uv
package cache (1,085,353,984 bytes by `du`, mostly shared hard links), and
`/home/nvidia/.npm/_cacache` (823,812,096 bytes by `du`). No source, map,
calibration, receiver log, or experiment artifact was removed. Final observed
available space was 894,726,144 bytes. This is enough for the next short
receiver log and supervised run, but still not a long-term storage solution.

Formal target arrival, success, and SPL remain unverified. A new exact operator
confirmation is required after WSJ is standing and both robots are clear.

## 2026-07-24 maploc disk recovery and v11 map restart

One exact operator authorization for `scene01-chair/run03` was consumed after
WSJ stood up. The live one-click attempt did not reach command publication:
WSJ's launcher found `maploc` missing while
`online-map/planning/goal-router/control` still existed, and the old all-or-none
launcher refused to replace `online-map`. Fail-closed cleanup restored Hub with
both goal outputs disabled. Read-only checks observed no WSJ Go2 bridge, no WSJ
live receiver, Yunji's debug receiver only, and WATER
`move_status=canceled`. No robot displacement was commanded by this attempt.

The missing `maploc` was traced to a Berkeley DB write failure in:

`/home/nvidia/.local/share/topofocus/maps/buildmap_online_20260723T135024Z.log`

The log ended with `No space left on device`. The corresponding BuildMap
directory was an unfinalized scratch database, not a saved/relocalizable map.
Before deletion, its exact paths, sizes and checksums were observed:

| Remote path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `.../buildmap_online_20260723T135024Z/depths.db` | 5,371,490,304 | `24c4cc20448439f0126c0efa9a7d77fead6bf97e96abcbf8b8a918cbf5a52f9a` | observed failed scratch database; deleted |
| `.../buildmap_online_20260723T135024Z/embeddings.db` | 13,922,304 | `bc45e7cf9ee3394063d3bcf5da418a0c9e83df2a1f1b036736dc78910aa80c69` | observed failed scratch database; deleted |
| `.../buildmap_online_20260723T135024Z/features.db` | 1,755,074,560 | `e8e60f55af93c2b9d43ac1a5ff8a5d8c61d9a95c19beddcc0eb8ee05a4d0619f` | observed failed scratch database; deleted |
| `.../buildmap_online_20260723T135024Z/tf_messages.npy` | 3,344 | `f96afa26b9ed7db143f4fbd6c25c4dd19fed118f16595f89253a906420324acd` | observed failed scratch metadata; deleted |
| `.../buildmap_online_20260723T135024Z.log` | 233,472 | `583365e8cff77d1dc14cf4c7010d297f32366696c8ca6525266271e8c7d45da1` | observed failure log; deleted after checksum |

The directory occupied 7,216,513,024 bytes by `du`. It and only its matching
log were deleted after hashing; they are not locally recoverable. WSJ
user-available space then increased to 8,111,292,416 bytes.

The WSJ launcher now has an explicit fail-closed `--repair-online-stack` path.
It may restore only a missing `maploc` when `online-map`, `planning`,
`goal-router`, and `control` already exist, while `/nav/paused=true`, the
guarded bridge is absent, and no live WSJ receiver exists. Any other partial
window set remains a hard error. The repaired native BuildMap output is
`/home/nvidia/.local/share/topofocus/maps/buildmap_20260723T165937Z`; WSJ then
returned to a complete debug stack.

At the operator's request, a fresh central 2D mapping session was started after
WSJ sequence 21,323 and Yunji sequence 197,103:

- session: `shared_maps_20260724_rebuild_v11_post_maploc`;
- WSJ output: `hub/runtime/map_out_wsj_20260724_rebuild_v11_post_maploc`;
- Yunji output: `hub/runtime/map_out_yunji_20260724_rebuild_v11_post_maploc`.

Both maps observed `mapping_blocked_reason=null`, stable three-frame ground
consensus, zero ground-drift events and zero pose-jump events. WSJ again
detected `chair`. The old v10 daemon was stopped without deleting either v10
output directory. Foxglove port 8765 was switched to v11; its external address
and layout remain unchanged.

The subsequent no-motion full-stack debug run completed
`DEBUG_FULLSTACK_READY`. It selected a 73-cell WSJ chair semantic mask at
shared display centroid `(0.521254,-0.454919)` m and allocated Yunji frontier
B; Hub published HOLD only.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/robot_overlay/start_go2_buildmap.sh` | 3,496 | `d963d4591e9b9a0059c37594335b0546c2ee0c74cf6006f842af1d6e6e6f4754` | implemented locally; checksum-verified on WSJ |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 8,194 | `5800ecf17efb724c3de85e83b87d45a735b78e8def06f3cad1d286f08e4af9d0` | implemented locally; checksum-verified on WSJ |
| `hub/scripts/realworld_oneclick.sh` | 13,289 | `6ff26c34627f3c5f5ae3ecb4b3f868b859aedbbdd822c47ccb80ceec62ce0ec0` | implemented locally; v11 paths selected |
| `hub/runtime/oneclick_debug_debug-v11-post-maploc_20260724_010225/shadow/shadow_manifest.json` | 19,702 | `ffd3f6ee8df31ddc3bfb9cc6fe221ce5c81305866e8e44f8d2252c43b05cc9c1` | observed real-model shadow, no motion |
| `hub/runtime/oneclick_debug_debug-v11-post-maploc_20260724_010225/shadow/source_goal_masks/wsj_chair.png` | 1,052 | `b2060022e1e81a6113adf7698731190ae5fc1cd70cb333f3821b9334bfa0c0e4` | source-derived projected chair mask |

Formal movement, arrival, SR and SPL remain unverified. The consumed run03
authorization is not reusable; a new exact operator confirmation is required.

## 2026-07-24 stale-occupancy bridge and v12 map restart

One fresh exact operator authorization was consumed by
`scene01-chair/run03-retry1`. Strict synchronization passed with WSJ sequence
21,348, Yunji sequence 197,311, 1.846 s capture skew, and source ages of
5.381 s and 7.227 s. The real-model cascade selected an 80-cell WSJ `chair`
semantic region at shared display centroid `(0.521254,-0.454919)` m and
allocated Yunji frontier B at `(-0.528746,-2.354919)` m.

Both local stacks accepted their high-level targets and moved. The controller
observed approximately 0.1780 m of WSJ path length and 0.2631 m of Yunji path
length before WSJ reported
`HOLD/OCCUPANCY_STALE_AFTER_MOTION`. The dual controller then issued HOLD and
automatic cleanup restored Hub with both GOAL outputs disabled, removed the
WSJ Go2 bridge, restored both read-only receivers, and cancelled Yunji's WATER
move. This was a failed engineering attempt, not an SR/SPL result.

Post-attempt, read-only WSJ diagnostics observed the online occupancy input at
approximately 0.08 Hz (about one synchronized geometry pair per 12.5 s), with
zero dropped pairs. The immutable source snapshot specifies a 0.20 m
translation keyframe threshold and a 0.05 m occupancy resolution, while the
deployment router had allowed only 0.10 m of base displacement after its
six-second map-age threshold. The exact causal timing inside the failed
interval was not captured, but the deployed 0.10 m bound was smaller than one
source keyframe interval and the observed rejection occurred at 0.1780 m.

Only deployment code under `hub/` was changed. The WSJ launcher now explicitly
passes a 0.25 m stale-map displacement bound: one 0.20 m source keyframe plus
one 0.05 m grid cell. The router's generic default remains 0.10 m. Unknown and
occupied cells remain non-traversable; the 0.05 m route clearance, 0.18 m
bounded start-footprint override, one-second odometry gate, TinyNav local
planner/controller, guarded bridge, expiring Hub lease, and robot-local stop
authority are unchanged. A displacement beyond 0.25 m still fails closed.
Stale-map HOLD status now includes occupancy age, measured anchor displacement,
configured displacement bound, and map timeout.

The updated files were checksum-verified on WSJ. With Hub GOAL disabled,
`/nav/paused=true`, and no Go2 bridge, a direct no-motion regression observed
`ONLINE_PARTIAL_PATH_READY_CACHED_MAP` after the map exceeded the six-second
age threshold, then invalidated the synthetic POI and returned to HOLD.
Unit regression covers acceptance at 0.24 m and rejection at 0.26 m. The
targeted local suite passed 47 tests.

At the operator's request, the old v11 central map was stopped after its WSJ
ground-drift gate later latched at sequence 21,358. Its artifacts were
preserved. A fresh map was started after WSJ sequence 21,437 and Yunji sequence
197,947:

- session: `shared_maps_20260724_rebuild_v12_router025`;
- WSJ output:
  `hub/runtime/map_out_wsj_20260724_rebuild_v12_router025`;
- Yunji output:
  `hub/runtime/map_out_yunji_20260724_rebuild_v12_router025`.

The first observed stable v12 states had 9 accepted WSJ frames and 18 accepted
Yunji frames. Both reported `mapping_blocked_reason=null`, accepted three-frame
ground consensus, zero ground-drift events, and zero pose-jump events.
Foxglove ports 8765/8766 were switched to
`foxglove_relay_20260724_rebuild_v12_router025`; the external WebSocket address
and layout did not change.

The final no-motion one-click run completed `DEBUG_FULLSTACK_READY`. It used
the v12 maps, selected a 27-cell WSJ `chair` semantic mask at shared display
centroid `(0.821185,-0.214799)` m, allocated Yunji frontier B, published HOLD
only, and confirmed Hub GOAL disabled, read-only receivers, and no WSJ Go2
bridge.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/robot_overlay/start_tinynav_buildmap_online_nav.sh` | 6,512 | `c3f578303d30b77027f74462da984bf370380666a82503c30e1d328ab541274f` | implemented locally; checksum-verified on WSJ |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 37,009 | `0ad57b6935cb3b77d8378a40f6a4e115947bdefa8fe37a01e3ec1a3c2ec12889` | implemented locally; checksum-verified on WSJ |
| `hub/tests/test_tinynav_buildmap_goal_router.py` | 11,359 | `bcaaec29ef43d6f767be29c4152f5554511871a2f8f0da94bba3508bc14eb772` | locally observed regression source |
| `hub/scripts/realworld_oneclick.sh` | 13,281 | `d446972808efce05a98164eb277243d6f05eac01a01f65c25b0699ba04b18ca1` | implemented locally; v12 paths selected |
| `hub/runtime/oneclick_live_scene01-chair_20260724_010416/episode/episode_report.json` | 9,693 | `d270148ff177abd17fed59dc0d51d05483abebd1acc5019fa1aacbf1cb332682` | observed failed supervised episode |
| `hub/runtime/oneclick_debug_debug-v12-router025_20260724_011651/shadow/shadow_manifest.json` | 19,545 | `3d6f49817daf220c82b213b1d3d83f1697ce6b5f813ebdcc06ef56e3c79919a1` | observed real-model shadow, no motion |
| `hub/runtime/oneclick_debug_debug-v12-router025_20260724_011651/shadow/source_goal_masks/wsj_chair.png` | 1,015 | `87354f0dd5049cc1b7bb4612275d07875f8e3c7c929780323ade47685b54d36b` | source-derived projected chair mask |

Formal target arrival, scene success, SR, and SPL remain unverified. The
authorization used by `run03-retry1` is consumed; another physical attempt
requires a new exact operator confirmation.

## 2026-07-24 official-run01 pre-publication IMU gate

One new exact operator authorization was consumed by
`scene01-chair/official-run01`. Strict input synchronization passed with WSJ
sequence 21,470, Yunji sequence 198,227, 0.474 s capture skew, and source ages
of 9.775 s and 9.301 s. The real-model shadow selected a 30-cell WSJ `chair`
region and a two-cell Yunji `chair` region.

No high-level GOAL was published and neither robot began this episode. The
live runtime readiness gate observed fresh WSJ health with
`localization_state=LOST`, `safety_state=HOLD`, and detail
`imu_interval_threshold`. It therefore blocked publication with
`HEALTH_NOT_READY`. Automatic cleanup restored Hub with GOAL disabled for both
robots, removed the WSJ guarded bridge, restored both debug receivers, and
left Yunji cancelled. The operator subsequently reported that the Go2 battery
was depleted.

The deployed source contained a deterministic numeric-policy mismatch:

- the independent sender accepted IMU coverage `>=0.80`, maximum sample gap
  `<=0.05 s`, and interval-end error `<=0.01 s`;
- the WSJ receiver independently recomputed the same fields with
  `>=0.95`, `<=0.02 s`, and `<=0.02 s`.

Thus a report already classified `slam_optimizer_imu_valid` by the sender
could be rejected by the receiver as `imu_interval_threshold`. The exact
failing interval values are unverified because `/slam/data` was no longer
available after the battery loss. Battery depletion may also have affected
the physical stream, but it does not explain or justify the contradictory
numeric policies.

The receiver thresholds now exactly mirror the sender values while still
recomputing every interval field rather than trusting the producer boolean.
Malformed reports, invalid intervals, buffer overwrite, optimizer failure,
two consecutive transient failures, stale local data, and every existing
planner/bridge/lease gate remain fail-closed. Regression tests enforce equality
between the sender and receiver constants and cover the three exact pass/fail
boundaries. The related local suite passed 68 tests. The updated receiver was
checksum-verified in the WSJ deployment directory but has not yet been
restarted or live-validated because the Go2 is unpowered.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/robot_overlay/v2_wsj_receiver.py` | 51,888 | `32f3242547895d8a63a3727208b0995d4ec2d0ed05decca7248727f6d627c27d` | implemented locally; checksum-verified on WSJ |
| `hub/tests/test_v2_receivers.py` | 11,135 | `f9dbb6b5da113a638bbbbc7cbf870d92d7055e4129752fa3475aaa8bd819b7c8` | locally observed regression source |
| `hub/runtime/oneclick_live_scene01-chair_20260724_011935/shadow/shadow_manifest.json` | 21,837 | `65fef32f735a89ff2f46c0eed209ba51735429cdb08b40caeac902e1374aaf07` | observed frozen real-model input and decision manifest |
| `hub/runtime/oneclick_live_scene01-chair_20260724_011935/episode/runtime_readiness.json` | 2,240 | `7423ff7eed1c8b79ff2c452dbca41eec32870524ae5cf4c73baf8465e0b9e9d1` | observed pre-publication runtime rejection |
| `hub/runtime/oneclick_live_scene01-chair_20260724_011935/episode/preflight_report.json` | 7,885 | `aa8a7b4dcb66a212733e031e1a068bda9f44d43011eeee2dd4a64432ea64aa16` | observed candidate-only preflight; no commands sent |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260723T171925Z.jsonl` | 354 | `f8147f41b4f5544d6890bdb1b4e0fd333ae047c066b66725129254388eddd225` | remotely observed WSJ live startup log |

This authorization is consumed. After charging and powering the Go2, the
updated debug receiver, live SLAM health, shared-frame pose, and v12 map must
be revalidated without motion before requesting another operator
authorization.

## 2026-07-24 post-power-cycle no-motion validation

After the operator reported that the Go2 was powered again, all checks used
the existing `focus_wsj_tunnel_20260722:sensor-audit` SSH/tmux pane. WSJ
published both `/slam/data` and `/semantic_mapping/occupancy_bev`; the observed
short `/slam/data` sample rate was approximately 1.5--2.1 Hz. The WSJ debug
receiver was restarted and loaded the checksum-verified threshold fix. It ran
without `--enable-live-go2-motion`; the observed Go2 bridge count and live
receiver count were both zero.

The fresh sender observation reported
`slam_optimizer_imu_valid;covariance_unavailable`. The v12 WSJ map resumed at
sequence 21,609 with `mapping_blocked_reason=null`, zero pose-jump events, and
zero ground-drift events. Relative to the frozen pre-power-cycle input, the
latest shared camera pose changed from `(1.8452, 1.2088)` m at
`-122.83 deg` to approximately `(1.8906, 1.2456)` m at `-120.96 deg`: about
0.058 m and 1.87 deg. This is an observed small same-placement discrepancy,
so the existing shared-frame calibration remains the current candidate; no
new board calibration was performed.

A complete no-motion one-click run returned `DEBUG_FULLSTACK_READY`. It used
the real VLM path, produced a 74-cell WSJ `chair` region and a four-cell Yunji
`chair` region, published versioned HOLD decisions only, and updated
Foxglove. Hub health still reported GOAL output disabled for both robots.
Therefore no physical robot command was issued by this validation.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-debug-20260723T173933Z.json` | 3,137 | `43a57890ccb9c1880b0acd68e1072bfe296266ca295a10cae6aded16453fc835` | remotely observed fresh alignment artifact; no motion |
| `hub/runtime/map_out_wsj_20260724_rebuild_v12_router025/live_status.json` | 4,392 | `a17070644ce6b89d45275f75be9328c0db5fe48ce763dae448f7a1431a6e8328` | locally observed post-power-cycle map status snapshot |
| `hub/runtime/oneclick_debug_post-powercycle-check_20260724_013942/shadow/shadow_manifest.json` | 21,998 | `bcba7f15506540ef6c30bea57945630278c844b60ddc5cfdb0182a4daa01598d` | observed real-model shadow manifest; HOLD only |
| `hub/runtime/oneclick_debug_post-powercycle-check_20260724_013942/shadow/source_goal_masks/wsj_chair.png` | 1,049 | `6d1462f01c1840cc0475129df46caf64dd42b0931252af414727b39a1e46077c` | model-inference map projection, unverified |

The debug receiver intentionally does not post a command-readiness heartbeat.
Consequently the patched receiver's live heartbeat, physical motor state,
actual movement, arrival, SR, and SPL remain unverified. Starting the live
receiver and guarded bridge requires a new exact operator confirmation; the
runtime gate will recheck health before any GOAL publication.

## 2026-07-24 official-run01-retry1 health-source race

The operator supplied a new exact dual-robot confirmation for
`scene01-chair/official-run01-retry1`. Strict synchronization passed at WSJ
sequence 21,627 and Yunji sequence 199,551 with 1.838 s capture skew. Both
robot-local heartbeats were `READY/TRACKING`, both high-level semantic GOALs
were accepted, and both local planners reported `NAVIGATING`. The operator
observed Yunji movement and a smaller WSJ movement. The receivers reported
0.3875 m of WSJ odometry path and 0.5745 m of Yunji odometry path before HOLD;
these are localization-derived path values, not independently measured ground
truth.

WSJ was not suppressed by a minimum-speed gate. The remotely observed
controller has a 0.10 m/s minimum effective nonzero linear command, and the
bridge has a 0.01 m/s deadband followed by a 0.10 m/s nonzero command floor.
The live launcher capped WSJ at 0.20 m/s linear and 0.50 rad/s angular. The
operator-observed physical motion additionally proves that the GOAL, TinyNav
controller, guarded topic, and Unitree bridge path all became active. Exact
per-command `SportClient.Move()` values were not preserved for this attempt
because the bridge tmux window was removed during fail-closed cleanup.

After approximately 71 seconds, lease 14 received `409 Conflict`. The
preserved Hub decision log gives the exact rejection:
`robot health does not permit a GOAL`. The race was reconstructed from
observed timestamps:

- a fresh receiver heartbeat had established command-ready health;
- WSJ RGB-D sequence 21,645 arrived at
  `1784828584157931641 ns` with its deliberately mapping-oriented
  `UNKNOWN/DEGRADED` observation health and
  `slam_optimizer_imu_valid`;
- the next renewal arrived about 61 ms later;
- `HubRegistry._freshest_health()` selected whichever transport had the later
  receive timestamp, so the RGB-D metadata transiently displaced the still
  fresh robot-local heartbeat.

This was a Hub health-authority race, not a network failure and not a Go2
velocity threshold. The rejected renewal triggered the intended fail-closed
path: a dual HOLD batch was accepted, both receivers reported `HOLDING` with
`velocity_zero_confirmed=true`, the WSJ bridge was removed, Yunji was
cancelled, both debug receivers were restored, and Hub returned with GOAL
disabled. This attempt is not an SR/SPL trial result.

Only `hub/` deployment code was changed. Once a robot-local heartbeat has
appeared in a Hub process, it is now the command-health authority for that
process. A later RGB-D observation cannot replace it between heartbeat ticks.
If that heartbeat becomes stale, Hub fails closed and does not fall back to
observation health. Regression tests cover both the race and the stale
heartbeat case; the related suite passed 88 tests. The WSJ live launcher now
persists the bridge's rate-limited `Move(vx, vy, wz)` log so the next attempt
has exact command evidence. The launcher was checksum-verified on WSJ and
passed remote `bash -n`.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/registry.py` | 17,245 | `99373ff8c356f5c096efc7a3195484d1f7a8b368035a92facc8f88e63f6491bc` | implemented locally; heartbeat authority remains fail-closed |
| `hub/tests/test_registry.py` | 13,811 | `86526132e5f478f227b5d3554e1a1b7d854640b131bf1dd89b30d4f2a82a11d1` | locally observed regression source |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 8,406 | `583882b8761368e2041cf48548273d6faedbb14e6928bface9b6cd676dc9423b` | implemented locally; checksum-verified on WSJ |
| `hub/runtime/oneclick_live_scene01-chair_20260724_014136/shadow/shadow_manifest.json` | 21,786 | `ebf3795240a91e3b98e3d57c871368460cc80c135d681ed4cd0e505bf5373f77` | observed frozen real-model decision input |
| `hub/runtime/oneclick_live_scene01-chair_20260724_014136/episode/controller_events.jsonl` | 9,512 | `649d01baf5d0f49f0a229716330bd6fe50990bb1cbd4a5586b0edf578b6d8295` | observed lease timeline and fail-closed HOLD |
| `hub/runtime/oneclick_live_scene01-chair_20260724_014136/episode/episode_report.json` | 11,805 | `70af2a2f35c8b567da5abc0dc76b7048bbe18ce9e924c76468a257e9a0d0bc39` | observed failed engineering attempt; not SR/SPL |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260723T174121Z.jsonl` | 44,350 | `810881ff8c651643c9b0ffcd60533dddcdaf22f64ad321e978e69e96f7b8bc7b` | remotely observed WSJ receiver decision/event log |
| `/home/nvidia/twork/tinynav/tinynav/platforms/cmd_vel_control.py` | 10,478 | `ea67c986934232b6ae42ffaca239dce21e3136efa7f133defb3037addde5350d` | remotely observed deployed TinyNav controller; unchanged |
| `/home/nvidia/twork/tinynav/scripts/run_go2_cmd_bridge.sh` | 3,514 | `f0d06edb8d1ac59b497aac77b099927d415baf63700a6cd73d240cbd0d7b9c21` | remotely observed deployed bridge launcher; unchanged |
| `/home/nvidia/twork/tinynav/tool/go2_cmd_bridge.py` | 11,678 | `8b81107f89ed4013529f325f75a80b39295e828a67e2e1d87c432d860f19ebb2` | remotely observed deployed Unitree bridge; unchanged |

The confirmation for this attempt is consumed. A new live attempt requires a
fresh operator confirmation after both robots are clear.

## 2026-07-24 official-run01-retry2: actual v2 registry path identified

The prior `HubRegistry` change was necessary for the v1 publication path but
was insufficient for the live experiment. The supervised controller publishes
atomic pairs through `V2DecisionRegistry`, which had a second, independent
`_freshest_health()` implementation with the old receive-timestamp race. This
corrects the retry1 diagnosis: the health-authority rule was right, but it had
initially been applied to only one of two registry implementations.

The operator supplied a fresh exact confirmation for
`scene01-chair/official-run01-retry2`. Strict synchronization passed at WSJ
sequence 21,689 and Yunji sequence 199,941 with 1.912 s capture skew. The
real-model run projected an 87-cell WSJ `chair` mask and a 19-cell Yunji
`chair` mask. Both initial GOALs were accepted; Yunji immediately reported
arrival and transitioned to HOLD, while WSJ remained active.

The bridge log preserved the first exact WSJ command evidence. The controller
spent most of the attempt rotating in place, then issued only about one second
of forward commands: `vx=0.100` followed by repeated `vx=0.148`, with
`wz=-0.449`. The receiver accumulated 0.1236 m of localization-derived path.
Lease four then received the same generic 409 health rejection because the
actual v2 registry still allowed a newer RGB-D observation to replace the
receiver heartbeat. Automatic dual HOLD and debug restoration succeeded.
This engineering attempt is not an SR/SPL trial.

`V2DecisionRegistry` now applies the same fail-closed authority rule: once a
command-receiver heartbeat has appeared, RGB-D health cannot replace it; an
expired heartbeat remains expired and cannot fall back to observation health.
Rejection text now identifies the robot and the exact safety/localization,
estop, collision-avoidance, motor and detail fields. The critical v2/v1/API
suite passed 32 tests and the remaining related receiver/runtime/router suite
passed 66 tests, for 98 related tests total. The Hub process was restarted
from this source before retry3.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/v2_registry.py` | 16,139 | `971fcb459416a38dd81ef44ec272c3909c30219dbe340c2ebcb90ea0aaeff483` | implemented locally and loaded by the Hub before retry3 |
| `hub/tests/test_v2_registry.py` | 14,325 | `f7c5b3017fd2dd00e97efd8d5fe0151a2be04dc24f62e89086ba342b622b1c76` | locally observed regression source |
| `hub/runtime/oneclick_live_scene01-chair_20260724_015031/shadow/shadow_manifest.json` | 21,768 | `8982b228645639fd9b389f3895ddd4ae6eac2d839003d6722468971ecb73b809` | observed frozen real-model decision input |
| `hub/runtime/oneclick_live_scene01-chair_20260724_015031/episode/controller_events.jsonl` | 3,266 | `43dd4c1f9e281d66c8381e8c7176ae0887d8389d936fb3eaa1b38fbaa144bb5f` | observed lease timeline and fail-closed HOLD |
| `hub/runtime/oneclick_live_scene01-chair_20260724_015031/episode/episode_report.json` | 9,390 | `6050402ecee86e4d36a831050c9fe32ccdeaa691a457fb09722e9f76c44be2cd` | observed failed engineering attempt; not SR/SPL |

## 2026-07-24 official-run01-retry3: v2 fix validated, effective-speed issue isolated

The operator supplied another exact confirmation for
`scene01-chair/official-run01-retry3`. Strict synchronization passed at WSJ
sequence 21,733 and Yunji sequence 200,236 with 3.976 s capture skew. The
real-model run projected a 99-cell WSJ `chair` mask and a 27-cell Yunji
`chair` mask. The initial pair, the Yunji arrival transition and all following
lease renewals were accepted: the prior Hub health race did not recur,
providing real-stack evidence that the v2 registry fix was active.

Yunji again classified itself as already inside its target arrival region and
transitioned to HOLD with only 0.0004 m of localization-derived path. WSJ
remained `NAVIGATING` until its local online router reported
`ODOMETRY_STALE`, at which point the receiver rejected the active leg,
confirmed zero velocity and the controller published the final dual HOLD.
Nine high-level batches were published in total. The report contains 0.2156 m
of WSJ localization-derived path, but the onsite operator observed no physical
motion; the path value must therefore be treated as SLAM drift, not measured
robot travel.

The persistent bridge log explains the lack of visible motion. It contains
151 `SportClient.Move()` calls and no SDK exception, but every call was
rotation-only at `vx=0.000`, `wz=-0.200 rad/s`; it contains zero forward
commands. The user observation is authoritative. The run is not a successful
trial and contributes neither SR nor SPL.

Two minimal follow-up changes were implemented locally after cleanup:

- the guarded WSJ launcher raises only the nonzero command floors to
  `0.15 m/s` linear and `0.30 rad/s` angular while preserving the existing
  hard maxima of `0.20 m/s` and `0.50 rad/s`;
- the online router gives odometry and large occupancy-grid conversion
  separate mutually exclusive callback groups and a three-thread executor,
  while retaining the one-second stale-odometry fail-closed timeout.

The focused router/receiver/runtime/registry suite passed 51 tests. The Go2
lost power before these two WSJ-side files could be transferred, checksum
verified or loaded. They are therefore implemented and locally tested, but
not remotely deployed or physically validated.

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_live_scene01-chair_20260724_015709/shadow/shadow_manifest.json` | 21,773 | `e31ed353d144ac644265622154f7e1754d57b082a749c89c22a5039a296a56d3` | observed frozen real-model decision input |
| `hub/runtime/oneclick_live_scene01-chair_20260724_015709/episode/controller_events.jsonl` | 5,475 | `a9384b83f25acfd9e55db462ac8abd4156d848b35d74a0191d73a204bc551ce9` | observed nine-batch controller timeline |
| `hub/runtime/oneclick_live_scene01-chair_20260724_015709/episode/episode_report.json` | 9,391 | `f049dc37f6bf3d7cb1fa519380d7641620016dee6f1a1c0bd9aaa8b1a0e6768a` | observed failed engineering attempt; not SR/SPL |
| `/home/nvidia/.local/state/topofocus/wsj-go2-bridge-20260723T175658Z.log` | 13,537 | checksum unverified after power loss | remotely observed exact rotation-only command log |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 37,147 | `4461bf961f307c9efb36158b5a928032e7131b32f754fa1f4be24b925255725c` | implemented and tested locally; not yet deployed to WSJ |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 8,444 | `4bc1fe9a16cc08c569080c98c6b40024fc2bfec1317d0a1e4365fa7c2e66027e` | implemented and tested locally; not yet deployed to WSJ |
| `hub/tests/test_tinynav_buildmap_goal_router.py` | 11,535 | `d56be72f034e0f55703b816ff128b491f9afba17b561cf68e1e1c7f266f9e593` | locally observed regression source |
| `hub/tests/test_v2_receivers.py` | 11,479 | `afaf84b1261a4846e601c77f5a1f187c1175d65f5a9cd016b7da853396bee0b0` | locally observed regression source |

At end of day, local Hub health reported GOAL output disabled for both robots,
there was no active one-click/live episode controller process, and the WSJ Go2
was unpowered. All operator confirmations used above are consumed. The next
session must first deploy and checksum-verify the two pending WSJ files,
restart only the read-only router/debug receiver, revalidate fresh odometry
and calibration without motion, and then obtain a new exact operator
confirmation.
