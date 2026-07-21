# Yunji WATER robot integration — observation sender + dry-run command guard

Date: 2026-07-18. This is the second robot's integration work, structurally
different from wsj's Unitree Go2. It is **not** G4 evidence: G4 needs a
physically verified `T_shared_world_robot_map` for each robot and this one
robot alone proves nothing about shared-frame fusion. It also does not touch
actuation: the command side is dry-run only, never wired to a real API call.

## The robot

云迹 WATER (Yunji Water) delivery chassis, product id `WTHT08E35C1338838`,
reachable at `192.168.10.10` over a USB-Ethernet link from `nyush-nuc`
(network access set up and made persistent earlier the same day — see
`~/workspace/tinynav/yunji-water-robot/README.md` section 17 on nyush-nuc).
ROS1 (Cartographer + AMCL + move_base) behind a documented vendor TCP API
(31001) — completely unlike TinyNav/Go2's live ROS2 topic stream. No GPU on
nyush-nuc, so this machine can only be a sender; mapping/decision stay
centralized on the hub as with wsj.

## What the sender does (`hub/robot_overlay/yunji_sender.py`)

Reads three real sources, all read-only:

- WATER TCP API (31001): `/api/robot_status` polled once per keyframe for
  `current_pose` (x, y, theta) and real telemetry (battery, e-stop,
  error_code, move_status).
- rosbridge WebSocket (9090): `/camera_front_up/rgb/image_raw` (rgb8) and
  `/camera_front_up/depth_registered/image_raw` (32FC1) fetched one-shot per
  keyframe with a hardened minimal WebSocket client (correct FIN/opcode
  parsing, so it doesn't wedge on a ping frame or a large payload the way a
  naive single-frame-only reader would).

Never calls `/api/move`, `/api/joy_control`, or anything else that could
move the robot.

## Approximations, stated plainly

Two of the three items below were originally unmeasured placeholders and
were later replaced with real values — see "Second pass" below for how.

- **Depth unit was empirically determined, not assumed.** The raw ROS
  message is tagged `32FC1` (nominally metres by ROS convention), but its
  numeric range (275–890) only makes physical sense as millimetres. Cross-
  checked against `/camera_front_up/depth/points` (a `PointCloud2`, always
  metres per REP103): the point cloud's z range was `[0.2750, 0.8740]` —
  exactly the raw image values ÷ 1000. Confirmed millimetres; the sender
  divides accordingly.
- `transform_version` is the explicit test label `yunji-water-robot-test-v1`
  (never a real `shared_world` calibration) and every upload is
  `mapping_only=true`. `robot_id=robot-1`, already present in the hub's
  shipped default config with `allow_goal=false`.

## A real debugging story: the HTTP snapshot endpoint was a dead end

The chassis also exposes RGB (and a colorized depth preview) as JPEG
snapshots over plain HTTP (`web_video_server` on port 8810,
`GET /snapshot?topic=...`). This looked like the simpler path and was tried
first. It was not reliable, and the investigation is worth recording so it
isn't repeated:

- `requests` and `urllib.request` both intermittently raised
  `RemoteDisconnected: Remote end closed connection without response`.
- Forcing `http.client.HTTPConnection` to send an HTTP/1.0 request line
  fixed it in one clean test run (5/5), which looked like a real root cause
  (a protocol-version mismatch with the server's `HTTP/1.0 + Connection:
  close` replies) — but did not reproduce reliably in further testing.
- Delays between the TCP-API call and the HTTP fetch (tested 0.5 s up to
  10 s) made no measurable difference.
- Shelling out to `curl` was 100% reliable across roughly a dozen calls
  early in the investigation, including interleaved with TCP-API calls —
  looked like the fix — and then failed with `rc=52` ("empty reply from
  server") on a later, otherwise-identical run.

No client-side fix was found because there wasn't a single deterministic
cause to find: the endpoint itself is intermittently unreliable regardless
of client. rosbridge (9090), in contrast, was reliable in every trial for
the depth topic across the whole investigation, so **RGB is fetched over
rosbridge instead of HTTP**, using the same fetch mechanism as depth. Every
network call to the robot is additionally wrapped in `retry_call()` (short
exponential backoff, defense in depth, not the primary fix).

## Result: 8/8 real frames end to end

```bash
FOCUS_ROBOT_TOKEN=... python3 yunji_sender.py \
  --base-url http://127.0.0.1:18089 --rate-hz 1 --max-frames 8
```

Against a local test hub (loopback, tunneled to nyush-nuc, random test
token, `robot-1` policy `allow_goal=false`): **8/8 frames accepted, 0
retries needed**, ~18 ms mean hub upload. Spot-checked the spooled data:

- Real 160×120 RGB and depth, depth range `[0.275, 0.890]` m matching the
  point-cloud cross-check.
- Real pose matrix composed from the chassis's actual `current_pose`
  (translation `(1.018, -0.369)`, rotation matching `theta≈1.428` rad).
- Real health: `battery_percent=100.0`, `estop_engaged=false`,
  `safety_state=READY` — genuine telemetry, better signal than wsj's replay
  sender ever had (which could only send `UNKNOWN` placeholders).
- Fed through the same `SpoolMappingPipeline` + RedNet used for G3: 8/8
  frames processed, no NaNs, sane bounded values (287 explored / 287
  obstacle cells — small because 8 frames from a nearly-stationary robot,
  not a red flag).

At this point, pose height (0.5 m) and RGB/depth registration (resize) were
still explicitly labelled placeholders — the next section replaces both with
real measured values.

## Second pass: real extrinsics from `/tf` (same day, after the above)

The first working sender used a placeholder camera height (0.5 m, zero
tilt) and downsampled RGB onto depth's grid with `cv2.resize` — both
explicitly labelled approximations, because an earlier `/tf_static` probe
had returned nothing. On a second look, plain `/tf` (not `/tf_static`) does
carry the same static mount edges as ordinary messages on this firmware;
subscribing for 20 s and aggregating distinct edges (1154 messages, 19
distinct `frame_id -> child_frame_id` pairs) recovered the full chain:

```
base_link -> camera_front_up_link            t=(0.2646, 0, 0.299) m, real rotation
camera_front_up_link -> ..._depth_frame       identity
..._depth_frame -> ..._depth_optical_frame    identity translation, standard optical rotation
..._depth_frame -> ..._color_frame            identity  (depth and color are CO-LOCATED)
..._color_frame -> ..._color_optical_frame    same standard optical rotation as depth's
```

Two consequences, both now implemented in `yunji_sender.py`:

1. Composing the edges confirms `camera_front_up_depth_optical_frame` and
   `camera_front_up_color_optical_frame` are related by an **exact identity
   transform** — RGB and depth differ only in intrinsics/FOV, not viewpoint.
   `reproject_rgb_onto_depth_grid()` replaced the `cv2.resize` approximation
   with a proper per-pixel reprojection: backproject each valid depth pixel
   through depth's intrinsics, project through RGB's own intrinsics to
   sample color. Result: 79.8% of the 160×120 grid gets a real per-pixel RGB
   sample (vs. ~85% valid-depth baseline; the gap is plausible sensor-edge
   effects, not a bug).
2. `base_link -> camera_front_up_depth_optical_frame` composes to
   `MEASURED_T_BASE_LINK_CAMERA` (translation `(0.2646, 0, 0.299)` m, a real
   tilt — not the flat 0.5 m guess). `pose_to_matrix()` now composes
   `T_map_camera = T_map_baselink(x, y, theta, z=0) @ MEASURED_T_BASE_LINK_CAMERA`
   using this real matrix. The `--camera-height-m` flag was removed —
   nothing left to override.

Remaining unverified assumption, stated plainly: `current_pose`'s `z=0`
(base_link on the floor plane in the chassis's own map frame) is the
standard convention for a wheeled ground robot's SLAM pose, not
independently measured on this unit.

Re-validated end to end after this change: 6/6 frames accepted (0 retries),
spot-checked — camera translation `(1.056, -0.108, 0.299)` exactly matches
the real measured height composed with the chassis's live 2D pose, 79.8%
non-black reprojected RGB coverage on the depth grid.

## Third pass: pose source switched to live odometry (2026-07-19)

Both earlier passes above used `current_pose` from `/api/robot_status` —
AMCL's pose in the chassis's own `map` frame, which only exists once a
saved map has been built and loaded (`/api/map/set_current_map`). That is
a real pose, but it silently assumes "build a map first, then
navigate/localize within it" — the opposite of what the original
Habitat/HPC codebase this project reproduces actually does. Upstream,
every episode resets both agents to the *same* `episode.start_position`
(only rotation randomized — `merge_sim_episode_config` in
habitat-lab/habitat/tasks/nav/nav.py:104-131) and each agent's
`EpisodicGPSSensor` reports pose relative to its own t=0 state, not
against a pre-built map. Faithfully reproducing that on real hardware
means each robot tracks its own pose from scratch, live, with no
pre-built-map dependency — not a "practical service robot" reframing,
a direct correction back to what the source actually does.

The chassis separately exposes `/sensors_fusion/odom` (fused IMU + wheel +
laser + visual odometry, `frame_id=odom`, `child_frame_id=base_link`, no
saved-map dependency). `yunji_sender.py` now fetches this via the same
rosbridge one-shot mechanism already used for RGB/depth, and composes
`T_odom_camera = T_odom_baselink(from /sensors_fusion/odom, full SE(3)
from the real quaternion, not a 2D x/y/theta-only approximation) @
MEASURED_T_BASE_LINK_CAMERA` — replacing the old 2D-only `pose_to_matrix`.
`--transform-version` default became `yunji-water-robot-live-odom-test-v1`.

Live-tested end to end against a fresh isolated test hub (own port/session/
tokens, torn down after): **6/6 frames accepted, 0 retries**. Spot-checked
one spooled observation's composed pose translation, `(0.246, -0.013,
0.299)`, against the raw `/sensors_fusion/odom` reading captured moments
earlier (`position=(-0.018, 0.0008, 0)`, `yaw≈-0.053 rad` from the
quaternion) composed analytically with `MEASURED_T_BASE_LINK_CAMERA`'s
translation `(0.2646, 0, 0.299)`: the analytical prediction matched the
spooled value to 3 decimal places. Health/depth/intrinsics fields
unaffected (still sourced from `/api/robot_status` and the depth topic as
before).

Accepted, unsolved tradeoff, stated plainly: Habitat's GPS/compass ground
truth never drifts across an episode. This live fused odometry does — there
is no loop closure here, only IMU/wheel/laser/visual fusion against its own
un-corrected running estimate. Nothing in this pass corrects for that; it
is a known limitation carried forward, not hidden.

## Shared-frame calibration tool (`hub/tools/calibrate_shared_frame.py`, 2026-07-19)

Companion to the pose-source switch above: once each robot publishes its
own live, map-independent pose, the two robots' pose streams are still each
in their own arbitrary local frame (wherever that robot's odometry stack
happened to initialize) — exactly the gap Habitat closes for free by
resetting every agent to the same `episode.start_position`. The real-world
analogue has to be built: a session-start calibration where an operator
physically co-locates the two robots (or measures a known offset between
them) and this tool converts that one shared instant into a fixed transform.

Core math lives in `hub/src/focus_hub/calibration.py`
(`compute_shared_frame_transform` / `apply_shared_frame_transform`), pure
and unit-tested (5 new tests in `tests/test_calibration.py`: identity sync,
round-trip recovery of the reference pose from a coincident sync, a later
motion mapped through the calibrated frame with a hand-verified expected
translation, a non-coincident sync honoring a measured offset, and
malformed-input rejection). Added `compose_rigid` to the existing
dependency-free `geometry.py` (already used by both `GoalGuard`s) alongside
`invert_rigid`/`transform_point`, rather than pulling numpy into that
module.

The CLI (`hub/tools/calibrate_shared_frame.py`) reads each robot's most
recently spooled observation, refuses to proceed if the two capture
timestamps are more than `--max-sync-skew-s` (default 5s) apart — fail
closed, since two observations captured seconds apart are not evidence the
robots were actually co-located — and writes a versioned JSON calibration
file. Smoke-tested against synthetic spooled observations: round-trip
recovery of the reference pose matched to floating-point precision, and the
skew guard correctly refused a 0.2s-apart pair under a 0.05s threshold
(exit 1).

`yunji_sender.py` gained a `--shared-frame-transform-file` flag that loads
this calibration and applies it (`shared_frame_transform @ T_odom_camera`)
to every published pose before upload; omitting it (the current default,
since no real cross-robot sync has been performed yet) leaves poses in the
robot's own local odometry frame, same as the third pass above. Verified
in isolation with a synthetic 45°-rotation + translation calibration
matrix: the composed pose matched the hand-computed expected value exactly.

**wsj's sender (`focus_ros_sender.py`) does not have this pose-source
switch or the `--shared-frame-transform-file` flag yet** — blocked on wsj
coming back online, since the GitHub mirror of its repo could not be
confirmed byte-identical to what's actually deployed there (see
`experiment.md`, 2026-07-18 entry, and the still-`pending` task tracking
this). A real dual-robot calibration run additionally requires an operator
to physically co-locate both robots and capture observations from both at
that moment — not yet scheduled.

## Fourth pass: sensor audit + real health signal + soak + fault injection (2026-07-19)

Triggered by a user question ("are these all stereo RGBD cameras, does Yunji
use multiple sensors?") that prompted re-checking the robot's actual `/tf`
tree rather than relying on what earlier passes had already documented.

**Sensor audit.** A fresh 15s `/tf` capture found two camera-related frames
never previously recorded: `camera_front_down_depth_optical_frame` (a second,
unused depth camera) and `vio_camera_proj_frame` + `vio_imu_link` (a separate
visual-inertial pair). Cross-checked against the vendor README (section
7.2, previously unread past the AMCL paragraph): `mfo_estimator` provides
visual/IMU odometry from this camera, and `sf2` fuses it with wheel/IMU/RF2O
laser odometry at ~20 Hz into `/sensors_fusion/odom` — our live pose source
since the third pass. So the pose we use has a real visual component from a
camera we'd never inspected, but it is a vendor-standard, documented part of
the robot's own navigation stack (not an experimental/unused sensor) — this
substantially de-risks the earlier open question about it. No separate
confidence/health topic exists for this pipeline beyond the fused message's
own `covariance` field, confirmed real and non-zero by sampling 10 live
messages (`~4e-6 m²` / `~1e-5 rad²` at rest, fluctuating cycle to cycle —
not a frozen placeholder). `/rosapi/topics` and 20+ guessed topic names were
tried while mapping this out; most produced nothing (confirmed genuinely
absent, not a probing bug, since `/camera_front_down/depth/camera_info` — a
working control — responded immediately with real content on the same
probe).

**`localization_state` was fake — replaced with the real covariance.**
Previously `"TRACKING" if status.get("move_status") is not None else
"UNKNOWN"` — true almost any time the TCP API merely responded, not a real
tracking-quality signal, despite `RobotHealth.ready_for_goal()` gating GOAL
issuance on exactly this field. `classify_localization_state()` now
thresholds `/sensors_fusion/odom`'s own covariance (TRACKING/DEGRADED/LOST
by position/yaw variance) and the real covariance is also now propagated
into the wire's `pose.covariance_6x6` field, which had separately been
hardcoded to all zeros since the first pass. Caveat stated up front in code:
the threshold *values* are a reasonable order-of-magnitude guess, not
calibrated against a real degraded/lost event — only a healthy, stationary
baseline has actually been observed; DEGRADED/LOST paths are exercised only
by unit-level synthetic covariance, not a real failure.

**Hardcoded camera intrinsics investigated, not fixable by live-fetch.**
Tried fetching `/camera_front_up/{depth,rgb}/camera_info` (and several
naming variants) over a 20s window — zero responses, while the sibling
`/camera_front_down/depth/camera_info` answered immediately, confirming this
is specific to the front_up driver on this firmware, not a probing bug. So
`K_DEPTH`/`K_RGB` stay hardcoded (their original provenance is undocumented
anywhere in this workspace — an honest gap, not papered over), but gained a
runtime guard: `verify_intrinsics_match_frame_size()` fetches a live
depth/RGB frame's actual resolution once at startup and raises loud if it no
longer matches what the constants were evidently calibrated for (confirmed
live: depth 160×120, RGB 640×360, matching the constants' implied principal
points exactly), instead of silently feeding stale intrinsics forever if a
firmware update ever changes resolution.

**Stale comment fixed.** `yunji_goal_guard.py`'s yaw-handling comment
blamed "the sender's pose composition is already coarse" — no longer true
since the third pass's real extrinsics. Reworded to describe the actual gap
correctly: `point`'s x/y/z is rotated through `shared_T_robot_map` via
`transform_point`, but `yaw_rad` is passed straight through unrotated — a
real, still-open approximation, currently dormant only because
`shared_T_robot_map` is still identity/placeholder.

**Sustained soak (600 frames, 17.9 min, fetch-bound ~0.56 Hz against a
requested 5 Hz).** 600/600 accepted, 0 sequence gaps, 0 duplicates, only 1
transient upload retry (self-recovered), `localization_state` and battery
stable throughout. Evidence: `data/robot_replays/yunji_soak_20260719/`.

**Transport fault injection**, using a new dependency-free raw-TCP proxy
(`tcp_proxy.py`, same technique as wsj's — an SSH tunnel kill doesn't
reliably interrupt an established connection because of `ControlPersist`,
so the proxy sits between sender and hub instead and is killed/restarted
directly):

- *Scenario A — dead at startup*: sender started against an unlistened
  proxy port; first upload retried for 6 attempts / 23.6s until the proxy
  came up, then succeeded; remaining 4 frames uploaded normally. 5/5
  accepted.
- *Scenario B — killed mid-run*: proxy killed after 5 frames had already
  gone through; the 6th frame's upload exhausted its full ~39.5s budget and
  the outer per-cycle handler logged `cycle failed (1/5): giving up on seq
  10 after 9 attempts` — genuinely failed, not silently retried forever.
  The *next* loop iteration re-attempted the same (unincremented) sequence
  fresh, and by then the proxy had been restarted, so it succeeded after
  15.55s of its own backoff. 8/8 frames ultimately accepted, 13 retries
  total, one genuine give-up-then-recover cycle — more thorough evidence
  than originally planned, since the timing landed on the give-up path
  rather than a clean single-retry recovery.
- *Scenario C — persistent outage*: proxy never restarted,
  `--max-consecutive-failures 2` (shortened from the default 5 to keep the
  test under 2 minutes instead of ~200s — same code path, just a lower
  threshold). Two consecutive full-budget give-ups (`giving up on seq 0
  after 9 attempts`, ×2, ~79s total), then `too many consecutive failures;
  aborting` and the process exited **1** (non-zero) — fail-loud, not a
  hang, 0 frames falsely reported as sent.
- Hub-side spool confirmed 13 total observations across all three
  scenarios (sequences 0–12), zero gaps, zero duplicates — scenario C
  correctly contributed nothing since it never succeeded.
- Evidence: `data/robot_replays/yunji_fault_injection_20260719/`.

All test infrastructure (test hub, tunnel, proxy, sender processes) confirmed
torn down and both machines clean after every scenario in this pass.

## Independent 2Hz heartbeat channel (2026-07-19, live-validated)

New hub-side wire model (`RobotHeartbeat`/`HeartbeatAck` in `models.py`) and
endpoint (`POST /v1/robots/{robot_id}/heartbeat`), separate from the RGBD
observation upload path entirely — no images, no pose, no map data, just
`{robot_id, sent_time_ns, health}`. `HubRegistry` now tracks heartbeat state
alongside (not instead of) observation state, and `publish_decision`'s
GOAL-gating uses whichever of the two channels reported most recently
(`_freshest_health`) for the staleness and readiness checks — an unhealthy
heartbeat can now block GOAL even over a cached-healthy observation, and a
fresh heartbeat can keep health non-stale even if the RGBD path itself goes
quiet. Heartbeat state is deliberately not persisted across hub restarts,
matching the existing fail-closed pattern for `last_observation`. 6 new
unit tests (5 registry, 1 API).

Sender-side: `yunji_sender.py` gained a real background `HeartbeatThread`
(a genuine OS thread, not just a faster loop iteration) that independently
polls the fast WATER TCP API (`/api/robot_status`, millisecond-scale) on
its own connection and posts a heartbeat every 0.5s by default —
decoupled from the RGBD fetch/encode/upload cycle, which is fetch-bound at
~1.8s/cycle (see the soak section above). `localization_state` in the
heartbeat reuses whatever the main loop most recently computed (via a
thread-safe `LatestLocalizationState` holder) rather than re-deriving it
independently, to avoid a second concurrent rosbridge connection just for
this.

**Live-tested** against a fresh isolated test hub: ran the sender at a
deliberately slow `--rate-hz 0.3` (so RGBD frames arrive far apart) with
`--heartbeat-hz 2.0` for ~10s — 3 RGBD frames sent (fetch-bound cadence
unaffected) alongside **21/21 heartbeats accepted, 0 failures**, confirmed
via the hub's own request log (all `200 OK`, no errors) and a manual direct
`curl` POST to the endpoint. This is real evidence the heartbeat thread
runs at its own independent cadence rather than being coupled to (or
starved by) the slow RGBD path. Test hub/tunnel torn down and confirmed
clean after.

## Investigated: does Yunji have an online-loop-closure pose available? Not in its current running state, at first (2026-07-19)

Follow-up to the same question asked about wsj (see `audit/LIVE_ROS2_SENDER.md`,
"does TinyNav support build-online-while-closing-loops"): Yunji's own vendor
README (section 7.1) confirms the chassis's mapping stack is Cartographer —
a real SLAM system with genuine online loop closure, architecturally the
same kind of capability wsj's `BuildMapNode` provides. The question is
whether it's actually running.

Checked live, today, two ways: `/rosapi/get_param` on
`/map_build_tool/mapbuilding` returned `"false"` — the chassis is not
currently in mapping mode. Subscribed to 7 candidate Cartographer live-SLAM
topics (`/tracked_pose`, `/submap_list`, `/trajectory_node_list`,
`/constraint_list`, `/scanmatched_points2`, `/map`,
`/landmark_poses_list`) for 8s — **zero responded**. Both independently
confirm: in normal operation this chassis runs AMCL-against-a-saved-map
only (matching what `/sensors_fusion/odom`'s already-documented pipeline
does — `sf2` fusing IMU/wheel/laser/visual odometry, no loop closure
anywhere in that chain either); Cartographer's own live SLAM node is not
active.

**Getting Yunji onto the same "no persistent map, bounded drift via online
loop closure" footing as wsj's prospective `BuildMapNode` approach would
require explicitly triggering the vendor's own mapping mode**
(`/opt/robot_install/share/cartographer_ros/launch/water2_mapping.launch`,
or the `/api/map/...` endpoints / the 9001 web console's mapping function)
— a real operational mode switch on the robot's own native navigation
stack, not a passive read the way every other Yunji investigation in this
project has been. Deliberately not attempted without explicit direction:
unlike observing already-running topics, switching the robot into mapping
mode is unfamiliar territory for this project and its interaction with the
vendor's own move_base/costmap/safety behavior during that mode has not
been characterized.

## Follow-up, same day: mapping mode triggered, live-verified, cleanly reverted

The above stopped at "would require explicitly triggering the vendor's own
mapping mode... deliberately not attempted without explicit direction." The
user then explicitly authorized this. Since neither SSH shell access to the
chassis nor a documented TCP/HTTP API endpoint for starting mapping exists,
the actual mechanism was reverse-engineered by fetching and reading the
9001/8809 web console's own JavaScript (via a local SSH port-forward to
each — `ssh -L 127.0.0.1:19001:192.168.10.10:9001` and `:18809:...:8809`,
purely to `curl` static JS/HTML assets, not to interact with the UI):

- The "建图工具" (mapping tools) button in the main console opens a
  separate page, `:8809/map-build/map.html?hotelid=...&floor=...`.
- That page's `Map.js` makes the actual call: `GET
  http://<ip>:8809/map-build-api/start?hotelid=<id>&floor=<n>&delta=<res>&
  map_type=<v1|v2>` to start, `GET .../map-build-api/stop` to stop-and-save
  (paired with a `/web_operation` topic publish for the console's own
  bookkeeping).

**Real risk identified before touching anything**: `/api/map/get_current_map`
showed the chassis's actual active map is `hotelid="ZhongNuo_factory_test"`,
`floor=11`. Calling `start` under that same identity without `&iscontinue=true`
looks, from the code, like it would begin a fresh map under the SAME name —
risking overwriting the map real navigation depends on. Used an isolated,
unrelated `hotelid=focus-hub-test` instead, explicitly to avoid this.

**Live-verified, in order:**
1. `GET .../map-build-api/start?hotelid=focus-hub-test&floor=1&delta=0.05&map_type=v2`
   → `{"msg": "success", "success": true}`.
2. `/rosapi/get_param` on `/map_build_tool/mapbuilding` → now `"true"`.
3. Re-subscribed to the same 7 candidate topics from the earlier (mapping-
   mode-off) check: **`/submap_list`, `/trajectory_node_list`,
   `/landmark_poses_list`, `/constraint_list` now respond** with real,
   freshly-timestamped data (`/constraint_list` specifically carries the
   pose-graph loop-closure constraints — direct proof the loop-closure
   machinery is live, not just submap accumulation). `/tracked_pose` and
   `/map` did not respond within 15s even with a longer window — this
   Cartographer deployment does not appear to publish those under their
   standard names (or does so on a slower cadence than tested).
4. **The actual usable pose was found via `/tf` instead**: a real, live
   `map -> odom` edge appeared (translation `(0.0167, -0.0011, 0)` m, yaw
   ≈5.3°, consistent with a small correction on a stationary robot) —
   Cartographer's standard pattern of publishing `map -> odom` and letting
   the *already-existing* `odom -> base_link` chain (the same
   `/sensors_fusion/odom` this project already uses) compose to a full,
   loop-closed `map -> base_link`. This means no new pose topic is even
   needed: `T_map_baselink = T_map_odom(live, from Cartographer) @
   T_odom_baselink(already read today)`.
5. `GET .../map-build-api/stop` → `{"msg": "success", "success": true}`.
   `mapbuilding` confirmed back to `"false"`. `/api/map/get_current_map`
   confirmed still `ZhongNuo_factory_test`/floor 11, byte-for-byte the same
   as before — the production map was never touched. `/api/map/list` now
   additionally lists `"focus-hub-test": [1]` — the test session did get
   saved as its own separate, harmless map entry (not deleted; no documented
   delete API was found or attempted — worth cleaning up later via the 9001
   console if desired, but not urgent since it doesn't conflict with
   anything).

**What this proves**: the "no persistent map, bounded drift via online loop
closure" capability genuinely exists and works on this specific robot,
right now — not just in vendor documentation. A real `map -> odom`
correction was observed, backed by a live, populated loop-closure
constraint list.

**What this does not prove / still open**: whether running mapping mode
continuously during an actual mission interferes with `move_base`/AMCL
navigation or `/api/move` command handling (mapping mode and normal
autonomous navigation were not exercised simultaneously); whether
`/tracked_pose`'s absence matters or whether the `/tf` composition is
equally good in practice; any resource-cost measurement of running
Cartographer continuously; and whether/how to fold this into
`yunji_sender.py` as an actual pose source — no sender code was changed by
this investigation, this was a feasibility test only, run for well under a
minute total and immediately reverted.

## Dry-run command guard (`hub/src/focus_hub/yunji_goal_guard.py`)

Same fail-closed contract as wsj's `GoalGuard` — robot-id match, STOP latch
(and STOP latching even under a wrong transform version), expiry,
out-of-order, map/transform version, health, distance limit — because the
safety envelope doesn't change with robot type. The only robot-specific part
is the final step: an accepted GOAL is reduced to a dry-run WATER TCP API
request string (`/api/move?location=x,y,theta&uuid=...`) instead of a
TinyNav POI JSON. **This module has no `socket`/`requests`/`urllib` import
at all — it is structurally incapable of calling the real API**, not just
configured not to; one of its own unit tests asserts this. 10/10 new unit
tests pass, mirroring wsj's `goal_guard.py` coverage. Hub test suite: **59
passed** (was 49).

## Post-conditions

Test hub stopped, tunnel closed (confirmed unreachable from nyush-nuc),
robot auth token removed from nyush-nuc, sender organized into
`~/focus_sender_yunji/` (matching wsj's `~/focus_sender/` overlay
convention). No command was ever sent to the robot; only `/api/robot_status`
(read) and rosbridge subscriptions (read) were exercised.

## What this proves / does not prove

Proves: a real second robot, on a completely different stack, can produce
wire-valid `mapping_only` observations that flow through the exact same hub
ingest → spool → RedNet pipeline as wsj, with genuine sensor data (RGB/depth
now reprojected through a real measured extrinsic chain read from the
robot's own `/tf`, not an approximation), real health telemetry now backed
by a real (not fabricated) covariance-derived localization signal, and a
fail-closed command guard for this robot's API shape that is implemented and
unit-tested. Also proves the sender holds up under sustained load (600
frames / 17.9 min, 0 gaps/duplicates) and under real, deterministically
injected transport outages at three different points in the retry/give-up
lifecycle (dead-at-startup recovery, mid-run kill-and-restart recovery, and
a persistent outage producing a clean non-zero-exit abort rather than a
hang) — the exact same proof shape as wsj's transport evidence.

Does not prove: G4 (needs a second robot's data fused with wsj's under a
*physically* verified shared frame — the extrinsic used here comes from the
robot's own internal TF tree, which is a real measurement, but still not the
cross-robot `T_shared_world_robot_map` G4 requires), any command actually
reaching the robot (guard is dry-run only, no HTTP/TCP client inside it),
that `current_pose`'s floor-plane (`z=0`) assumption is independently
verified on this unit, that the `localization_state` DEGRADED/LOST
thresholds are correct (only synthetic covariance has exercised those
branches — no real degraded-tracking event has been observed on this unit),
or that the hardcoded `K_DEPTH`/`K_RGB` intrinsics are themselves accurate
(their original provenance is undocumented; only a resolution-mismatch
guard exists, not an independent re-derivation).

## Network path switched from USB adapter to nyush-nuc's native Ethernet port (2026-07-20)

The operator physically moved the robot's Ethernet cable from a USB-to-
Ethernet dongle (`enx00e04c2536b0`, NetworkManager profile with static
`192.168.10.112/24`, the config the earlier USB-network fix in this document
targeted) to nyush-nuc's onboard NIC (`enp114s0`, MAC `48:21:0B:6E:1F:BD`),
expecting lower latency. That port already had an existing NetworkManager
profile named "Lidar" (static `192.168.1.2/24`, a different subnet, for a
different, unrelated device that also uses this same physical port at other
times) auto-connected on it, so the robot was unreachable at first — no
route existed to `192.168.10.0/24`.

Diagnosed read-only first (`ip addr`, `nmcli device status`, `nmcli
connection show`), then verified the fix would work with a non-persistent
`ip addr add` before touching any NetworkManager config. Confirmed the robot
became reachable (ping, and all four known service ports: TCP API `31001`,
rosbridge `9090`, web console `9001`/`8809`) as soon as an address in the
right subnet existed on the interface — the physical/link layer was never
the problem, only the missing route.

**Asked before persisting anything** (this port has another use — "Lidar" —
that the operator confirmed is still needed): rather than overwrite the
"Lidar" profile's address, created a new independent NetworkManager
connection, `Yunji-Robot`, bound to the port's MAC address (not the
interface name, so it survives renaming) with the same static
`192.168.10.112/24` the old USB profile used, `autoconnect-priority 10`
(higher than "Lidar"'s default `0`, so this is NetworkManager's default pick
on that port going forward). Brought it up with `nmcli connection up`,
confirmed it cleanly replaced "Lidar" as the active connection with no
duplicate/stray addresses left behind.

**Real limitation, stated plainly**: NetworkManager cannot detect which
physical device is actually plugged into an Ethernet port — `autoconnect-
priority` only sets a default preference for which profile activates
automatically; it does not and cannot dynamically switch based on what's
actually on the other end of the cable. If "Lidar"'s device is plugged into
this same port at a different time, the "Yunji-Robot" profile will still be
what NetworkManager tries first (or whichever the OS decides at boot/link-up
time) — the operator will need to run `nmcli connection up Lidar` (or
`Yunji-Robot`) manually when swapping which device occupies this port,
same as this session did.

**Verified against the real application, not just raw ports**: ran the
actual `yunji_sender.py` (already deployed at `~/focus_sender_yunji/`) end
to end against a fresh isolated test hub through the new network path:
6/6 real frames sent and accepted (0 retries), 22/22 heartbeats accepted (0
failed), mean upload 22.2 ms. Ping RTT to the robot dropped to sub-1ms
(0.65-1.03 ms across three pings) — the USB adapter's own latency was never
measured in a comparable way in this project's history, so "faster" is
plausible and directionally consistent with expectation but not proven via
a same-session before/after comparison; `mean_fetch_ms` (rosbridge/TCP-API
round trip to the robot itself, ~1.8s/frame at the requested 1Hz rate) is
dominated by the robot's own processing, not link speed, and wasn't
meaningfully different from prior sessions.
- Test hub, tunnel, and spooled test tokens cleaned up afterward; the 6 real
  frames from this test remain in `hub/runtime/spool/robot-1/` alongside 30
  real wsj frames from the same day's separate deployment test — genuine
  evidence, not scratch, left in place on purpose.
