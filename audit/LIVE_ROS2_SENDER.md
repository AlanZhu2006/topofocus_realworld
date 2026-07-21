# Live ROS 2 sender + one-click startup on both ends

Date: 2026-07-18. This is a transport/wiring validation, not a gate. It
proves the sender subscribes to real ROS 2 topics (not the custom TinyNav
map-record format G3/E2E used) and that both ends of the system now start
with one command, mirroring TinyNav's own `tinynav_semantic_auto_*.sh`
convention. It does not touch actuation and does not claim G4/G5.

## What was built

### `hub/robot_overlay/focus_ros_sender.py` (deploy target: the robot)

An rclpy node subscribing to exactly the topics TinyNav's own semantic
mapping stack publishes live during `tinynav_semantic_auto_nav.sh`:

```
/camera/camera/color/image_raw                    sensor_msgs/Image
/camera/camera/aligned_depth_to_color/image_raw    sensor_msgs/Image (16UC1, mm)
/camera/camera/aligned_depth_to_color/camera_info  sensor_msgs/CameraInfo
/semantic_mapping/camera_pose                      geometry_msgs/PoseStamped
```

Traced on the robot (`semantic_mapping/semantic_pointcloud_node.py`):
`/semantic_mapping/camera_pose` is `T_map_camera`, republished with the
*same* `header.stamp` as the RGB/depth pair it was computed from — so a
single 4-way `message_filters.ApproximateTimeSynchronizer` pairs all four
exactly, with a generous queue (40) to absorb the pose topic's extra
processing latency (it waits on a TF lookup) without needing a large slop.

Two properties this sender has that the replay sender (`focus_sender.py`,
G3/E2E/transport-test) does not:
- No depth realignment needed — `aligned_depth_to_color` is already
  resampled into the color frame by the RealSense driver, and its 16UC1
  millimetre encoding is byte-identical to this project's `png16` wire
  format at `depth_scale_m=0.001`, so depth is forwarded as raw bytes.
- `--capture-time-source header` (default) trusts the real ROS timestamp —
  correct for a genuinely live camera. `--capture-time-source wall`
  re-stamps to now and is explicitly logged as rehearsal-only, needed only
  because a replayed bag's historical timestamps would otherwise be rejected
  by the hub's 3 s freshness window.

Pose frame caveat, stated up front and enforced by construction: TinyNav's
"map" frame is per-session and relocalization-dependent, not a physically
verified `shared_world`. This sender only ever uploads `mapping_only=true`
with a distinctive `--transform-version` test label
(`wsj-live-map-frame-test-v1`), never a real calibration.

### `hub/robot_overlay/run_live_rehearsal.sh` / `stop_live_rehearsal.sh`

One-click rehearsal, styled after `tinynav_semantic_auto_nav.sh` (tmux
session, `wait_for_topic`/`wait_for_message` health checks, usage banner).
Starts only `perception_node` (SLAM), `map_node` (relocalization against an
existing built map, read-only) and `semantic_pointcloud_node` (pose/cloud
publishing) — **never** `planning_node`, `cmd_vel_control` or
`go2_cmd_bridge**, so it structurally cannot move the robot. Default source
is `ros2 bag play` of a real recorded bag (no camera/robot hardware needed);
`--live` switches to the real camera (operator-presence only).

## What was validated (real ROS 2 messages, not the custom map-record format)

Ran the one-click rehearsal against the real bag
`~/.local/share/tinynav/rosbags/semantic_map_record_20260717_102052`
(confirmed via `ros2 bag info`: real `sensor_msgs/Image`/`CameraInfo` topics,
34,392 messages) with `perception_node` + `map_node` (relocalizing against
the already-built `output/semantic_map_record_20260717_102052`) +
`semantic_pointcloud_node`, single pass (no loop — see limitation below):

- **26 observations genuinely reached the hub** over authenticated HTTP,
  built from real synchronized ROS 2 messages end to end.
- Spot-checked one: 848×480 RGB, depth range 0.54–1.85 m, a proper
  orthonormal rotation matrix (not identity/garbage), `fx≈605.6` matching the
  camera's known intrinsics from the G3 dataset, `distortion_model=plumb_bob`
  (real `CameraInfo`, not a placeholder).
- Translation spread across the 26 frames: 5.66 m × 1.58 m × 0.54 m —
  physically plausible robot motion, not a stationary/degenerate pose.
- Fed through the same `SpoolMappingPipeline` as G3/E2E: 11,881 explored /
  4,417 obstacle cells, no NaNs, values properly bounded — a sane partial
  map from genuinely live-sourced data.

## Bugs found and fixed during this validation (all now fixed in the files)

1. `run_live_rehearsal.sh` defaulted `--from-bag` to the *built map*
   directory (`output/...`) instead of the *raw rosbag* directory
   (`~/.local/share/tinynav/rosbags/...`) — `ros2 bag play` failed with "No
   storage could be initialized". Fixed the default path.
2. `tmux set-option -g remain-on-exit on` was called before any tmux
   session/server existed; that specific `tmux` subcommand does not
   implicitly start a server the way most others do, so it failed and (under
   `set -e`) aborted the whole launcher before creating any window. Moved it
   to after the first `tmux new-session`.
3. `focus_ros_sender.py` read `FOCUS_ROBOT_TOKEN` into a local `token`
   variable but never attached it to `args`, while `HubTransport` read
   `args.token` — `AttributeError` on startup. Fixed by setting `args.token`
   directly. `remain-on-exit` (added while fixing #2) is what surfaced this
   one immediately instead of the window silently vanishing.

## Known limitation: `--bag-loop` wedges the TF-dependent pose after one pass

With `--bag-loop`, `semantic_pointcloud_node`'s diagnostics showed
`processed` frozen at 63 while `received` kept climbing — it stopped
producing new poses shortly after the bag wrapped back to its start
timestamp. This is consistent with tf2's `Buffer`/`message_filters`
time-indexing assuming monotonically increasing timestamps: a bag loop makes
`header.stamp` jump backward, which is exactly the fault case the hub's own
clock-skew checks are designed to catch on outputs, but it also confuses the
*consuming* ROS 2 machinery upstream of that point. This is a rehearsal/bag
looping artifact only — a live camera's clock is always monotonic, so it
cannot occur in real deployment. The launcher now prints an explicit warning
when `--bag-loop` is passed and recommends repeated single-pass launches
instead for an extended rehearsal. Nothing in TinyNav's own code was touched
to investigate or work around this.

## Hub-side one-click start/stop (`hub/scripts/focus_hub_up.sh` / `_down.sh`)

Matching UX on this side: one command starts hub API + GLM-4V + the
incremental mapping/decision pipeline daemon together (tmux session, health
waits, usage banner); one command stops them and confirms ports/GPU are
released without touching durable data (spool, registry state, tokens).

Tested end to end on port 8588/GLM 31611 (kept off the default ports so it
never collides with a real deployment): fresh run generated per-robot tokens
and an admin token (printed once, `chmod 600` on disk), started hub+GLM+
pipeline, health-checked all three, accepted 5 real uploaded frames from
`replay_sender.py` and the pipeline daemon picked them up and initialized
the mapper automatically (`mapper_init` logged) with **zero extra steps**.
`focus_hub_down.sh` then confirmed both ports free and GPU back to the
pre-existing baseline, while `runtime/spool` and `runtime/state` (5 frames,
correct `last_sequence=4`) were left intact.

Token bootstrap: if `hub/runtime/tokens.json` doesn't exist, the script
generates one random token per robot in `hub/config/robots.json` (or
`robots.local.json` if present — already-gitignored per `hub/.gitignore`),
prints them once for copying to each robot's `FOCUS_ROBOT_TOKEN`, and never
re-prints them on subsequent runs.

## Post-conditions

Both machines confirmed clean after every run: wsj shows no `ros2 node
list` entries and no tmux server; this machine shows hub/GLM ports free and
GPU at the pre-existing unrelated-process baseline. The shipped
`hub/config/robots.json` (`allow_goal=false` for both robots) was never
modified by any of this work.

## What this does and does not prove

Proves: the live ROS 2 topic architecture, message synchronization strategy,
and wire upload all work with real sensor/SLAM/relocalization messages, and
both the robot side and hub side now have a working one-click start/stop
matching TinyNav's own operational style.

Does not prove: sustained live-camera operation (only bag replay was
exercised; `--live` exists but was not run — no operator/hardware session
for this turn), G4/G5, or any actuation path (this sender and rehearsal
launcher structurally cannot reach `/cmd_vel`).

## Pose-source pivot drafted, NOT yet validated (2026-07-19)

Per the same HPC-fidelity correction already validated on the Yunji sender
(`audit/YUNJI_WATER_SENDER.md`, "Third pass"): `focus_ros_sender.py` no
longer defaults to `/semantic_mapping/camera_pose` (TinyNav's own
map-relocalized pose, which assumes a pre-built map — the opposite of
Habitat's per-episode fresh-start semantics this project reproduces).
Default is now `/slam/odometry_visual`, and `nav_msgs/Odometry`-shaped
covariance now drives the same `classify_localization_state()` logic
already live-validated on Yunji.

This is genuinely unvalidated, not a smaller version of validated — stated
plainly because wsj has been offline for this entire pivot:

- Topic name and message type: `hub/docs/ROBOT_WSJ_AUDIT.md` (a real
  direct-observation audit) documents `/slam/odometry_visual` and
  `/slam/odometry` exist, but never records their message type or which of
  the two is the right one for a synchronized-keyframe sender.
  `nav_msgs/Odometry` and "visual over high-rate" are informed guesses, not
  verified facts.
- Frame semantics: assumed to report the camera's own pose directly (no
  `base_link`->camera composition applied), by analogy to how visual SLAM
  naturally tracks the sensor frame — not confirmed.
- Never deployed to or run against wsj. Only checked: `python3 -m py_compile`
  passes, and `odom_to_matrix`/`classify_localization_state` were unit-
  tested in isolation with mocked ROS message objects (correct SE(3)
  composition from an identity-quaternion pose, and all four
  TRACKING/DEGRADED/LOST/UNKNOWN branches behave as expected).

Do not treat this as validated real-machine evidence until wsj is back
online and this has actually been run: verify the topic/type assumptions
with `ros2 topic info` / `ros2 topic echo --once`, then deploy and repeat
the same live-frame + sanity-check pass already done for Yunji.

## Pose topic revised on stronger evidence: `/slam/keyframe_odom` (2026-07-19, later)

A user question ("is there a way to build the map online while also closing
loops, the way TinyNav's own stack does?") prompted reading
`perception_node.py` and `build_map_node.py` directly, rather than relying
on the topic-existence-only audit the first version of this pivot was based
on. Read via a cached local clone of the same GitHub mirror used earlier
(`AlanZhu2006/go2_tinynav`, commit `629c79b`, dated 2026-06-17 — still not
proven identical to wsj's exact deployed commit, same caveat as always, but
strong evidence for the general architecture). This settled the topic
choice with much higher confidence than the first version had:

- `perception_node.py` publishes `/slam/keyframe_odom`, `/slam/keyframe_image`,
  and `/slam/keyframe_depth` all in the same code block with the identical
  `left_msg.header.stamp` — an exact-timestamp match by construction.
- TinyNav's own `BuildMapNode` (see below) subscribes to exactly these three
  topics via `ApproximateTimeSynchronizer`, for precisely the same purpose
  this sender has (one pose per synchronized RGB-D keyframe) — strong
  corroboration this is the intended topic, not `/slam/odometry_visual`
  (continuous, not keyframe-paired) or `/slam/odometry` (a separate stream
  `BuildMapNode` subscribes to independently as `continuous_odom`,
  confirming it is not the per-keyframe pose either).
- `perception_node.py` publishes both `odom_pub` and `keyframe_pose_pub` via
  `np2msg(pose, stamp, "world", "camera", velocity)` — confirming
  `parent_frame="world"` (this node's own live session-fresh origin, not a
  persistent map name) and `child_frame="camera"` (the camera's own pose
  directly, no `base_link` involved) — the first version's frame-semantics
  assumption was correct, now from direct evidence rather than analogy.

`--pose-topic` default changed from `/slam/odometry_visual` to
`/slam/keyframe_odom` accordingly. RGB/depth topics deliberately NOT
changed to `/slam/keyframe_image`/`/slam/keyframe_depth` despite their even
tighter sync guarantee: that depth comes from `perception_node`'s own
`depth_engine.infer(...)` (computed, disparity-derived) rather than the
RealSense driver's native hardware depth this sender has always used — a
real trade-off needing live evaluation, not folded in as a side effect of
the pose-topic fix. Still never run against wsj; still offline throughout.

## Investigated: does TinyNav support "build the map online while closing loops"? Yes — `BuildMapNode`, not adopted yet (2026-07-19)

Direct source reading (`build_map_node.py`, `perception_node.py`) answers
this precisely — there are three distinct capabilities in this codebase,
not one:

1. **`perception_node.py`** (what this sender's pose comes from): pure VIO,
   GTSAM factor graph over IMU + stereo visual factors, but a fixed 5-frame
   sliding window (`_N = 5`) — local smoothing only, confirmed zero loop-
   closure code anywhere in the file. Pure odometry: bounded local
   consistency, unbounded long-term drift.
2. **`map_node.py`** (the old pre-pivot pose source): DOES do online
   pose-graph optimization with loop closure (`find_loop_and_pose_graph`,
   `solve_pose_graph`) — but `MapNode.__init__` requires `tinynav_map_path`
   (a mandatory constructor arg) and loads `poses.npy`/`occupancy_grid.npy`/
   `sdf_map.npy`/etc. at startup. Cannot start without a pre-built map — the
   exact dependency this whole pivot exists to avoid.
3. **`build_map_node.py`**'s `BuildMapNode` — the answer to the question.
   Its constructor takes only an output path (`map_save_path`); it builds
   completely from scratch, calls `detect_loop_closure()` and periodically
   `solve_pose_graph_online()` (triggered by a COLMAP-style frames-ratio
   rule) as new keyframes arrive, and publishes a continuously-refined
   trajectory (`pose_graph_trajectory_publish`) every keyframe. It
   subscribes to standard live ROS topics (`/slam/keyframe_odom`,
   `/slam/keyframe_image`, `/slam/keyframe_depth`, `/slam/odometry`,
   `/camera/camera/color/camera_info`) — the same ones `perception_node.py`
   (already required by `run_live_rehearsal.sh`) publishes — not bag-replay
   machinery specifically, so nothing about its subscription design
   prevents pointing it at a live, moving robot.

This is architecturally the ideal middle ground for HPC fidelity: no
persistent-map dependency (matches Habitat's per-episode fresh start) AND
bounded drift via online loop closure (matches Habitat's non-drifting
ground truth) — resolving the tension the third-pass odometry pivot
explicitly could not.

**Not adopted — scoped only, real open questions remain:**
- TinyNav's own workflow uses `BuildMapNode` as an offline, pre-mission
  step (typically fed by a bag replay of a manually-driven session, then
  `MapNode` relocalizes against the saved result later) — never confirmed
  running live against a moving robot's real-time topics in this codebase's
  own launch scripts. Nothing in its subscription design rules this out,
  but it has not been demonstrated.
- Real added GPU cost: loop closure runs `SuperPointTRT`/`LightGlueTRT`/
  `Dinov2TRT` (TensorRT-dependent, confirmed earlier in this project)
  continuously, on top of `perception_node.py`'s own GTSAM VIO and this
  sender's own encode/upload work, all on the Jetson Orin NX. Not measured.
  Not compared against the RTX 4090 hub side, which is where this project's
  compute budget was designed to concentrate.
- No code has been written for this yet (unlike the pose-topic/heartbeat
  changes above) — this would mean a new launch-script addition (running
  `BuildMapNode` alongside `perception_node`) and a new sender-side
  consumer of its `/mapping/pose_graph_trajectory` output, a materially
  larger change than a topic-name swap. Deferred pending explicit
  direction on whether the GPU-cost tradeoff is worth it, and pending wsj
  connectivity to test any of it.

## Independent heartbeat thread drafted, NOT yet validated (2026-07-19)

Same hub-side protocol addition described in `audit/YUNJI_WATER_SENDER.md`
("Independent 2Hz heartbeat channel", live-validated there against the real
Yunji robot): `focus_ros_sender.py` gained a `HeartbeatThread` — a real OS
thread, not an rclpy `Timer` (a same-thread timer would NOT be independent
here, since `rclpy.spin_once` only runs one callback at a time and
`HubTransport.upload()`'s retry loop can legitimately block `process()` for
up to ~40s on a bad connection).

Honest limitation specific to wsj, unlike Yunji: this sender has no
independent fast health source at all (no TCP-API equivalent) — the
heartbeat thread can only repost the same `localization_state`
`process()` already computes per synced frame (via a thread-safe
`LatestHealth` holder), faster and more reliably than waiting for the next
successful RGBD upload. It cannot invent `estop_engaged`/battery data this
sender has no source for; `safety_state` stays `UNKNOWN`, same as before.

Never run against wsj — checked only: `python3 -m py_compile` passes, and
`LatestHealth`/`HeartbeatThread` construction was exercised in isolation
with mocked ROS imports (no real network round-trip, since there is
nothing to connect to). Validate live once wsj is reachable, the same way
the Yunji heartbeat thread was live-tested (slow `--rate-hz` alongside
`--heartbeat-hz`, confirm heartbeats keep flowing independently via the
hub's request log).
