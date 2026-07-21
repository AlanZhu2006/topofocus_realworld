# wsj sender pose-source switch: deployed and live-validated — 2026-07-20

## Background

`focus_ros_sender.py`'s HPC-fidelity pose-source pivot (default `--pose-topic`
changed from `/semantic_mapping/camera_pose`, TinyNav's map-relocalized pose,
to `/slam/keyframe_odom`, `perception_node`'s own live SLAM estimate with no
pre-built-map dependency — see `audit/LIVE_ROS2_SENDER.md`) had been written
and unit-tested (mocked ROS messages) since 2026-07-19, but never deployed or
run against wsj because it was offline. The same pivot's independent 2Hz
heartbeat thread was similarly code-complete but unverified on this robot.
wsj came back online 2026-07-20; this deploys and live-validates both.

## Deployment

- `ssh wsj` reachable again (`tegra-ubuntu`, confirmed via a fresh interactive
  check).
- Backed up the previously-deployed file at
  `/home/nvidia/focus_sender/focus_ros_sender.py.bak_pre_odom_pivot_20260720`
  before overwriting, then `scp`'d the current
  `hub/robot_overlay/focus_ros_sender.py` to the same path.
  `run_live_rehearsal.sh`/`stop_live_rehearsal.sh` were already byte-identical
  to the deployed copies (diffed, no changes needed).
- Verified on-robot under the real TinyNav ROS 2 environment (`source
  /home/nvidia/twork/tinynav_setup.bash`): `ast.parse` + `py_compile` pass,
  and `import focus_ros_sender` succeeds with real `rclpy`/`message_filters`
  available (not mocked) — confirms `HeartbeatThread`, `odom_to_matrix`, and
  `classify_localization_state` are all present and importable in the actual
  deployed environment, not just off-robot.

## Live validation (isolated test hub, bag replay, dry-run only)

Followed the same isolated-test-hub discipline used throughout this project
(off-default port, fresh generated tokens, torn down after): started
`hub/scripts/focus_hub_up.sh --port 8590 --no-glm --no-pipeline` on this
machine, opened a reverse SSH tunnel (`ssh -R 127.0.0.1:18590:127.0.0.1:8590
wsj`, run from the hub side — the robot-to-hub direction the project's own
scripts recommend), and ran `run_live_rehearsal.sh --base-url
http://127.0.0.1:18590 --max-frames 30` on wsj (bag replay, the script's
default — never `--live`; `run_live_rehearsal.sh` structurally never starts
planning/control/go2-bridge, so this cannot move the robot).

**Two real problems hit and fixed during this pass, recorded honestly:**

1. **Stale robot token.** wsj's `/home/nvidia/focus_sender/.token` held a
   token from some earlier session, not matching the freshly-generated token
   for this test hub — every upload got 401'd. Fixed by writing the correct
   token to `.token` (the file `run_live_rehearsal.sh`'s sender window reads
   at launch) and restarting the rehearsal. Restored the original token value
   afterward, since it's equally stale either way until a real production hub
   run generates the actual deployment token — noted as an open operational
   step below, not silently "fixed" to a guessed-correct value.
2. **A stuck ROS 2 daemon.** After a couple of rapid rehearsal start/stop
   cycles during token debugging, `ros2 topic list` started silently
   returning empty (stderr showed an `xmlrpc.client.Fault:
   "<class 'RuntimeError'>:!rclpy.ok()"` from the long-lived `ros2-daemon`
   background process) — this made `run_live_rehearsal.sh`'s `wait_for_topic`
   time out even though the bag was actively playing and publishing. Root
   cause confirmed directly (`ros2 topic list` failing with that exact
   fault), not guessed. Fixed with `ros2 daemon stop && kill -9 <pid> && ros2
   daemon start`. This is a ROS 2 CLI daemon quirk from my own rapid
   iteration, not a defect in the deployed sender or launcher code — recorded
   here since a future operator hitting the same "topic list looks empty but
   the bag is clearly running" symptom should know the fix.

**Once both were resolved, a clean run**: 30/30 frames sent and accepted by
the hub (0 retries, 0 upload errors), synchronizer saw 65 candidate
tuples before matching (2Hz requested against a denser raw stream — normal),
mean upload 29.4 ms, mean pose/frame sync skew 8.9 ms. 52/52 heartbeats sent,
0 failed (`HeartbeatThread` genuinely live-verified on wsj for the first
time, matching Yunji's earlier 21/21 result).

**Confirmed the pose source itself is real and correct**, not a stale
default: every spooled frame's `metadata.json` reports `pose.transform_version
= "wsj-live-map-frame-test-v1"`, `child_frame = "camera_color_optical_frame"`,
and a genuinely time-varying `shared_T_camera` — translation moved from
`(0.19, 0.46, 0.03)` at sequence 0 to `(-2.99, 2.80, 0.25)` at sequence 29
(~4 m of real trajectory across the bag), not a degenerate fixed pose. This
is `/slam/keyframe_odom`, not the old `/semantic_mapping/camera_pose` — the
sender was rebuilt from the current source, which only knows the new topic.

**Real, honest limitation found**: every frame's `health.localization_state`
reported `"TRACKING"`, but the underlying `covariance_6x6` is all zeros in
every single frame. `classify_localization_state()`'s TRACKING branch
triggers on low-variance covariance, and zero technically satisfies
"low" — so **`perception_node.py`'s GTSAM VIO does not appear to populate a
real covariance on `/slam/keyframe_odom`**, meaning wsj's `localization_state`
signal is not currently a real discriminator the way Yunji's is (Yunji's
`/sensors_fusion/odom` covariance is genuinely non-zero and varies). This
was not previously known because the sender had never been run against real
wsj messages before. Not fixed in this pass — recorded as an open finding,
since fixing it would require either confirming this is really how
`perception_node` always publishes this topic (vs. a bag-replay artifact) or
deriving an alternative uncertainty proxy.

## Cleanup

- `stop_live_rehearsal.sh` run on wsj: tmux session stopped, rehearsal scratch
  directories removed, confirmed "TinyNav's own map/bag/output directories
  were never touched" (the script's own post-condition message).
- `focus_hub_down.sh` run here: hub port 8088/GLM port 31511 (script's
  hardcoded health-check ports, unrelated to this test's actual 8590) both
  free; GPU memory back to 991 MiB baseline; test hub's 8590 confirmed
  unreachable (`curl` connection refused).
- Reverse SSH tunnel process killed; confirmed unreachable from wsj afterward
  (empty `curl` response).
- wsj confirmed clean afterward: no tmux server, `ros2 node list` empty.
- `.token` restored to its pre-test value (see "stale token" above for why
  this isn't a real fix, just restoring prior state).
- Test spool/state data under `hub/runtime/{spool,state}` on this machine was
  left in place (30 real frames from robot-0) — this is genuine, useful
  evidence of the deployment, not scratch to delete, and nothing else reads
  or depends on that directory being empty.

## What this proves and doesn't

**Proves**: the pose-source pivot (already live-validated on Yunji) now also
works end to end on wsj against real ROS 2 SLAM output — real, time-varying
`/slam/keyframe_odom` poses reach the hub through the full sync/encode/upload
path, and the independent heartbeat channel works on this robot for the first
time. Both of Task #25's and part of Task #37's remaining "unverified, wsj
offline" caveats in `audit/LIVE_ROS2_SENDER.md` are resolved.

**Does not prove**: sustained/soak operation on wsj specifically (only a
bounded 30-frame run; Yunji has a proven 600-frame/17.9-minute soak, wsj does
not yet), live-camera operation (`--live` was not used, bag replay only,
consistent with every prior wsj rehearsal), or anything about
`localization_state`'s newly-discovered always-zero-covariance limitation
being fixed — it is not.

## Open follow-ups

- wsj's `localization_state` is not currently a real discriminator (always
  `TRACKING`, zero covariance) — investigate whether `perception_node`
  ever publishes non-zero covariance on `/slam/keyframe_odom`, and if not,
  consider whether there's another real uncertainty signal on wsj (e.g.
  ISAM optimization residual/factor count, already visible in
  `perception_node`'s own log output) worth deriving instead.
- The token-sync step (robot's `.token` file vs. whatever the real production
  `hub/scripts/focus_hub_up.sh` run eventually generates) is a manual
  operational step, not automated — worth remembering before a real
  deployment, not just this test.
- No sustained soak or fault-injection test has been run for wsj's
  odometry-pose-source + heartbeat combination the way Yunji has both; this
  pass proves the path works, not that it's robust under load/outage the way
  Yunji's has been separately proven to be.
