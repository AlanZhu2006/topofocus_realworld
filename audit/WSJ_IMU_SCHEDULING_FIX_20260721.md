# WSJ TinyNav IMU scheduling repair — 2026-07-21

## Scope and safety boundary

This run repaired and tested the WSJ RealSense/TinyNav perception failure without
starting `planning_node`, `cmd_vel_control`, `go2_cmd_bridge`, sport-mode control,
or any other actuator path. Hub policy remained `allow_goal=false`; all ten Hub
observations used `mapping_only=true`.

The original dirty TinyNav checkout at `/home/nvidia/twork/tinynav` was not
modified. Work was isolated in:

- worktree: `/home/nvidia/focus_sender/tinynav_imu_fix_worktree_20260721`
- branch: `focus/imu-scheduling-fix-20260721`
- base: `933fce54ae65e775a1262c346180341f5657c0e4`
- commits:
  - `a9710abbec870b3c034891fa906f4862b4721abe` — decouple IMU callbacks from stereo inference
  - `39783be71d76538ce6b4b0b2c3f97d2bdda32377` — fail closed on incomplete IMU intervals

## Observed findings

1. Before the scheduling patch, camera + perception alone repeatedly built
   keyframe factors with zero IMU measurements; GTSAM reported `inf -> inf`.
   This occurred without Hub upload, map localization, pointcloud, Foxglove
   preview, or actuator processes, excluding those consumers as the primary cause.
2. TinyNav put synchronized stereo processing, TensorRT and GTSAM work on a path
   that prevented the 200 Hz Python IMU callback from being serviced reliably.
3. The first full TensorRT/GTSAM pass after process start took 18–21 seconds.
   The former IMU subscription depth of 500 retained only about 2.5 seconds at the
   observed 199.7 Hz input rate, so cold start could still create a large sensor-time
   gap even after moving inference to a worker thread.
4. In the final isolated build, cold start was explicitly treated as warm-up,
   its stale result was not published, DDS retained 50 seconds, and the in-process
   queue retained 100 seconds. The node then re-anchored on the newest sensor frame.
5. A final live sample at `process_cnt=125` reported:
   - four current IMU intervals all `valid=true`;
   - coverage ratios `1.0049`, `1.0058`, `1.0063`, `1.0065`;
   - maximum sample gaps between 5.0 and 10.0 ms;
   - `imu_messages_rejected=0` and `imu_messages_overwritten=0`;
   - finite, non-increasing optimizer errors;
   - zero occurrences in captured final logs of `only used 0 imu`, invalid-interval
     optimization, or rejected non-finite optimization.
6. The new sender completed a no-actuation ten-frame Hub run, sequences 3203–3212:
   - 10/10 accepted, zero retries;
   - mean upload 37.1 ms;
   - mean pose synchronization skew 0.0 ms;
   - 65/65 heartbeats succeeded;
   - all observations were `DEGRADED` with detail
     `slam_optimizer_imu_valid;covariance_unavailable` (never false `TRACKING`).
7. A fresh map pipeline bound to `wsj-imu-fix-20260721-live-v1` processed exactly
   those ten frames, then shut down cleanly with zero decisions emitted.
8. The pre-existing legacy WSJ daemon was also tailing the shared spool and,
   because its already-running code predated the new version gate, consumed
   sequences 3203–3212 into the old 0–3202 map. This was detected immediately
   after the end-to-end run. The mixed map was retained but marked invalid,
   its daemon was stopped cleanly, and the Foxglove relay was restarted with
   Yunji as its only source. No rollback or deletion was attempted.

## Implemented protections

- IMU callback now only validates and enqueues; stereo/TRT/GTSAM runs in a dedicated worker.
- Stereo queue keeps the newest frame instead of accumulating stale work.
- Image processing waits for an IMU sensor-timestamp watermark.
- IMU deque extraction is synchronized; integration no longer double-counts the
  image-boundary segment.
- Per-keyframe IMU count, expected count, coverage, maximum gap and end-time error
  are published in `/slam/data`.
- Any incomplete interval or non-finite optimizer result is rejected before odometry
  publication and before a `CombinedImuFactor` can be trusted.
- The sender independently revalidates interval fields. Complete optimizer/IMU
  health with absent covariance is only `DEGRADED`; faults are `LOST` or `UNKNOWN`.
- A map pipeline binds to one `transform_version` before segmentation/integration
  and raises on a version change.
- Live rehearsal launchers generate a unique transform/session version by default,
  preventing a restarted odometry origin from silently entering an old map.

## Verification

- Remote pure health tests: `6 passed`.
- Remote syntax and diff checks: passed.
- Local sender/session tests plus regressions: passed.
- Full local Hub suite: `127 passed`.
- Final no-actuation live run: 125 perception cycles observed, then a separate
  10-frame sender + fresh-map end-to-end run passed.

## Provenance

| Artifact | Size | SHA-256 | Status |
|---|---:|---|---|
| Original `/home/nvidia/twork/tinynav/tinynav/core/perception_node.py` | 31,964 B | `cfb91db07e48b4e6f1858c2f2c4e25c6da56419c8b8b86d91b29fdb338fdab0d` | observed, unchanged |
| Patched `tinynav/core/perception_node.py` | 39,472 B | `7e20431bba524fac380018b66b8b8d5ff79cc4bab90ea34be2edd03b26116c8a` | observed/live-tested |
| Patched `tinynav/core/perception_health.py` | 1,900 B | `620c5628aab448e84485557155a269cda71e67984995327fb6f15128b89a9e2e` | source-derived/tested |
| Remote `tests/test_perception_health.py` | 1,496 B | `914c43ca97544dd3ef892a169472251725ef8144b58dd55765ec4c294e52da15` | tested |
| Remote live-tested `focus_ros_sender_imu_fix_20260721.py` | 35,397 B | `589c09e0d98b56c54b162495d83f3ffeaa630ef0842f37ff32b7097e0f9ea1c8` | observed/live-tested |
| Local `hub/robot_overlay/focus_ros_sender.py` after provenance-comment correction | 35,173 B | `c0ff5530735248f98562711991b40a5f200e1f2931308c733ec9aaaf0160296d` | functionally identical; syntax/unit-tested |
| `hub/src/focus_hub/pipeline.py` | 7,063 B | `baa088960666cb161e596c5fff52aa6f819b6308fabddd165b98bf1ce279f7fa` | tested |
| `hub/tools/hub_pipeline_daemon.py` | 18,800 B | `693599308bd45d95fc0114c585c22cb313451aff94c1f4b2272c375a69d30c4c` | tested; preceding functionally equivalent revision live-tested |
| `hub/robot_overlay/run_live_rehearsal.sh` | 9,677 B | `625d070f703ec104fd4b81de8a577998f4e08315bc3a6cb2bdf53e24a2a6ae23` | syntax-tested |
| Remote ten-frame metrics JSON | 4,460 B | `58eebf51415788000452a4e94fa3a050b1d400921bf40585d3cedd0bee33cced` | observed |
| Fresh ten-frame `central_map.npz` | 19,887 B | `27fae8a3b3bde38b92543fc438f8d479957de671d429e4706c4b3c45674f81d2` | observed |
| Fresh ten-frame `latest_rgb.jpg` | 91,259 B | `d85897eab0ecc4e4880db6d232b64607cd994318001f35d86823373b1ec2c53c` | observed |
| Fresh ten-frame `map_summary.json` | 187 B | `b277560b5753e0f80d1ebb03c1ec81b385dc01497a7f17dee0808ca6e6e8a0b4` | observed |

Fresh map outputs are preserved under
`hub/runtime/map_out_wsj_imu_fix_20260721/`; they were not substituted into the
existing Foxglove relay configuration. The mixed legacy map is explicitly
marked by `hub/runtime/map_out_wsj/INVALID_TRANSFORM_MIX_20260721.md`; its final
`central_map.npz` SHA-256 is
`bc2f81ebf4c323af5f1ef854e161ee6875039002b3e8574fb1d08547c117d2b4`.

## Still unverified / not claimed fixed

- TinyNav still publishes all-zero pose covariance. Computing full GTSAM
  marginals online was rejected because it was too expensive on this Jetson;
  command-capable `TRACKING` therefore remains intentionally blocked.
- The RealSense IMU-to-camera/body extrinsic and noise values have not yet been
  physically calibrated or validated by a moved-robot ground-truth trajectory.
- This test was stationary/no-actuation and about two minutes, not a long moving soak.
- The WSJ camera/pose flow has now been resumed only in the mapping-only lane;
  command-capable navigation remains unverified and blocked. See the activation
  update below.

## Foxglove activation update

After explicit user approval, WSJ was reconnected at approximately 18:12 local
time using only the isolated session:

- the WSJ map daemon was restarted with `--start-after-sequence 3202` and
  `--expected-transform-version wsj-imu-fix-20260721-live-v1`;
- Foxglove now reads WSJ from
  `runtime/map_out_wsj_imu_fix_20260721` and Yunji from its unchanged directory;
- the WSJ sender resumed at sequence 3213 with the same transform version;
- the raw WSJ camera preview resumed at 5 Hz; its first two checkpoints were
  50/50 and 100/100 successful pushes;
- at the map validation checkpoint, sequences 3203–3230 were continuous, all
  28 observations had the one expected transform version, all were
  `mapping_only=true`, and all reported `DEGRADED` with
  `slam_optimizer_imu_valid;covariance_unavailable`;
- the next map snapshot reached sequence 3231 / 29 frames and `latest_rgb.jpg`
  advanced during a five-second observation window;
- a subsequent 18:15 liveness checkpoint reached sequence 3277 / 75 mapped
  frames; sender logs showed 60 accepted live uploads, camera preview reached
  400 pushes with zero failures, and no actuator process was present;
- the invalid legacy map remained frozen and was not reintroduced.

The activation manifest is
`hub/runtime/map_out_wsj_imu_fix_20260721/SESSION_MANIFEST_20260721.json`.

Operational rollback is non-destructive: stop the remote `sender` and
`wsj_camera_preview` tmux panes, stop local `dash_daemons:wsj`, then restart the
relay with only `robot-1:yunji:runtime/map_out_yunji`. Neither WSJ map directory
needs to be deleted or overwritten.
