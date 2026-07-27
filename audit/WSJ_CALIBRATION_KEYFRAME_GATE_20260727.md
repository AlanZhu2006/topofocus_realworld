# WSJ calibration keyframe-gate recovery — 2026-07-27

## Outcome

The WSJ calibration launcher no longer treats sparse TinyNav keyframes as a
continuous perception heartbeat. Camera/perception recovery is now gated by
continuous depth, continuous visual odometry, camera intrinsics and the native
infrared stream. Calibration geometry is unchanged: the read-only sender must
still synchronize `/slam/keyframe_depth` with `/slam/keyframe_odom`, upload the
tuple, and advance the Hub observation sequence before board fitting can begin.

Remote-command timeouts in both calibration and the formal one-click launcher
now interrupt the timed-out foreground job before fail-closed cleanup. If the
job does not release the shared shell within ten seconds, the existing
SSH/tmux pane is respawned and probed before cleanup continues.

## Observed evidence and provenance

- observed host: `tegra-ubuntu` (`aarch64`);
- inspected source:
  `/home/nvidia/twork/tinynav/tinynav/core/perception_node.py`;
- observed source size: `31,964` bytes;
- observed source SHA-256:
  `cfb91db07e48b4e6f1858c2f2c4e25c6da56419c8b8b86d91b29fdb338fdab0d`;
- observed TinyNav checkout:
  `933fce54ae65e775a1262c346180341f5657c0e4`;
- source-derived publication rule: keyframe depth and keyframe odometry are
  emitted together only when `keyframe_check(...)` accepts the current frame
  or the sparse-keyframe interval exceeds three seconds;
- observed before the fix: infrared, `/slam/depth`, `/slam/odometry_visual`,
  `/slam/keyframe_odom` and `/slam/camera_info` were fresh while one
  `/slam/keyframe_depth` probe timed out;
- observed after leaving the healthy perception process running: fresh
  keyframe depth arrived in 6 seconds with a best-effort subscriber and in
  4 seconds with a reliable subscriber.

This proves that the prior pre-sender keyframe heartbeat could reject and
restart a healthy stationary perception stack. It does not replace the final
end-to-end sender sequence gate.

## Verification

- `bash -n` passed for the WSJ calibration launcher, calibration wrapper and
  formal one-click wrapper;
- 64 focused operator/receiver tests passed;
- all 499 Hub tests passed;
- throughout diagnosis the Hub reported `goal_output_enabled=false` for both
  robots; no planner, receiver, Go2 bridge or WATER motion path was started.
