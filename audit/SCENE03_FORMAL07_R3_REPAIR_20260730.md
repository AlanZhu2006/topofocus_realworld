# Scene 03 Formal 07 Restart r3 repair — 2026-07-30

## Scope and safety state

The observed run completed seven source rounds and then entered a fail-closed
Hub HOLD. Both final navigation events reported `HOLDING` and
`velocity_zero_confirmed=true`. No physical command was issued while preparing
this repair.

## Observed evidence

- Robot 1 travelled 10.6241394008 m and produced the semantic `plant`
  override. Round 07 then failed in the Hub validator with
  `remaining VLM history set does not match allocations`.
- Robot 0 travelled 2.7675678059 m. Its receiver recorded five bounded
  `ODOMETRY_STALE` recovery waits and later rejected one leg as
  `LOCAL_PLANNER_PATH_STALE`.
- During the same sensor-time window, Robot 0 perception repeatedly rejected
  incomplete IMU intervals. The largest explicit timestamp jump in the run
  window was 0.1202056408 s.
- Robot 0 bridge commands in the run alternated angular sign during ordinary
  in-place alignment. The deployment wrapper only latched turns after a
  75-degree entry threshold, leaving moderate pure-yaw path alignment
  unlatched.

## Repair

- Validate and consume the source exploration choice before validating a later
  semantic override. Source history `target_id` is normalized to the frozen
  candidate's `frontier_id` only for complete provenance comparison.
- Retain one second of Robot 0 IMU DDS history at the observed 200 Hz rate
  (200 samples), with a hard two-second maximum (400 samples). This covers the
  observed scheduling gap without restoring the former roughly 50-second
  backlog.
- Latch a real, pure-yaw, traversable-path turn at 15 degrees and release it
  below 8 degrees. Existing pause, stale-pose, stale-path, collision,
  deadline, and final robot stop authority remain unchanged.

## Verification state

- Exact Round 07 frozen manifest: validator passed after the repair.
- Targeted scene-batch tests: 16 passed.
- Targeted controller and perception-entry tests: 61 passed.
- Complete Hub regression suite: 745 passed.
- Python compilation and `git diff --check`: passed.
- Physical motion after this repair: unverified pending a new operator
  authorization.

## Provenance

Checkpoint time: `2026-07-30T16:52:24+08:00`.

| Artifact | Size (bytes) | SHA-256 | Classification |
|---|---:|---|---|
| `hub/runtime/oneclick_scene03-plant-20260730-1425-recalibration5-formal07-r3_live_scene03-plant-long_20260730_163057_848839915/episode_report.json` | 10,021 | `25aa9c8369f46cafb96a6d719fde587f0507ad38a79ff31446639201a1d3c3f5` | observed |
| `hub/runtime/oneclick_scene03-plant-20260730-1425-recalibration5-formal07-r3_live_scene03-plant-long_20260730_163057_848839915/round_07_step_174/shadow/shadow_manifest.json` | 38,868 | `0d1e1f04c3dca02b3a0afa119209ac1f5bbcdb1864dbe56b4ea83973220429fe` | observed |
| Robot 0 `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260730T083042Z.jsonl` | 139,824 | `0e0630ab54c0fdd3f0dda597efe0addb011b49f2a548f1c816c4c1f54407d0b1` | observed |
| Robot 0 `/home/nvidia/.local/state/topofocus/wsj-go2-bridge-20260730T083042Z.log` | 97,065 | `1b26ed0476bc6c89889f6e082e875fc00e2c9d8ae70c4b513696e323c55481e0` | observed |
| First 9,852,451 bytes of Robot 0 `/home/nvidia/.local/state/topofocus/perception-20260730T063024Z.log` | 9,852,451 | `3118d0d9d876580e507da297f31d508d2ff3abfbeea83c938dec6e3ec24f8e9a` | observed append-only prefix |
| `hub/src/focus_hub/v2_scene_batch.py` | 62,708 | `e89738303707de092f00ba3afea6ea5538afdafa9088bbcba8d405f556544604` | source-derived, locally tested |
| `hub/robot_overlay/yunji_tinynav_cmd_vel_control.py` | 62,255 | `10ab9c15189a60740c9be6ea9ccdbb1be1acffcf35004cf73bce28115b938735` | source-derived, locally tested |
| `hub/robot_overlay/wsj_perception_entry.py` | 3,102 | `d3c2dab2f68a58916c3cb79f71366ac94503b39a07282eee99bb9633b2571c74` | source-derived, locally tested |
