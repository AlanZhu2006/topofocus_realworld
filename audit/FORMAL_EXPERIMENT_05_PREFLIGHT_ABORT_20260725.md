# Formal experiment 05 preflight abort — 2026-07-25

## Outcome

Formal experiment 05 did **not** start. Both preparation attempts stopped
inside strict no-motion debug:

- live mode was never entered;
- no episode directory/report was created;
- no high-level GOAL batch was published;
- no motion-capable robot command path was authorized;
- no robot movement occurred through this workflow;
- there is no SR or SPL row for experiment 05.

The operator then reported:

> 已经下电并且去充电了 先不标定了 先做文档整理

Calibration is therefore intentionally deferred. Both robots are recorded as
powered down for charging, and no previous motion authorization remains
valid.

## Preflight sequence

Both session contracts used archival commit
`203d5500c769cf3fef9f06da672d3733f919ca5b`, the existing lab21 shared-frame
calibration and the existing WSJ/Yunji SSH tmux paths.

### Attempt 1: historical replay boundary

Session `20260725-lab21-formal05-203d550` verified both remote release roots
and started a clean Hub with `GOAL=false`. Its Yunji map boundary preceded
formal experiment 04, so the SegFormer-backed daemon replayed roughly 700
historical frames. Foxglove did not obtain the Yunji/fused semantic snapshots
inside the 90-second readiness window. The launcher exited before creating a
debug run directory.

This was a preparation-boundary error, not a robot, VLM or navigation
failure. A replacement session used current stream boundaries.

### Attempt 2: current stream boundary

Session `20260725-lab21-formal05-fast-203d550` removed the replay backlog.
During the read-only robot gate, WSJ then failed at:

```text
WSJ calibrated sensor epoch is stale at /slam/camera_info.
Refusing to restart camera/perception after calibration because that would
change the tracking origin; run a new board-calibration session.
```

Read-only checks established:

- the raw D435i camera-info stream remained approximately `29.597–30.387 Hz`;
- the raw IMU remained approximately `198.070–201.216 Hz`;
- `/slam/camera_info` still advertised one publisher but produced no sample
  during a 20-second observation;
- `/slam/odometry_visual` still advertised one publisher but produced no
  sample during a 20-second observation;
- the WSJ bounded perception process still existed, but its log stopped
  changing at Unix second `1784968734`.

Thus the raw sensor was alive while the calibrated perception/tracking output
was frozen. Restarting that process would create a new tracking epoch. The
old `shared_world ← WSJ tracking` transform was not reused because its
continuity could no longer be proved.

## Safety closeout

Before local shutdown, Hub health reported:

```json
{
  "goal_output_enabled": {
    "robot-0": false,
    "robot-1": false
  }
}
```

The local Hub plus both formal05 map sessions and the formal05 Foxglove relay
were stopped. Runtime/spool/calibration evidence was preserved. The GLM model
service is read-only and was left running.

After charging, formal experiment 05 must not reuse either failed preparation
session. Since WSJ perception requires a restart and the robots have since
been powered down, the next live workflow requires a fresh two-position board
calibration unless a separately validated continuity/re-anchor procedure is
completed. The operator has explicitly deferred that work.

## Provenance

The full ignored runtime index is:

| Evidence | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/analysis/formal05_preflight_abort_20260725_1645/manifest.json` | 4,952 | `bd64cfbc7a52a4faac491057812778f0c9cad22a13c019b3d9265e465cfae243` | observed operator closeout joined with source-derived read-only preflight evidence |
| `hub/runtime/sessions/20260725-lab21-formal05-203d550/session.json` | 3,492 | `d9b491661c843dd4150e28a7f5feb53252c2ca9c97a3a97d806ef8256a735473` | source-derived fail-closed session contract |
| `hub/runtime/sessions/20260725-lab21-formal05-fast-203d550/session.json` | 3,527 | `5f6250d89569eed70f683704bb331148529284365e2c1f984e7bf843acc678e6` | source-derived fail-closed session contract |
| `hub/runtime/map_out_wsj_20260725-lab21-formal05-203d550/live_status.json` | 5,208 | `3cc55cb7ef189fdf0b86b05a8b82c41fb8cc620e9d535ff296c5544097b5eaa6` | frozen source-derived read-only mapping status |
| `hub/runtime/map_out_yunji_20260725-lab21-formal05-203d550/live_status.json` | 9,034 | `2f73a04aed1c67109dfb2d12ed9680d816b957e38bebfc4fe604dfc75a670540` | frozen source-derived read-only mapping status |
| `hub/runtime/map_out_wsj_20260725-lab21-formal05-fast-203d550/live_status.json` | 4,457 | `56eea4ac594e77b239c4040fbc6e0d06ca27b37efe7fa2b962cf0bbccf166b3f` | frozen source-derived read-only mapping status |
| `hub/runtime/map_out_yunji_20260725-lab21-formal05-fast-203d550/live_status.json` | 4,652 | `60c9699a079977c7ecb5a729e625e21aaaf44c86fb44ee3399f00d2526b6add5` | frozen source-derived read-only mapping status |

Three workspace-only media files remain untouched and unbound:

| Candidate file | Bytes | Duration | SHA-256 | Status |
| --- | ---: | ---: | --- | --- |
| `media/video/third_view/experiment_1/experiment_1_success_2.mp4` | 2,403,359 | 18.900 s | `145c7a334816d845a757ef714f0cbe74a0983ab7b728bf2c05e3e61551a2069e` | observed user file; exact runtime binding unverified; not committed |
| `media/video/dashboard/experiment_1/experiment_1_success_2_dashboard.mov` | 17,640,290 | 28.733333 s | `4e060ba8fe006a27113e1c3aa5ea0fddbc9d7e58812959b74ff34a69d492231d` | observed user file; exact runtime binding unverified; not committed |
| `media/video/dashboard/experiment_1/experiment_1_success_4_dashboard.mov` | 41,979,329 | 67.663 s | `dd3518091413d7267e50b896364071882321c8a3802fd8c35d8efdb9e7c6a633` | observed user-labelled file; exact formal04 runtime binding unverified; not committed |

No file under immutable `source/` or `dependencies/` was changed.
