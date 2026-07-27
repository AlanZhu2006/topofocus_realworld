# Scene 02 · Plant：正式实验 02 Failure

日期：2026-07-28
Session：`scene02-plant-20260728-semanticreplan4`
Episode（内部 ID）：`formal-03`（操作者归档为 Formal 02）

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` |

官方归因说明：

> Formal 02 failed during coordinated execution: both robots' assigned
> frontiers were rejected as locally unreachable before any plant semantic
> region was found, and the episode then timed out waiting for a fresh
> synchronized round after Robot 1's map was blocked by ground-plane drift.

分类为 `navigation_policy_failure`，不是 VLM failure：两轮探索中的 frontier
候选均为 `FRONTIER_POINT`（未曾出现 `plant` 语义区域候选），失败发生在机器人
本地导航执行与建图健康检查阶段，与目标识别/选择无关。

## 双机动作

| Robot | 路径 | 动作 |
| --- | ---: | --- |
| Robot 0 | `0.208027 m` | 第 0 轮 frontier 被判 `LOCAL_GOAL_UNREACHABLE` 隔离；第 1 轮回退 frontier 再次被判 `LOCAL_GOAL_UNREACHABLE`，持续隔离。 |
| Robot 1 | `1.488125 m` | 第 0 轮独立推进；第 1 轮其 frontier 也被判 `LOCAL_GOAL_UNREACHABLE`，两机同时隔离，触发 `all_frontier_failures_replan_hold`。 |

两机隔离后，控制器等待下一轮同步输入 45 秒超时：

```text
no fresh synchronized round input within 45.0s: ValueError: robot-1 map
blocked: ground plane drift requires a fresh calibrated map session:
sequence=303069, consecutive_frames=7, duration_s=5.965,
tilt_delta_deg=3.691, height_delta_m=0.060
```

Episode 以 `controller_error_TimeoutError_holding` 结束，两台机器人最终均为
`HOLDING` 且 `velocity_zero_confirmed=true`，Hub 已恢复 `GOAL=false`。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260728-semanticreplan4_live_scene02-plant_20260728_054405_113292494/episode_report.json` | 9,789 | `5ae86477a50ea0a9da2f0f9785042f3726dec5ffb235e17d0fb23a7f5ea92a0d` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene02-plant-20260728-semanticreplan4_live_scene02-plant_20260728_054405_113292494/controller_events.jsonl` | 20,341 | `b6106e529379b88fc735985351b3b56607627c0cc13fb1646925fe0274bf0dbe` | source-derived controller event log |
| `hub/runtime/sessions/scene02-plant-20260728-semanticreplan4/session.json` | 4,294 | `77fca1b56ba177bed30d77a2cf13b975a8819fdb883c201f6f23840cc106e165` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene02-plant-20260728-044246-recal2/shared_frame.json` | 6,029 | `5f95966f2c201467af549fc845d0265e9ea50cd12595c5931082451a647cde07` | observed and source-derived calibration (reused, unchanged) |

机器可读归档：
[`scene02_plant_formal_experiment_02_failure_20260728.json`](../manifests/scene02_plant_formal_experiment_02_failure_20260728.json)。

## 媒体

操作者提供的第三视角与 Dashboard 录屏已通过 Git LFS 按字节原样提交到
`media/video/**`；浏览器可播放 H.264 版本与内嵌预览 GIF 位于 `media/demo/`，
命名与编码方式与 Formal 01 一致（`libx264`，GIF 为整段视频加速压缩至
8fps/8s 循环，无海报截图）。

| 素材 | 字节 | 时长 | SHA-256 |
| --- | ---: | ---: | --- |
| `media/video/third_view/experiment_2/experiment_2_failure_2.mp4` | 13,267,358 | 99.2 s | `16ad3ea2222795c59355f99f72509e4668ec1322e8f2ce61678bc1d3db8f8e34` |
| `media/video/dashboard/experiment_2/experiment_2_failure_2_dashboard.mov` | 77,734,720 | 113.128 s | `340b2ddeb64ff35da9eff5e7b23809d35b569636b45ba3037acd85e5b5d66a68` |
| `media/demo/scene02_formal_02_third_view.mp4` | 10,854,652 | 99.2 s | `ce16b8130a70f3084f3f5a79243b8f5c07cae36285c1f0f920c8e4bc91689a20` |
| `media/demo/scene02_formal_02_dashboard.mp4` | 1,810,405 | 113.067 s | `378597877305a3ac77490fcaa2fedb0e79d3430fa9b1d5f37ce917aae9997bed` |
| `media/demo/scene02_formal_02_preview.gif` | 10,179,646 | 8.01 s | `a86d3ffe4440a4763445db193825db9429496d37b5bc89edec10888ec5cb86dd` |
| `media/demo/scene02_formal_02_dashboard.gif` | 1,930,298 | 8.01 s | `5f6c7363c248f4b03e17672d304304755ddcdefc45ccce24026014bcc2187e9d` |
