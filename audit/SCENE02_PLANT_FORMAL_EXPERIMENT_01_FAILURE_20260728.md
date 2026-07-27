# Scene 02 · Plant：正式实验 01 Failure

日期：2026-07-28
Session：`scene02-plant-20260728-044246-recal2`
Episode：`scene02-plant-recal1-20260728-044959`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` |

官方归因说明：

> Formal 01 failed during coordinated execution: one assigned frontier was
> rejected as locally unreachable, and the remaining semantic-navigation leg
> terminated before arrival.

分类为 `navigation_policy_failure`，不是 VLM failure：`plant` 语义区域已被
正确识别并分配给 Robot 0，失败发生在机器人本地导航执行阶段。

## 双机动作

| Robot | 路径 | 动作 |
| --- | ---: | --- |
| Robot 0 | `6.104564 m` | 探索后被分配 `plant` 语义区域并开始导航，随后本地导航段在到达前终止（`LOCAL_PLANNER_PATH_STALE`）。 |
| Robot 1 | `1.905387 m` | 探索后其 frontier 被本地判为 `LOCAL_GOAL_UNREACHABLE`，单机隔离。 |

两台机器人最终均为 `HOLDING`，`velocity_zero_confirmed=true`，Hub 已恢复
`GOAL=false`。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260728-044246-recal2_live_scene02-plant_20260728_045111_880186006/episode_report.json` | 9,919 | `1a96f0d2364050b577897875607878886b3c8b568b1a165606871d73ce83ffe5` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene02-plant-20260728-044246-recal2_live_scene02-plant_20260728_045111_880186006/controller_events.jsonl` | 38,550 | `ed0bd91a5de6204559be2df642cdb087115a0569a8626afd9474698c9a984f02` | source-derived controller event log |
| `hub/runtime/sessions/scene02-plant-20260728-044246-recal2/session.json` | 4,283 | `1bdebe420a829d1ff0190128c93f1efc2fbee4610040c3ef40f17bca934b8212` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene02-plant-20260728-044246-recal2/shared_frame.json` | 6,029 | `5f95966f2c201467af549fc845d0265e9ea50cd12595c5931082451a647cde07` | observed and source-derived calibration |

机器可读归档：
[`scene02_plant_formal_experiment_01_failure_20260728.json`](../manifests/scene02_plant_formal_experiment_01_failure_20260728.json)。

## 媒体

操作者提供的第三视角与 Dashboard 录屏已通过 Git LFS 按字节原样提交到
`media/video/**`；浏览器可播放 H.264 版本、海报截图与内嵌预览 GIF 位于
`media/demo/`。

| 素材 | 字节 | 时长 | SHA-256 |
| --- | ---: | ---: | --- |
| `media/video/third_view/experiment_2/experiment_2_failure_1.mp4` | 8,098,606 | 68.767 s | `66f31368a56f635c47f46755465ef44491a9164101ff28f15fa5ec4d0ad9df5c` |
| `media/video/dashboard/experiment_2/experiment_2_failure_1_dashboard.mov` | 61,809,856 | 99.73 s | `76b6a8fcffe98dd430d004291eb84a07a112118cad22c0a16e14b3c9a708c1bd` |

Derivatives, posters and preview GIFs follow the existing Scene 01 naming and
encoding convention; full paths and hashes are in the machine-readable
archive above.
