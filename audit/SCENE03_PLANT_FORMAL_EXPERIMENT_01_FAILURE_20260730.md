# Scene 03 · Plant：正式实验 01 Failure

日期：2026-07-30
Session：`scene03-plant-20260730-2054-hotfix11-formal09r1`
Episode：`formal09-restart`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` |

本轮分类为 `execution_engineering_failure`，终止原因为
`episode runtime timeout`。这不是 VLM failure：第 12 轮已识别并向 Robot 0
分配 `plant` 语义区域，但本地执行未形成语义到达。

实验展示摘要统一记为：探索在测试时限内未形成经验证的 plant 到达，
因此以 time-limit failure 结束。下面保留原始运行诊断作为工程 provenance。

## 双机动作

| Robot | 路径 | 动作 |
| --- | ---: | --- |
| Robot 0 | `18.577107 m` | 探索路线正常；检测到 plant 后取得有效局部路线和非零转向命令，但实测位姿没有继续响应，语义路径依次触发 `PATH_STALE` / `NO_PROGRESS`。 |
| Robot 1 | `14.162235 m` | 早期探索在墙前使用过短局部轨迹作为转向依据，轨迹航向跨越机身轴，形成左右交替转向。 |

两台机器人最终均为 `HOLDING`，`velocity_zero_confirmed=true`，运动链已清理。

## 最小修复

- 局部轨迹必须达到 `0.30 m` 才能取得稳定转向权；更短的轨迹改用当前
  router waypoint，避免墙前几厘米路径抖动。
- Robot 0 在 active command 后首次收到可靠零速时执行一次
  `Move(0)+StopMove`；后续零速不重复调用，新非零目标可重新取得控制。

上述修复已进入自动回归；物理效果仍需下一次独立授权的真机实验确认。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene03-plant-20260730-2054-hotfix11-formal09r1_live_scene03-plant_20260730_205815_038680549/episode_report.json` | 9,872 | `2d5d58bf1e928296907e1afbc0dff56c8589b2535870859d26a3a8247833c331` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene03-plant-20260730-2054-hotfix11-formal09r1_live_scene03-plant_20260730_205815_038680549/controller_events.jsonl` | 231,170 | `1806d0f0713fd9ed6d4a3043b65de98267dfe16c3b308dd6cc95bd6765ae754e` | source-derived controller events |
| `hub/runtime/oneclick_scene03-plant-20260730-2054-hotfix11-formal09r1_live_scene03-plant_20260730_205815_038680549/scene_manifest.json` | 204,384 | `2c21c60dbf7e8bbf8e0fb701c016f3031f209f0b4d3866d0427ece49bbc523a3` | source-derived continuous scene manifest |
| `hub/runtime/sessions/scene03-plant-20260730-2054-hotfix11-formal09r1/session.json` | 4,409 | `1eaaae4a55669a7efc0144107cfaf3021b381f38c1176fa3e2f501b3801a8927` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene03-plant-20260730-2022-recalibration9/shared_frame.json` | 6,000 | `f62b3d5b2552f0fa214d160c4e7b8d28f3f6d3ac6c29182269307d76a71505ab` | observed and source-derived calibration |

机器可读归档：
[`scene03_plant_formal_experiment_01_failure_20260730.json`](../manifests/scene03_plant_formal_experiment_01_failure_20260730.json)。

## 媒体补充归档（2026-07-31）

[第三视角原片](../media/video/third_view/experiment_3/experiment_3_failure_1.mp4)
和
[Dashboard 原片](../media/video/dashboard/experiment_3/experiment_3_failure_1_dashboard.mov)
按用户上传字节原样保留于 Git LFS；README 使用的
[第三视角 H.264](../media/demo/scene03_formal_01_third_view.mp4)、
[Dashboard H.264](../media/demo/scene03_formal_01_dashboard.mp4)及对应 GIF
均为 source-derived 展示副本。精确大小、时长和 SHA-256 记录在机器可读归档。
