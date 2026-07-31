# Scene 04 · Plant：正式实验 04 Failure

时间：2026-07-31 22:16:27 至 22:26:28（Asia/Shanghai）

Session：`scene04-plant-20260731-recalibration8-start1`

Episode：`formal04`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` | `17` |

本轮由操作者指定归档为 Scene 04 Formal 04 Failure。探索运行达到预设
`600 s` 测试时限，17 个 source rounds 内没有找到并抵达经验证的 `plant`
目标，因此本轮以 time-limit failure 计入指标。失败轮的 SR、
source-compatible SPL 和 Standard SPL 均为 `0`。

## 双机轨迹

| Robot | Actual path | Start-to-stop displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `12.917490 m` | `4.723973 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `19.608723 m` | `6.219133 m` | `HOLDING`，零速度已确认 |

运行覆盖 source steps `0–399`，共发布 87 个带版本和有效期的高层批次。
最终没有生成语义到达终态证据，超时批次将两台机器人同步置于 HOLD。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260731-recalibration8-start1_live_scene04-plant_20260731_221626_794883343/episode_report.json` | 9,820 | `fac40bd00d4e512729831d52b29df6d316c4089ff356360b64b138912f8ecae1` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene04-plant-20260731-recalibration8-start1_live_scene04-plant_20260731_221626_794883343/controller_events.jsonl` | 320,444 | `3c2cb6b0f7a43266f615b4164f52517793a19af8e57625e83cbba4a24c72e5b2` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260731-recalibration8-start1_live_scene04-plant_20260731_221626_794883343/scene_manifest.json` | 313,871 | `2c7d0da485396dc6bd7350e2865b0788defe245b62ca4a400605002ff76fd981` | source-derived continuous scene manifest |
| `hub/runtime/sessions/scene04-plant-20260731-recalibration8-start1/session.json` | 4,386 | `ca251e5f2f38a0ac5914620f8ce78017cc4902bdc448de2e906a3c5c69a64cbc` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260731-recalibration8/shared_frame.json` | 6,493 | `c317d18d201b7e76dbdef5261b8349369cf1b42235d8e07a4012c4a9a170c0cb` | observed and source-derived calibration |

机器可读归档：
[`scene04_plant_formal_experiment_04_failure_20260731.json`](../manifests/scene04_plant_formal_experiment_04_failure_20260731.json)。

## 媒体

[第三视角原片](../media/video/third_view/experiment_4/experiment_4_failure_1.mp4)
和
[Dashboard 原片](../media/video/dashboard/experiment_4/experiment_4_failure_1.mov)
按用户上传字节原样保留于 Git LFS。README 使用的
[第三视角 H.264](../media/demo/scene04_formal_04_third_view.mp4)、
[Dashboard H.264](../media/demo/scene04_formal_04_dashboard.mp4)及两个 64 帧
GIF 均为 source-derived 展示副本；精确大小、时长和 SHA-256 记录在
机器可读归档中。

## 安全终态

- 超时终止批次已由两台机器人确认；最终均为 `HOLDING`、`HUB_HOLD`。
- 两台机器人均记录 `velocity_zero_confirmed=true`。
- Hub 只发布高层目标，机器人本地停车与拒绝权限保持不变。
- 实验退出后 GOAL 通路已关闭，仅保留观察、地图与 Foxglove。
