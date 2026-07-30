# Scene 03 · Plant：正式实验 04 Success

时间：2026-07-31 03:12:47 至 03:16:54（Asia/Shanghai）

Session：`scene03-plant-20260731-0311-formal04`

Episode：`formal04`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `SUCCESS` | `1` | `1.0` | `0.693557` | `待测` |

Robot 1 自动报告语义目标 `ARRIVED`，本地规划器确认停车，现场操作者随后
确认成功抵达并指定归档为 Scene 03 Formal 04 Success。自动报告中的
`official_success_verified=false` 仅表示机器流程本身不具备独立现场标注；
原始报告保持不变，现场确认作为独立观察证据记录。

Robot 1 的实际路径为 `13.010775 m`，起终点净位移为 `9.023718 m`，
因此按 `S × D / max(D, P)` 得到 source-compatible SPL `0.693557`。
Scene 03 尚未独立测量最短可行路径，所以 Standard SPL 不作估计。

## 双机动作

| Robot | 实际路径 | 净位移 | 结果 |
| --- | ---: | ---: | --- |
| Robot 0 | `9.391253 m` | `6.409542 m` | 沿独立 frontier 探索；Robot 1 到达后安全 HOLD。 |
| Robot 1 | `13.010775 m` | `9.023718 m` | 获得 plant 语义区域并自动 ARRIVED；现场确认成功。 |

## 场景累计指标

Scene 03 当前共 4 次正式实验：2 次成功、2 次失败，SR 为 `2/4=0.5`。
失败轮按 0 计入后，mean source-compatible SPL 为 `0.345770`；
mean Standard SPL 等待 Scene 03 的独立最短可行路径测量。

## 安全终态

- 终止批次将两台机器人置于 `HOLDING`，且均确认零速度。
- Hub `/healthz` 显示 Robot 0、Robot 1 的 GOAL 输出均为 `false`。
- Hub 只发布带版本和有效期的高层目标，机器人保留本地拒绝与停车权。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene03-plant-20260731-0311-formal04_live_scene03_20260731_031247_078020375/episode_report.json` | 20,145 | `98a7e5655c2ccd7c52db1e2259f0302d07cecf327c94b83305e1c82e336a16f3` | source-derived navigation report |
| `hub/runtime/oneclick_scene03-plant-20260731-0311-formal04_live_scene03_20260731_031247_078020375/controller_events.jsonl` | 76,759 | `6f7fcbbdd90208a7363af011bc535458bbafd9c9e7b6ef743365bb1953e3fe34` | source-derived controller log |
| `hub/runtime/oneclick_scene03-plant-20260731-0311-formal04_live_scene03_20260731_031247_078020375/terminal/terminal_evidence.json` | 10,123 | `d25a1886958bd0ff71c4b9da4b23465d16f347c15ec7edc38f36c5f809a04004` | terminal evidence index |
| `hub/runtime/oneclick_scene03-plant-20260731-0311-formal04_live_scene03_20260731_031247_078020375/terminal/robot-1/rgb.jpg` | 39,489 | `b101003f1748a76edc4dee2df56a9d3c87a38d25d619d8093f493520b594c7d5` | observed post-arrival RGB |
| `hub/runtime/oneclick_scene03-plant-20260731-0311-formal04_live_scene03_20260731_031247_078020375/terminal/robot-1/depth.png` | 210,247 | `073d6f8d5ba6a533adee9feaa69fb057fcaab43d95a03c9d3d18f60787f3d351` | observed post-arrival aligned depth |
| `hub/runtime/sessions/scene03-plant-20260731-0311-formal04/session.json` | 4,292 | `95bcfcf7132128563fb9d64133c739897cc1feebb0d6ff1fc535b04120d78483` | exact session identity |
| `hub/runtime/calibration_sessions/scene03-plant-20260731-0300-yunjireanchor1/shared_frame.json` | 18,506 | `ce1162d2e6870d16898b81907eaa8858ea75e8a9c6ace4f8aa5430e8ef5cc757` | shared-frame calibration |

机器可读归档：
[`scene03_plant_formal_experiment_04_success_20260731.json`](../manifests/scene03_plant_formal_experiment_04_success_20260731.json)。
