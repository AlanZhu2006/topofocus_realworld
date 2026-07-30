# Scene 03 · Plant：正式实验 03 Success

时间：2026-07-31 02:31:44 至 02:35:10（Asia/Shanghai）
Session：`scene03-plant-20260731-022816-formal04`
Episode：`scene03-formal04`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `SUCCESS` | `1` | `1.0` | `0.689524` | `待测` |

现场操作者在运行后确认机器人最终实际到达 `plant`，并将本轮指定为
Scene 03 Formal 03 Success。运行时使用的 `formal04` 只是临时标签；
正式实验序号以这次操作者指定为准。

Robot 1 在 round 4 获得冻结的 176-cell `plant` 语义区域，累计路径
`11.606679 m`，起终点净位移 `8.003082 m`。按
`S × D / max(D, P)` 计算，本轮 source-compatible SPL 为 `0.689524`。
Scene 03 尚未独立测量最短可行路径，因此不虚构 Standard SPL。

## 双机动作

| Robot | 实际路径 | 净位移 | 结果 |
| --- | ---: | ---: | --- |
| Robot 0 | `9.037490 m` | `4.610633 m` | 沿独立 frontier 推进，最终安全 HOLD。 |
| Robot 1 | `11.606679 m` | `8.003082 m` | 获得 plant 语义目标；现场操作者确认最终物理到达。 |

## 自动证据边界

不可变的自动报告没有生成 semantic `ARRIVED`：其
`automatic_terminal_candidate_complete=false`、
`official_success_verified=false`，原始 outcome 为
`failed_robot_or_controller_holding`。最终控制事件是 Robot 0 的
`HEALTH_NOT_READY`；本地传感、里程计、SLAM、occupancy、对齐和平台检查
均通过，但 `tiny_nav_poi_subscriber=false` 使 `graph_ready=false`，随后
全局安全 HOLD。归档不改写这段机器证据；本轮成功依据是独立记录的现场
操作者物理到达确认。

## 场景累计指标

Scene 03 当前共 3 次正式实验：1 次成功、2 次失败，SR 为 `1/3`。
失败轮按 0 计入后，mean source-compatible SPL 为 `0.229841`；
mean Standard SPL 等待 Scene 03 的独立最短可行路径测量。

## 安全终态

- 两台机器人最终均为 `HOLDING`、`HUB_HOLD`，
  `velocity_zero_confirmed=true`。
- Hub 只发布带版本和有效期的高层目标，机器人保留本地拒绝和停车权。
- 本轮结束后 Hub GOAL 输出已关闭，没有遗留运动授权。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene03-plant-20260731-022816-formal04_live_scene03-formal04_20260731_023143_863454849/episode_report.json` | 9,820 | `22aca9b46ca4f2ee44c61a90aba37da89a953c99766bb2dc752a479ee3455d5c` | source-derived navigation report |
| `hub/runtime/oneclick_scene03-plant-20260731-022816-formal04_live_scene03-formal04_20260731_023143_863454849/controller_events.jsonl` | 55,722 | `70206309180b369c6c472260d181a8a83380aefe3c0c88eaaf4b331ad60885f0` | source-derived controller log |
| `hub/runtime/oneclick_scene03-plant-20260731-022816-formal04_live_scene03-formal04_20260731_023143_863454849/scene_manifest.json` | 47,377 | `3bc68879eaea4b799aad4cd989325de5356fce2f7071cc57a79f660947947e86` | source-derived scene manifest |
| `hub/runtime/oneclick_scene03-plant-20260731-022816-formal04_live_scene03-formal04_20260731_023143_863454849/round_04_step_099/shadow/shadow_manifest.json` | 39,010 | `1b5141f86f212865d8c2e3b380765eb8218eaaa56c6a13e80985f1e814bc58a9` | frozen VLM/semantic evidence |
| `hub/runtime/sessions/scene03-plant-20260731-022816-formal04/session.json` | 4,250 | `95d9a9605f8e227600ae33dc1c86ba764bf2ea44309d25be2b6d7ce4fe716f96` | exact session identity |
| `hub/runtime/calibration_sessions/scene03-plant-20260731-0128-recal/shared_frame.json` | 5,947 | `f9cd9f7162bb2bdea689159841d8e99132eb43ab986b044b3a689de21b551288` | shared-frame calibration |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260730T183128Z.jsonl` | 169,164 | `6a0e63638cf6f704a28879b60f6bb388ccbc6ab5a7645b7e0d022b6bdc2edb83` | observed Robot 0 receiver log; remains on robot |
| `/home/nyu/.local/state/topofocus/yunji-v2-tinynav-live-20260730T183109Z.jsonl` | 161,611 | `150b095b690226764240ede30022d7313be9ff8673cc4e74905c06577cab7e66` | observed Robot 1 receiver log; remains on robot |

机器可读归档：
[`scene03_plant_formal_experiment_03_success_20260731.json`](../manifests/scene03_plant_formal_experiment_03_success_20260731.json)。
