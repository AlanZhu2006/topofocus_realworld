# Scene 04 · Plant：正式实验 04 Success

时间：2026-08-01 14:22:42 至 14:25:32（Asia/Shanghai）

Session：`scene04-plant-20260801-recal10-forwardfix1-start2`

Episode：`forwardfix-rerun01`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| `SUCCESS` | `1` | `1.0` | `0.822180` | `0.905411` | `4` |

操作者在实验后确认本轮成功，归档为 Scene 04 Formal 04。Robot 1 在第四个
source round 切换至检测到的 plant 语义区域，自动发出
`LOCAL_PLANNER_ARRIVED`；0.461 s 后保存的 RGB-D 继续提供到达后证据。
Robot 0 完成其分配的探索段，随后与 Robot 1 一起接受终止 HOLD。

Standard SPL 使用操作者独立测得的近似最短可行路径 `L≈11 m`：
`11 / max(11, 12.149176) = 0.905411`。

## 双机轨迹

| Robot | Actual path | Start-to-stop/arrival displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `5.709166 m` | `1.963621 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `12.149176 m` | `9.988810 m` | `ARRIVED` 后同步 `HOLDING`，零速度已确认 |

Source-compatible SPL 使用到达机器人 Robot 1 的起点至到达点位移作为参考：
`9.988810 / 12.149176 = 0.822180`。运行覆盖 source steps
`[0, 24, 49, 74]`，共发布 23 个带版本和有效期的高层批次。

## 到达证据

- Robot 1：`potted plant`，置信度 `0.837681`，语义区域 130 个栅格。
- 自动到达事件：sequence `415289`，reason `LOCAL_PLANNER_ARRIVED`。
- 到达后 RGB-D：sequence `415345`，相对到达事件延迟 `0.460868 s`。
- 自动报告保持其不可变边界 `official_success_verified=false`；独立的现场操作者
  确认“归档为成功”提供正式物理协议分类。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260801-recal10-forwardfix1-start2_live_scene04-forwardfix-rerun01_20260801_142241_722401259/episode_report.json` | 21,142 | `1394dfbb83e6772ce1b5b0c49d105ce35fc11fee8fddf789c11622ac4dfada46` | source-derived automatic episode report |
| `hub/runtime/oneclick_scene04-plant-20260801-recal10-forwardfix1-start2_live_scene04-forwardfix-rerun01_20260801_142241_722401259/controller_events.jsonl` | 56,321 | `913c0dce2ceeddd52ce17160898ea065b66a5b2533ab5232ada59ee1f63361ae` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260801-recal10-forwardfix1-start2_live_scene04-forwardfix-rerun01_20260801_142241_722401259/scene_manifest.json` | 66,469 | `daa9ed8283c63cbc4b891a1a950cb49cce7fc020c7654733b7783a1c1f97eed7` | source-derived continuous scene manifest |
| `hub/runtime/oneclick_scene04-plant-20260801-recal10-forwardfix1-start2_live_scene04-forwardfix-rerun01_20260801_142241_722401259/terminal/terminal_evidence.json` | 10,889 | `9993c63fa0248052ebd1a5fc7c5ff61b2953a9d746729c346212d7a8cb3a98a8` | source-derived terminal evidence index |
| `hub/runtime/oneclick_scene04-plant-20260801-recal10-forwardfix1-start2_live_scene04-forwardfix-rerun01_20260801_142241_722401259/terminal/robot-1/rgb.jpg` | 120,280 | `869caa2fef3a6395f033eae9cb4660eeedcafcb3e6dc085f7efb6b772c6a765a` | observed post-arrival RGB |
| `hub/runtime/oneclick_scene04-plant-20260801-recal10-forwardfix1-start2_live_scene04-forwardfix-rerun01_20260801_142241_722401259/terminal/robot-1/depth.png` | 202,013 | `508f91548e1eab19add034492dc3f30bc103f7c48cbb5e9a4ab86adc6e4e165f` | observed post-arrival aligned depth |
| `hub/runtime/sessions/scene04-plant-20260801-recal10-forwardfix1-start2/session.json` | 4,358 | `45c65528f2a54a076c771cff2939110862971cde19cd203bf9efcd2e5ce140f7` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260801-recal10/shared_frame.json` | 6,442 | `9ddc80061af595a7fafa3fd125e9d420846520b542a96f53595a6e98a1354bf7` | observed and source-derived calibration |

机器可读归档：
[`scene04_plant_formal_experiment_04_success_20260801.json`](../manifests/scene04_plant_formal_experiment_04_success_20260801.json)。
独立最短路径记录：
[`scene04_plant_shortest_feasible_path_20260801.json`](../manifests/scene04_plant_shortest_feasible_path_20260801.json)。

## 媒体

[第三视角原片](../media/video/third_view/experiment_4/experiment_4_success_1.mp4)、
[Dashboard 原片](../media/video/dashboard/experiment_4/experiment_4_success_1_dashboard.mov)
和 [探索地图](../media/image/experiment_4_map.png) 按用户上传字节原样保留；
精确大小、时长/分辨率及 SHA-256 记录在机器可读归档中。

## 安全终态

- 语义到达终止批次已被两台机器人确认，最终均为零速度 `HOLDING`。
- Hub 仅发布有版本和有效期的高层目标；底盘本地停车与拒绝权限保持不变。
- 本轮运动许可已随实验结束而消费；下一轮需要新的现场运动确认。
