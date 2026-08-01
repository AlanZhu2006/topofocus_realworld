# Scene 04 · Plant：正式实验 05 Success

时间：2026-08-01 15:58:34 至 16:03:25（Asia/Shanghai）

Session：`scene04-plant-20260801-recal11-formal05-persistent1-start1`

Episode：`formal05-restart3`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| `SUCCESS` | `1` | `1.0` | `0.829375` | `0.901648` | `7` |

操作者在实验后确认本轮成功，归档为 Scene 04 Formal 05。Robot 1 在第七个
source round 切换至检测到的 plant 语义区域，自动发出
`LOCAL_PLANNER_ARRIVED`；0.078 s 后保存的 RGB-D 提供到达后证据。Robot 0
完成其分配的探索段，Robot 1 到达后双机同步进入终止 HOLD。

Standard SPL 使用操作者独立测得的近似最短可行路径 `L≈11 m`：
`11 / max(11, 12.199885) = 0.901648`。

## 双机轨迹

| Robot | Actual path | Start-to-stop/arrival displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `3.738739 m` | `0.785709 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `12.199885 m` | `10.118279 m` | `ARRIVED` 后同步 `HOLDING`，零速度已确认 |

Source-compatible SPL 使用到达机器人 Robot 1 的起点至到达点位移作为参考：
`10.118279 / 12.199885 = 0.829375`。运行覆盖 source steps
`[0, 24, 49, 74, 99, 124, 149]`，共发布 42 个带版本和有效期的高层批次。

Robot 0 的路径较短有明确运行证据：启动检查中其局部可达连通区为 143 个
栅格，而 Robot 1 为 1,782 个；第 1 轮 Robot 0 的候选点因没有足迹净空
接近点而 HOLD，后续远端 frontier 多次被投影为启动区附近的 bounded-safe
点。最后一轮 Robot 0 的剩余距离 `0.709 m` 在 `20.438 s` 内未改善
`0.05 m`，因此作为可恢复的 `LOCAL_PLANNER_NO_PROGRESS` 被隔离，Robot 1
继续执行语义到达。本轮没有 `LOCAL_CHASSIS_COMMAND_NOT_EXECUTED` 事件。

## 到达证据

- Robot 1：`potted plant`，YOLO 置信度 `0.866008`，语义区域 54 个栅格。
- 自动到达事件：sequence `420108`，reason `LOCAL_PLANNER_ARRIVED`。
- 到达后 RGB-D：sequence `420165`，相对到达事件延迟 `0.077891 s`。
- 自动报告保持其不可变边界 `official_success_verified=false`；独立的现场操作者
  确认“归档为成功”提供正式物理协议分类。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/episode_report.json` | 20,909 | `6a66e9e8a16ed488d90b7c7dbf5762df40c0f28493178ed5fcf965dde601816b` | source-derived automatic episode report |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/controller_events.jsonl` | 95,665 | `a62f81ee74046cd98f195cf1a32570ea773f2919972500abf99498ae0930d7aa` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/scene_manifest.json` | 94,422 | `3cb34a92ec0c6c22c9050022abbdaa5cc426dd9ff75e33203a041a3f59ed341f` | source-derived continuous scene manifest |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/round_06_step_149/shadow/source_goal_masks/yunji_plant.png` | 1,044 | `8a28e390b9666a456d9a837ce58b86797bc8170750ccb0b656ed1652b68e45df` | source-derived Robot 1 plant goal mask |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/terminal/terminal_evidence.json` | 10,792 | `b86c622c80e9889e366cb697188602d5ceacc8efdfa47f9e0d72021e8c045558` | source-derived terminal evidence index |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/terminal/robot-1/rgb.jpg` | 103,949 | `16c421779bd71ce8b0450e35056bc35112999286d68d85cc8a32bfca1aa2b803` | observed post-arrival RGB |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal05-persistent1-start1_live_scene04_20260801_155833_621487143/terminal/robot-1/depth.png` | 204,577 | `b42ebbe39a59b6ccd87f9a40671b4b49bf15b8cf57f8acbaba182b93bf5da1d8` | observed post-arrival aligned depth |
| `hub/runtime/sessions/scene04-plant-20260801-recal11-formal05-persistent1-start1/session.json` | 4,429 | `691ccb57ee5740f31f5920f6f25677e089210c7b103402a55ce6e3c104003530` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260801-recal11/shared_frame.json` | 6,445 | `b91fa62a5f27929876397faad2700b9c230397aa4db64745f6cc02b947665d0a` | observed and source-derived calibration |

机器可读归档：
[`scene04_plant_formal_experiment_05_success_20260801.json`](../manifests/scene04_plant_formal_experiment_05_success_20260801.json)。
独立最短路径记录：
[`scene04_plant_shortest_feasible_path_20260801.json`](../manifests/scene04_plant_shortest_feasible_path_20260801.json)。

## 媒体

[第三视角原片](../media/video/third_view/experiment_4/experiment_4_success_2.mp4)
和 [Dashboard 原片](../media/video/dashboard/experiment_4/experiment_4_success_2_dashboard.mov)
按用户上传字节原样保留；精确大小、时长、编码、分辨率及 SHA-256 记录在
机器可读归档中。

## 安全终态

- 语义到达终止批次已被两台机器人确认，最终均为零速度 `HOLDING`。
- Hub 仅发布有版本和有效期的高层目标；底盘本地停车与拒绝权限保持不变。
- 本轮运动许可已随实验结束而消费；下一轮需要新的现场运动确认。
