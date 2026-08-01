# Scene 04 · Plant：正式实验 02 Failure

时间：2026-08-01 16:19:17 至 16:27:04（Asia/Shanghai）

Session：`scene04-plant-20260801-recal11-formal06-start1`

Episode：`formal06`

本记录按操作者指令替换先前的 Scene 04 Formal 02；旧运行仍保留在 Git
历史中，但不再属于当前五轮指标轨道。

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` | `13` |

Robot 1 在冻结 RGB 中正确识别到 `plant`，但现场确认机器人仍与实体目标
存在明显距离且没有继续接近，因此本轮按物理协议计为失败。自动报告中的
semantic `ARRIVED` 仅是未验证候选，不能覆盖现场物理验证。

## 双机轨迹

| Robot | Actual path | Start-to-stop displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `12.313735 m` | `2.632082 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `6.858408 m` | `5.928786 m` | `HOLDING`，零速度已确认 |

运行完成 13 个 source rounds，覆盖 source steps `0–299`，共发布 69 个
带版本和有效期的高层批次。终止后 Hub 的双机 GOAL 输出均关闭，两端
命令通路均验证为 `STOPPED`。

## Robot 1 为什么看到目标后没有继续动作

直接原因是语义目标的空间位置误投影到了 Robot 1 附近，而不是目标没有
送达或底盘没有执行：

1. 冻结帧 `421545` 的 YOLO 检测为 `potted plant`，置信度
   `0.886242`。它只确认当前 RGB 中存在 plant，不提供三维位置权威。
2. 语义地图只生成了 6 个 plant 栅格，其中心距 Robot 1 基座仅
   `0.642535 m`。执行适配器按源码保留 10 栅格、即 `0.50 m` 的语义
   approach 膨胀，并选择距机器人最近的可达边界点。
3. 解析后的本地 approach point 为
   `(3.656504, -1.342553) m`；当时 Robot 1 为
   `(3.592199, -1.377703) m`，两者仅相距 `0.073286 m`，已经小于
   `0.15 m` 的本地到达半径。因此新 POI 发布约 `0.287 s` 后，TinyNav
   就对同一个 decision 返回 `nav_done=true`，Hub 随即让双机 HOLD。
4. 这不是上一目标遗留的 `nav_done`：Robot 1 对本轮 decision 依次记录了
   `RECEIVED → LOCAL_GOAL_ACCEPTED → POI published → ARRIVED`，ID 完全一致。

因此，精确的失败归因是
`perception_mapping_failure/semantic_spatial_misprojection_premature_arrival`。
分类识别本身是正确的，错误发生在像素语义到共享地图位置的几何投影。

## 为什么判断为前景深度污染

冻结 RGB 显示 plant 被近处立柱部分遮挡。同一 YOLO 框内的对齐深度呈
明显双峰：

| Statistic | Value |
| --- | ---: |
| Valid depth pixels | `4,441` |
| `<1 m` pixels | `40.40%` |
| `>3 m` pixels | `59.60%` |
| Depth quantiles `q10/q25/q50/q75/q90` | `0.391 / 0.489 / 5.300 / 6.548 / 6.764 m` |

近处约 `0.4–0.5 m` 的立柱与远处约 `5–7 m` 的 plant/背景同时落入语义
窗口，和“plant 栅格被投到基座附近”的结果一致。现有归档没有单独封存
后端逐像素 segmentation mask，因此可以确定的是“语义空间误投影”；
更底层究竟是 segmentation 泄漏到立柱，还是 mask 与深度关联时混入前景，
属于由冻结 RGB-D 支持的机制判断，而不是直接观测事实。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal06-start1_live_scene04_20260801_161917_324159904/episode_report.json` | 20,550 | `e72a431a77cb5a8a77a96dfb0d3b492e2b9e6e11144a75b1c431d391c8874eb4` | source-derived navigation and automatic-terminal report |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal06-start1_live_scene04_20260801_161917_324159904/controller_events.jsonl` | 220,071 | `28ace9a700b4e57a820d04903118e6ed91b864cae0ec5789edc15a798240c1b8` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal06-start1_live_scene04_20260801_161917_324159904/round_12_step_299/vlm_candidate_batch.json` | 6,451 | `a7e8aa333443a99918b034503a57bd0c7ea2a419e15cba825454ddd3b7f49243` | unmodified VLM semantic-region candidate |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal06-start1_live_scene04_20260801_161917_324159904/round_12_step_299/semantic_execution_guard.json` | 2,184 | `5f5153b424803c156ad09bcd3eb5af1fbfed0b494667effe3ea86fc5286fd17f` | same-frame detector confirmation |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal06-start1_live_scene04_20260801_161917_324159904/round_12_step_299/shadow/inputs/yunji/source_421545.jpg` | 109,635 | `25236a59337d7e397e7b396acdfb5cc4b0e25867a227085394aa99cf95ce83c6` | observed frozen RGB |
| `hub/runtime/oneclick_scene04-plant-20260801-recal11-formal06-start1_live_scene04_20260801_161917_324159904/round_12_step_299/shadow/inputs/yunji/source_421545_depth.png` | 198,241 | `0082c1d7adf58232f72715834b3e7d5e036169d9ff063b007c939866a073a384` | observed frozen aligned depth |
| `hub/runtime/sessions/scene04-plant-20260801-recal11-formal06-start1/session.json` | 4,324 | `83d5f323ae4f79a057d37c8decdfb8ef8cd87748d037fd7beb8a2de0ba36eb81` | exact session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260801-recal11/shared_frame.json` | 6,445 | `b91fa62a5f27929876397faad2700b9c230397aa4db64745f6cc02b947665d0a` | reused shared-frame calibration |

机器可读归档：
[`scene04_plant_formal_experiment_02_failure_20260801.json`](../manifests/scene04_plant_formal_experiment_02_failure_20260801.json)。

## 媒体

- 第三视角：`media/video/third_view/experiment_4/experiment_4_failure_2.mp4`
  （`149.966667 s`，SHA-256
  `0605b0711577c96d0d15ff21da030a9c516d89a5551b4070ac61057ce690c131`）
- Dashboard：`media/video/dashboard/experiment_4/experiment_4_failure_2_dashboard.mov`
  （`302.719 s`，SHA-256
  `6c234a01d4c57e72ef921da85202e551160249d1f5795c578cd1e86c77f5c0ba`）

## 安全终态

- semantic-arrival 终止批次和最终双机 HOLD 均已确认。
- 两台机器人最终均为 `HOLDING`，且 `velocity_zero_confirmed=true`。
- Hub `goal_output_enabled` 对 Robot 0、Robot 1 均为 `false`。
- 两端命令通路只读核验均为 `STOPPED`；观察、地图和 Foxglove 保持运行。
