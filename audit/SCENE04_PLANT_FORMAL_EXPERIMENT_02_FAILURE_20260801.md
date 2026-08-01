# Scene 04 · Plant：正式实验 02 Failure

时间：2026-08-01 09:41:10 至 09:51:25（Asia/Shanghai）

Session：`scene04-plant-20260801-recal8-navfix1`

Episode：`navfix1-live01`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` | `16` |

本轮由操作者指定归档为 Scene 04 Formal 02 Failure。约 `600 s` 的正式
测试窗口内完成 16 个 source rounds，但没有生成经验证的 `plant` 语义
到达证据；最终双机同步 HOLD。本轮作为失败计入 Scene 04 指标，SR、
source-compatible SPL 和 Standard SPL 均为 `0`。

## 双机轨迹

| Robot | Actual path | Start-to-stop displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `16.910834 m` | `2.655550 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `13.391200 m` | `1.218478 m` | `HOLDING`，零速度已确认 |

运行覆盖 source steps `0–374`，共发布 92 个带版本和有效期的高层批次。
Robot 1 的路径长度约为净位移的 `10.9901×`，与现场观察到的走廊往返一致。

## Robot 1 为什么没有沿走廊持续前进

逐轮冻结证据表明，这不是定位漂移、地图阻塞或底盘拒绝。Robot 1 全程
`pose_jump_events=0`、`mapping_blocked_reason=null`，没有产生任何
`FAILED/REJECTED` 本地导航事件。往返来自高层目标连续性与备用 frontier
排序的组合：

1. 第 0 轮 Robot 1 沿走廊正向移动。第 1 轮新目标 C 位于已观测前进方向
   后方 `173.683°`，但仅距机器人 `0.885258 m`；第 2 轮目标 D 仍在后方
   `179.775°`，距离 `1.740169 m`。现行反向保护只把距离至少 `2 m` 且
   反向角超过 `90°` 的目标视为严重回退，因此两次都被记录为
   `target_within_direction_guard` 并放行，形成第一次掉头。
2. Robot 1 随后沿反方向继续推进。第 10 轮 VLM 的 B 仍在该方向前方，
   相对已观测进展向量仅偏 `27.592°`，但 B 在执行净空阶段命中了此前的
   `CROSS_ROUND_SOURCE_STALL` 失败记忆而被拒绝。系统改选备用 A；A 相对
   已观测进展方向反向 `170.551°`，执行点位于
   `(5.846684, -0.270020) m`，因此触发第二次掉头并沿走廊返回。
3. 反向候选排序只在“当前原始目标本身已经被判为严重回退”时才把非反向
   候选提前。第 10 轮的原始 B 是正向目标，所以 `backtrack_redirected=false`；
   下游净空层拒绝 B 后，仍按源排序首先尝试了反向 A。这是本轮往返的直接
   工程原因，不是“走廊不可达”，也不能单独归因于 VLM。
4. 返回开始后，第 12 轮 B 相对于新的正向进展变成 `154.928°` 的反向
   目标，保护器转而保留 A，继续驱动 Robot 1 返回。由此形成现场看到的
   “回头—又回来”，并消耗了正式测试窗口。

因此，本轮公开结果按“时限内未找到目标”记录；详细诊断归类为
`execution_engineering_failure/high_level_frontier_direction_oscillation`。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/episode_report.json` | 9,766 | `53c0340251e50b9965972dd642a511182d5a63667ed778d616693189fc401bf1` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/controller_events.jsonl` | 230,249 | `412886c11daf5da2ff6c80f6cd7c7082828bbf1ab9d358ee097da694a434bcb1` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/scene_manifest.json` | 193,191 | `fafd0819bbb9212faba5e1078be69014e8c4dafe733bede2323af2cfab31072c` | source-derived continuous scene manifest |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/navigation_failure_memory.json` | 25,563 | `366f1f702bf260c59f7c35866f175d5408bc54e47c50d42740d3ca640428059d` | source-derived bounded navigation-failure memory |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/round_01_step_024/source_replan_guard.json` | 9,298 | `a1a6f65cfa021a8548fe85c35717ee8cea0b6bfeb9f8e812f54e7836c9e096e9` | first short-range reversal evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/round_02_step_049/source_replan_guard.json` | 10,434 | `600686d0f518921927455d1802606f628d0eb279f9894a8a441c191aa272d8ee` | continued short-range reversal evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/round_10_step_249/source_replan_guard.json` | 36,910 | `05779d5c4450f8d8459eddbbf10778c6eebe2e2c88ba7552314610274b8401d1` | source target, failure-memory and reverse-fallback ordering evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/round_10_step_249/frontier_clearance_guard.json` | 39,617 | `f28882888a53ed4f36eea06f6e0895ac5c1938969ef0395d9164e6c3ccb1ab07` | reverse A execution-selection evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/round_12_step_299/source_replan_guard.json` | 38,167 | `71ddb3c140a85434a4e1884eff9d283b469de685bb7b2f332ae79f5bdf64b638` | return-direction retention evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-navfix1_live_scene04_20260801_094110_284492569/round_15_step_374/accepted/yunji/live_status.json` | 15,979 | `9be964f30282fde163f8cd1d57429a2edf10029825b5cd837eba94194f3f82ed` | observed Robot 1 terminal map and tracking health |
| `hub/runtime/sessions/scene04-plant-20260801-recal8-navfix1/session.json` | 4,266 | `01f4f80bc296d073d3243093a755e9c8f536fdc6fbb4a9be6c5534a3c439cc96` | exact session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260801-recal8/shared_frame.json` | 6,457 | `26b6d1533d23544f9488d770e11b7a0253f8f5bb1161fba86bfd2c09d1b14af0` | reused observed and source-derived shared-frame calibration |

机器可读归档：
[`scene04_plant_formal_experiment_02_failure_20260801.json`](../manifests/scene04_plant_formal_experiment_02_failure_20260801.json)。

## 安全终态

- 终止批次已由两台机器人确认；最终均为 `HOLDING`、`HUB_HOLD`。
- 两台机器人均记录 `velocity_zero_confirmed=true`。
- Hub 只发布带版本、有效期的高层目标，机器人本地停车与拒绝权限保持不变。
- 实验退出后 GOAL 通路已关闭，仅保留观察、地图与 Foxglove。
