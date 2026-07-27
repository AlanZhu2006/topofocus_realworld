# Scene 02 · Plant：正式实验 02 Failure

日期：2026-07-28

Session：`scene02-plant-20260728-0720-yunjireanchor1-single2-r4`

Episode：`yunji-single-02`

范围：Yunji 单机运动；WSJ 底盘下电并始终 `HOLD`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` |

本轮由操作者指定替换原 Formal 02，归档为
`navigation_policy_failure / algorithmic_exploration_failure`：

> 源码一致的探索策略在 13 个完整 source round 内没有产生 plant 语义到达。
> Yunji 累计行驶 `7.425951 m`，其中操作者现场观察到一条探索分支朝目标反方向
> 延伸；随后连续两轮位移低于 `0.05 m`，进度保护以
> `failed_cross_round_no_progress_holding` 安全结束本轮。

这不是工程链路故障：当前提交与标定绑定的严格只读 debug 已通过，正式运行持续
接收新鲜同步输入，controller log 中无工程 error。公开分类保持“算法探索/导航
策略失败”；由于本轮没有封存独立目标区域标注，不提升为更严格的
`vlm_decision_failure`。

## 动作与终态

| Robot | 角色 | 动作 |
| --- | --- | --- |
| WSJ | inactive | 底盘下电，不在运动授权范围内；所有批次均为 `HOLD`。 |
| Yunji | active | 13 轮均为非语义 `FRONTIER_POINT` 探索或其净空投影；未出现 plant semantic arrival；累计路径 `7.425951 m`。 |

Round 7 的代表性 frontier `B` 位于 shared-world
`(-1.551943, -1.833756) m`，与操作者观察到的反向探索分支对应。终止前两次
跨轮位移分别为 `0.003037 m` 与 `0.002637 m`；第二次 stagnant 触发
`cross_round_no_progress_hold`。最终两机均 `HOLDING`、
`velocity_zero_confirmed=true`，Hub 已恢复双机 `GOAL=false`。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260728-0720-yunjireanchor1-single2-r4_live_scene02-plant_20260728_073321_124117639/episode_report.json` | 9,878 | `635564912651ebde53ada98d133bae6d37894fccf4aa4035eba81ca6e46269a5` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene02-plant-20260728-0720-yunjireanchor1-single2-r4_live_scene02-plant_20260728_073321_124117639/controller_events.jsonl` | 112,902 | `32547ff4a78710eb10c41506b2bfaad209134d131b6d81a1dd9eb08a5130f566` | source-derived controller event log |
| `hub/runtime/oneclick_scene02-plant-20260728-0720-yunjireanchor1-single2-r4_live_scene02-plant_20260728_073321_124117639/round_07_step_174/initial_batch.json` | 4,357 | `fcbf8dc15df753bf5a2c61aacd6502904233408a3298416e394c2d2a21f2af84` | source-derived representative frontier batch |
| `hub/runtime/oneclick_scene02-plant-20260728-0720-yunjireanchor1-single2-r4_live_scene02-plant_20260728_073321_124117639/round_13_step_324/cross_round_progress_guard.json` | 651 | `cf31d1d7dd8a9c8725562a02c3fe01cbbdf9bd93d4a07be7ea2d1bbf7becb8b6` | source-derived terminal no-progress guard |
| `hub/runtime/sessions/scene02-plant-20260728-0720-yunjireanchor1-single2-r4/session.json` | 4,456 | `0b11a923724b60c8604a97203a0ab661799a1430a1b2e169a22c9bb0944b1d55` | exact code/calibration/runtime session identity |
| `hub/runtime/calibration_sessions/scene02-plant-20260728-0720-yunjireanchor1/shared_frame.json` | 22,423 | `30a746b251aeca417e13cbfcdb53ff302104489517f56342963b163e3789e622` | observed/source-derived validated stationary re-anchor |

完整机器可读证据表见
[`scene02_plant_formal_experiment_02_failure_20260728.json`](../manifests/scene02_plant_formal_experiment_02_failure_20260728.json)。

## 替换与媒体边界

原 Formal 02 在提交 `dcc8812b027c40fad2716b8a097e45d226d46686` 中完整保留；
旧 manifest 的 SHA-256 为
`e237ad271f31707fa118cc40b252d0a627a785f61aa9e518b99ec7a1f546f33c`。
其视频不作为本轮替换样本的证据；本轮第三视角与 Dashboard 素材待独立绑定。
