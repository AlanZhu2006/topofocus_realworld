# Scene 02 · Plant：正式实验 01 Failure

日期：2026-07-28

Session：`scene02-plant-20260728-044246-recal2`

Episode：`scene02-plant-recal1-20260728-044959`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` |

失败 episode 的 SPL 贡献恒为 `0`，因此本行不依赖尚未独立测量的
Scene 02 最短可行路径。此次归因为
`execution_engineering_failure/local_planner_trajectory_stale`，不是 VLM
failure。

## 双机动作

| Robot | 路径 | 动作 |
| --- | ---: | --- |
| WSJ / Robot 0 | `6.104564 m` | 第 1 轮 HOLD；第 2、3 轮探索；第 4 轮切换到 plant 语义区域并继续导航，随后因轨迹发布失联超时而 fail-closed。 |
| Yunji / Robot 1 | `1.905387 m` | 第 1 轮探索；第 2、3 轮由净空/路线协调 HOLD；第 4 轮 frontier 被本地判为 `LOCAL_GOAL_UNREACHABLE` 后单机隔离。 |

第 4 轮的冻结 VLM 候选已为 WSJ 生成 `plant` 语义区域：
9 个语义栅格，展示质心为 shared-frame
`(1.014268, 7.828397) m`。因此目标识别与高层选择不是本轮终止原因。

## 终止原因

WSJ 机器人端的最后证据为：

- Router：`NAVIGATING / ONLINE_PATH_READY`
- 剩余有效路线：约 `2.5 m`
- Router waypoint：`(1.025, 5.425) m`
- 轨迹曾在当前 authority 内出现：`true`
- 轨迹 age：`3.365452042 s`
- 当时终止恢复阈值：`3.0 s`
- 最新 raw command：`[0.0, 0.0]`

轨迹失联满 `1.0 s` 后物理速度门已经关闭并输出零；随后仅因
`3.365 > 3.0 s` 被终止为 `LOCAL_PLANNER_PATH_STALE`。这说明有效路由
仍在，但局部规划轨迹发布发生短暂空窗。

## 最小修复

修复仅修改 `hub/` 部署层：

1. 轨迹超过 `1.0 s` 未更新时，物理速度门仍立即关闭；
2. 已经产生过有效轨迹的 semantic leg，其静止重规划窗口由
   `3.0 s` 增至 `5.0 s`；
3. 从未产生过轨迹的目标仍使用原有 `1.5 s` start grace，不会把真实
   no-path 延迟为长等待；
4. WSJ 与 Yunji 启动器均显式传入相同的 `1.0/5.0 s` 合同；
5. WATER watchdog、TinyNav 碰撞检查、lease、occupancy、定位、路线及
   robot-local stop/reject authority均未放宽。

| Deployment file | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/robot_overlay/v2_wsj_receiver.py` | 119,935 | `86887b869989df0324efda3a4893176e08ab07c890f588beefde73d56dbaa9dc` | source-derived common TinyNav receiver repair |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 33,408 | `007ffe8128049669c6ad23e3067c1fe801095ba57e9c6ee38c925ab3b01e508b` | source-derived explicit WSJ timeout contract |
| `hub/robot_overlay/start_yunji_v2.sh` | 24,669 | `6411477da2665d8670cb047f87c35605b14fa2904c1c770c2b3c626b73b3aafc` | source-derived explicit Yunji timeout contract |

Regression coverage reproduces the observed `3.365452042 s` gap: the physical
gate is closed but the leg is non-terminal at that point, and becomes terminal
only after `5.001 s`. The targeted receiver suite passed 45 tests and the full
Hub suite passed 599 tests. These are offline source-derived validations;
post-fix physical behavior remains unverified.

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260728-044246-recal2_live_scene02-plant_20260728_045111_880186006/episode_report.json` | 9,919 | `1a96f0d2364050b577897875607878886b3c8b568b1a165606871d73ce83ffe5` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene02-plant-20260728-044246-recal2_live_scene02-plant_20260728_045111_880186006/controller_events.jsonl` | 38,550 | `ed0bd91a5de6204559be2df642cdb087115a0569a8626afd9474698c9a984f02` | source-derived controller event log |
| `hub/runtime/oneclick_scene02-plant-20260728-044246-recal2_live_scene02-plant_20260728_045111_880186006/scene_manifest.json` | 31,929 | `beea265906bb24a9a70ebc59df26c21cd5ede02878ae23cc5801061a6658f353` | source-derived continuous scene manifest |
| `hub/runtime/oneclick_scene02-plant-20260728-044246-recal2_live_scene02-plant_20260728_045111_880186006/round_03_step_074/vlm_candidate_batch.json` | 6,548 | `bbbfc5d3681f9e68334716506107c1ce7d8eca56433c0928cd90be638832718d` | source-derived unmodified VLM candidate batch |
| `hub/runtime/sessions/scene02-plant-20260728-044246-recal2/session.json` | 4,283 | `1bdebe420a829d1ff0190128c93f1efc2fbee4610040c3ef40f17bca934b8212` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene02-plant-20260728-044246-recal2/shared_frame.json` | 6,029 | `5f95966f2c201467af549fc845d0265e9ea50cd12595c5931082451a647cde07` | observed and source-derived calibration |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260727T205051Z.jsonl` | 97,692 | `bfbdc28e3740990fdcff7d1add67c4f38a1e1d540e2e726c6ba5a9f22bcba690` | remotely observed WSJ log; retained on robot |

机器可读归档：
[`scene02_plant_formal_experiment_01_failure_20260728.json`](../manifests/scene02_plant_formal_experiment_01_failure_20260728.json)。
episode 清理后两台机器人均为 `HOLDING` 且
`velocity_zero_confirmed=true`，Hub 已恢复 `GOAL=false`。
