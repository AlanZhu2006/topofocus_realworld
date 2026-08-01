# Scene 04 · Plant：正式实验 03 Failure

时间：2026-08-01 10:23:56 至 10:30:18（Asia/Shanghai）

Session：`scene04-plant-20260801-recal8-direction1`

Episode：`direction-fix01-live01`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration loops |
| --- | ---: | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` | `16` |

本轮由现场操作者指定归档为 Scene 04 Formal 03 Failure。正式实验的
16 次探索循环达到本轮最大限额；末端已能看到 `plant`，但两台机器人均未
进入目标成功区域，也没有形成经验证的语义到达终态，因此按失败计入指标。
SR、source-compatible SPL 和 Standard SPL 均为 `0`。

## 双机轨迹

| Robot | Actual path | Start-to-stop displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `5.303672 m` | `2.459419 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `8.733911 m` | `7.989057 m` | `HOLDING`，零速度已确认 |

## 证据边界

- `16` 次探索循环、Formal 03 编号以及“末端看到目标但没有走到”来自现场
  操作者记录。
- 冻结 `episode_report.json` 提供双机实际里程、终态和没有语义 ARRIVED 的
  自动证据；该报告在结束封存时记录了 `9` 个完整 source rounds。人工记录
  的实验循环与自动 source-round 计数是不同口径，机器归档同时保留两者。
- 公开失败原因按正式实验协议记为“达到最大探索循环限额但未抵达目标”；
  自动报告的内部终止字符串仅作为工程溯源，不改变本轮 SR/SPL 结果。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-direction1_live_scene04-direction-fix01_20260801_102355_660281940/episode_report.json` | 9,903 | `74d0baad8484183eb3f158125269bf7424277d06de20284f71dbfa9aa9f9bc78` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-direction1_live_scene04-direction-fix01_20260801_102355_660281940/controller_events.jsonl` | 130,429 | `e48f8d6f06ddb37f1929ebe86eddde492c3963611be1710c6931d63e09599951` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260801-recal8-direction1_live_scene04-direction-fix01_20260801_102355_660281940/scene_manifest.json` | 110,624 | `302138af3a264b19a1a861b3b9566915ba36f0db8e512ce76c6d6ae53cdcb3a1` | source-derived continuous scene manifest |
| `hub/runtime/sessions/scene04-plant-20260801-recal8-direction1/session.json` | 4,271 | `4bbc313e39f9e8ce6b283238706e4abdcc75c4d87c3acb1068813f237f3f70e9` | exact session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260801-recal8/shared_frame.json` | 6,457 | `26b6d1533d23544f9488d770e11b7a0253f8f5bb1161fba86bfd2c09d1b14af0` | reused observed and source-derived shared-frame calibration |

机器可读归档：
[`scene04_plant_formal_experiment_03_failure_20260801.json`](../manifests/scene04_plant_formal_experiment_03_failure_20260801.json)。

## 安全终态

- 终止后两台机器人均为 `HOLDING`、`HUB_HOLD`。
- 两台机器人均记录 `velocity_zero_confirmed=true`。
- Hub 仅发布带版本和有效期的高层目标，机器人本地停车与拒绝权限保持不变。
- 实验结束后未保留运动权限。
