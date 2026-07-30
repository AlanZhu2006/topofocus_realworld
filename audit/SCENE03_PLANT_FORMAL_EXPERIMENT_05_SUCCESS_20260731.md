# Scene 03 · Plant：正式实验 05 Success

时间：2026-07-31 04:55:16 至 05:02:33（Asia/Shanghai）

Session：`scene03-plant-20260731-0444-robot1-continue01-start`

Episode：`robot1-continue02`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `SUCCESS` | `1` | `1.0` | `0.446727` | `待测` |

本轮采用双机协同角色分配：Robot 0 由协调策略保持 `HOLD`，持续保留观测、
地图和里程计 provenance；Robot 1 执行长程 frontier 探索与 plant 语义目标
接近。Robot 1 最终自动报告语义区域 `ARRIVED`，本地规划器确认停车，终端
RGB 中 plant 与白色花盆清晰位于画面中央，现场操作者随后确认成功并指定
归档为 Scene 03 Formal 05 Success。

自动报告中的 `official_success_verified=false` 仅表示机器流程本身不具备
独立现场目标区域标注。原始报告保持不变，现场操作者确认作为独立物理成功
证据记录。

## 指标

Robot 1 的实际路径为 `P=19.152682984 m`，起点与终点分别为
`(-3.238882, -2.718769) m` 和 `(5.203149, -1.326732) m`，净位移
`D=8.556029879 m`。因此：

```text
source-compatible SPL = S * D / max(D, P)
                      = 1 * 8.556029879 / 19.152682984
                      = 0.446727484
```

Scene 03 尚未提供独立测量的最短可行路径，因此 Standard SPL 保持未计算。

## 双机协同动作

| Robot | 协同角色 | 路径与位姿证据 | 结果 |
| --- | --- | --- | --- |
| Robot 0 | 策略 `HOLD` | 原始累计里程计字段 `2.477757 m`；起终点净变化仅 `0.006053 m`，作为静止观测 provenance 保留，不解释为实际路线执行。 | 全程维持 HOLD，保留共享观测/地图输入，终态零速。 |
| Robot 1 | 长程探索与语义接近 | 实际路径 `19.152683 m`，净位移 `8.556030 m`。 | 完成 frontier 探索、plant 语义区域接近并自动 ARRIVED；现场确认成功。 |

## 规划过程

- 共完成 `11` 个 source planning round，对应 source step
  `0, 24, 49, 74, 99, 124, 149, 174, 199, 224, 249`。
- Robot 1 先沿长程 frontier B 探索；source step `74` 的延续目标被本地
  规划器判定 `LOCAL_GOAL_UNREACHABLE`，系统立即隔离该目标并在确认 HOLD
  后重新规划。
- 后续 bounded-safe approach 向 frontier C/A 推进；source step `124`
  再次出现局部不可达，系统同样隔离失败区域，后续轮次离开该区域并继续取得
  有效位移。
- source step `249` 首次形成最终 plant 语义目标：检测标签
  `potted plant`，模型置信度 `0.851619`，投影语义区域为 `5` 个栅格。
- 最终语义 leg 返回 `LOCAL_PLANNER_ARRIVED`；到达观测序列为 `393032`，
  `0.413156 s` 后封存序列 `393102` 的 RGB-D 与地图快照。

## 场景累计指标

Scene 03 当前共 5 次正式实验：3 次成功、2 次失败，SR 为 `3/5=0.6`。
失败轮按 0 计入后，mean source-compatible SPL 为 `0.365962`；
mean Standard SPL 等待 Scene 03 的独立最短可行路径测量。

## 安全终态

- 语义到达终止批次将 Robot 0 与 Robot 1 同时置于 `HOLDING`，两端均确认
  `velocity_zero_confirmed=true`。
- 清理阶段关闭 Hub GOAL 输出和两端运动命令路径，仅保留只读观测、地图与
  Foxglove。
- Hub 只发布带版本和有效期的高层目标；机器人始终保留本地拒绝与停车权。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/episode_report.json` | 20,612 | `ccff4dac6e11f15c141b3ec3a6ddd49382e9467e51b613d9bfa83eae0b73df75` | source-derived navigation report |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/controller_events.jsonl` | 131,570 | `08219d14859d8bd0b6b66893472ac7e727f8429ea2d71b6ad0bd402fd861423e` | source-derived controller log |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/scene_manifest.json` | 122,353 | `36cc1fd1f448e2559817c7fa7420b7e8bad2aa8f2cf97854bb2316207ad63f08` | continuous scene manifest |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/round_10_step_249/shadow/shadow_manifest.json` | 37,870 | `34f04f15a9fcde2b7e08cdf67880422ce1fcac4fde51185d3f94025e7a77883c` | frozen VLM/semantic projection |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/round_10_step_249/shadow/source_goal_masks/yunji_plant.png` | 997 | `d854d82cb1800b718ee8f266b0b303c17770a1bc51e4dacd919376d15e2be570` | five-cell semantic goal mask |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/terminal/terminal_evidence.json` | 10,476 | `ef3a6699f6e33a995c6e963815dc04fc33036c6e31bddb0136fb7c4bd241fd17` | terminal evidence index |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/terminal/robot-1/rgb.jpg` | 111,337 | `417ae2c55701cb2884bb0d385a59026f8076fa7cb6149c2a0b75135be28844a1` | observed post-arrival RGB clearly containing plant |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/terminal/robot-1/depth.png` | 222,621 | `ab56ee39d1c4c89dbbc9920c4f0c2a63f8d4de3f6d13c165909ccd7a5a49ec3c` | observed aligned depth |
| `hub/runtime/sessions/scene03-plant-20260731-0444-robot1-continue01-start/session.json` | 4,483 | `19d8c6d628d149d8d868c96b83475cc2b82cd8576933da09101c221ea7bdfc35` | exact session identity |
| `hub/runtime/calibration_sessions/scene03-plant-20260731-dual-power-reanchor/shared_frame.json` | 29,278 | `c6c924305602fd8a8417959a1c1413aa10aaae754960589c0646bffa9612df8c` | dual stationary-reanchor shared frame |

机器可读归档：
[`scene03_plant_formal_experiment_05_success_20260731.json`](../manifests/scene03_plant_formal_experiment_05_success_20260731.json)。
