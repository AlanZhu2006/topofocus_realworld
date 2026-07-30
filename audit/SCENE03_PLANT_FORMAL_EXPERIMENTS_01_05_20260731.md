# Scene 03 · Long-range Cooperative Plant：正式实验 01–05

日期：2026-07-30 至 2026-07-31

目标：`plant`

协议：双机长程协同探索，物理成功半径 `0.5 m`

## 汇总

| Formal episodes | Successes | Failures | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: | ---: |
| `5` | `3` | `2` | `0.6` | `0.365961721746991` | `0.54619361696575` |

Standard SPL 使用操作者独立提供的近似最短可行路径 `L≈14 m`：

```text
Standard SPL = S * L / max(L, P)
```

两次 failure 均按零计入两个均值。Formal 03–05 的 Standard SPL 分别为
`1.000000`、`1.000000` 和 `0.730968`。

## 双机轨迹与结果

| Run | Result | Robot 0 trajectory | Robot 1 trajectory | Source-compatible SPL | Standard SPL |
| --- | --- | ---: | ---: | ---: | ---: |
| Formal 01 | FAILURE | `18.577107 m` | `14.162235 m` | `0.0` | `0.0` |
| Formal 02 | FAILURE | `5.754388 m` | `17.902160 m` | `0.0` | `0.0` |
| Formal 03 | SUCCESS | `9.037490 m` | `11.606679 m` | `0.689524` | `1.000000` |
| Formal 04 | SUCCESS | `9.391253 m` | `13.010775 m` | `0.693557` | `1.000000` |
| Formal 05 | SUCCESS | policy `HOLD`, `0.006053 m` net | `19.152683 m` | `0.446727` | `0.730968` |

Formal 05 的 Robot 0 原始累计里程计字段为 `2.477757 m`，起终点净变化仅
`0.006053 m`；该字段作为观测 provenance 保留，不解释为真实路线执行。
Robot 1 为 Formal 03–05 的到达机器人。

## 运行摘要

- Formal 01：探索在测试时限内未完成经验证的 plant 到达，记为
  time-limit failure。
- Formal 02：探索在测试时限内未发现并抵达经验证的 plant 目标，记为
  time-limit failure。
- Formal 03：双机沿长程 frontier 探索；Robot 1 切换到 plant 语义区域，
  现场操作者确认物理到达。
- Formal 04：Robot 0 沿独立 frontier 推进；Robot 1 完成 plant 语义路线并
  自动 `LOCAL_PLANNER_ARRIVED`，现场操作者确认成功。
- Formal 05：协调策略令 Robot 0 保持 `HOLD` 并保留共享观测；Robot 1 完成
  长程探索、切换到 plant 语义区域并自动 `LOCAL_PLANNER_ARRIVED`，终端 RGB-D
  与现场确认共同支持成功结论。

## 地图与视频

融合语义地图
[`experiment_3_map.png`](../media/image/experiment_3_map.png)
包含 Robot 0 蓝色轨迹和 Robot 1 绿色轨迹。该用户提供观测图大小为
`156,500 bytes`，SHA-256 为
`95f9c92f31f84c59dcdc691ece700c41e725da3171bcbb488573905c325fce32`。

| Run | Third-view master | Dashboard master |
| --- | --- | --- |
| Formal 01 | [`experiment_3_failure_1.mp4`](../media/video/third_view/experiment_3/experiment_3_failure_1.mp4) | [`experiment_3_failure_1_dashboard.mov`](../media/video/dashboard/experiment_3/experiment_3_failure_1_dashboard.mov) |
| Formal 02 | [`experiment_3_failure_2.mp4`](../media/video/third_view/experiment_3/experiment_3_failure_2.mp4) | [`experiment_3_failure_2_dashboard.mov`](../media/video/dashboard/experiment_3/experiment_3_failure_2_dashboard.mov) |
| Formal 03 | [`experiment_3_success_1.mp4`](../media/video/third_view/experiment_3/experiment_3_success_1.mp4) | [`experiment_3_success_1_dashboard.mov`](../media/video/dashboard/experiment_3/experiment_3_success_1_dashboard.mov) |
| Formal 04 | [`experiment_3_success_2.mp4`](../media/video/third_view/experiment_3/experiment_3_success_2.mp4) | [`experiment_3_success_2_dashboard.mov`](../media/video/dashboard/experiment_3/experiment_3_success_2_dashboard.mov) |
| Formal 05 | [`experiment_3_success_3.mp4`](../media/video/third_view/experiment_3/experiment_3_success_3.mp4) | [`experiment_3_success_3_dashboard.mov`](../media/video/dashboard/experiment_3/experiment_3_success_3_dashboard.mov) |

原片按上传字节通过 Git LFS 保留；README 使用 H.264 `yuv420p` MP4 与
64-frame / 8 fps GIF 副本。每一轮的精确大小、时长、SHA-256、运行绑定和
证据分类均记录在对应机器可读清单中。

## 证据索引

- [Formal 01](SCENE03_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260730.md) /
  [machine record](../manifests/scene03_plant_formal_experiment_01_failure_20260730.json)
- [Formal 02](SCENE03_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260731.md) /
  [machine record](../manifests/scene03_plant_formal_experiment_02_failure_20260731.json)
- [Formal 03](SCENE03_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260731.md) /
  [machine record](../manifests/scene03_plant_formal_experiment_03_success_20260731.json)
- [Formal 04](SCENE03_PLANT_FORMAL_EXPERIMENT_04_SUCCESS_20260731.md) /
  [machine record](../manifests/scene03_plant_formal_experiment_04_success_20260731.json)
- [Formal 05](SCENE03_PLANT_FORMAL_EXPERIMENT_05_SUCCESS_20260731.md) /
  [machine record](../manifests/scene03_plant_formal_experiment_05_success_20260731.json)
- [Five-run aggregate](../manifests/scene03_plant_formal_experiments_20260731.json)
- [Independent shortest-path measurement](../manifests/scene03_plant_shortest_feasible_path_20260731.json)

## 安全终态

每轮终止后，Hub 运动输出均关闭；两台机器人保持零速或策略 `HOLD`。Hub
只发布带版本和有效期的高层目标，机器人本地拒绝与停车权始终保留。
