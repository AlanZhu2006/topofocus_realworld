# Scene 01 · Chair：五次正式真机实验归档

日期：2026-07-25 · 场景：`scene01-chair` · 目标：`chair`
正式结果：`5/5 SUCCESS`

## 定量结果

| Run | 到达机器人 | 判定 | 实际路径 `P` | Source-compatible SPL | Standard SPL |
| --- | --- | --- | ---: | ---: | ---: |
| Formal 01 | WSJ | 自动 `LOCAL_PLANNER_ARRIVED` + 操作者确认 | `3.850792 m` | `0.628399` | `0.843982` |
| Formal 02 | WSJ | 自动 `LOCAL_PLANNER_ARRIVED` + 现场成功注释 | `2.760377 m` | `0.892911` | `1.000000` |
| Formal 03 | Yunji | 操作者按 `0.5 m` 真机终点半径确认 | `4.048842 m` | `0.864048` | `0.802699` |
| Formal 04 | Yunji | 自动 `LOCAL_PLANNER_ARRIVED` + 操作者归档 | `3.210222 m` | `0.956361` | `1.000000` |
| Formal 05 | WSJ | 自动 `LOCAL_PLANNER_ARRIVED` + 操作者确认实际到达 | `12.582981 m` | `0.288722` | `0.258285` |

汇总：

- `SR = 5 / 5 = 1.0`
- mean source-compatible SPL：`0.7260879584850242`
- mean Standard SPL：`0.7809932415154623`

Standard SPL 统一使用操作者独立测量的 Scene 01 最短可行路径
`L≈3.25 m`，计算式为 `S × L / max(L, P)`。Source-compatible SPL
使用成功机器人的起终点位移 `D`，计算式为 `S × D / max(D, P)`。

## 双机动作分析

| Run | WSJ 动作 | Yunji 动作 |
| --- | --- | --- |
| Formal 01 | 第一轮探索前沿；第二轮切换至 chair 语义区域并自动到达。 | 第一轮协调 HOLD；第二轮沿独立前沿推进，WSJ 到达后同步 HOLD。 |
| Formal 02 | 完成前沿探索后切换至 chair 语义区域并自动到达。 | 路径协调期间保持近静止，场景成功后同步 HOLD。 |
| Formal 03 | 完成首轮前沿探索，随后让出活动权并 HOLD。 | 接力完成前沿探索，再驶入 chair 的 `0.5 m` 成功区域。 |
| Formal 04 | 参与交替与并行前沿探索，终局保持独立探索腿。 | 完成前沿探索后切换至 chair 语义区域并自动到达。 |
| Formal 05 | 独立完成前沿探索，再持续导航至 chair 并自动到达。 | 两轮均按路径协调保持 HOLD。 |

## 正式实验与媒体绑定

| Run | Session / episode | 第三视角主文件 | Dashboard 主文件 |
| --- | --- | --- | --- |
| Formal 01 | `20260725-lab19-scene01-8ca1d52-yunjireboot1-r5 / trial-r5-01` | [`experiment_1_success_1.mp4`](../media/video/third_view/experiment_1/experiment_1_success_1.mp4) | [`experiment_1_success_1_dashboard.mov`](../media/video/dashboard/experiment_1/experiment_1_success_1_dashboard.mov) |
| Formal 02 | `lab-20260725-132014-wsjreanchor1-r2 / trial-reanchor1-r1` | [`experiment_1_success_2.mp4`](../media/video/third_view/experiment_1/experiment_1_success_2.mp4) | [`experiment_1_success_2_dashboard.mov`](../media/video/dashboard/experiment_1/experiment_1_success_2_dashboard.mov) |
| Formal 03 | `20260725-lab17-nearwall-fix / trial-05-nearwall-fix` | [`experiment_1_success_3.mp4`](../media/video/third_view/experiment_1/experiment_1_success_3.mp4) | [`experiment_1_success_3_dashboard.mov`](../media/video/dashboard/experiment_1/experiment_1_success_3_dashboard.mov) |
| Formal 04 | `20260725-lab21-wallfix-imudebounce-3a2d953 / trial-wallfix-imudebounce-r1` | [`experiment_1_success_4.mp4`](../media/video/third_view/experiment_1/experiment_1_success_4.mp4) | [`experiment_1_success_4_dashboard.mov`](../media/video/dashboard/experiment_1/experiment_1_success_4_dashboard.mov) |
| Formal 05 | `20260725-lab22-formal05-recalibration-ddsfix / scene01-chair-run05` | [`experiment_1_success_5.mp4`](../media/video/third_view/experiment_1/experiment_1_success_5.mp4) | [`experiment_1_success_5_dashboard.mov`](../media/video/dashboard/experiment_1/experiment_1_success_5_dashboard.mov) |

Formal 05 的公开展示片段覆盖实验开始至实际到达；其 SR/SPL 仍使用完整
episode report 中的真实里程，不使用视频时长推算路径。每次实验均提供
8 秒第三视角与 Dashboard 循环 GIF；完整 H.264 文件保留在 `media/demo/`。

## 证据与复现

机器可读归档：

- [`scene01_chair_formal_experiments_20260725.json`](../manifests/scene01_chair_formal_experiments_20260725.json)
- [`scene01_chair_shortest_feasible_path_20260725.json`](../manifests/scene01_chair_shortest_feasible_path_20260725.json)

关键 runtime 身份如下；runtime 文件由项目规则保持本地忽略，提交的
manifest 保存其源路径、字节数、SHA-256 与证据分类。

| Run | Episode report SHA-256 | 字节数 |
| --- | --- | ---: |
| Formal 01 | `e25e6853c4272be698b3049a3fec11a9f296c1d8d7b55397a82e5631b6c264c4` | `20,686` |
| Formal 02 | `d51e394ec9be12ab5044d970ad343f41f47ad0d00839aae821452912977ae913` | `20,310` |
| Formal 03 | `e31d66e4c2a8ed136238b8670c22b9505daf520fc7715f05d4b4ce020dda769d` | `9,777` |
| Formal 04 | `2f9036f80879d142f0beac4f6165a40be83634630d41d84b4aa26c76de1df169` | `20,592` |
| Formal 05 | `e91cbc392b692cfd2fab5dcc97d7986c3a5291eb9269ae16d771b6cae85e3d41` | `20,752` |

所有用户提供的主视频均按原始字节通过 Git LFS 保存；网页 MP4 与 GIF
均标记为 source-derived。未修改 `source/` 或 `dependencies/`。
