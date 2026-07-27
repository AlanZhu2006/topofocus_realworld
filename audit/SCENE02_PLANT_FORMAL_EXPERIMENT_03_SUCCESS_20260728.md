# Scene 02 · Plant：正式实验 03 Success

日期：2026-07-28
Session：`scene02-plant-20260728-yunji-single2`
Episode：`yunji-single-01`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `SUCCESS` | `1` | `1.0` | `0.865191906` | `0.837668869` |

Yunji 的实际路径 `P=8.356523990 m`，起点到终点位移
`D=7.229996923 m`，因此 source-compatible SPL 为
`D/max(D,P)=0.865191906`。本轮结束后 Scene 02 已归档三次正式实验，
其中一次成功：`SR=1/3=0.333333333`，计入失败零贡献后的平均
source-compatible SPL 为 `0.288397302`。

## Standard SPL 更新 — 2026-07-28

操作者事后提供了独立测量的最短可行路径：

> 大概直线距离应该是7m 可以计算standard spl

据此记录 `L≈7 m`（近似值，不复用 Scene 01 的 `3.25 m`），得到官方 Standard
SPL：

```text
standard SPL = S * L / max(L, P)
             = 1 * 7 / max(7, 8.356523990369512)
             = 0.8376688690258246
```

计入本次成功后，Scene 02 计入失败零贡献的平均 Standard SPL 为
`0.27922295634194155`（Formal 01/02 两次失败各贡献 `0`）。该 `L≈7 m`
适用于使用同一起点/目标布局的任何当前或未来 Scene 02 正式实验；失败
episode 无论 `L` 为何均因 `S=0` 而 Standard SPL 恒为 `0`。

## 执行数据

- 本轮是明确授权的 Yunji 单机 live scope：Yunji 是唯一 active robot。
- WSJ 在全部 11 个源规划轮中均为 `HOLD`，未获得 live motion authority。
- 前 6 轮为 frontier 探索；第 6 轮首次生成 `plant` 语义区域目标。
- 第 10 轮 Yunji 返回 `LOCAL_PLANNER_ARRIVED`，终点图像清晰包含 plant。
- 终止后自动发布双机 HOLD、封存终点 RGB-D/地图，并确认 Hub 双机
  `GOAL=false`。

这不是实际路径冲突记录：11 次 route-conflict guard 均为
`single_or_no_active_route`。WSJ HOLD 的可验证原因是操作者限定的单机执行
范围。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260728-yunji-single2_live_scene02-plant_20260728_070156_944397147/episode_report.json` | 20,525 | `6dbb8ade6043c1bf90e34027d65ef00168be9cb402799c3e8b8c34cd892fb302` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene02-plant-20260728-yunji-single2_live_scene02-plant_20260728_070156_944397147/controller_events.jsonl` | 86,252 | `f1c30e92b64bc6693ea7a2496314100ae7a7f7367f4871cd27149e249ae19d32` | source-derived controller event log |
| `hub/runtime/oneclick_scene02-plant-20260728-yunji-single2_live_scene02-plant_20260728_070156_944397147/terminal/terminal_evidence.json` | 10,400 | `496eb777513b8236f863491c9334d1c275c5f7cae7c537ed72228f8bc0beb801` | source-derived terminal evidence index |
| `hub/runtime/oneclick_scene02-plant-20260728-yunji-single2_live_scene02-plant_20260728_070156_944397147/terminal/robot-1/rgb.jpg` | 127,823 | `9bfcebb1d9162fe2bbb20b3cdfc80ca1b44217ed698572cac7df9b157d6bbe4f` | observed terminal RGB visibly containing plant |
| `hub/runtime/sessions/scene02-plant-20260728-yunji-single2/session.json` | 4,267 | `2b6f3cb63e6e4260299a38b69d56b52a12f219ed6aaa667247fe4dcba13ed011` | source-derived exact session identity |
| `hub/runtime/calibration_sessions/scene02-plant-20260728-062248-recal3/shared_frame.json` | 6,008 | `bab3dfc7f09699f20d8977cfe66c3985c9e5050fb18d9ded9af68b384df3ee3c` | observed/source-derived moved-board calibration |

机器可读归档：
[`scene02_plant_formal_experiment_03_success_20260728.json`](../manifests/scene02_plant_formal_experiment_03_success_20260728.json)。

## 媒体

操作者提供的第三视角与 Dashboard 录屏已通过 Git LFS 按字节原样提交到
`media/video/**`；浏览器可播放 H.264 版本与内嵌预览 GIF 位于 `media/demo/`，
命名与编码方式与 Formal 01/02 一致（`libx264`，GIF 为整段视频加速压缩至
8fps/8s 循环，无海报截图）。

| 素材 | 字节 | 时长 | SHA-256 |
| --- | ---: | ---: | --- |
| `media/video/third_view/experiment_2/experiment_2_success_1.mp4` | 15,100,041 | 199.1 s | `69ce83683ca33775fbc1c9af1a1239671b7776ad3c2f3413cebbc40a1e8d65ef` |
| `media/video/dashboard/experiment_2/experiment_2_success_2_dashboard.mov` | 193,595,030 | 289.368 s | `ba360ba29117906b2e935a1726d7c5f2ed5aeaefe7011eeca25d1687da19356e` |
| `media/demo/scene02_formal_03_third_view.mp4` | 19,119,038 | 199.1 s | `1fa34b70f29ed75829dde9ed0f95d88e346eed7fed39f27f985ad089137a062e` |
| `media/demo/scene02_formal_03_dashboard.mp4` | 3,496,392 | 289.3 s | `1c9ce1a52d31ee718fdb3086b2f0b528994924c1cad29548c7d166c1eb438b84` |
| `media/demo/scene02_formal_03_preview.gif` | 10,365,072 | 8.01 s | `5e049f1c7759c04f5a9381a914e8dc4c495016491c6b4c872a82cbab6d7acbea` |
| `media/demo/scene02_formal_03_dashboard.gif` | 1,912,186 | 8.01 s | `8be21251cf6c35df22e761eaeff11b23e7b063b15b70f2c6a43c4c0cc7a7b77d` |

README 已更新以展示本次成功实验；具体见项目主 `README.md` 的 Scene 02 章节
与 [`media/README.md`](../media/README.md)。
