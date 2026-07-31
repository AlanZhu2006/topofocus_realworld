# Scene 04 · Plant：正式实验 01 Failure

时间：2026-08-01 05:06:48 至 05:15:01（Asia/Shanghai）

Session：`scene04-plant-20260801-formal01-recal3`

Episode：`scene04-formal01-recal3`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL | Exploration rounds |
| --- | ---: | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` | `15` |

本轮由操作者指定归档为 Scene 04 Formal 01 Failure。15 个完整 source
rounds 内没有确认并抵达 `plant`；终止分配轮中，所有剩余 source-derived
frontier 均没有通过足迹净空执行分配，因此系统自动同步 HOLD。失败轮的
SR、source-compatible SPL 和 Standard SPL 均为 `0`。

## 双机轨迹

| Robot | Actual path | Start-to-stop displacement | Final state |
| --- | ---: | ---: | --- |
| Robot 0 | `10.461142 m` | `1.347643 m` | `HOLDING`，零速度已确认 |
| Robot 1 | `14.271182 m` | `3.937275 m` | `HOLDING`，零速度已确认 |

运行覆盖已完成 source steps `0–349`，并在 step `374` 完成终止分配判定；
共发布 64 个带版本和有效期的高层批次，最终没有生成语义到达终态证据。

## Robot 1 的 A 点

A 点并非在融合图上不可达。第 12 轮的冻结执行图记录：A 是已知空闲且
与起点连通的单元，目标净空为 `0.412311 m`，目标附近有 196 个安全接近
单元，起点连通的足迹净空单元共 7,189 个。执行点距离当时 Robot 1
`7.496566 m`。

问题发生在机器人本地执行层。Robot 1 当时朝向 `-124.864°`，A 的方位
为 `9.890°`，第一段需要约 `134.754°` 的大角度转向。TinyNav 连续
`7.220 s` 将 `416/416` 条局部轨迹全部判为碰撞，随后以
`LOCAL_GOAL_UNREACHABLE` 拒绝并确认零速度。因而日志能证明的是当前姿态
下的近场局部规划无法启动，而不是 A 区域在全局上不可接近。

## Robot 0 的开局与地图

操作者现场报告 Robot 0 开局发生墙面接触。日志没有显示接触前出现离散
位姿跳变：round 0 的 `pose_jump_events=0`、地面估计为 `accepted`。
但全局安全投影没有从真实底座单元起步，而是允许将起点吸附到距离真实
底座 `0.375959 m` 的虚拟安全种子；Robot 0 的投影路径净空仅为
`0.05 m`，明显小于记录的 `0.35 m` 足迹净空要求。这说明开局存在
start-seed 与足迹净空合同不一致，不能将接触归因于一次已记录的定位跳变。

接触后的确出现了持续地图完整性问题：Robot 0 的几何图最后停在序列
`154467`，终止观测已到 `154981`，间隔 514 帧；累计 509 帧地面候选
不足，几何图不再更新。全程仍未记录离散 pose jump，所以更准确的描述是
“几何地图更新饥饿并伴随里程计继续演化”，而不是已证实的 tracking epoch
丢失。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/episode_report.json` | 10,189 | `93f59092a5fa169ab83e9a38a605948ebc75a3d263ff654fb7abdc709d914e8a` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/controller_events.jsonl` | 252,139 | `0088910f71a61e750326aefced12d9f9423a3e3c580fa429377c26206192d501` | source-derived controller event log |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/scene_manifest.json` | 237,613 | `be7c143bb60b4a57ef024426bfda723e80b11aecb13793a93a1af4de82179ea3` | source-derived continuous scene manifest |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/round_00_step_000/frontier_clearance_guard.json` | 11,954 | `e0b82848d5da3dd0a0033419449edd401b814bc8bd7e8e220c86a54f803a1762` | Robot 0 launch/start-seed evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/round_12_step_299/frontier_clearance_guard.json` | 28,583 | `143cf799981a84abf41563ad0789b9e8249692c1ca5cfcf752935fde583b1cf7` | Robot 1 A-point global reachability evidence |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/round_12_step_299/navigation_failure_memory_after_round.json` | 15,842 | `50b65abb8e845b6503f82ea89279787cdf96705e1c3e1bd78470febc57baf6b1` | Robot 1 observed local rejection mirror |
| `hub/runtime/oneclick_scene04-plant-20260801-formal01-recal3_live_scene04_20260801_050647_761155680/round_15_step_374/freeze_result.json` | 8,797 | `ed93e0f7facbe678359b920c892dd562789634f5560bde265e9a04e7805626ad` | strict terminal input and map-gap evidence |
| `hub/runtime/sessions/scene04-plant-20260801-formal01-recal3/session.json` | 4,269 | `fb3405e7991c7b2d7c1148dcf716344ea805a6a0c1ac21e4e43d632203c7fd2a` | exact session identity |
| `hub/runtime/calibration_sessions/scene04-plant-20260801-recal3/shared_frame.json` | 6,443 | `d726fab35f5518fe970d87abf99fefded89db5b98ae14b0223020b9c661454b3` | shared-frame calibration |

机器可读归档：
[`scene04_plant_formal_experiment_01_failure_20260801.json`](../manifests/scene04_plant_formal_experiment_01_failure_20260801.json)。

## 媒体

[第三视角原片](../media/video/third_view/experiment_4/experiment_4_failure_1.mp4)
和
[Dashboard 原片](../media/video/dashboard/experiment_4/experiment_4_failure_1_dashboard.mov)
的内嵌创建时间分别为本轮结束后约 2 分 36 秒和 3 分 40 秒，按用户上传
字节原样绑定。README 使用的
[第三视角 H.264](../media/demo/scene04_formal_01_third_view.mp4)、
[Dashboard H.264](../media/demo/scene04_formal_01_dashboard.mp4)及两个 64 帧
GIF 均为 source-derived 展示副本；精确大小、时长和 SHA-256 记录在
机器可读归档中。

## 安全终态

- 终止批次已由两台机器人确认；最终均为 `HOLDING`、`HUB_HOLD`。
- 两台机器人均记录 `velocity_zero_confirmed=true`。
- Hub 只发布高层目标，机器人本地停车与拒绝权限保持不变。
- 实验退出后 GOAL 通路已关闭，仅保留观察、地图与 Foxglove。
