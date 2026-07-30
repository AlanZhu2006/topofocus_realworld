# Scene 03 当日运行问题完整修复

记录时间：2026-07-31T06:27:48+08:00

## 结论

本次在不修改 `source/`、`dependencies/`、不连接底盘且不下发任何机器人
命令的前提下，复核了 Scene 03 两次正式失败、三次成功及 Formal 05
重标定前后的冻结证据。当天暴露的新增执行问题已分别落到共享控制器、
跨轮目标连续性、语义执行确认和所有命令批次入口中；Robot 0、Robot 1
使用同一套修复，不再存在仅修一端的分叉。

代码和回放测试已经完成。修复后的真机运动效果尚未执行，仍需下一次现场
操作者授权后验证；这不是本次离线检查能够替代的结论。

## 问题与根因

| 问题 | 证据结论 | 根因 | 修复 |
| --- | --- | --- | --- |
| Robot 1 在墙前左右转、持续原地转 | Formal 01 已记录 5–20 cm 的短碰撞轨迹在机身轴两侧跳变；普通转向过去只由较慢的平移进展 watchdog 兜底 | 太短的局部路径没有稳定航向；连续纯转向没有独立的本地终止理由 | 仅当碰撞检测路径达到 `0.30 m` 才取得航向权，短路径使用固定 router waypoint；`15°/8°` 方向迟滞保持转向符号；任意连续原地恢复超过 `12 s` 时零速并上报 `LOCAL_PLANNER_TURN_STALLED`，Hub 隔离该腿后重新规划 |
| Robot 1 走回头路、在两个方向间往返 | Formal 02 round 16 时 Robot 1 距上一目标仍有 `0.943568 m`，源码已进入 `1.25 m` 切换区并从 `history-1` 改选相反方向的 `history-6` | 模拟器的 25-cell 高层切换边界早于真机 10-cell 到达边界，上一物理腿尚未完成就被换点 | 保留源码候选和 `1.25 m` 规则作为 provenance；执行侧继续使用上一轮**同一个已接受目标**，直到进入 `0.50 m` 到达区、收到 `ARRIVED` 或明确拒绝。不会生成新目标，且每轮仍重新通过当前地图净空检查 |
| Robot 0 出现 `plant` 误标并提前 ARRIVED | 两个误标区域分别只有 `7` 和 `2` cells，同帧检测均没有 `potted plant`；成功样本分别是 `176` cells 的强源语义区域，以及 `5` cells + `0.851619` 同帧检测 | 源码 `Find_Goal` 对持久语义图中任意正值都会生成语义目标；微小陈旧区域可直接获得真机语义目标权，并可能因局部距离很近立即到达 | 完整保留源码语义图和候选。执行侧只接受两类证据：`>=25` cells 的强源语义区域，或 `>=3` cells 且同一冻结 RGB 的目标检测置信度 `>=0.50`。其余情况退回该轮已冻结的原始 frontier/history；没有退路时 HOLD。YOLO 不写入语义图 |
| 负速度/身后路径导致不走或错误转向 | Formal 02 Robot 0 多次产生源码固定 `-0.2 m/s` 请求；负速度均被本地安全层拦截 | 身后 lookahead 在固定控制器中表现为负线速度和零角速度，直接使用零角速度无法完成 rotate-first | 永不向底盘发送负速度；使用已经验证的碰撞路径航向或 router 航向生成零线速度转向。缺少航向时仍拒绝，超时使用独立的 turn-stalled 原因，不再伪装成 reverse |
| 一个入口修好、换入口后重新出现误标 | 正式 runner、旧 supervised runner 和离线 batch builder 原先并非共享语义执行确认 | 保护只接在单一运行链会形成可绕过的命令批次 | 三个入口全部先保存未修改的 source candidate，再生成带版本的 guarded execution batch；批次构造器只允许使用冻结清单中精确的 pre-semantic selection 或 HOLD，拒绝任何临时发明坐标 |
| PATH_STALE、NO_PROGRESS、重复失败 frontier、运动后 ground drift | 当日运行中均有明确的可恢复拒绝或映射阻断记录 | 目标刷新生命周期、跨轮失败记忆和地面基线更新曾分别不完整 | 保留并复核既有修复：同一 DDS participant 内刷新 planner target；局部失败进入有界空间/方向记忆；同 XY 即使 frontier 标签变化也不立即重发；运动后的地面基线只按稳定帧更新。新 `LOCAL_PLANNER_TURN_STALLED` 也进入同一可恢复链 |

## 语义保护为何不是“硬加 YOLO”

源码 `Find_Goal`、融合语义图、VLM 结果和原始决策批次均原样保存。YOLO
只作为小区域取得**物理执行权**时的同帧交叉证据，并不回写或强化语义图。
较大的源语义区域仍可完全依赖源码语义结果，因此不会破坏 Formal 03
中 `176`-cell、当帧无 plant 检测但最终物理成功的既有行为。紧凑真实目标
则可通过 Formal 05 的 `5`-cell + `0.851619` 检测路径取得执行权。

## 安全和一致性

- 两端共享同一控制器文件、接收器逻辑和 `12 s` 转向失败合同。
- `turn_stalled` 按当前 authority 锁存；后续一次 false 消息不能抹掉失败，
  只有新高层 authority 才能清除。
- Hub 对该失败发布零速拒绝并进行 source replan；机器人本地仍保留最终
  停车、拒绝和速度门控权。
- 未删除 reachability、occupancy、SLAM、轨迹新鲜度或控制器暂停门控。
- `source/` 与 `dependencies/` 未修改；本次没有网络访问、远端同步或真机
  命令。

## 离线验证

- 当日问题专项回放：`204 passed`。
- 完整 Hub 回归：`826 passed`；仅有一条第三方
  `fastapi.testclient`/`httpx` 弃用警告，无失败。
- Python 编译、两个机器人启动脚本的 shell 语法、JSON 解析、
  `git diff --check` 全部通过。
- 可复现部署清单中的 `20` 个文件合同均重新核对了大小和 SHA-256。
- 修复后真机运动：`unverified_not_run`。

## 关键证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `audit/SCENE03_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260730.md` | 3,687 | `5858ec8c1c4c362fad261856f83c1dab92e15ad7d54fb99c615622e8faef1f1d` | source-derived from observed run evidence |
| `hub/runtime/oneclick_scene03-plant-20260730-2349-formal-start_live_scene03_20260730_235458_576402645/round_15_step_374/shadow/shadow_manifest.json` | 36,580 | `763980758264168d42150c42133a8544dc8aad0624ec99336e955c8a7e082c8b` | source-derived frozen VLM/history evidence |
| `hub/runtime/oneclick_scene03-plant-20260730-2349-formal-start_live_scene03_20260730_235458_576402645/round_16_step_399/shadow/shadow_manifest.json` | 36,939 | `2357a8c5572a6c4324146bbcdfdeae58e46c3937459a07ef5475e47f66a2b5c6` | source-derived frozen VLM/history evidence |
| `hub/runtime/oneclick_scene03-plant-20260731-0325-formal05_live_scene03_20260731_032816_045070790/round_07_step_174/shadow/shadow_manifest.json` | 39,447 | `b4666d9b520ec1f01a0f62e4c4bf9f399bd63ab7083676a20fffe257a9782418` | source-derived seven-cell false semantic candidate |
| `hub/runtime/oneclick_scene03-plant-20260731-0325-formal05_live_scene03_20260731_032816_045070790/round_07_step_174/shadow/inputs/wsj/source_161240.jpg` | 142,975 | `260bb5318ab8135cc5095214736d72affbc3c10481f4d1e387cfeee6628c6bac` | observed exact frozen RGB |
| `hub/runtime/oneclick_scene03-plant-20260731-0421-formal05-rerun2_live_scene03_20260731_042657_466212878/round_05_step_124/shadow/shadow_manifest.json` | 38,166 | `091a1f93215cd74a512978d4007f8b58b59e766e10748c7ac6df257457c85d6a` | source-derived two-cell false semantic candidate |
| `hub/runtime/oneclick_scene03-plant-20260731-0421-formal05-rerun2_live_scene03_20260731_042657_466212878/round_05_step_124/shadow/inputs/wsj/source_164121.jpg` | 147,970 | `a22055878f0546386b83385f125c4b1ccfb1cfb7556ed372701fa52f33a15d22` | observed exact frozen RGB |
| `hub/runtime/oneclick_scene03-plant-20260731-022816-formal04_live_scene03-formal04_20260731_023143_863454849/round_04_step_099/shadow/shadow_manifest.json` | 39,010 | `1b5141f86f212865d8c2e3b380765eb8218eaaa56c6a13e80985f1e814bc58a9` | source-derived 176-cell successful semantic candidate |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/round_10_step_249/shadow/shadow_manifest.json` | 37,870 | `34f04f15a9fcde2b7e08cdf67880422ce1fcac4fde51185d3f94025e7a77883c` | source-derived five-cell detector-confirmed candidate |
| `hub/runtime/oneclick_scene03-plant-20260731-0444-robot1-continue01-start_live_scene03_20260731_045516_219395014/round_10_step_249/shadow/inputs/yunji/source_393032.jpg` | 143,642 | `cae7795166399e7abb5b54785594f78325fd7da2a10ef20d4f83387967d0be0f` | observed exact frozen RGB |

机器可读记录：
[`scene03_runtime_issues_complete_repair_20260731.json`](../manifests/scene03_runtime_issues_complete_repair_20260731.json)。
