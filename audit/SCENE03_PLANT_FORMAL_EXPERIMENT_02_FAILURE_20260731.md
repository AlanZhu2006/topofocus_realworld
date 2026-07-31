# Scene 03 · Plant：正式实验 02 Failure

时间：2026-07-30 23:54:58 至 2026-07-31 00:05:00（Asia/Shanghai）  
Session：`scene03-plant-20260730-2349-formal-start`  
Episode：`formal02`

## 结果

| Result | Success | SR | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| `FAILURE` | `0` | `0.0` | `0.0` | `0.0` |

本轮由操作者明确指定为 Scene 03 Formal 02 Failure。运行达到 600 秒上限，
18 个 source rounds 均未形成 `plant` 语义到达，因而不是成功候选。
失败的主要归因是 `execution_engineering_failure`；Robot 1 同时暴露了源码
历史点回访策略在长程真机场景中的跨轮折返行为。

实验展示摘要统一记为：探索在测试时限内未发现并抵达经验证的 plant
目标，因此以 time-limit failure 结束。下面保留原始运行诊断作为工程
provenance。

## 双机动作与原因

| Robot | 实际路径 | 净位移 | 结论 |
| --- | ---: | ---: | --- |
| Robot 0 | `5.754388 m` | `3.746004 m` | 前两轮正常推进；随后 11 次由 TinyNav 固定 `-0.2 m/s` 反向请求触发 `LOCAL_PATH_REVERSE_REQUIRED`，每次均零速拒绝并被隔离。 |
| Robot 1 | `17.902160 m` | `2.918764 m` | TinyNav 按下发目标持续运动；后段目标切换到旧历史点，造成明显回走和往返，路径/净位移比为 `6.133472`。 |

### Robot 0

Robot 0 不是因为协调 HOLD 长期不动。原始 TinyNav 控制器在局部轨迹
lookahead 位于机身后方时会生成固定负线速度，并把角速度清零。当前共享
部署包装器先检查负线速度，再进入 rotate-first，因此在安全的零线速度
转向逻辑有机会执行之前就发布了 `reverse_required=true`。Hub 随后按设计
隔离该腿并让其 HOLD。

本轮从 round 2 开始共记录 11 次
`LOCAL_PATH_REVERSE_REQUIRED`；控制器日志中的请求均为
`linear=-0.200000 m/s`。所有负线速度均未发送到底盘。

最小修复保持 forward-only 合同：当且仅当稳定的碰撞检测路径航向（短路径
时使用固定 router waypoint）可用时，把源码负速度请求转换为有超时的
`linear=0` 原地航向对齐；缺少航向依据或超时仍然拒绝。相同共享控制器
同时适用于 Robot 0 和 Robot 1。

### Robot 1

Robot 1 的折返来自源码决策，而不是局部规划器擅自改路：

1. round 12 的当前局部目标以 `LOCAL_GOAL_UNREACHABLE` 结束，所以下一轮
   不再保留该连续目标。
2. round 13 的 Judgment VLM `Yes=0.495690 < 0.5`，且 source step 324
   已超过前 125 步强制 frontier 窗口，严格进入
   `Final_PR = history_score_copy` 的首个最大值分支，选择旧
   `history-1=(-1.085073, 0.990670) m`。
3. round 15、16、17 的 `Yes` 分别为 `0.441136`、`0.386554`、
   `0.441136`，继续选择 `history-1`、`history-6`、`history-1`。
4. 源码只在同一轮的双机顺序分配中移除已经选中的 history 节点，没有跨轮
   “已回访冷却”或“两点循环抑制”，因此旧高分点可以在后续轮次再次成为
   最大值。

这与 `source/Focus_realworld/main.py` 的实际执行分支一致。若禁止这种
折返，需要显式增加不同于论文源码的部署策略；本次归档不把它伪装成
局部控制故障，也不擅自修改不可变源码。

## 运行后修复状态

- 控制器修复提交：`b202827`。
- 控制器专项测试：`74/74`；完整 Hub 测试：`774/774`。
- 同一控制器文件已原子同步到 Robot 0 与 Robot 1 部署目录，三端大小均为
  `71,983 bytes`、SHA-256 均为
  `7449fcf061c108e15c0e4083e213e3ab13e7ad8638f9765b579944112a7244bc`。
- 同步没有重启正在运行的控制器；原进程 PID 保持不变，Hub GOAL 仍为双侧
  `false`，临时传输缓冲与远端临时文件均已删除。
- 修复后的物理效果尚未被本轮数据验证，必须在新的现场运动授权下单独测试。

## 安全终态

- 最终两台机器人均为 `HOLDING`、`HUB_HOLD`、
  `velocity_zero_confirmed=true`。
- Hub 的 Robot 0 / Robot 1 `goal_output_enabled` 均已恢复为 `false`。
- live episode 进程已退出；只保留观察、地图和 Foxglove。
- Hub 只发布带版本和过期时间的高层目标，机器人本地拒绝与停车权保持不变。

## 证据

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene03-plant-20260730-2349-formal-start_live_scene03_20260730_235458_576402645/episode_report.json` | 9,790 | `53bc2d03431836a6bc7955d93bcdf397546a461f35e1af612ba957897ef73038` | source-derived from observed navigation feedback |
| `hub/runtime/oneclick_scene03-plant-20260730-2349-formal-start_live_scene03_20260730_235458_576402645/controller_events.jsonl` | 270,677 | `646dd57c59153c0da84dc714c89b2ce251d789fbcc7e7ff0723f1ef33e86a59c` | source-derived controller event log |
| `hub/runtime/oneclick_scene03-plant-20260730-2349-formal-start_live_scene03_20260730_235458_576402645/scene_manifest.json` | 250,031 | `0fd7179666db5ebd8dd867906f7ab311aee0c081e60009a3512af1d116adf4ac` | source-derived continuous scene manifest |
| `hub/runtime/oneclick_scene03-plant-20260730-2349-formal-start_live_scene03_20260730_235458_576402645/round_13_step_324/shadow/shadow_manifest.json` | 35,820 | `38e5acc28ae6c5e2c39d05b12a90e768034809ed1ec48bed2aca1b3ad1f969c5` | source-derived VLM/history decision evidence |
| `hub/runtime/sessions/scene03-plant-20260730-2349-formal-start/session.json` | 4,368 | `d87520c13a04f2f2b9d00daf279113f6271d8a721c82ab21b34ee9af14f279d6` | source-derived session identity |
| `hub/runtime/calibration_sessions/scene03-plant-20260730-2332-turncontract-recal/shared_frame.json` | 6,039 | `92330b7633ab1365771ddf9ee5a3607c0037287dd95578742a0dff985dd56606` | observed and source-derived calibration |
| `/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-live-20260730T155445Z.jsonl` | 217,743 | `f0a336f30f72d709d8b1ad1d3104d4c2ab3059faff5f2102438de0a4af6280e5` | observed Robot 0 receiver log, verified through existing SSH/tmux; remains on Robot 0 |
| `/home/nyu/.local/state/topofocus/yunji-v2-tinynav-live-20260730T155428Z.jsonl` | 362,158 | `5a99db651324b4e395788883318d39811d75da7acbb46a49ce438307f833b1a8` | observed Robot 1 receiver log, verified through existing SSH/tmux; remains on Robot 1 |

机器可读归档：
[`scene03_plant_formal_experiment_02_failure_20260731.json`](../manifests/scene03_plant_formal_experiment_02_failure_20260731.json)。

## 媒体补充归档（2026-07-31）

[第三视角原片](../media/video/third_view/experiment_3/experiment_3_failure_2.mp4)
和
[Dashboard 原片](../media/video/dashboard/experiment_3/experiment_3_failure_2_dashboard.mov)
按用户上传字节原样保留于 Git LFS；README 使用的
[第三视角 H.264](../media/demo/scene03_formal_02_third_view.mp4)、
[Dashboard H.264](../media/demo/scene03_formal_02_dashboard.mp4)及对应 GIF
均为 source-derived 展示副本。精确大小、时长和 SHA-256 记录在机器可读归档。
