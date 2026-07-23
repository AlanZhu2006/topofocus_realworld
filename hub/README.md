# Focus 真机 Hub

`hub/` 是唯一的真机部署开发区。`../source/` 与 `../dependencies/` 是不可修改的来源快照。

当前双机部署身份、最近一次真机结果和剩余门禁以
[`../CURRENT_STATUS.md`](../CURRENT_STATUS.md) 为准。带日期的操作记录和
旧命令只作历史证据，不覆盖该页。

## 已实现边界

- 认证 HTTP/TCP 观测上传，含序号、UTC 时间、RGB-D hash、相机内外参、shared-frame pose、健康状态和 transform version；
- 只追加 spool、断点恢复、重复一致性和乱序/陈旧/未来帧拒绝；
- RedNet 源码基线与校验锁定的 SegFormer 实机像素语义适配器、带启动姿态/地面门禁的实时地图、frontier/VLM 决策；
- 可逆 occupancy 证据、关键帧过滤和位姿跳变锁止；
- 显式 frame/calibration 契约下的双图对齐融合；新的持久会话必须包含
  独立移动标定板留出验证；
- TinyNav 原生 occupancy 导入，保留原 frame，并拒绝错误 frame 融合；
- Foxglove camera/map relay；
- Foxglove 位姿、轨迹、像素语义、标签与前沿合成 2-D overview；
- 版本化、可过期 `GOAL/HOLD/STOP` 与机器人端 fail-closed GoalGuard；
- v2 原子双机目标、独立续租/到达反馈、WSJ TinyNav 在线
  BuildMap + guarded `cmd_vel` 接收器以及 Yunji WATER 高层目标接收器；
  双机链路已受控试跑并能故障闭环到 HOLD，但尚无可计入 SR/SPL 的成功场景；
- WSJ ROS 2 sender、云迹 ROS1/RealSense 回滚 sender、Odin1 ROS 2 适配器和 Go2 可复现部署层。

默认配置始终 `allow_goal=false`。当前只完成了部分 HIL 工程验证，尚未
完成一次带终点独立确认的正式场景；任何脚本都不应把“能建图”或
`ARRIVED` 解释为自主导航成功。

新摆位先运行一次标定与无运动全栈：

```bash
bash hub/scripts/calibrate_realworld_session.sh \
  --session-id <unique-session-id> \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY \
  --goal-category chair
```

之后使用两种启动模式：

```bash
# 无运动：地图、Foxglove、真实 VLM 和只读接收器
bash hub/scripts/realworld_oneclick.sh \
  --session-file current --mode debug --goal-category chair

# 有运动：还必须在现场一次性提供当次授权
bash hub/scripts/realworld_oneclick.sh --mode live \
  --session-file current \
  --scene-id scene01-chair --episode-id run01 --goal-category chair \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR
```

标定命令自动保存 Git/标定/transform/map 边界、机器人部署根目录和 tmux
身份；debug/live 不再要求人工替换旧 v12 常量。完整说明见
[持久会话一键流程](docs/ONECLICK_SESSION_WORKFLOW.md)。该流程已通过本机
测试，尚未在两台真机上完整执行；旧 July v3 会话不会被自动提升为
`current`。

## 轻量开发环境

从仓库根目录：

```bash
bash hub/scripts/bootstrap_dev.sh
bash hub/scripts/verify_repository.sh --tests
```

`hub/uv.lock` 只锁定协议、映射工具和测试所需的轻量环境。GLM/RedNet/YOLO/CLIP 的 CUDA 环境与外部权重属于 G0/G1/G2，见 `../docs/REPRODUCE.md`。

## 启动 Hub（默认回环、mapping-only）

```bash
cp hub/config/robots.example.json hub/config/robots.json
bash hub/scripts/focus_hub_up.sh
```

脚本在 `hub/runtime/` 生成 chmod-600 token；该目录被 Git 强制忽略。不要把 token 复制进配置示例、文档或命令历史。

停止：

```bash
bash hub/scripts/focus_hub_down.sh
```

## Go2

新 Go2 从 `robot_overlay/bootstrap_go2.sh` 开始。它从固定 TinyNav 上游 commit 重建 WSJ 已验证源码，不修改现有生产 checkout，也不启动 ROS/控制。

```bash
bash hub/robot_overlay/bootstrap_go2.sh \
  --destination /home/nvidia/twork/tinynav-topofocus
bash hub/robot_overlay/verify_go2.sh --hardware --tests
```

完整顺序、USB bind 后电源策略和 native BuildMap 保存协议见 [复现手册](../docs/REPRODUCE.md)。

## 协议与坐标

- [传输协议](docs/TRANSPORT.md)
- [Triple-AI 真机 Demo：历史图预演与 4×5 SR/SPL 协议](docs/TRIPLE_AI_REALWORLD_DEMO.md)
- [持久标定、无运动调试与正式实验一键流程](docs/ONECLICK_SESSION_WORKFLOW.md)
- [坐标系](docs/COORDINATE_FRAMES.md)
- [TinyNav 原生地图适配](docs/TINYNAV_NATIVE_MAP_ADAPTER.md)
- [实时地图、Foxglove 与融合契约](docs/LIVE_MAPPING.md)
- [Yunji Odin1 部署](docs/YUNJI_ODIN1_DEPLOYMENT.md)
- [v2 双机真机最短上线清单](docs/V2_PHYSICAL_QUICKSTART.md)
- [离线地图诊断、移动验收与既有标定脚本复用](docs/OFFLINE_MAP_VALIDATION.md)
- [WSJ 初始审计](docs/ROBOT_WSJ_AUDIT.md)
