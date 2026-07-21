# Focus 真机 Hub

`hub/` 是唯一的真机部署开发区。`../source/` 与 `../dependencies/` 是不可修改的来源快照。

## 已实现边界

- 认证 HTTP/TCP 观测上传，含序号、UTC 时间、RGB-D hash、相机内外参、shared-frame pose、健康状态和 transform version；
- 只追加 spool、断点恢复、重复一致性和乱序/陈旧/未来帧拒绝；
- RedNet 语义 BEV、带启动姿态/地面门禁的实时地图、frontier/VLM 决策；
- 可逆 occupancy 证据、关键帧过滤和位姿跳变锁止；
- 显式 frame/calibration 契约下的双图对齐融合（当前会话未重新标定，默认关闭）；
- TinyNav 原生 occupancy 导入，保留原 frame，并拒绝错误 frame 融合；
- Foxglove camera/map relay；
- 版本化、可过期 `GOAL/HOLD/STOP` 与机器人端 fail-closed GoalGuard；
- WSJ ROS 2 sender、云迹 ROS1 sender 和 Go2 可复现部署层。

默认配置始终 `allow_goal=false`。当前仍未通过 G5 HIL，任何脚本都不应把“能建图”解释为“允许自主运动”。

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
- [坐标系](docs/COORDINATE_FRAMES.md)
- [TinyNav 原生地图适配](docs/TINYNAV_NATIVE_MAP_ADAPTER.md)
- [实时地图、Foxglove 与融合契约](docs/LIVE_MAPPING.md)
- [WSJ 初始审计](docs/ROBOT_WSJ_AUDIT.md)
