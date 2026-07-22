# TopoFocus Realworld

TopoFocus 的真机仓库：一台 GPU Hub 接收机器人观测、构建/融合语义地图并发布可过期的高层决策；Go2 端保留最终停止与拒绝权限。本仓库同时保存审计过的研究源码快照、Hub 实现和 WSJ Jetson 的可复现部署层。

目标仓库：`git@github.com:AlanZhu2006/topofocus_realworld.git`

## 当前结论（2026-07-22）

- Hub 的协议、spool、单机语义映射、前沿/VLM 决策、Foxglove relay、TinyNav 原生 occupancy 适配器及 fail-closed 守门器已实现；本机测试基线为 197 项通过。
- 实时地图现要求稳定姿态窗和三帧 RANSAC 地面共识，过滤静止重复帧，以可逆 log-odds 融合障碍；位姿跳变会锁止当前地图而不是继续污染。
- WSJ 的稳定观测路径为 D435i 双红外 + RGB + IMU、TinyNav 修复后的 perception。修复避免重型 stereo inference 阻塞 IMU 回调，并在无效 IMU 区间后重新锚定。
- Yunji 新接入的传感器已按其 TinyNav `odin1_deployment.md` 确认为 Odin1（序列号 `O1-P070100205`），不是另一台 RealSense。RGB/SLAM 点云/里程计适配、真实地面门禁、WSJ/Odin 标定板拟合和独立移动板留出均已通过；主 Foxglove 已在保持原端口/topic/layout 的情况下切换到全新的 Odin 地图目录，随后 1.193 m 操作者低速移动地图门禁也已通过。
- 原生 BuildMap 静止门禁已验证保存；刚结束的 2026-07-21 21:35 会话保存 161 个 pose、1024.75 秒，优化路径累计 0.1294 m、首尾仅 0.00155 m，属于静止抖动测试，不是受控移动测试。
- Hub 和机器人端默认均禁止 `GOAL`。没有通过 G5 真机安全门禁，不得宣称已完成自主双机导航。
- Foxglove 当前显示 WSJ 与 Odin 两张新地图，并发布共享融合图；两端必须同时保持 `shared-board-odin1-20260722-v1`，否则 relay 会拒绝融合。旧 D455 地图只保留作回滚和审计，不能继续写入新帧。
- Foxglove 默认使用独立几何频道（灰未知、白自由、黑障碍），语义叠加在真实相机门禁通过前保持隐藏；离线参数扫描和移动验收工具已加入。

## 从干净克隆开始

```bash
git clone git@github.com:AlanZhu2006/topofocus_realworld.git
cd topofocus_realworld

# 只安装轻量 Hub/测试依赖，不下载模型或仿真数据。
bash hub/scripts/bootstrap_dev.sh
bash hub/scripts/verify_repository.sh --tests
```

完整 GPU 推理还需要仓库外的 RedNet、YOLO、CLIP 和 GLM 权重。它们的固定路径、大小和 SHA-256 见 `manifests/artifacts.json`；Git 不保存模型、录包、地图、token 或虚拟环境。

## 在另一台 Go2 Jetson 上复现

以下步骤只构造源码和检查环境，不会移动机器人：

```bash
git clone https://github.com/AlanZhu2006/topofocus_realworld.git
cd topofocus_realworld

# 从固定 TinyNav 上游 commit 重建 WSJ 已验证源码状态。
bash hub/robot_overlay/bootstrap_go2.sh \
  --destination /home/nvidia/twork/tinynav-topofocus

# 先查看，再显式安装 USB 稳定性配置。
bash hub/robot_overlay/install_go2_host_config.sh
sudo bash hub/robot_overlay/install_go2_host_config.sh --apply

# 只读检查；--hardware 要求 D435i 已连接且 power/control=on。
bash hub/robot_overlay/verify_go2.sh --hardware --tests
```

环境检查全部通过、操作者在现场后，才可启动“相机 + perception”观测栈：

```bash
cp hub/robot_overlay/config/go2.env.example hub/robot_overlay/go2.env
bash hub/robot_overlay/start_go2_observation.sh \
  --env hub/robot_overlay/go2.env
```

这个入口明确不启动 planner、`cmd_vel`、Unitree bridge 或 Hub GOAL receiver。原生 BuildMap 的人工移动与安全保存步骤见 [复现手册](docs/REPRODUCE.md)。

## 仓库边界

| 路径 | 作用 | Git 策略 |
| --- | --- | --- |
| `source/Focus_realworld/` | 原始集中式 Habitat/TopoFocus 研究代码 | 只读快照，逐文件校验 |
| `dependencies/` | RedNet 与修改版 Habitat 参考源码 | 只读快照，逐文件校验 |
| `hub/` | 真机协议、Hub、工具、测试和机器人部署层 | 主开发区 |
| `hub/robot_overlay/tinynav_snapshot/` | WSJ 所用 TinyNav 固定基线补丁与实验快照 | 可审计、可重建 |
| `audit/` | 已观察结果与门禁证据 | 入库 |
| `manifests/` | 来源、环境、外部资产和 SHA-256 | 入库 |
| `artifacts/`, `data/`, `logs/`, `hub/runtime/` | 权重、录包、地图、日志、token、运行状态 | 永不入库 |

不要在 `source/` 或 `dependencies/` 中开发部署代码；新真机代码只进入 `hub/`。

## 文档入口

- [从零复现本机与新 Go2](docs/REPRODUCE.md)
- [WSJ 已观察基线与遗留问题](docs/WSJ_BASELINE_20260721.md)
- [Git 分支、发布与快照更新规则](docs/GIT_WORKFLOW.md)
- [系统架构](ARCHITECTURE.md)
- [操作与验证门禁](RUNBOOK.md)
- [传输协议](hub/docs/TRANSPORT.md)
- [坐标系约束](hub/docs/COORDINATE_FRAMES.md)
- [实时地图与 Foxglove 契约](hub/docs/LIVE_MAPPING.md)
- [Yunji Odin1 替换部署、校验与重新标定](hub/docs/YUNJI_ODIN1_DEPLOYMENT.md)
- [离线地图诊断、移动验收和既有标定脚本复用](hub/docs/OFFLINE_MAP_VALIDATION.md)
- [来源与第三方说明](SOURCE_MANIFEST.md)

任何让机器人运动的工作都必须另行通过标定、replay、超时/断网、急停和 HIL 门禁；克隆或运行本仓库的默认脚本本身不构成运动授权。
