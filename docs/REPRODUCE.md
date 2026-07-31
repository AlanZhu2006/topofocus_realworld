# 从零复现

本仓库的标准真机配置为 RTX 4090 Hub、Robot 0 的 Jetson Orin NX +
Unitree Go2 + RealSense D435i，以及 Robot 1 的 NUC + Odin1。Go2 的完整
clean-room 安装、版本锁、规划控制逻辑和分级验收见
[`ROBOT0_REPRODUCIBLE_BASELINE.md`](ROBOT0_REPRODUCIBLE_BASELINE.md)。

证据分类统一为：

- **observed**：从参考主机或设备实际读取；
- **source-derived**：由固定源码、补丁或配置推导；
- **unverified**：尚未在目标硬件执行。

## 1. 代码级复现

以下命令不连接机器人，也不能产生运动：

```bash
git clone https://github.com/AlanZhu2006/topofocus_realworld.git
cd topofocus_realworld

bash hub/scripts/bootstrap_dev.sh
bash hub/scripts/verify_repository.sh --tests
python3 hub/tools/verify_public_baseline.py --workspace .
```

Robot 0 的 TinyNav 确定性重建应得到：

```bash
repro_root="$(mktemp -d)"
bash hub/robot_overlay/bootstrap_go2.sh \
  --destination "$repro_root/tinynav"
git -C "$repro_root/tinynav" rev-parse HEAD HEAD^{tree}
```

```text
a6290559b13cedf19c05f7ec64ff91a29b685cbd
5281e70451f2f9cc1d5f5464315d803f6f0972bd
```

## 2. RTX 4090 Hub

在 Ubuntu 22.04 和已安装 NVIDIA 驱动的 RTX 4090 主机上：

```bash
bash hub/scripts/bootstrap_gpu_hub_cleanroom.sh
bash hub/scripts/bootstrap_gpu_hub_cleanroom.sh \
  --apply \
  --fetch-models \
  --accept-model-licenses
```

默认命令只打印计划。`--apply` 创建固定 Python 3.10.20/CUDA 12.8
环境、安装 Hub、构建 Detectron2，并执行模型、CUDA、源码语义和测试门禁。
模型已由其他方式合法放入 `manifests/artifacts.json` 约定位置时，可省略
两个模型参数；校验不会省略。

脚本明确不下载 HM3D 场景、ObjectNav 数据集、overlay、SIF、录包或地图。

## 3. Jetson Orin NX + Unitree Go2

先刷入清单记录的 JetPack 6.2.1/L4T 36.4.7，再克隆相同 Git commit：

```bash
bash hub/robot_overlay/bootstrap_robot0_cleanroom.sh
bash hub/robot_overlay/bootstrap_robot0_cleanroom.sh --apply

bash hub/robot_overlay/configure_go2_network.sh
bash hub/robot_overlay/configure_go2_network.sh --apply
```

安装脚本从固定 revision 构建 CycloneDDS、GTSAM、librealsense、
realsense-ros 和 message_filters，重建 TinyNav，校验 5 个 ONNX，并
生成 4 个 Jetson TensorRT plan。配置默认写入 XDG 目录，不依赖用户名或
仓库绝对路径；认证 token 必须另存为 mode-600 文件。

硬件门禁是只读的：

```bash
config_root="${XDG_CONFIG_HOME:-$HOME/.config}/topofocus"
set -a
source "$config_root/robot-0.env"
set +a
source "$TINYNAV_SETUP"
"$TINYNAV_PYTHON" hub/robot_overlay/verify_robot0_cleanroom.py \
  --level hardware
```

它不会初始化 Unitree DDS，不会启动 ROS 节点，也不会发送机器人命令。

## 4. 观测、建图与标定

观测和原生 BuildMap 不包含 planner、controller、receiver 或 Go2 bridge：

```bash
config_root="${XDG_CONFIG_HOME:-$HOME/.config}/topofocus"
bash hub/robot_overlay/start_go2_observation.sh \
  --env "$config_root/robot-0.env"
```

新摆位必须生成新的双相机标定与地图会话。完整入口见
[`hub/docs/ONECLICK_SESSION_WORKFLOW.md`](../hub/docs/ONECLICK_SESSION_WORKFLOW.md)；
Robot 1 的 Odin1 安装见
[`hub/docs/YUNJI_ODIN1_DEPLOYMENT.md`](../hub/docs/YUNJI_ODIN1_DEPLOYMENT.md)。
会话记录 commit、输入路径、大小、SHA-256、标定 ID、变换、地图和端口，
但不把 token、传感器数据或机器路径提交进 Git。

## 5. 分级验收

| 级别 | 必须具备 | 运动 |
| --- | --- | --- |
| R0 | 仓库合同和静态测试通过 | 否 |
| R1 | TinyNav commit/tree 确定性重建 | 否 |
| R2 | Hub 与 Jetson clean-room 主机/硬件门禁通过 | 否 |
| R3 | 仅观测与原生地图保存成功 | 无自主运动 |
| R4 | 新标定与严格 debug 通过 | 否 |
| R5 | 操作者现场逐轮授权的 live episode | 是 |

Hub 只发布有版本和过期时间的高层目标；机器人始终保留停止或拒绝执行的
最终权限。不要把 R0–R4 描述成已完成真机运动复现。

## 发布边界

模型权重、固件、录包、地图、运行态和密钥均为外部资产。仓库根目录目前
没有项目级许可证，`dependencies/RedNet` 的再分发许可仍待确认，因此公开
发布前仍需由项目所有者选择根许可证并完成第三方许可审查。详见
[`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)。
