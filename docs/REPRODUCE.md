# 从零复现

本手册区分三个层次：代码复现、外部资产复现、真机行为复现。仓库可以完整重建代码和 WSJ 部署差异；模型权重、录包和硬件固件因体积或授权原因必须单独提供并校验。

文档中的证据标签：

- **observed**：在本机或 WSJ 实际读取/运行得到；
- **source-derived**：由固定源码、Git diff 或配置推导；
- **unverified**：尚未在第二台硬件上执行。

## 1. 克隆与仓库自检

要求 Git、GNU coreutils、Bash、`uv`，以及 Python 3.10。轻量 Hub 测试不需要 CUDA 或模型。

```bash
git clone git@github.com:AlanZhu2006/topofocus_realworld.git
cd topofocus_realworld
bash hub/scripts/bootstrap_dev.sh
bash hub/scripts/verify_repository.sh --tests
```

`bootstrap_dev.sh` 严格使用 `hub/uv.lock`，且拒绝覆盖已有环境。若当前机器已有专门 CUDA 环境，可用不同路径：

```bash
bash hub/scripts/bootstrap_dev.sh --env-dir hub/.venv-clean --python /usr/bin/python3.10
```

## 2. Hub 外部资产

以下内容不进入 Git：

- GLM-4V-9B Hugging Face cache；
- RedNet HM3D checkpoint；
- YOLOv10m 与 OpenAI CLIP ViT-B/32 权重；
- 真机 ROS bag、TinyNav map、Foxglove 运行快照和认证 token。

将合法取得的文件放到 `manifests/artifacts.json` 指定的相对路径，然后执行：

```bash
/usr/bin/python3.10 hub/tools/g0_audit.py --workspace "$PWD" --full-hash
```

不要为了真机部署下载 HM3D、Matterport scenes、HPC overlay 或 SIF。它们只属于明确批准的模拟器门禁。

当前 GPU G1 环境继承了本机已经做过 kernel 测试的 `memnav` Python/CUDA 栈；这是 **observed** 的本机恢复路径，不是假装跨机器通用的 lock：

```bash
FOCUS_TORCH_PYTHON=/path/to/tested/python \
FOCUS_UV_BIN=/path/to/uv \
bash hub/scripts/create_g1_env.sh

hub/.venv/bin/python hub/tools/g1_preflight.py --workspace "$PWD"
```

新的 Hub 主机必须重新通过 G0/G1/G2，不能仅凭相同 GPU 型号继承结论。

## 3. 新 Go2/Jetson 的硬件前提

WSJ 的已观察基线是 Jetson Orin NX、Ubuntu 22.04、JetPack 6.2.1/L4T 36.4.7、ROS 2 Humble、D435i 固件 5.17.0.10。另一台机器允许小版本不同，但每个差异必须记录并重跑预检。

本仓库不会自动安装 JetPack，也不会下载 TinyNav 模型。先按厂商/上游方式准备：

1. ROS 2 Humble；
2. GTSAM 与 TinyNav C++ 扩展；
3. `message_filters` workspace；
4. librealsense `v2.58.1` 与 realsense-ros `4.58.1`；
5. TinyNav Python 3.10 环境及其模型资产。

WSJ 的精确版本和哈希见 [基线文档](WSJ_BASELINE_20260721.md)。

## 4. 重建 WSJ TinyNav 源码

不要在已有生产 checkout 上套补丁。创建一个新目录：

```bash
git clone https://github.com/AlanZhu2006/topofocus_realworld.git
cd topofocus_realworld

bash hub/robot_overlay/bootstrap_go2.sh \
  --destination /home/nvidia/twork/tinynav-topofocus
```

该脚本执行以下可审计步骤：

1. clone `UniflexAI/tinynav`，跳过 Git LFS smudge；
2. checkout 固定基线 `576c082e69580f618a5ff313a3e74f3672abb69f`；
3. 校验并应用 `tinynav-required.patch`；
4. 创建本地 `topofocus/wsj-repro-20260721` commit；
5. 不 source ROS、不启动节点、不访问 Unitree 控制。

可选的 `--with-experimental-semantic` 会再恢复 WSJ 主 checkout 中尚未提交的语义包。它不是原生 BuildMap 必需条件，不应作为首次部署默认项。

补丁应用后，使用重建 checkout 中的 `DEPLOYMENT.md` 完成上游 build。复制并调整环境入口：

```bash
cp hub/robot_overlay/tinynav_setup.bash.example \
  /home/nvidia/twork/tinynav_setup.bash
```

## 5. 安装 USB 稳定性配置

先审阅 dry-run：

```bash
bash hub/robot_overlay/install_go2_host_config.sh
sudo bash hub/robot_overlay/install_go2_host_config.sh --apply
```

这会安装：

- `usbfs_memory_mb=1000` 的开机 oneshot；
- 通过 vendor/product 匹配 D435i 与 USB3 hub 的 udev 规则；
- 可在 RealSense driver bind 后再次调用的 power-policy service。

WSJ 上已观察到 udev 初始事件之后 driver bind 仍可能把 `power/control` 改回 `auto`。因此 `start_go2_observation.sh` 会在相机启动后重新运行 service，并在状态不是 `on` 时停止整个启动流程。

## 6. 只读预检

```bash
bash hub/robot_overlay/verify_go2.sh \
  --tinynav-root /home/nvidia/twork/tinynav-topofocus \
  --hardware --tests
```

必须看到：固定基线存在、净化补丁完整、Python 3.10、`usbfs_memory_mb>=1000`、D435i/hub power 为 `on`、IMU health tests 通过、没有已知规划/执行进程。

## 7. 启动仅观测栈

操作者在现场，Go2 处于稳定姿态且急停可用时：

```bash
cp hub/robot_overlay/config/go2.env.example hub/robot_overlay/go2.env
# 编辑路径，不要写 token。

bash hub/robot_overlay/start_go2_observation.sh \
  --env hub/robot_overlay/go2.env
```

启动内容只有：

- RealSense：848×480×30 RGB、infra1、infra2，gyro+accel 合并 IMU；
- 修复后的 TinyNav `perception_node.py`。

明确不包含：`planning_node.py`、`cmd_vel_control`、Nav2 controller、`go2_cmd_bridge` 或 Hub GOAL receiver。

## 8. 原生 BuildMap

观测栈健康后：

```bash
bash hub/robot_overlay/start_go2_buildmap.sh \
  --output "$HOME/.local/share/topofocus/maps/room-$(date -u +%Y%m%dT%H%M%SZ)"
```

脚本只启动 TinyNav BuildMap；机器人由人在现场手动移动。完成时必须走保存协议：

```bash
bash hub/robot_overlay/save_go2_buildmap.sh
```

只有看到 `/benchmark/data_saved=true` 且 `maploc` 退出，才能关闭相机：

```bash
bash hub/robot_overlay/stop_go2_observation.sh
```

禁止直接断电或 kill BuildMap 后把残缺目录当成有效地图。

## 9. 导入 Hub 与 Foxglove

地图目录只读复制到 Hub 的忽略目录，例如 `data/robot_replays/`，记录源路径、字节数和 SHA-256，然后：

```bash
hub/.venv/bin/python hub/tools/import_tinynav_occupancy.py \
  --record data/robot_replays/<map> \
  --out-dir hub/runtime/map_out_<map> \
  --robot-id robot-0 \
  --frame-id <unique_world_frame> \
  --transform-version <unique_version>
```

不要把不同 TinyNav 重启产生的 world origin 放进同一个 transform version。
Foxglove relay 会拒绝 frame 不一致的融合；这个拒绝应保留。

实时 Hub 地图还会写入 `shared_frame_calibration_id`。两台地图只有在各自
daemon 显式使用同一个、独立验证过的
`--shared-frame-calibration-id` 时才允许 `foxglove_relay.py --fuse`；仅仅都叫
`shared_world` 不构成标定。启动姿态、地面、关键帧和位姿跳变规则见
[实时地图契约](../hub/docs/LIVE_MAPPING.md)。

## 10. 复现完成的判据

代码层完成：仓库验证、轻量测试、TinyNav patch verification 全通过。

观测层完成：相机/IMU 频率连续、perception health 正常、静止测试无持续漂移、BuildMap 能收到正向保存确认。

运动层仍未完成：必须另行通过受控移动、长时间 soak、断网/超时/急停和 G5 HIL。不得因地图能更新就打开 `allow_goal`。
