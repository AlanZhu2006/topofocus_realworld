# 从零复现

本手册区分三个层次：代码复现、外部资产复现、真机行为复现。仓库可以完整重建代码和 WSJ 部署差异；模型权重、录包和硬件固件因体积或授权原因必须单独提供并校验。

当前真机版本、标定 ID、地图目录和未完成门禁见
[`CURRENT_STATUS.md`](../CURRENT_STATUS.md)。本文中的旧日期路径只用于复现
对应历史证据，不能自动替代当前会话参数。

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

## 9. Yunji Odin1 当前路径与 RealSense 回滚

Yunji 当前替换传感器是 Odin1 `O1-P070100205`。先从固定驱动 commit
重建并应用仓库保存的 mode-1 补丁，再用设备自身的 serial-specific
`calib.yaml`；完整路径、hash 和 systemd 单元见
[Odin1 部署文档](../hub/docs/YUNJI_ODIN1_DEPLOYMENT.md)。只读检查为：

```bash
bash hub/robot_overlay/verify_odin1.sh
bash hub/robot_overlay/verify_odin1.sh --hardware
python3 hub/robot_overlay/odin1_sender.py --help
```

Odin 适配器使用 `/odin1/image`、`/odin1/cloud_slam` 和
`/odin1/odometry`。观测 sender 不调用 WATER 运动接口；只有显式启动的
v2 live receiver 才能发送受租约约束的 `/api/move`。旧 D455 共享变换
不得用于 Odin。

`yunji_odin1_board_20260722_v1.json` 是首个 Odin 历史门禁。最后一次
现场 predecessor 会话使用：

- shared ID `shared-board-odin1-20260723-v3`;
- WSJ transform `wsj-tinynav-depth-20260723-powercycle-v3`;
- Yunji transform `yunji-odin1-board-20260723-powercycle-v6`.

这些实测 JSON 位于机器人/Hub 的 runtime state，因包含会话路径而不进入
Git。它们保留为历史证据，但旧 v3 artifact 没有新的定量
`board_moved_independently` 字段，不能手工提升为 persistent `current`
会话。其源路径、大小和 hash 见
[`audit/SHARED_FRAME_ODIN1_20260723.md`](../audit/SHARED_FRAME_ODIN1_20260723.md)
和
[`audit/YUNJI_REBOOT_CALIBRATION_REVALIDATION_20260723.md`](../audit/YUNJI_REBOOT_CALIBRATION_REVALIDATION_20260723.md)。

这个结果只适用于记录中的相机安装和两端 odom 会话。新的现场摆位、另一台
Go2/Yunji、传感器拆装或无法证明 origin 未变时，运行：

```bash
bash hub/scripts/calibrate_realworld_session.sh \
  --session-id <unique-session-id> \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY
```

该入口沿用已有标定板 detector 与 `--other-pose-is-camera` solver，自动
采集拟合帧和独立移动板留出、部署相同 hash、创建新 map 输出目录、保存
完整 session，并执行 strict debug。通过之前只允许独立单机地图，不允许
`--fuse`。标定 JSON 内保存每个输入的源路径、大小、SHA-256 以及 observed /
source-derived 分类。完整操作见
[`hub/docs/ONECLICK_SESSION_WORKFLOW.md`](../hub/docs/ONECLICK_SESSION_WORKFLOW.md)。

以下 D455 内容保留为回滚/历史复现路径：

Yunji 的外接 D455 不在底盘 `/tf` 树中。不要只把口头测量写成新的源码常量；发送器支持显式版本化文件：

```bash
python3 hub/robot_overlay/yunji_sender.py --help
# 启动参数必须同时包含：
# --camera-source local-realsense
# --local-camera-model d455
# --camera-extrinsic-file <base_link-camera artifact>
# --shared-frame-transform-file <gravity-preserving board artifact>
```

历史 D455 实机验证产物位于：

- `hub/config/calibration/yunji_d455_mount_nominal_20260721.json`：旧口头测量，仅作为推导输入；
- `hub/config/calibration/yunji_d455_ground_extrinsic_20260722.json`：九个双朝向地面帧推导；
- `hub/config/calibration/shared_board_gravity_20260722_v3.json`：标定板 yaw-only 共享变换，含独立移动板留出结果。

这些 D455 文件只能复现其记录时的 Yunji、安装位置和 odom 会话，不能证明另一台机器的机械安装相同。相机被拆装、机器人姿态基准改变或 odom 重置后，按
[离线标定流程](../hub/docs/OFFLINE_MAP_VALIDATION.md) 重新运行：先用多朝向地面帧执行
`derive_ground_camera_extrinsic.py`，再用同步标定板和独立移动板留出执行
`calibrate_gravity_shared_frame_via_board.py`。

切换发送器时保留最后一个旧序号。新 map daemon 必须使用新的输出目录、从该序号之后开始，并同时绑定新的 `transform_version` 和 `shared_frame_calibration_id`。绝不把旧/新外参帧写进同一张图。认证值通过服务环境传递，不写进 Git、命令示例或标定 JSON。

## 10. 导入 Hub 与 Foxglove

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

## 11. 复现完成的判据

代码层完成：仓库验证、轻量测试、TinyNav patch verification 全通过。

观测层完成：相机/IMU 频率连续、perception health 正常、静止测试无持续漂移、BuildMap 能收到正向保存确认。

Yunji Odin 的操作者低速移动地图门禁已通过（1.193 m 有效路径、85 个新增关键帧、无位姿跳变或地面拒绝）。

双机代码层还应核对 persistent-session publication 对应的部署快照：

```text
WSJ   /home/nvidia/topofocus_buildmap_v2_20260723
Yunji /home/nyu/topofocus_buildmap_v2_20260723
```

历史 retry3 双端归档 SHA-256 为
`e1b9001fb188a3890037f5e33927d25afa44473fb50a6b8c40b61a6e123b1b72`；
persistent-session publication 的最终 archive identity 见
[`audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md`](../audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md)。
归档 hash 只证明文件已同步，不证明运行进程已加载。

完整运动层仍未完成。`official-run01` 及三个 retry 都是工程尝试，
全部排除在 SR/SPL 之外。新 persistent session 必须依次通过：

1. 一键标定、独立移动板留出和 fresh map 创建；
2. 两端 debug-only fresh health/pose/map 与真实 VLM 全栈；
3. 一次新授权下的有限运动、物理到达和独立成功确认；
4. 用 `record_realworld_trial.py` 立即绑定 shortest-path/terminal evidence；
5. 随后才是四场景 × 五次正式采集。

不得因地图、VLM 或命令日志能更新就把工程尝试登记为正式成功。
