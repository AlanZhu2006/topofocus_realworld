# WSJ 基线（2026-07-21）

> 这是 2026-07-21 的历史基线。之后已经加入实测 body-camera 外参、
> 在线 BuildMap 目标路由和 v2 租约门控；当前状态以
> [`../CURRENT_STATUS.md`](../CURRENT_STATUS.md) 为准。

以下均为 **observed**，除非明确标注 source-derived。它描述被检查的 WSJ，不保证另一台 Jetson 自动相同。

## 主机与中间件

| 项目 | 已观察值 |
| --- | --- |
| Hostname | `tegra-ubuntu` |
| Hardware | NVIDIA Jetson Orin NX Engineering Reference Developer Kit |
| Architecture | `aarch64` |
| OS | Ubuntu 22.04.5 LTS |
| Kernel | `5.15.148-tegra` |
| JetPack | `6.2.1+b38` |
| L4T core | `36.4.7-20250918154033` |
| ROS | Humble, `ros-humble-ros-base 0.10.0-1jammy.20260310.230016` |
| RMW | CycloneDDS `1.3.4-1jammy.20260605.112510` |
| Python | 3.10.12 |
| uv | 0.11.2 aarch64 |

## TinyNav 与传感器

| 项目 | 已观察值 |
| --- | --- |
| TinyNav upstream base | `UniflexAI/tinynav@576c082e69580f618a5ff313a3e74f3672abb69f` |
| WSJ deployment head | `933fce54ae65e775a1262c346180341f5657c0e4` |
| IMU fix head | `29f26bc058886ff450f02cdc0d6e9977e1c57010` |
| `pyproject.toml` SHA-256 | `17037c48b5bcf5c0eb8346ef1338c73e334cd4295af9825f6b561405663ca926` |
| `uv.lock` SHA-256 | `b9b451020635612b3900d73d193ab439206089fd37d8a5b4e2b3be6a463daa79` |
| Camera | Intel RealSense D435i, USB id `8086:0b3a` |
| USB3 hub | Genesys Logic `05e3:0625` |
| librealsense | `v2.58.1`, commit `bf2778061d5dd29776e9aca8765f75852671760b` |
| realsense-ros | `4.58.1`, commit `53aceeac29e2e848e5e1ede5430124f07b2fe924` |
| Camera firmware | 5.17.0.10（先前设备审计 observed） |
| USBFS memory | 1000 MB |

稳定相机 profile：

```text
initial_reset=false
848x480x30 color + infra1 + infra2
depth=false
gyro=true, accel=true, unite_imu_method=2
enable_sync=false, align_depth=false
publish_tf=true, tf_publish_rate=1.0
```

这是 TinyNav stereo/IMU 原生 BuildMap 路径；不要与早期启用 aligned hardware depth 的实验 profile 混为一谈。

## USB 遗留问题

`usbfs-memory-fix.service` 已在重启后保持 1000 MB。`99-realsense-usb-power.rules` 的文件 SHA-256 为：

```text
908883197f0c3f79fa3d7d9801f3834a5e4b92f4d88600196cd512a9ed05ad90
```

但曾实际观察到：设备 add 后为 `on`，RealSense driver bind 又恢复成 `auto`。所以 udev 规则本身不是充分证明；启动器必须在 bind 后重设并读取验证。

## 最新原生 BuildMap 保存

远端路径：`/home/nvidia/focus_sender/buildmap_native_move_gate_20260721_2135`。

| 项目 | observed |
| --- | ---: |
| 目录占用 | 346 MiB（`du -sh`） |
| pose 数 | 161 |
| 时间跨度 | 1024.753068288 s |
| 优化 pose 累计路径 | 0.1294259495 m |
| 首尾位移 | 0.0015471353 m |
| occupancy shape | `(102, 101, 22)` |
| `occupancy_grid.npy` SHA-256 | `576d9a63e3c996a5090e860cfafbd8fa684d7f80e765eb3671cfb27f6ac73dd0` |
| `occupancy_meta.npy` SHA-256 | `85ecd6501539bf2560089e742375edf2f7bc59c2ebca38fb6d0995c21387cd3f` |
| `occupancy_2d_image.png` SHA-256 | `f601c856305e5c987b0c850b62dfb913d54880c60a31451e70ad85de47acb82a` |
| `poses.npy` SHA-256 | `a9613f397807a1b6f873146c7b816e9d4794ed642a56b7837cc26d3d4efb68a0` |

收到 `/benchmark/data_saved=true` 后 wrapper 正常退出。操作者在这段会话没有执行受控移动，因此它只能证明长时间静止采集、优化与保存，不能代替下一次移动门禁。

## 当前不成立的声明

- 没有证明另一台 Go2 已一键复现；本仓库提供的是首次可执行复现合同。
- 没有通过 G5 hardware-in-the-loop。
- 没有允许 Hub 直接控制 Unitree。
- 没有证明 udev 规则在所有 RealSense driver/firmware 组合下单独充分。
- 没有证明双机器人标定在相机重新安装后仍有效；移动相机后必须重新标定并更换 transform version。
