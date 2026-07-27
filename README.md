# TopoFocus Real-World

TopoFocus 的双机器人真机系统：GPU Hub 负责在线地图、语义理解、VLM
决策与高层协调，Robot 0 和 Robot 1 在本地完成规划、控制与安全停止。

## 实验平台与设备

<p align="center">
  <img src="media/image/platforms_annotated.jpg" width="900" alt="Real-world dual-robot platforms: Robot 0 (Unitree Go2) with RealSense D435i, Robot 1 wheeled chassis with Odin1 fisheye+LiDAR+IMU">
</p>

- **Robot 0**：Unitree Go2 四足底盘（可复现的商用平台），头部前向搭载
  Intel RealSense D435i RGB-D 相机与机载计算单元，用于实时建图与局部规划。
- **Robot 1**：轮式配送底盘，后部搭载 Odin1 鱼眼相机 + 激光雷达 + IMU
  一体式传感单元与机载计算单元。
- 两台机器人均在本地完成路径规划、速度控制与安全停止；GPU Hub 只发布
  带过期时间的高层语义导航目标，不直接下发底层速度指令。

## Scene 01 · Chair

两台机器人从同一实验室起始区域协同探索并到达白色椅子。五次正式实验均
成功。

| Trials | Success | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `5` | `1.0` | `0.726088` | `0.780993` |

Standard SPL 使用独立测量的最短可行路径 `L≈3.25 m`；source-compatible
SPL 使用成功机器人的起终点位移 `D` 作为源码兼容参考。

### Real-robot rollouts

<table>
  <tr>
    <td width="50%" align="center">
      <strong>Formal 01 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_01_preview.gif" width="440" alt="Formal 01 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_01_dashboard.gif" width="440" alt="Formal 01 dashboard"><br>
      Robot 0 先探索前沿，随后切换至 chair 语义区域并自动 ARRIVED；Robot 1
      沿独立前沿推进后同步 HOLD。<br>
      <a href="media/demo/scene01_formal_01_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_01_dashboard.mp4">Dashboard</a>
    </td>
    <td width="50%" align="center">
      <strong>Formal 02 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_02_preview.gif" width="440" alt="Formal 02 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_02_dashboard.gif" width="440" alt="Formal 02 dashboard"><br>
      Robot 0 完成前沿探索后进入 chair 语义区域并自动 ARRIVED；Robot 1
      由协调器保持近静止。<br>
      <a href="media/demo/scene01_formal_02_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_02_dashboard.mp4">Dashboard</a>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <strong>Formal 03 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_03_preview.gif" width="440" alt="Formal 03 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_03_dashboard.gif" width="440" alt="Formal 03 dashboard"><br>
      Robot 0 完成首轮前沿探索；Robot 1 接力探索并驶入 chair 的
      0.5 m 成功区域，由操作者确认到达。<br>
      <a href="media/demo/scene01_formal_03_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_03_dashboard.mp4">Dashboard</a>
    </td>
    <td width="50%" align="center">
      <strong>Formal 04 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_04_preview.gif" width="440" alt="Formal 04 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_04_dashboard.gif" width="440" alt="Formal 04 dashboard"><br>
      双机交替并行探索；Robot 1 切换至 chair 语义区域并自动 ARRIVED，
      Robot 0 完成独立前沿后同步 HOLD。<br>
      <a href="media/demo/scene01_formal_04_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_04_dashboard.mp4">Dashboard</a>
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <strong>Formal 05 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_05_preview.gif" width="440" alt="Formal 05 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_05_dashboard.gif" width="440" alt="Formal 05 dashboard"><br>
      Robot 0 完成前沿探索并持续导航至 chair 后自动 ARRIVED；Robot 1
      按路径协调保持 HOLD。<br>
      <a href="media/demo/scene01_formal_05_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_05_dashboard.mp4">Dashboard</a>
    </td>
  </tr>
</table>

### Per-run metrics

| Run | Arriving robot | Actual path `P` | Source-compatible SPL | Standard SPL |
| --- | --- | ---: | ---: | ---: |
| Formal 01 | Robot 0 | `3.850792 m` | `0.628399` | `0.843982` |
| Formal 02 | Robot 0 | `2.760377 m` | `0.892911` | `1.000000` |
| Formal 03 | Robot 1 | `4.048842 m` | `0.864048` | `0.802699` |
| Formal 04 | Robot 1 | `3.210222 m` | `0.956361` | `1.000000` |
| Formal 05 | Robot 0 | `12.582981 m` | `0.288722` | `0.258285` |

[完整五次实验归档](audit/SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md)
· [机器可读结果](manifests/scene01_chair_formal_experiments_20260725.json)
· [媒体清单](media/README.md)
· [运行说明](hub/docs/ONECLICK_SESSION_WORKFLOW.md)
