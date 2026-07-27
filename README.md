# TopoFocus Real-World

TopoFocus's real-world dual-robot system: a GPU hub handles online mapping,
semantic understanding, VLM decisions and high-level coordination, while
Robot 0 and Robot 1 handle planning, control and safety stops locally.

## Experiment platforms and equipment

<p align="center">
  <img src="media/image/platforms_annotated.jpg" width="900" alt="Real-world dual-robot platforms: Robot 0 (Unitree Go2) with RealSense D435i, Robot 1 wheeled chassis with an Odin1 spatial memory module">
</p>

- **Robot 0**: a Unitree Go2 quadruped chassis (a reproducible commercial
  platform), carrying a head-mounted, forward-facing Intel RealSense D435i
  RGB-D camera and onboard compute for real-time mapping and local planning.
- **Robot 1**: a wheeled delivery chassis, carrying an
  [Odin1 spatial memory module](https://www.manifoldtech.cn/) (RGB-D camera
  + LiDAR + IMU, by Manifold Tech) and onboard compute on its rear deck.
- Both robots run path planning, velocity control and safety stops locally;
  the GPU hub only publishes high-level, expiring semantic navigation
  targets and never issues low-level velocity commands.

## Cross-robot calibration

<p align="center">
  <img src="media/image/calibration_annotated.jpg" width="900" alt="Shared cross-robot calibration using a circle-grid board">
</p>

With both robots stationary, each robot's own camera captures one shared
7×10 symmetric circle-grid board (40 mm spacing) and solves its pose by
PnP; the two per-camera poses compose into a fixed transform between the
robots' camera frames, registering their local odometry into one shared
frame. The board is then moved by at least 10 cm or rotated by at least 5°
and re-observed as an independent holdout — the calibration is accepted
only if it also explains this second, independently moved observation.

## Scene 01 · Chair

Both robots explore cooperatively from the same lab starting area and reach
a white chair. All five formal experiments succeeded.

| Trials | Success | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `5` | `1.0` | `0.726088` | `0.780993` |

Standard SPL uses the independently measured shortest feasible path
`L≈3.25 m`; source-compatible SPL uses the arriving robot's start-to-arrival
displacement `D` as a source-compatible reference.

### Real-robot rollouts

<table>
  <tr>
    <td width="50%" align="center">
      <strong>Formal 01 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_01_preview.gif" width="440" alt="Formal 01 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_01_dashboard.gif" width="440" alt="Formal 01 dashboard"><br>
      Robot 0 explores a frontier first, then switches to the chair semantic
      region and auto-ARRIVED; Robot 1 advances along an independent
      frontier and HOLDs in sync.<br>
      <a href="media/demo/scene01_formal_01_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_01_dashboard.mp4">Dashboard</a>
    </td>
    <td width="50%" align="center">
      <strong>Formal 02 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_02_preview.gif" width="440" alt="Formal 02 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_02_dashboard.gif" width="440" alt="Formal 02 dashboard"><br>
      Robot 0 finishes frontier exploration and enters the chair semantic
      region, auto-ARRIVED; Robot 1 is held near-stationary by the
      coordinator.<br>
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
      Robot 0 completes an initial round of frontier exploration; Robot 1
      relays exploration and enters the chair's 0.5 m success region,
      confirmed on arrival by the operator.<br>
      <a href="media/demo/scene01_formal_03_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene01_formal_03_dashboard.mp4">Dashboard</a>
    </td>
    <td width="50%" align="center">
      <strong>Formal 04 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene01_formal_04_preview.gif" width="440" alt="Formal 04 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene01_formal_04_dashboard.gif" width="440" alt="Formal 04 dashboard"><br>
      Both robots explore in alternating parallel rounds; Robot 1 switches
      to the chair semantic region and auto-ARRIVED, while Robot 0 finishes
      its independent frontier and HOLDs in sync.<br>
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
      Robot 0 finishes frontier exploration and continuously navigates to
      the chair, auto-ARRIVED; Robot 1 HOLDs under route coordination.<br>
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

[Full five-experiment archive](audit/SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md)
· [Machine-readable results](manifests/scene01_chair_formal_experiments_20260725.json)
· [Media manifest](media/README.md)
· [Run instructions](hub/docs/ONECLICK_SESSION_WORKFLOW.md)

## Scene 02 · Plant

Both robots explore cooperatively toward a plant target.

| Trials | Success | SR | Mean source-compatible SPL | Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `3` | `1` | `0.333333` | `0.288397` | pending |

Mean source-compatible SPL counts both failures at zero contribution.
Standard SPL awaits an independent Scene 02 shortest-feasible-path
measurement (Scene 01's `3.25 m` is not reused).

<table>
  <tr>
    <td width="50%" align="center">
      <strong>Formal 01 · FAILURE</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene02_formal_01_preview.gif" width="440" alt="Formal 01 failure rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene02_formal_01_dashboard.gif" width="440" alt="Formal 01 failure dashboard"><br>
      Formal 01 failed during coordinated execution: one assigned frontier
      was rejected as locally unreachable, and the remaining
      semantic-navigation leg terminated before arrival.<br>
      <a href="media/demo/scene02_formal_01_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene02_formal_01_dashboard.mp4">Dashboard</a>
    </td>
    <td width="50%" align="center">
      <strong>Formal 02 · FAILURE</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene02_formal_02_preview.gif" width="440" alt="Formal 02 failure rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene02_formal_02_dashboard.gif" width="440" alt="Formal 02 failure dashboard"><br>
      Formal 02 failed during coordinated execution: both robots' assigned
      frontiers were rejected as locally unreachable before any plant
      semantic region was found, and the episode then timed out waiting for
      a fresh synchronized round after Robot 1's map was blocked by
      ground-plane drift.<br>
      <a href="media/demo/scene02_formal_02_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene02_formal_02_dashboard.mp4">Dashboard</a>
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <strong>Formal 03 · SUCCESS</strong><br>
      <small>Third view</small><br>
      <img src="media/demo/scene02_formal_03_preview.gif" width="440" alt="Formal 03 rollout"><br>
      <small>Dashboard</small><br>
      <img src="media/demo/scene02_formal_03_dashboard.gif" width="440" alt="Formal 03 dashboard"><br>
      Robot 1 ran solo under an operator-scoped single-robot live authorization
      (Robot 0 held throughout); it explored frontiers, switched to the plant
      semantic region and auto-ARRIVED, confirmed by the operator.<br>
      <a href="media/demo/scene02_formal_03_third_view.mp4">Third view</a> ·
      <a href="media/demo/scene02_formal_03_dashboard.mp4">Dashboard</a>
    </td>
  </tr>
</table>

### Per-run metrics

| Run | Result | Robot 0 path | Robot 1 path | Source-compatible SPL |
| --- | --- | ---: | ---: | ---: |
| Formal 01 | FAILURE | `6.104564 m` | `1.905387 m` | `0.0` |
| Formal 02 | FAILURE | `0.208027 m` | `1.488125 m` | `0.0` |
| Formal 03 | SUCCESS | `0.728655 m`* | `8.356524 m` | `0.865192` |

\* Robot 0 had no live motion authority in Formal 03 (operator-scoped
single-robot run); its accumulated odometry is retained as observed
provenance, not commanded travel — net displacement was `0.002418 m`.

[Formal 01 failure record](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md)
· [Formal 02 failure record](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md)
· [Formal 03 success record](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md)
· [Machine-readable results](manifests/realworld_experiment_progress.json)
