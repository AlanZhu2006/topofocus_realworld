# Current project status

Snapshot: **2026-07-28, after Scene 02 formal experiment 01 failure archival
and local trajectory-recovery verification.**

## Scene 01 formal results

Scene `scene01-chair`, target `chair`, contains five formal successful
real-robot experiments.

| Episodes | Successes | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `5` | `1.0` | `0.7260879584850242` | `0.7809932415154623` |

Standard SPL uses the operator-provided independently measured shortest
feasible path `L≈3.25 m`.

| Formal run | Runtime episode | Arriving robot | Result |
| --- | --- | --- | --- |
| 01 | `trial-r5-01` | WSJ | automatic `LOCAL_PLANNER_ARRIVED` |
| 02 | `trial-reanchor1-r1` | WSJ | automatic `LOCAL_PLANNER_ARRIVED` |
| 03 | `trial-05-nearwall-fix` | Yunji | operator-confirmed inside `0.5 m` |
| 04 | `trial-wallfix-imudebounce-r1` | Yunji | automatic `LOCAL_PLANNER_ARRIVED` |
| 05 | `scene01-chair-run05` | WSJ | automatic `LOCAL_PLANNER_ARRIVED` |

The complete experiment/action/media binding is in
[`audit/SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md`](audit/SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md).
Exact machine-readable metrics, paths, hashes and evidence classes are in
[`manifests/scene01_chair_formal_experiments_20260725.json`](manifests/scene01_chair_formal_experiments_20260725.json).

## Scene 02 formal results

Scene `scene02-plant`, target `plant`, currently contains one archived formal
episode.

| Episodes | Successes | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `1` | `0` | `0.0` | `0.0` | `0.0` |

Formal 01, episode `scene02-plant-recal1-20260728-044959`, completed four
source-derived rounds. WSJ travelled `6.104564 m` and Yunji travelled
`1.905387 m`; both ended in confirmed zero-velocity HOLD. The terminal result
is `execution_engineering_failure/local_planner_trajectory_stale`, not a VLM
failure: round 4 had already selected the plant semantic region for WSJ while
its local router remained `NAVIGATING/ONLINE_PATH_READY`.

The complete record is
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md);
the machine-readable result is
[`manifests/scene02_plant_formal_experiment_01_failure_20260728.json`](manifests/scene02_plant_formal_experiment_01_failure_20260728.json).

## Published media

- Five original third-view masters and five Dashboard masters are preserved
  under `media/video/**` through Git LFS.
- Five standardized H.264 third-view/Dashboard pairs are under `media/demo/`.
- Five third-view and five Dashboard animated previews play directly in the
  main README.
- Formal 05 display media covers the run beginning through physical arrival;
  its SR/SPL uses the complete episode report.
- Scene 02 formal experiment 01's third-view and Dashboard masters are
  likewise preserved under `media/video/**` through Git LFS, with an H.264
  pair, terminal-frame posters and ~8 s README preview GIFs under
  `media/demo/`. See
  [`media/README.md`](media/README.md#scene-02-media).

## Implementation state

The physical chain has exercised:

```text
RGB-D / pose
  -> online robot-local maps
  -> shared semantic/fused maps
  -> source-derived VLM decisions
  -> versioned expiring high-level targets
  -> robot-local planning and control
  -> navigation feedback
  -> coordinated HOLD
```

The current deployment repair keeps the stale-trajectory physical zero gate
at `1.0 s` and changes only the terminal republish window from `3.0 s` to
`5.0 s` for an already-started semantic leg. Both robot launchers pass this
contract explicitly. A never-started path retains its shorter `1.5 s` grace,
and WATER/TinyNav collision, watchdog, lease, localization, occupancy and
robot-local stop/reject authority remain unchanged.

The exact observed `3.365452042 s` gap is covered by regression tests; 45
targeted receiver tests and all 599 Hub tests pass locally.

No file under immutable `source/` or `dependencies/` was changed.

## Physical-runtime boundary

The failed episode was cleaned up with Hub `GOAL=false` and both robots
reporting `HOLDING` plus `velocity_zero_confirmed=true`. The validated
calibration artifact remains preserved, but session
`scene02-plant-20260728-044246-recal2` is bound to pre-fix commit
`a3dbe09bd543a7b26a10241712b6c9b1c60192e5`; it cannot authorize post-fix live
motion. A new code-bound session and strict no-motion debug are required before
the next run. Calibration reuse additionally requires unchanged camera mounts
and tracking epochs.

No physical robot command was issued by the archival or code verification.

Engineering attempts and diagnostic media remain separate from the formal
results in
[`audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md`](audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md).

## Next formal run

Scene 02 formal experiment 02 is next. Before live motion: synchronize the
committed repair byte-identically to both robot roots, create a new session
bound to that commit, pass strict no-motion debug, and obtain a fresh onsite
motion confirmation. A successful episode still requires an independent
Scene 02 shortest-feasible-path measurement before Standard SPL finalization.
