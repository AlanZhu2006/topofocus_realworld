# Current project status

Snapshot: **2026-07-28, after replacing Scene 02 formal experiment 02 with the
Yunji-only algorithmic-exploration failure and retaining formal experiment 03
as the first Scene 02 success.**

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

Scene `scene02-plant`, target `plant`, currently contains three archived
formal episodes: two failures and one operator-confirmed success.

| Episodes | Successes | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `3` | `1` | `0.3333333333333333` | `0.2883973021635285` | `0.27922295634194155` |

Standard SPL uses the operator-provided independently measured shortest
feasible path `L≈7 m` ("大概直线距离应该是7m 可以计算standard spl").

Formal 01, episode `scene02-plant-recal1-20260728-044959`: failed during
coordinated execution — one assigned frontier was rejected as locally
unreachable, and the remaining semantic-navigation leg terminated before
arrival. WSJ travelled `6.104564 m` and Yunji travelled `1.905387 m`.

Formal 02, episode `yunji-single-02` (session
`scene02-plant-20260728-0720-yunjireanchor1-single2-r4`): an operator-scoped
Yunji-only run with WSJ powered off and forced `HOLD`. Yunji travelled
`7.425951 m` through 13 completed non-semantic frontier rounds without a
plant arrival; the operator observed one branch exploring away from the
target. Two consecutive sub-`0.05 m` progress intervals then triggered
`failed_cross_round_no_progress_holding`.

Both failures contribute zero SR/SPL. Formal 02 is specifically attributed
`navigation_policy_failure / algorithmic_exploration_failure`, not an
engineering-chain failure.

Formal 03, episode `yunji-single-01` (session
`scene02-plant-20260728-yunji-single2`): an operator-scoped Yunji-only live
run (WSJ forced HOLD throughout, no live motion authority). Yunji explored,
switched to the plant semantic region in round 6, and emitted
`LOCAL_PLANNER_ARRIVED` in round 10 with the plant visible in its terminal
RGB; the operator confirmed the physical success afterward
("可以 把这个归档为formal 003 success"). Yunji travelled `8.356524 m`
(source-compatible `SPL=0.865192`; standard `SPL=0.837669` using the
operator-measured `L≈7 m`).

Complete records:
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md) /
[`manifests/scene02_plant_formal_experiment_01_failure_20260728.json`](manifests/scene02_plant_formal_experiment_01_failure_20260728.json),
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md) /
[`manifests/scene02_plant_formal_experiment_02_failure_20260728.json`](manifests/scene02_plant_formal_experiment_02_failure_20260728.json)
and
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md) /
[`manifests/scene02_plant_formal_experiment_03_success_20260728.json`](manifests/scene02_plant_formal_experiment_03_success_20260728.json).

## Published media

- Five original third-view masters and five Dashboard masters are preserved
  under `media/video/**` through Git LFS.
- Five standardized H.264 third-view/Dashboard pairs are under `media/demo/`.
- Five third-view and five Dashboard animated previews play directly in the
  main README.
- Formal 05 display media covers the run beginning through physical arrival;
  its SR/SPL uses the complete episode report.
- Scene 02's previously published third-view and Dashboard masters remain
  preserved under `media/video/**` through Git LFS. The old Formal 02 media
  belongs to the superseded run and is not evidence for the replacement
  `yunji-single-02`; replacement media has not yet been bound.

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

A deployment-layer repair to the local-navigation stale-route recovery timing
has been applied and is covered by regression tests, but has not yet been
re-verified against a live physical run. WATER/TinyNav collision, watchdog,
lease, localization, occupancy and robot-local stop/reject authority remain
unchanged. No file under immutable `source/` or `dependencies/` was changed.

## Physical-runtime boundary

Replacement Formal 02 was cleaned up with Hub `GOAL=false` for both robots
and `velocity_zero_confirmed=true`; live motion authority was disabled at
episode end. Session
`scene02-plant-20260728-0720-yunjireanchor1-single2-r4` was bound to commit
`dcc8812b027c40fad2716b8a097e45d226d46686` and used a Yunji-only
operator-scoped execution contract — WSJ never held live motion authority.
A new code-bound session and strict no-motion debug are required before the
next live run; calibration reuse additionally requires unchanged camera
mounts and tracking epochs.

No physical robot command was issued by the archival or code verification
steps themselves.

Engineering attempts and diagnostic media remain separate from the formal
results in
[`audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md`](audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md).

## Next formal run

Scene 02 formal experiment 04 is next. Before live motion: create a new
session bound to the current commit, pass strict no-motion debug, and obtain
a fresh onsite motion confirmation. A successful episode still requires an
independent Scene 02 shortest-feasible-path measurement before Standard SPL
finalization.
