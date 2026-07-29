# Current project status

Snapshot: **2026-07-29, after completing the five-run Scene 02 campaign and
pausing Scene 03 for a fresh calibration following repository/workspace
consolidation.**

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

Scene `scene02-plant`, target `plant`, contains five archived formal episodes:
two failures and three operator-confirmed successes.

| Episodes | Successes | SR | Mean source-compatible SPL | Mean Standard SPL |
| ---: | ---: | ---: | ---: | ---: |
| `5` | `3` | `0.6` | `0.557366832135995` | `0.531689377642298` |

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

Formal 03-05 are successes. Robot 1 is the arriving robot in all three runs:

| Formal run | Robot 0 path | Robot 1 path | Source-compatible SPL | Standard SPL |
| --- | ---: | ---: | ---: | ---: |
| 03 | `0.728655 m` | `8.356524 m` | `0.865192` | `0.837669` |
| 04 | `0.454227 m` | `7.579081 m` | `0.961379` | `0.923595` |
| 05 | `2.032182 m` | `7.802197 m` | `0.960264` | `0.897183` |

Complete records:
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md) /
[`manifests/scene02_plant_formal_experiment_01_failure_20260728.json`](manifests/scene02_plant_formal_experiment_01_failure_20260728.json),
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md) /
[`manifests/scene02_plant_formal_experiment_02_failure_20260728.json`](manifests/scene02_plant_formal_experiment_02_failure_20260728.json)
and
[`audit/SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md`](audit/SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md) /
[`manifests/scene02_plant_formal_experiment_03_success_20260728.json`](manifests/scene02_plant_formal_experiment_03_success_20260728.json),
[`manifests/scene02_plant_formal_experiment_04_success_20260728.json`](manifests/scene02_plant_formal_experiment_04_success_20260728.json),
[`manifests/scene02_plant_formal_experiment_05_success_20260728.json`](manifests/scene02_plant_formal_experiment_05_success_20260728.json)
and the
[`five-run aggregate`](manifests/scene02_plant_formal_experiments_20260728.json).

## Published media

- Five original third-view masters and five Dashboard masters are preserved
  under `media/video/**` through Git LFS.
- Five standardized H.264 third-view/Dashboard pairs are under `media/demo/`.
- Five third-view and five Dashboard animated previews play directly in the
  main README.
- Formal 05 display media covers the run beginning through physical arrival;
  its SR/SPL uses the complete episode report.
- Scene 02's third-view and Dashboard masters for Formal 01-05 are preserved
  under `media/video/**` through Git LFS, each with an H.264 derivative pair
  and a time-lapsed ~8 s README preview GIF under `media/demo/`. Formal 02's
  replacement run (`yunji-single-02`) is now bound to its own masters, which
  reuse the original `experiment_2_failure_2.*` filenames with replaced
  content; the pre-replacement Formal 02 media remains recoverable in Git
  history at commit `dcc8812b027c40fad2716b8a097e45d226d46686` but is no
  longer current evidence.

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

On 2026-07-29 the local Hub and Foxglove relays were stopped, and no current
session pointer was retained. The 55 completed calibrations and all 163
session contracts remain archived and hash-valid; 30 never-completed
calibration directories were moved intact to the failed-attempt archive.
The latest Scene 03 map diagnosis and previous session pointer were preserved,
but neither is eligible for implicit reuse.

No physical robot command was issued by the archival or code verification
steps themselves.

Engineering attempts and diagnostic media remain separate from the formal
results in
[`audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md`](audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md).
The full repository/runtime organization record is
[`audit/REPOSITORY_WORKSPACE_ORGANIZATION_20260729.md`](audit/REPOSITORY_WORKSPACE_ORGANIZATION_20260729.md).

## Next formal run

Scene 03 formal experiment 01 is next. It requires a fresh shared-frame
calibration, a new code-bound session, strict no-motion debug and a new onsite
motion confirmation before any live target can be enabled.
