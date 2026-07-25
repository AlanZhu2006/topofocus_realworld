# Current project status

Snapshot: **2026-07-25, after Scene 01 formal experiment 05, media
publication and local arrival-stabilization verification.**

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

## Published media

- Five original third-view masters and five Dashboard masters are preserved
  under `media/video/**` through Git LFS.
- Five standardized H.264 third-view/Dashboard pairs are under `media/demo/`.
- Five third-view and five Dashboard animated previews play directly in the
  main README.
- Formal 05 display media covers the run beginning through physical arrival;
  its SR/SPL uses the complete episode report.

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

Commit `c4b1116f524b691522c34a18dfb0d214da5011d1` contains the minimal WSJ
semantic-arrival stabilization:

- a `0.15 m` inward semantic planning margin while preserving the official
  `0.5 m` arrival radius;
- zeroing tiny negative longitudinal segments before pinned-controller
  reverse quantization;
- explicit launcher configuration and regression coverage.

The change passed 94 targeted tests and the full repository test gate. Details
are in
[`audit/WSJ_SEMANTIC_ARRIVAL_STABILIZATION_20260725.md`](audit/WSJ_SEMANTIC_ARRIVAL_STABILIZATION_20260725.md).

No file under immutable `source/` or `dependencies/` was changed.

## Physical-runtime boundary

Both robots were subsequently power-cycled. The previous tracking/shared-frame
session is therefore archival; the next physical run requires a fresh
calibration/session and a new explicit onsite motion confirmation. No physical
robot command is issued by this documentation/archive work.

Engineering attempts and diagnostic media remain separate from the formal
results in
[`audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md`](audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md).

## Next campaign

Scene 02 is prepared as `scene02-plant`, target `plant`, with five planned
formal episodes `scene02-plant-run01` through `scene02-plant-run05`. The
operator command sheet now binds calibration, debug and live commands to
`plant`; the Scene 01 session remains archival.

No Scene 02 episode or metric sample has been created. Standard SPL remains
unset until the Scene 02 shortest feasible path is independently measured.
The preparation record is
[`audit/SCENE02_PLANT_PREPARATION_20260725.md`](audit/SCENE02_PLANT_PREPARATION_20260725.md).
