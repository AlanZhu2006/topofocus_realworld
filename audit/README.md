# Evidence archive

This directory contains dated observed and source-derived records. They
preserve the boundary of a particular experiment or engineering gate and must
not be rewritten as current state. Start with
[`CURRENT_STATUS.md`](../CURRENT_STATUS.md) for the released result.

## Formal campaigns

| Scene | Human-readable archive | Machine-readable aggregate |
| --- | --- | --- |
| Scene 01 · Chair | [Five-run archive](SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md) | [Manifest](../manifests/scene01_chair_formal_experiments_20260725.json) |
| Scene 02 · Plant | [Formal 01](SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md) · [Formal 02](SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md) · [Formal 03](SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md) | [Manifest](../manifests/scene02_plant_formal_experiments_20260728.json) |
| Scene 03 · Plant | [Five-run archive](SCENE03_PLANT_FORMAL_EXPERIMENTS_01_05_20260731.md) | [Manifest](../manifests/scene03_plant_formal_experiments_20260731.json) |
| Scene 04 · Plant | [Formal 01](SCENE04_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260801.md) · [Formal 02](SCENE04_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260801.md) · [Formal 03](SCENE04_PLANT_FORMAL_EXPERIMENT_03_FAILURE_20260801.md) · [Formal 04](SCENE04_PLANT_FORMAL_EXPERIMENT_04_SUCCESS_20260801.md) · [Formal 05](SCENE04_PLANT_FORMAL_EXPERIMENT_05_SUCCESS_20260801.md) | [Manifest](../manifests/scene04_plant_formal_experiments_20260731.json) |

The cross-scene machine-readable index is
[`manifests/realworld_experiment_progress.json`](../manifests/realworld_experiment_progress.json).

## Engineering provenance

- Repository and deployment:
  [`REPOSITORY_WORKSPACE_ORGANIZATION_20260729.md`](REPOSITORY_WORKSPACE_ORGANIZATION_20260729.md),
  [`REPOSITORY_AND_ONECLICK_AUDIT_20260724.md`](REPOSITORY_AND_ONECLICK_AUDIT_20260724.md),
  [`DUAL_ROBOT_CODE_SYNC_20260723.md`](DUAL_ROBOT_CODE_SYNC_20260723.md)
- Calibration and sensors:
  [`SHARED_FRAME_ODIN1_20260723.md`](SHARED_FRAME_ODIN1_20260723.md),
  [`WSJ_CALIBRATION_SENSOR_EPOCH_RECOVERY_20260724.md`](WSJ_CALIBRATION_SENSOR_EPOCH_RECOVERY_20260724.md),
  [`WSJ_CALIBRATION_KEYFRAME_GATE_20260727.md`](WSJ_CALIBRATION_KEYFRAME_GATE_20260727.md)
- Mapping and semantics:
  [`SEMANTIC_OVERVIEW_REAUDIT_20260724.md`](SEMANTIC_OVERVIEW_REAUDIT_20260724.md),
  [`PIXEL_SEMANTIC_OVERVIEW_20260723.md`](PIXEL_SEMANTIC_OVERVIEW_20260723.md),
  [`LIVE_MAP_RECOVERY_20260722.md`](LIVE_MAP_RECOVERY_20260722.md)
- Decision and control:
  [`SOURCE_DERIVED_VLM_SCENE_RUNNER_20260723.md`](SOURCE_DERIVED_VLM_SCENE_RUNNER_20260723.md),
  [`SCENE03_RUNTIME_ISSUES_COMPLETE_REPAIR_20260731.md`](SCENE03_RUNTIME_ISSUES_COMPLETE_REPAIR_20260731.md),
  [`FAILURE_ATTRIBUTION_PROTOCOL_20260725.md`](FAILURE_ATTRIBUTION_PROTOCOL_20260725.md)
- Safety and transport:
  [`DUAL_ROBOT_COLLISION_20260725.md`](DUAL_ROBOT_COLLISION_20260725.md),
  [`G5_FAULT_INJECTION.md`](G5_FAULT_INJECTION.md),
  [`ONECLICK_RELIABILITY_AND_LATENCY_20260725.md`](ONECLICK_RELIABILITY_AND_LATENCY_20260725.md)

All other files remain searchable by date and subsystem in this directory.
An older record may contain paths, calibration IDs or conclusions valid only
for that run; do not use it as a launch instruction without checking the
current runbook and supervised workflow.
