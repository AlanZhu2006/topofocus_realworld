# Audit index

The files in this directory are dated evidence records. They preserve what was
observed at a particular gate and should not be silently rewritten to look
current. For the authoritative project state, read
[`CURRENT_STATUS.md`](../CURRENT_STATUS.md).

## Current physical-chain evidence

- [`ONECLICK_RELIABILITY_AND_LATENCY_20260725.md`](ONECLICK_RELIABILITY_AND_LATENCY_20260725.md):
  post-experiment calibration/debug/live/cleanup audit, bounded self-recovery,
  safe startup parallelism, per-phase timing and the current Yunji
  synchronization boundary.
- [`SCENE02_PLANT_PREPARATION_20260725.md`](SCENE02_PLANT_PREPARATION_20260725.md):
  strict no-motion Scene 02 campaign preparation, plant-category path
  verification, five planned episode IDs and fresh-calibration boundary.
- [`SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md`](SCENE01_CHAIR_FORMAL_EXPERIMENTS_01_05_20260725.md):
  unified Scene 01 formal archive with five successes, per-robot behavior,
  SR/source-compatible SPL/Standard SPL and complete media/runtime provenance.
- [`SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md`](SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md):
  separate index for development attempts, collision/preflight records and
  engineering diagnostics excluded from the five formal results.
- [`WSJ_SEMANTIC_ARRIVAL_STABILIZATION_20260725.md`](WSJ_SEMANTIC_ARRIVAL_STABILIZATION_20260725.md):
  post-formal-05 arrival analysis, minimal terminal-planning/velocity fix and
  local verification boundary.
- [`FORMAL_EXPERIMENT_05_PREFLIGHT_ABORT_20260725.md`](FORMAL_EXPERIMENT_05_PREFLIGHT_ABORT_20260725.md):
  earlier experiment-05 preparation stopped inside strict no-motion debug
  after WSJ tracking output froze; no live, GOAL, episode, movement or SR/SPL
  row; retained only as an engineering preparation record.
- [`SCENE01_CHAIR_FORMAL_EXPERIMENT_04_SUCCESS_20260725.md`](SCENE01_CHAIR_FORMAL_EXPERIMENT_04_SUCCESS_20260725.md):
  operator-designated formal experiment 04 success, automatic Yunji
  `LOCAL_PLANNER_ARRIVED`, exact SR/source-compatible SPL, the operator's
  independently measured approximate `L≈3.25 m` and standard `SPL=1.0`,
  strict-debug and
  terminal-evidence provenance.
- [`FAILURE_ATTRIBUTION_PROTOCOL_20260725.md`](FAILURE_ATTRIBUTION_PROTOCOL_20260725.md):
  evidence requirements separating preflight/engineering, perception, VLM
  decision, navigation-policy and unclassified failures, plus current Scene 01
  attribution.
- [`R6_FUSION_PREFLIGHT_ABORT_20260725.md`](R6_FUSION_PREFLIGHT_ABORT_20260725.md):
  operator-caught cross-map wall conflict after strict debug but before live,
  frozen-map geometry diagnosis, exclusion from SR/SPL and the required fresh
  calibration gate.
- [`SCENE01_CHAIR_SUCCESS_R5_20260725.md`](SCENE01_CHAIR_SUCCESS_R5_20260725.md):
  second normal Scene 01 success, automatic WSJ
  `LOCAL_PLANNER_ARRIVED`, exact SR/source-compatible SPL and paired
  external/Dashboard media provenance.
- [`SCENE01_MEDIA_PUBLICATION_20260725.md`](SCENE01_MEDIA_PUBLICATION_20260725.md):
  initial ten-file publication, the later second-success pair and three
  additional unbound candidate masters, with byte hashes, web derivatives and
  exact runtime-binding boundaries.
- [`LAB18_CALIBRATION_ABORT_20260725.md`](LAB18_CALIBRATION_ABORT_20260725.md):
  aborted repeat-2 calibration after the Go2 was moved to charge, the repaired
  cross-launcher release-root mismatch, preserved incomplete runtime
  provenance and the required full-calibration restart boundary.
- [`SCENE01_CHAIR_SUCCESS_20260725.md`](SCENE01_CHAIR_SUCCESS_20260725.md):
  first normal success under the `0.5 m` physical protocol, exact
  source-compatible SPL, paired external/Dashboard video provenance, preserved
  legacy `LOCAL_PLANNER_PATH_STALE` log and the forward automatic-arrival fix.
- [`DUAL_ROBOT_COLLISION_20260725.md`](DUAL_ROBOT_COLLISION_20260725.md):
  first observed concurrent end-to-end physical motion, operator-confirmed
  collision, paired third-view/Dashboard provenance, runtime hashes, route
  intersection replay, network-causality boundary and the locally verified
  post-incident serialization guard.
- [`YUNJI_TINYNAV_MIGRATION_20260724.md`](YUNJI_TINYNAV_MIGRATION_20260724.md):
  Yunji's migration from WATER saved-map navigation to online TinyNav with a
  guarded WATER velocity-only bridge, including pinned-source provenance and
  the current physical-verification boundary.
- [`REPOSITORY_AND_ONECLICK_AUDIT_20260724.md`](REPOSITORY_AND_ONECLICK_AUDIT_20260724.md):
  complete repository/startup-chain audit, persistent-session replacement,
  local verification boundary and final robot synchronization record.
- [`V2_ROBOT_RECEIVERS_20260723.md`](V2_ROBOT_RECEIVERS_20260723.md): v2
  receiver implementation, online BuildMap routing, official-run engineering
  attempts, exact failures and retry3 follow-up.
- [`DUAL_ROBOT_CODE_SYNC_20260723.md`](DUAL_ROBOT_CODE_SYNC_20260723.md):
  byte-verified WSJ/Yunji deployment snapshots, including the final retry3
  fixes.
- [`MINIMAL_ONECLICK_DEPLOYMENT_20260723.md`](MINIMAL_ONECLICK_DEPLOYMENT_20260723.md):
  initial one-click debug deployment and its boundary.
- [`TRIPLE_AI_IMAGE_PREFLIGHT_20260723.md`](TRIPLE_AI_IMAGE_PREFLIGHT_20260723.md):
  historical-image VLM preflight; not a real-world SR/SPL result.

## Current calibration and sensors

- [`YUNJI_WATER_LINK_PREFLIGHT_20260727.md`](YUNJI_WATER_LINK_PREFLIGHT_20260727.md):
  observed no-carrier failure, fail-fast MAC/profile/TCP recovery and
  sequence-backed calibration readiness.
- [`WSJ_CALIBRATION_KEYFRAME_GATE_20260727.md`](WSJ_CALIBRATION_KEYFRAME_GATE_20260727.md):
  observed sparse-keyframe false failure, continuous-health/keyframe-tuple
  gate separation, and fail-closed remote-timeout recovery.
- [`YUNJI_ODIN1_BOOT_RECOVERY_20260724.md`](YUNJI_ODIN1_BOOT_RECOVERY_20260724.md):
  observed post-reboot disabled-driver failure, read-only recovery, and the
  calibration-only automatic startup guard.
- [`WSJ_CALIBRATION_SENSOR_EPOCH_RECOVERY_20260724.md`](WSJ_CALIBRATION_SENSOR_EPOCH_RECOVERY_20260724.md):
  observed RGB-live/depth-stale calibration failure, coordinated pre-board
  recovery, and the post-calibration tracking-epoch guard.
- [`SHARED_FRAME_ODIN1_20260723.md`](SHARED_FRAME_ODIN1_20260723.md): current
  WSJ/Odin board calibration session and holdout.
- [`YUNJI_REBOOT_CALIBRATION_REVALIDATION_20260723.md`](YUNJI_REBOOT_CALIBRATION_REVALIDATION_20260723.md):
  calibration reuse decision after power cycling.
- [`YUNJI_ODIN1_INTEGRATION_20260722.md`](YUNJI_ODIN1_INTEGRATION_20260722.md):
  Odin1 hardware/source integration.
- [`WSJ_POST_REBOOT_READINESS_20260722.md`](WSJ_POST_REBOOT_READINESS_20260722.md):
  WSJ USB and observation readiness.
- [`WSJ_IMU_SCHEDULING_FIX_20260721.md`](WSJ_IMU_SCHEDULING_FIX_20260721.md):
  TinyNav perception/IMU repair provenance.

## Mapping, semantics and Foxglove

- [`SEMANTIC_OVERVIEW_REAUDIT_20260724.md`](SEMANTIC_OVERVIEW_REAUDIT_20260724.md)
- [`PIXEL_SEMANTIC_OVERVIEW_20260723.md`](PIXEL_SEMANTIC_OVERVIEW_20260723.md)
- [`YOLO_SEMANTIC_BEV_LIVE_20260722.md`](YOLO_SEMANTIC_BEV_LIVE_20260722.md)
- [`LIVE_MAP_RECOVERY_20260722.md`](LIVE_MAP_RECOVERY_20260722.md)
- [`OFFLINE_MAP_DIAGNOSTICS_20260722.md`](OFFLINE_MAP_DIAGNOSTICS_20260722.md)
- [`CENTRAL_MAPPING_RAY_FILL_20260721.md`](CENTRAL_MAPPING_RAY_FILL_20260721.md)
- [`FOXGLOVE_DASHBOARD_20260720.md`](FOXGLOVE_DASHBOARD_20260720.md)

## VLM and source-fidelity work

- [`SOURCE_DERIVED_VLM_SCENE_RUNNER_20260723.md`](SOURCE_DERIVED_VLM_SCENE_RUNNER_20260723.md)
- [`LIVE_VLM_SHADOW_20260722.md`](LIVE_VLM_SHADOW_20260722.md)
- [`VLM_DECISION_CASCADE_20260720.md`](VLM_DECISION_CASCADE_20260720.md)
- [`REDNET_DOMAIN_GAP_20260719.md`](REDNET_DOMAIN_GAP_20260719.md)

## Protocol, transport and safety gates

- [`TRANSPORT_V2_DEMO_DRAFT_20260723.md`](TRANSPORT_V2_DEMO_DRAFT_20260723.md)
- [`G5_FAULT_INJECTION.md`](G5_FAULT_INJECTION.md)
- [`SOAK_FULL_CHAIN_20260718.md`](SOAK_FULL_CHAIN_20260718.md)
- [`TRANSPORT_WSJ_TEST.md`](TRANSPORT_WSJ_TEST.md)
- [`E2E_SINGLE_ROBOT.md`](E2E_SINGLE_ROBOT.md)

## Historical baseline gates

- [`G0_LOCAL_VERIFICATION.md`](G0_LOCAL_VERIFICATION.md)
- [`G1_LOCAL_ENVIRONMENT.md`](G1_LOCAL_ENVIRONMENT.md)
- [`G2_LOCAL_GLM_REQUEST.md`](G2_LOCAL_GLM_REQUEST.md)
- [`G3_LOCAL_REPLAY_MAPPING.md`](G3_LOCAL_REPLAY_MAPPING.md)
- [`G4_REAL_CALIBRATION_20260720.md`](G4_REAL_CALIBRATION_20260720.md)

An older audit may contain paths, calibration IDs or conclusions that were
valid only for that dated run. Do not use it as a launch command without
checking the canonical status and current runbook.
