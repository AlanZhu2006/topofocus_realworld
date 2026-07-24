# Audit index

The files in this directory are dated evidence records. They preserve what was
observed at a particular gate and should not be silently rewritten to look
current. For the authoritative project state, read
[`CURRENT_STATUS.md`](../CURRENT_STATUS.md).

## Current physical-chain evidence

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
