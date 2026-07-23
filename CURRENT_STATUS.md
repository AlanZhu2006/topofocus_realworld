# Current project status

Snapshot time: **2026-07-24 02:16 CST**

This is the canonical current-state document. Dated files under `audit/` are
append-only evidence records; they do not supersede this page.

## Executive outcome

The real two-robot chain has reached:

`RGB-D/pose -> central maps -> semantic/VLM decision -> atomic v2 targets ->
TinyNav/WATER local planning -> robot feedback -> lease renewal -> fail-closed
HOLD`

No official scene has completed successfully yet. The engineering attempts
described below must not be included in SR or SPL.

The last experiment (`official-run01-retry3`) proved that the actual v2 Hub
health-authority fix was active: nine high-level batches were accepted without
the earlier health-source 409. Yunji immediately classified itself inside its
selected arrival region. WSJ received only rotation commands at
`vx=0.000, wz=-0.200 rad/s`; the onsite operator observed no physical motion.
The router then reported `ODOMETRY_STALE` and the complete stack returned both
robots to HOLD.

## Current deployment identity

| Item | Current value |
| --- | --- |
| Hub API | `http://127.0.0.1:8188` |
| GLM endpoint | `http://127.0.0.1:31511/v1` |
| Foxglove | `ws://10.208.2.249:8765` |
| Shared calibration ID | `shared-board-odin1-20260723-v3` |
| WSJ transform | `wsj-tinynav-depth-20260723-powercycle-v3` |
| Yunji transform | `yunji-odin1-board-20260723-powercycle-v6` |
| WSJ map | `hub/runtime/map_out_wsj_20260724_rebuild_v12_router025` |
| Yunji map | `hub/runtime/map_out_yunji_20260724_rebuild_v12_router025` |
| Map tmux session | `shared_maps_20260724_rebuild_v12_router025` |
| Foxglove tmux session | `foxglove_relay_20260724_rebuild_v12_router025` |
| WSJ deployment root | `/home/nvidia/topofocus_buildmap_v2_20260723` |
| Yunji deployment root | `/home/nyu/topofocus_buildmap_v2_20260723` |

Power cycling alone does not invalidate calibration. Reuse is allowed only
after a no-motion pose-delta check confirms that the robot, sensor mount and
starting placement did not move.

## What is implemented and verified

### Observation, maps and visualization

- WSJ uses the D435i/TinyNav observation path with the deployed IMU scheduling
  repair, registered RGB to TinyNav depth, measured body-camera calibration
  and independent command-health heartbeat.
- Yunji uses Odin1 `O1-P070100205`, not the retired RealSense lane. The adapter
  consumes native RGB, SLAM cloud and odometry and preserves the factory
  calibration.
- Both current maps use gravity/ground gates, free-space ray fill, reversible
  obstacle evidence, pose-jump blocking and online status artifacts.
- Foxglove publishes camera, occupancy, pose, trajectory, frontiers, semantic
  pixel regions, labels and a fused shared-frame overview.
- Real YOLO inference is enabled by default for the live semantic target path.
  Its map projection remains model inference, not ground truth.

### VLM behavior

- The source-derived Perception, Judgment/FN and Decision stages are ported.
- Directional history is shared across agents.
- Agent 0 selects first and its frontier is removed before Agent 1 selects.
- A detected target semantic component can override a frontier target.
- The continuous shadow runner preserves the executable HPC decision schedule.
- The current physical one-click runner uses one frozen VLM round followed by
  supervised lease renewal. Multi-round physical exploration after reaching a
  frontier is implemented only in the non-motion shadow runner and remains a
  physical integration gate.

### Transport and robot authority

- Transport v2 atomically publishes one decision per configured robot.
- Targets are versioned, expiring high-level frontier or semantic regions.
- Each robot owns its local planner, collision handling, velocity controller
  and final stop/reject authority.
- WSJ routes a v2 semantic/point target into online BuildMap A*, TinyNav
  control and a guarded Go2 bridge.
- Yunji routes a v2 target into WATER `/api/move`, cancel and feedback. Its
  observed firmware lacks the newer accessible-point API, so the receiver uses
  bounded legacy receding-horizon goals.
- Hub never emits motor velocity commands.

### Evaluation

- The four-scene, five-run-per-scene protocol is documented.
- Episode reports preserve decisions, feedback, path length and failure
  reasons.
- Standard SPL and the source-compatible SPL variant are implemented.
- No valid official trial exists yet, so current SR/SPL are **not available**.

## Physical attempt record

| Episode | Observed outcome | Official metric status |
| --- | --- | --- |
| `official-run01` | Blocked before GOAL by contradictory WSJ IMU thresholds; thresholds were aligned and deployed. | Excluded |
| `official-run01-retry1` | Both local planners navigated and the operator observed motion; lease 14 hit the observation/heartbeat race and triggered dual HOLD. | Excluded |
| `official-run01-retry2` | The actual v2 registry still had the same race; Yunji held after immediate arrival and WSJ had a short translation segment before fail-closed cleanup. | Excluded |
| `official-run01-retry3` | v2 health race did not recur. Yunji immediately arrived; WSJ sent 151 rotation-only commands, operator saw no motion, then local `ODOMETRY_STALE` rejected the leg. | Excluded |

The detailed evidence, hashes and exact command observations are in
[`audit/V2_ROBOT_RECEIVERS_20260723.md`](audit/V2_ROBOT_RECEIVERS_20260723.md).

## Closed root causes

- RealSense/TinyNav IMU callback starvation and invalid-interval recovery.
- Sender/receiver IMU threshold mismatch.
- RGB-D observation health temporarily replacing command-receiver health.
- The same health-source bug existing independently in v1 and v2 registries.
- Missing exact Go2 bridge command evidence.
- Frozen/blank Foxglove panels, ray-fill interpretation and map-session
  contamination across pose discontinuities.
- Cross-view YOLO depth selection using an unrelated foreground surface.
- Yunji D405-era assumptions after the Odin1 hardware replacement.

## Implemented and synchronized, but not yet physically revalidated

After retry3, two WSJ changes were locally tested and synchronized to both
versioned robot deployment roots:

- nonzero Go2 command floors are now `0.15 m/s` linear and `0.30 rad/s`
  angular, while hard maxima remain `0.20 m/s` and `0.50 rad/s`;
- odometry and occupancy conversion use independent callback groups and a
  three-thread router executor; the one-second stale-odometry fail-closed
  threshold remains unchanged.

The code is on both robot computers but robot-side processes were deliberately
not restarted during the final synchronization. These changes will load on
the next controlled stack start and are still physically unverified.

The synchronized 392-entry deployment archive was 2,371,165 bytes with
SHA-256
`e1b9001fb188a3890037f5e33927d25afa44473fb50a6b8c40b61a6e123b1b72`.
Both robots independently observed the same hash before extraction. See
[`audit/DUAL_ROBOT_CODE_SYNC_20260723.md`](audit/DUAL_ROBOT_CODE_SYNC_20260723.md).

## Current safety state

At the final check:

- local Hub was running the debug robot configuration;
- `goal_output_enabled=false` for both robots;
- WSJ live receiver count was zero and Go2 bridge count was zero;
- Yunji live receiver count was zero;
- all previously supplied operator confirmations were consumed.

Never reuse an earlier confirmation after cleanup, restart or a failed
attempt.

## Remaining gates

1. Load the synchronized WSJ router/command-floor changes in debug mode.
2. Verify fresh WSJ odometry, map status, camera registration and pose delta
   without a bridge.
3. Verify Yunji/Odin observation freshness, WATER canceled state and pose
   delta.
4. Run a no-motion full-stack debug round and preserve its session manifest.
5. Place/choose a target outside both arrival radii and verify its semantic
   projection before arming motion.
6. Obtain one fresh operator-present confirmation at the live entry point.
7. Complete one bounded scene with physical arrival and independent operator
   success confirmation.
8. Only then begin four scenes times five official runs and compute SR/SPL.
9. Integrate the continuous multi-round VLM runner with physical execution if
   a scene requires exploration beyond one frozen decision.

## Git and reproducibility state

The final robot synchronization was made from a pre-publication working tree
based on commit `ee8f84b6646cb08fbcb30fab072b9d0437bf485b`. The Git commit
containing this page captures the intended code, tests, configuration, audit
and documentation as the reproducible baseline. The two robot roots have not
yet been redeployed from that Git commit; their exact pre-publication archive
identity is retained above so the distinction remains auditable.

Runtime maps, camera frames, model files, credentials, tokens and robot-local
calibration state remain intentionally outside Git. Their paths and hashes are
recorded in manifests and dated audits.
