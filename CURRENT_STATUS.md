# Current project status

Snapshot time: **2026-07-24 04:09 CST**

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

Since that attempt, the dated launcher has been replaced by a persistent
physical-session workflow. Its schemas, launch chain and regression tests pass
locally. The implementation commit has also been byte-verified in both robot
release roots without restarting robot-side processes, but the new workflow
has not yet completed a physical debug or live run. There is intentionally no
`hub/runtime/sessions/current.json` until a new onsite board calibration
succeeds.

## Last observed physical identity

These identifiers describe the last July 24 predecessor session. They remain
useful evidence, but are **not** promoted to a reusable persistent session:

| Item | Last observed value |
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
starting placement did not move. The legacy v3 artifact predates the new
quantitative `board_moved_independently` holdout field, so it cannot seed the
new strict session by assertion. The next onsite run must use
`calibrate_realworld_session.sh` once and create a new session ID.

## What is implemented and verified

### Observation, maps and visualization

- WSJ uses the D435i/TinyNav observation path with the deployed IMU scheduling
  repair, registered RGB to TinyNav depth, measured body-camera calibration
  and independent command-health heartbeat.
- Yunji uses Odin1 `O1-P070100205`, not the retired RealSense lane. The adapter
  consumes native RGB, SLAM cloud and odometry and preserves the factory
  calibration.
- Both last observed maps use gravity/ground gates, free-space ray fill, reversible
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
- The persistent physical one-click runner uses one frozen VLM round followed by
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
- Navigation reports now preserve each robot's local start/stop pose, path and
  planner STOP evidence. `record_realworld_trial.py` binds independent
  terminal evidence and surveyed shortest paths to each 4 × 5 trial.
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

## Persistent operator workflow

The new authoritative sequence is:

```text
one board-calibration command
  -> persisted, hash-bound session
  -> strict no-motion debug
  -> one freshly authorized live episode
  -> immediate SR/SPL evidence recording
```

It removes all operator-supplied map, calibration and tmux identifiers from
normal startup. Debug/live resolve one immutable session contract, checksum
both remote code trees, start a clean Hub epoch, reject stale/torn inputs and
replace a mismatched managed Foxglove relay. Each map directory separately
binds its code/sequence/transform/calibration/backend contract; a missing or
blocked map can be reconstructed from that exact boundary before strict input
freezing. Live validates a HOLD-only VLM result before either motion-capable
receiver is armed. Cleanup always restores mapping-only Hub policy and
robot-side stop/reject authority.

The implementation and complete Hub regression suite are locally observed.
Physical execution of this new sequence is still unverified. See
[`hub/docs/ONECLICK_SESSION_WORKFLOW.md`](hub/docs/ONECLICK_SESSION_WORKFLOW.md)
and
[`audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md`](audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md).

The episode controller also preserves a robot's observed `ARRIVED` event
across the subsequent coordination HOLD, preventing its start/stop/path seed
from being overwritten before trial recording.

## Latest robot code availability

After retry3, two WSJ changes were locally tested and synchronized to both
versioned robot deployment roots:

- nonzero Go2 command floors are now `0.15 m/s` linear and `0.30 rad/s`
  angular, while hard maxima remain `0.20 m/s` and `0.50 rad/s`;
- odometry and occupancy conversion use independent callback groups and a
  three-thread router executor; the one-second stale-odometry fail-closed
  threshold remains unchanged.

Those changes are included in the newer persistent-session implementation
commit
`90dd8fe43dad16515017fe4fd9bd017e02277bf6`. A code-only archive from that
exact Git object was synchronized to:

- WSJ `/home/nvidia/topofocus_buildmap_v2_20260723`;
- Yunji `/home/nyu/topofocus_buildmap_v2_20260723`.

The latest archive contained 326 entries, was 2,133,790 bytes and had
SHA-256
`4298f048591ca8b6a7cfa9d9aa3fe3ba34058965329f32bfba827af72f2a097f`.
Both robots matched that archive before extraction and then independently
matched all 175 tracked files under `hub/src/focus_hub` and
`hub/robot_overlay`. Both parsed the archive's 196 Python files and passed
`bash -n` for all 39 shell files using Python 3.10.12.

Robot-side processes were deliberately not restarted: the observed final
state was WSJ receiver 0 / Go2 bridge 0 and Yunji receiver 0 / live service
inactive / debug service inactive. The files will first be loaded by the next
controlled stack start and remain physically unverified.

The older retry3 archive contained 392 entries, was 2,371,165 bytes and had
SHA-256
`e1b9001fb188a3890037f5e33927d25afa44473fb50a6b8c40b61a6e123b1b72`.
It remains historical evidence. See
[`audit/DUAL_ROBOT_CODE_SYNC_20260723.md`](audit/DUAL_ROBOT_CODE_SYNC_20260723.md).

The newer transfer and independently observed checks are recorded in
[`audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md`](audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md).
Archive availability must not be described as process loading or physical
verification.

## Current safety state

At the last physical check:

- local Hub was recreated from the current checkout on `127.0.0.1:8188`
  with the debug robot configuration after the code-only transfer;
- `goal_output_enabled=false` for both robots;
- WSJ live receiver count was zero and Go2 bridge count was zero;
- Yunji live receiver count was zero and its live/debug services were
  inactive;
- temporary transfer files and the loopback HTTP tmux session were removed;
- all previously supplied operator confirmations were consumed.

Never reuse an earlier confirmation after cleanup, restart or a failed
attempt.

## Remaining gates

1. Onsite, run the one-command board calibration with a new session ID. It
   also creates fresh maps and runs strict no-motion debug.
2. Confirm `DEBUG_FULLSTACK_READY`, fresh WSJ/Yunji health, correct Foxglove
   views and a target outside both arrival radii.
3. Obtain one fresh operator-present confirmation and complete one bounded
   live episode.
4. Record surveyed shortest paths, goal-region judgments and independent
   terminal evidence immediately; only a complete record is metric-eligible.
5. Then collect four scenes × five official trials.
6. Integrate physical multi-round VLM re-planning if a chosen scene cannot be
   completed by the current one-frozen-decision leg.

## Git and reproducibility state

The robot-synchronized persistent-session implementation is commit
`90dd8fe43dad16515017fe4fd9bd017e02277bf6`. The two robot release roots
contain its exact critical runtime bytes. Repository `main` now also contains
the synchronization record and direct `command.txt` operator wrapper through
`eefa8f801682f27bf188f6496b30dbf760e28a37`; GitHub PR #1 is merged.
Those later wrapper/documentation changes do not alter the remotely checked
`hub/src/focus_hub` or `hub/robot_overlay` trees.

Runtime maps, camera frames, model files, credentials, tokens and robot-local
calibration state remain intentionally outside Git. Their paths and hashes are
recorded in manifests and dated audits.
