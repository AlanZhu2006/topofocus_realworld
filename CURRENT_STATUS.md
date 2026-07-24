# Current project status

Snapshot time: **2026-07-25, after the first concurrent dual-robot live
motion and the operator-reported collision**

This is the canonical current-state document. Dated files under `audit/` are
append-only evidence records; they do not supersede this page.

## Executive outcome

The real two-robot chain has now physically exercised:

```text
RGB-D/pose
  -> online TinyNav maps
  -> central semantic/fused maps
  -> source-derived Perception/Judgment/Decision VLM
  -> atomic expiring v2 high-level targets
  -> robot-local TinyNav planning/control
  -> guarded Go2/WATER output
  -> robot feedback and lease renewal
  -> fail-closed HOLD
```

Both robots moved during episode `scene01-chair-run01-fastfix`. This is the
first observed concurrent physical execution of the current end-to-end
architecture, but it is **not a success**: their assigned frontiers were
different while the corresponding start-to-target routes crossed. The robots
made physical contact, the operator stopped the run, and the episode is
excluded from SR/SPL.

The pre-incident controller allowed both robots to be active because source
coordination removes Agent 0's selected frontier before Agent 1 chooses; that
prevents duplicate frontier allocation but does not reserve collision-free
multi-robot routes. A post-incident real-world execution guard now preserves
the original two-agent VLM candidate while serializing physical authority when
the buffered shared-frame start-to-target segments come within `0.9 m`.
Replay of the observed starts and targets reports a minimum predicted
separation of `0.0 m` and reduces authority to one robot. This fix is committed
and locally tested, but has not yet been deployed into a new calibrated
physical session or verified in motion.

No official scene has completed successfully. Current SR and SPL therefore
remain **not available**, rather than zero-valued experimental estimates.

## Latest validated physical session

The most recent session that passed strict no-motion debug before the
collision was:

| Item | Observed value |
| --- | --- |
| Session | `20260725-lab05-yunjireboot4` |
| Session Git commit | `cdcd7e70560f8bd782d83b5176bda6f5fca36780` |
| Calibration ID | `shared-board-odin1-20260725-lab05-yunjireboot1-v1` |
| Calibration kind | validated stationary reanchor of moved-board calibration |
| WSJ transform | `wsj-tinynav-depth-20260725-lab05-raw-v1` |
| Yunji transform | `yunji-odin1-stationary-reanchor-20260725-lab05-yunjireboot1-v1` |
| WSJ map boundary | sequence `24082` |
| Yunji map boundary | sequence `222279` |
| Debug manifest | strict no-motion debug passed on the same commit |
| Live episode | `scene01-chair-run01-fastfix` |
| Hub API | `http://127.0.0.1:8188` |
| GLM endpoint | `http://127.0.0.1:31511/v1` |
| Foxglove | `ws://10.208.2.249:8765` |

That session must **not** be reused for another live run:

- repository HEAD now includes the route-conflict fix at
  `b79879bfc96805aa7e7b63cf3a8ebbfe59679730`;
- Yunji's Odin host and tracking driver restarted after the incident;
- the Go2 chassis was subsequently powered down and is crouched;
- the next standing pose and tracking epoch therefore need a new shared-frame
  calibration and fresh session binding.

The old session remains immutable runtime evidence. It is not edited or
migrated to claim compatibility with the new commit.

## Observed physical episode

Episode `scene01-chair-run01-fastfix` used session
`20260725-lab05-yunjireboot4` and goal category `chair`.

Observed/source-derived facts:

- the source-derived VLM assigned WSJ frontier `D` at
  `(-1.2897, 4.9773)` and Yunji frontier `B` at
  `(2.1103, 4.5773)` in `shared_world`;
- both decisions listed `robot-0` and `robot-1` as active;
- WSJ accumulated approximately `1.4340 m` and Yunji approximately
  `1.5976 m` of robot-reported path length;
- the uploaded third-view video directly shows the platforms converge and
  make physical contact;
- the paired Dashboard recording shows the two short trajectories converging,
  followed by close-range occlusion of the WSJ camera;
- the operator reported the collision and confirmed
  `ROBOTS_STOPPED_AFTER_COLLISION`;
- the Hub changed both decisions to HOLD after feedback disappeared;
- Yunji/Odin host connectivity also disappeared near the incident, but timing
  alone does not prove that the collision caused the network loss.

The controller-generated outcome was `controller_error_TimeoutError`, while
the operator incident record is the authoritative physical termination:
`collision`. The run has no semantic arrival, no independently verified target
evidence, no completed terminal bundle and no metric eligibility.

Detailed hashes, video provenance and the distinction between observation and
inference are recorded in
[`audit/DUAL_ROBOT_COLLISION_20260725.md`](audit/DUAL_ROBOT_COLLISION_20260725.md).

## Corrective execution guard

Commit `b79879bfc96805aa7e7b63cf3a8ebbfe59679730` adds:

- frozen `shared_world` base-pose extraction from the accepted map snapshots;
- pairwise buffered straight-segment separation checks before any GOAL batch;
- deterministic single-robot execution when routes conflict;
- one-robot execution when only one valid shared pose exists;
- dual HOLD when neither shared pose is available;
- preservation of the unmodified VLM output as
  `vlm_candidate_batch.json`;
- preservation of the applied decision as `initial_batch.json`;
- an explicit `route_conflict_guard.json` provenance report.

The guard changes physical concurrency authority only. It does not change the
source VLM's Perception, Judgment/FN, Decision, directional memory or
sequential frontier allocation.

Its current limitation is explicit: straight start-to-target segment
separation is conservative and does not certify robot-local planner detours.
The next physical run must first verify that the observed crossing case
produces one GOAL plus one HOLD before allowing movement.

## Subsystem status

### Sensors, mapping and Foxglove

- **WSJ:** D435i, repaired TinyNav perception/IMU path, online BuildMap,
  TinyNav local planner/controller and guarded Go2 bridge.
- **Yunji:** Odin1 `O1-P070100205` RGB/depth/cloud/odometry, online TinyNav
  occupancy/A*/local planner/controller and guarded WATER
  `/api/joy_control` bridge.
- The formal Yunji path does not use a WATER saved map,
  `accessible_point_query`, `make_plan` or `/api/move`.
- Foxglove publishes both cameras plus WSJ, Yunji and fused semantic overview
  images with occupancy, frontiers, base poses, headings, trajectories,
  pixel masks and labels.
- The collision Dashboard contains a projected `chair` region. It is real
  model inference and map projection, not independent semantic ground truth.

### Source-derived VLM behavior

- Perception VLM, Judgment/FN, Decision VLM and the executable source gate are
  preserved.
- Directional history is shared across agents.
- Agent 0 chooses first and its frontier is removed before Agent 1 chooses.
- Positive target-semantic evidence can replace a frontier with the largest
  connected target component.
- The physical runner preserves the source decision schedule
  `0,24,49,...,499`, with acknowledged HOLD between rounds.
- Frontier arrival means replan; only a semantic-region arrival creates an
  automatic terminal candidate.

### Transport and robot authority

- The Hub publishes only versioned, expiring high-level targets.
- Each robot owns local planning, velocity control and final stop/rejection.
- Leases renew only while command feedback remains fresh.
- Cleanup restores Hub `GOAL=false`, local HOLD/zero behavior and removes live
  bridge authority.
- The route-conflict guard is an additional real-world execution adapter, not
  a replacement for robot-local collision avoidance.

### Evaluation

- Four scenes × five trials are documented.
- Standard SPL and source-compatible SPL are implemented.
- Episode reports preserve robot-local start/stop poses and accumulated path.
- Official success still requires surveyed shortest paths, goal-region
  membership and independent terminal target evidence.
- Collision, operator stop, controller error, incomplete terminal evidence
  and debug/shadow runs are excluded.

## Today's implementation delta

Relative to the previous remote `origin/main`
`89ca34bd9d621bc9a2b46e1988a8490eb9e220e0`, the runtime series through
`b79879b` contains 19 commits across 31 files, with 4,091 insertions and 317
deletions. All are authored as `AlanZhu2006 <yz11502@nyu.edu>`.

The work groups into:

1. live input ordering, measured local start projection and cropped-map start
   admission;
2. persistent WSJ goal-router loading and Yunji depth preservation;
3. WSJ stationary startup, ROS graph discovery, camera/SLAM freshness and
   BuildMap readiness;
4. measured Odin odometry tolerance and bounded robot health diagnostics;
5. source-derived multi-round live episodes with terminal evidence handling;
6. native WSJ calibration geometry, restored RGB preview and stationary
   tracking reanchor;
7. Yunji startup compute-starvation prevention;
8. post-incident dual-route serialization.

No file under immutable `source/` or `dependencies/` was changed by this
series.

## Current safety state

At closeout:

- the operator confirmed both robots stopped after the collision;
- Hub health reports `goal_output_enabled=false` for both robots;
- no previous operator motion confirmation remains valid;
- the Go2 chassis is powered down/crouched;
- Yunji/Odin restarted, its driver is active, and both existing SSH/tmux
  tunnels were restored;
- old session `20260725-lab05-yunjireboot4` is retained only as evidence;
- no live command should be run until a new standing calibration/session
  succeeds.

## Next valid workflow

1. Power and stand the Go2 in its intended operating posture; keep both robots
   fixed and place the complete 7 × 10 board in both camera views.
2. Run the new-session calibration block in [`command.txt`](command.txt).
   It deploys the committed code, creates fresh maps/Foxglove and performs
   strict no-motion debug.
3. Confirm `DEBUG_FULLSTACK_READY`, both data-plane verification reports and
   the new route-guard code/session identity.
4. In a bounded live preflight, inspect `route_conflict_guard.json`. A crossing
   allocation must expose only one active robot.
5. Obtain one new onsite motion confirmation and run one supervised episode.
6. Record independent terminal evidence and surveyed shortest paths only if
   the episode reaches a valid terminal candidate.

The calibration command already includes the required debug. A second manual
debug run is optional unless the host, code, tracking epoch or visualization
state changes afterward.

## Git and provenance

- Repository: `git@github.com:AlanZhu2006/topofocus_realworld.git`
- Default branch: `main`
- Route-conflict implementation: `b79879bfc96805aa7e7b63cf3a8ebbfe59679730`
- Runtime maps, observations, tokens, calibration state and full episode
  directories stay outside Git.
- Original high-resolution user videos stay under ignored `media/video/`.
- Size-bounded H.264 derivatives, posters, hashes and classifications are
  committed under `media/demo/`.
- Physical/runtime facts are labelled observed; algorithm outputs are labelled
  source-derived; causal claims without evidence remain unverified.
