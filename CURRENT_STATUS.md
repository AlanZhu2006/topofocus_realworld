# Current project status

Snapshot time: **2026-07-25, after `trial-05-nearwall-fix`, its 0.5 m
operator adjudication, the automatic-arrival fix, a subsequent Yunji power
cycle and the aborted `20260725-lab18-repeat2` calibration**

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

The earlier episode `scene01-chair-run01-fastfix` remains a collision and is
excluded. The newer session `20260725-lab17-nearwall-fix`, episode
`trial-05-nearwall-fix`, completed three source-scheduled rounds with the
post-collision route guard serializing physical authority. Yunji detected the
chair and approached the semantic point; WSJ was intentionally HOLD whenever
its route conflicted.

The pre-incident controller allowed both robots to be active because source
coordination removes Agent 0's selected frontier before Agent 1 chooses; that
prevents duplicate frontier allocation but does not reserve collision-free
multi-robot routes. A post-incident real-world execution guard now preserves
the original two-agent VLM candidate while serializing physical authority when
the buffered shared-frame start-to-target segments come within `0.9 m`.
Replay of the observed starts and targets reports a minimum predicted
separation of `0.0 m` and reduces authority to one robot. This fix is committed
and locally tested. It is included in the newer calibrated/debugged
`20260725-lab12` deployment and has now been observed in motion over all three
rounds of `trial-05-nearwall-fix`.

The old receiver stopped the episode with `LOCAL_PLANNER_PATH_STALE` at
`0.321133 m` from its semantic approach point because that run still used a
`0.15 m` arrival radius. The onsite operator explicitly adjudicated it
successful under the subsequently declared `0.5 m` physical radius. The
separate `operator_adjudicated_0p5m_demo` progress track therefore contains
one sample with `SR=1.0` and exact source-compatible `SPL=0.864048`; standard
SPL remains unavailable because no shortest path was surveyed before the run.
The original automatic failure record is unchanged.

Commit `b1762d15e1059281056ef1e6b4e472e9d25258e1` now passes the `0.5 m`
semantic radius explicitly to both physical launchers, so future equivalent
terminals can report automatic `ARRIVED`.

## Latest validated physical session

The latest session that passed strict no-motion debug is:

| Item | Observed value |
| --- | --- |
| Session | `20260725-lab17-nearwall-fix` |
| Session Git commit | `cbe6d9b02f50ded3607037c46abb0230d1639ebc` |
| Calibration ID | `shared-board-odin1-20260725-lab15-reanchor1-v1` |
| Calibration kind | validated stationary re-anchor of board calibration |
| Calibration SHA-256 | `825d8ef714cbf06f24b783f476b21becd5ccf0af28dbb78075ba97e45ac32d0a` |
| WSJ transform | `wsj-tinynav-depth-20260725-lab15-raw-v1` |
| Yunji transform | `yunji-odin1-reanchor-20260725-lab15-reanchor1-v1` |
| WSJ map boundary | sequence `26943` |
| Yunji map boundary | sequence `238267` |
| Debug manifest SHA-256 | `176650ecac3dd802188fbc0a010ed98e1f89d1d23b8a72633754a613354b07ac` |
| Live episode | `trial-05-nearwall-fix`; operator-adjudicated 0.5 m success |
| Hub API | `http://127.0.0.1:8188` |
| GLM endpoint | `http://127.0.0.1:31511/v1` |
| Foxglove | `ws://10.208.2.249:8765` |

After this run, Yunji was fully power-cycled. Read-only inspection observed a
new host boot at `2026-07-25 09:26:54 +08:00` and the Odin driver active from
`09:27:02`. A later `20260725-lab18-repeat2` calibration attempt was cancelled
before a board fit because the Go2 was moved to its charging position. There
is therefore no lab18 calibration to reuse, and the old lab17 shared placement
is no longer valid for the next physical run.

The lab18 release check also exposed stale cross-robot launcher copies in the
remote release roots. Both cross-launchers were synchronized atomically and a
subsequent complete byte-identical verification passed on WSJ and Yunji before
the calibration was cancelled. Exact hashes and the incomplete-attempt
boundary are recorded in
[`audit/LAB18_CALIBRATION_ABORT_20260725.md`](audit/LAB18_CALIBRATION_ABORT_20260725.md).

## Latest near-chair episode

The exact evidence and arithmetic for `trial-05-nearwall-fix` are in
[`audit/NEAR_CHAIR_SUCCESS_20260725.md`](audit/NEAR_CHAIR_SUCCESS_20260725.md)
and the machine-readable
[`manifests/realworld_experiment_progress.json`](manifests/realworld_experiment_progress.json).
The key facts are:

- Yunji path length: `4.048842201890332 m`;
- start-to-stop displacement: `3.498394160748945 m`;
- final distance to the selected semantic point: `0.321133366707683 m`;
- third-view terminal frame independently shows Yunji beside the chair;
- posthoc 0.5 m track: `SR=1`, source-compatible `SPL=0.864048038008398`;
- pre-surveyed standard SPL: unavailable.

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
The latest physical run verified that crossing/near-crossing allocations
produce one GOAL plus one HOLD, but this is still serialization rather than
full multi-robot trajectory planning.

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
- Repeated live startup now distinguishes chassis power from tracking reset by
  proving that the live WSJ perception/SLAM and Yunji Odin processes predate strict
  debug. Matching warm TinyNav/map/Foxglove cores are reused; both robot
  launchers run concurrently. Cleanup removes both chassis command paths but
  deliberately keeps the read-only observation/map core warm.
- The route-conflict guard is an additional real-world execution adapter, not
  a replacement for robot-local collision avoidance.

### Evaluation

- Four scenes × five trials are documented.
- Standard SPL and source-compatible SPL are implemented.
- Episode reports preserve robot-local start/stop poses and accumulated path.
- The operator-adjudicated 0.5 m progress track currently has one success:
  `SR=1`, source-compatible `SPL=0.864048`.
- The pre-surveyed standard track still requires surveyed shortest paths,
  automatic terminal/goal-region membership and independent target evidence.
- Collision, operator stop, controller error, incomplete terminal evidence
  and debug/shadow runs remain excluded from that standard track.

## Today's implementation delta

The runtime series through session `20260725-lab17-nearwall-fix` is authored
as `AlanZhu2006 <yz11502@nyu.edu>`. In addition to the startup acceleration,
the current code includes the physically exercised route guard and the
explicit `0.5 m` semantic-arrival setting for future physical runs.

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
8. post-incident dual-route serialization;
9. same-session tracking continuity and warm repeated-trial orchestration.

No file under immutable `source/` or `dependencies/` was changed by this
series.

## Current safety state

At closeout:

- `trial-05-nearwall-fix` ended with both robots HOLD and zero velocity
  confirmed in the episode report;
- no previous operator motion confirmation remains valid;
- Yunji was subsequently power-cycled; its host and Odin driver are active,
  but its tracking epoch is new;
- the Go2 was moved to charge, so the old shared physical placement is
  invalid regardless of whether Yunji stayed fixed;
- `20260725-lab18-repeat2` was cancelled before any valid board fit, holdout,
  fresh-map debug or live episode;
- the WSJ connection closed during the charging transition and is not assumed
  available;
- a local health check after cancellation reports
  `goal_output_enabled=false` for `robot-0` and `robot-1`, and no calibration
  or live runner process remains;
- old collision session `20260725-lab05-yunjireboot4` remains immutable
  evidence;
- the fail-closed Hub process may remain available for observation, but no
  physical command path is authorized.

## Next valid workflow

1. After charging, return both robots to the intended experiment start poses
   and keep them stationary.
2. Restore the existing WSJ SSH/tmux connection and verify both tracking
   streams without enabling either chassis command path.
3. Run the normal two-position board calibration with a fresh session ID.
   Lab18 cannot be reused, and a stationary re-anchor is insufficient because
   the Go2 was physically moved.
4. Let the calibration command complete fresh maps/Foxglove and strict
   no-motion debug. Verify that both physical launchers report
   `semantic_arrival_radius_m=0.5`.
5. Obtain a new `OPERATOR_PRESENT_AND_ROBOTS_CLEAR` authorization and run one
   supervised episode.
6. Preserve automatic terminal evidence and record the path/metric result
   immediately. Survey `L` before the run if standard SPL is required.

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
