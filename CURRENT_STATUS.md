# Current project status

Snapshot time: **2026-07-25, after formal experiment 05 preparation stopped
inside strict no-motion debug and both robots were powered down for charging**

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

The earlier episode `scene01-chair-run01-fastfix` remains a route-coordination
engineering collision and is excluded. Session
`20260725-lab17-nearwall-fix`, episode `trial-05-nearwall-fix`, then completed
three source-scheduled rounds with the post-collision route guard serializing
physical authority. Yunji reached `0.321133 m` from its selected chair
semantic point and passes the declared `0.5 m` physical protocol.

The earlier automatic physical success is
`20260725-lab19-scene01-8ca1d52-yunjireboot1-r5 / trial-r5-01`. WSJ emitted
`LOCAL_PLANNER_ARRIVED`, stopped `0.406692832069 m` from the chair semantic
goal and triggered the automatic terminal-evidence bundle. Its
source-compatible SPL is `0.628398923`.

The latest completed physical episode is
`20260725-lab21-wallfix-imudebounce-3a2d953 /
trial-wallfix-imudebounce-r1`, which the operator designated **formal
experiment 04: success**. Yunji emitted `LOCAL_PLANNER_ARRIVED`, stopped with
zero velocity, and triggered a complete post-arrival candidate bundle whose
time-aligned terminal RGB visibly contains the white chair. Its actual path
was `3.2102223806424663 m`, start-to-arrival displacement was
`3.070130248072939 m`, per-episode `SR=1`, and source-compatible
`SPL=0.956360614325575`.

The operator later reported an approximate shortest-path distance of
`L≈3.25 m` for formal experiment 04. Substitution into the standard formula
gives `1.0`, and the operator explicitly confirmed that this value is an
independently measured shortest feasible path. It is therefore the official
standard SPL for this episode. The approximate `L` exceeds the
odometry-derived executed path by `0.0397776193575337 m`; `max(L,P)` normally
caps the result at `1.0`.

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

The later `trial-reanchor1-r2` exposed a separate robot-local frontier
feasibility problem. Yunji's selected frontier had only `0.10 m` clearance
and no `0.34 m`-clear approach cell inside its unchanged `0.50 m` arrival
disk. TinyNav eventually emitted a backward lookahead segment; its
forward-only controller rotated toward that segment and then timed out with
`LOCAL_PLANNER_NO_PROGRESS`. The deployment now filters such frontier
landings before publication, immediately rejects a fresh reverse-required
trajectory, and isolates that frontier failure so the other robot can
continue. Exact evidence and replay are in
[`audit/YUNJI_WALL_TURN_REJECTION_20260725.md`](audit/YUNJI_WALL_TURN_REJECTION_20260725.md).

The old receiver stopped `trial-05-nearwall-fix` with
`LOCAL_PLANNER_PATH_STALE` because that run still used a `0.15 m` arrival
radius. The project now uses a `0.5 m` real-world success radius and retains
that legacy log unchanged. The separate `physical_0p5m_protocol` progress
track now contains three evidence-bound normal successes with `SR=3/3=1.0`
and mean source-compatible `SPL=0.816269191777991`. The operator subsequently
identified `trial-05-nearwall-fix` as the manually annotated formal experiment
03 and the newest episode as formal experiment 04. Formal experiment 04 is
the one sample with an operator-confirmed independently measured shortest
feasible path: standard `SPL=1.0`. The other two runs still have no standard
SPL.

Commit `b1762d15e1059281056ef1e6b4e472e9d25258e1` now passes the `0.5 m`
semantic radius explicitly to both physical launchers, so future equivalent
terminals can report automatic `ARRIVED`.

After r5, session `20260725-lab19-scene01-8ca1d52-yunjireboot1-r6` passed
automated strict no-motion debug. Before live, the operator noticed that the
WSJ fused pose was close to/inside a wall. Frozen-map analysis placed WSJ
`3.131345 m` from its own map's nearest occupied-cell boundary but only
`0.224118 m` from Yunji's. Its relative two-robot geometry differed from r5
by `0.602050 m` separation and about `13.8°` heading. r6 was therefore
rejected as a calibration/fusion preflight failure: no target, command,
movement or episode, and no SR/SPL entry.

Formal experiment 05 preparation then made two fail-closed attempts at strict
no-motion debug. The first used a stale Yunji replay boundary and timed out
before semantic snapshots became ready. The current-boundary retry found live
raw D435i camera/IMU streams but no fresh calibrated WSJ camera-info or visual
odometry output. It stopped before live because restarting perception would
change the tracking epoch and invalidate direct reuse of the old shared-frame
transform. No GOAL, episode, robot command or movement was produced. The
operator then powered both robots down for charging and deferred calibration.
Exact evidence is in
[`audit/FORMAL_EXPERIMENT_05_PREFLIGHT_ABORT_20260725.md`](audit/FORMAL_EXPERIMENT_05_PREFLIGHT_ABORT_20260725.md).

## Latest completed physical session

The latest session that completed live is:

| Item | Observed value |
| --- | --- |
| Session | `20260725-lab21-wallfix-imudebounce-3a2d953` |
| Session Git commit | `3a2d953e7ab5c1b7891829b2101dc5bb52126e77` |
| Calibration ID | `shared-board-odin1-20260725-lab21-wallfix-dualreanchor2-v1` |
| Calibration kind | validated dual stationary re-anchor of board calibration |
| Calibration SHA-256 | `c688becd72e9f320014812c57e3aae15798ad4e731978a063f5bdf2137bae8ed` |
| WSJ transform | `wsj-tinynav-depth-20260725-lab21-wallfix-wsjreanchor4-v1` |
| Yunji transform | `yunji-odin1-board-20260725-lab21-wallfix-yunjireanchor1-v1` |
| WSJ map boundary | sequence `28988` |
| Yunji map boundary | sequence `254465` |
| Debug manifest SHA-256 | `018ab08c62db3cffb58758d6dafe159880c8ff759f7b683e2514b8a1febcb9c3` |
| Live episode | `trial-wallfix-imudebounce-r1`; automatic Yunji `ARRIVED`, operator-designated formal experiment 04 success |
| Hub API | `http://127.0.0.1:8188` |
| GLM endpoint | `http://127.0.0.1:31511/v1` |
| Foxglove | `ws://10.208.2.249:8765` |

The earlier r5 result is preserved independently of the later r6
calibration/fusion rejection and the newer lab21 success. The r6 debug
manifest SHA-256 is
`e55447da56308c35c2a8fbb165fe983c071f31fd82e9f1bbe813de3aa78d077e`;
passing that automated gate did not override the contradictory fused
visualization. Exact diagnosis and frozen-input hashes are in
[`audit/R6_FUSION_PREFLIGHT_ABORT_20260725.md`](audit/R6_FUSION_PREFLIGHT_ABORT_20260725.md).

The lab18 release check also exposed stale cross-robot launcher copies in the
remote release roots. Both cross-launchers were synchronized atomically and a
subsequent complete byte-identical verification passed on WSJ and Yunji before
the calibration was cancelled. Exact hashes and the incomplete-attempt
boundary are recorded in
[`audit/LAB18_CALIBRATION_ABORT_20260725.md`](audit/LAB18_CALIBRATION_ABORT_20260725.md).

## Scene 01 chair successes

The exact evidence and arithmetic are in
[`audit/SCENE01_CHAIR_SUCCESS_20260725.md`](audit/SCENE01_CHAIR_SUCCESS_20260725.md),
[`audit/SCENE01_CHAIR_SUCCESS_R5_20260725.md`](audit/SCENE01_CHAIR_SUCCESS_R5_20260725.md),
[`audit/SCENE01_CHAIR_FORMAL_EXPERIMENT_04_SUCCESS_20260725.md`](audit/SCENE01_CHAIR_FORMAL_EXPERIMENT_04_SUCCESS_20260725.md)
and the machine-readable
[`manifests/realworld_experiment_progress.json`](manifests/realworld_experiment_progress.json).
The three metric samples are:

| Episode | Arriving robot | Goal distance | Path | SR | Source-compatible SPL |
| --- | --- | ---: | ---: | ---: | ---: |
| Formal 03: `trial-05-nearwall-fix` | Yunji | `0.321133 m` | `4.048842 m` | `1` | `0.864048038008398` |
| `trial-r5-01` | WSJ | `0.406693 m` | `3.850792 m` | `1` | `0.628398923` |
| Formal 04: `trial-wallfix-imudebounce-r1` | Yunji | automatic semantic `ARRIVED`; scalar goal distance unavailable | `3.210222 m` | `1` | `0.956360614325575` |
| **Current mean** | — | — | — | **`3/3=1.0`** | **`0.816269191777991`** |

Formal experiment 04 has independently measured `L≈3.25 m` and official
standard `SPL=1.0`. The standard track therefore has one successful sample:
`SR=1/1=1.0`, mean standard `SPL=1.0`. The other two physical samples do not
have independently measured shortest paths.

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
- A second execution adapter now requires a footprint-clear known-free cell
  inside each frontier arrival disk. A robot-local reverse/no-progress
  frontier rejection removes only that robot's authority and requests a fresh
  source round; semantic, transform, localization and e-stop failures remain
  coordinated fail-closed conditions.

### Evaluation

- Four scenes × five trials are documented.
- Standard SPL and source-compatible SPL are implemented.
- Episode reports preserve robot-local start/stop poses and accumulated path.
- The `physical_0p5m_protocol` track currently has three evidence-bound normal
  successes: `SR=3/3=1.0`, mean source-compatible
  `SPL=0.816269191777991`.
- The standard track has one sample: formal experiment 04 with independently
  measured approximate `L≈3.25 m`, automatic semantic arrival, independent
  terminal target evidence, `SR=1.0` and standard `SPL=1.0`.
- Failure attribution is now explicit. The two user-labelled approach-failure
  videos remain unclassified because they lack exact runtime bindings. The
  collision is a route-coordination engineering failure; r6 is a
  calibration/fusion preflight failure; formal experiment 05 preparation is a
  WSJ tracking-output preflight failure. None is a VLM decision failure.
- A VLM decision failure requires healthy sensing/calibration/mapping/
  transport/control, independently established target presence, faithful
  target execution and a completed declared budget. Verified YOLO/SegFormer
  errors are recorded separately as perception failures.
- Collision, operator stop, controller error, incomplete terminal evidence,
  preflight aborts and debug/shadow runs remain outside the current SR/SPL
  track but remain in the engineering reliability record.

## Today's implementation delta

The runtime series through session
`20260725-lab21-wallfix-imudebounce-3a2d953` is authored
as `AlanZhu2006 <yz11502@nyu.edu>`. In addition to the startup acceleration,
the current code includes the physically exercised route guard and the
explicit `0.5 m` semantic-arrival setting, the safe remaining-frontier
fallback from `05447b9`, and the bounded WSJ IMU-skip debounce from
`3a2d953`. The latest run exercised the latter two and ended in automatic
Yunji `ARRIVED`.

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

- `trial-wallfix-imudebounce-r1` ended with automatic Yunji semantic arrival,
  zero velocity and both robots held;
- both formal experiment 05 preparations stopped before live, GOAL, episode,
  robot command or movement;
- before shutdown Hub health showed `goal_output_enabled=false` for both
  robots;
- the local Hub, both formal05 map sessions and formal05 Foxglove relay were
  stopped; the read-only GLM service remains running;
- both robots are powered down for charging and calibration is explicitly
  deferred;
- no previous operator motion confirmation remains valid;
- WSJ calibrated perception/tracking output is frozen and must be restarted
  before a future session; direct reuse of the lab21 WSJ transform is not
  authorized;
- old collision session `20260725-lab05-yunjireboot4` remains immutable
  evidence;
- no physical command path is authorized.

## Next valid workflow

1. Wait until charging is complete; do not reuse either formal05 preparation
   session.
2. Start a new session and restore observation paths without motion. Restart
   the frozen WSJ perception/tracking process.
3. Complete a fresh two-position board calibration before live. A different
   path is valid only if a separately reviewed continuity/re-anchor procedure
   independently validates the new WSJ tracking epoch; calibration remains
   deferred until the operator chooses to resume.
4. Check each robot marker against both its own map and the other robot's map;
   reject any fused-wall conflict.
5. Complete strict no-motion debug and verify
   `semantic_arrival_radius_m=0.5`.
6. Obtain a new `OPERATOR_PRESENT_AND_ROBOTS_CLEAR` authorization and run one
   supervised episode.
7. Preserve automatic terminal evidence and classify any non-success using
   [`audit/FAILURE_ATTRIBUTION_PROTOCOL_20260725.md`](audit/FAILURE_ATTRIBUTION_PROTOCOL_20260725.md).
   Survey `L` before the run if standard SPL is required.

## Git and provenance

- Repository: `git@github.com:AlanZhu2006/topofocus_realworld.git`
- Default branch: `main`
- Route-conflict implementation: `b79879bfc96805aa7e7b63cf3a8ebbfe59679730`
- Runtime maps, observations, tokens, calibration state and full episode
  directories stay outside Git.
- The 12 archived original Scene 01 videos are committed through Git LFS under
  `media/video/`; the largest is 80,244,552 bytes. Three additional
  workspace-only media candidates remain unbound and uncommitted. Web-playable
  H.264 derivatives, posters, hashes and classifications for the archived set
  are committed under `media/demo/`.
- Physical/runtime facts are labelled observed; algorithm outputs are labelled
  source-derived; causal claims without evidence remain unverified.
