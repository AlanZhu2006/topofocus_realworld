# Persistent one-click physical workflow

This is the canonical operator path for a new physical placement and for
repeated trials in one placement. It replaces the dated, hard-coded July 24
launcher. The workflow has three explicit states:

`board calibration -> strict no-motion debug -> one bounded multi-round live episode`

The first two states are non-motion. The live state still requires one fresh,
onsite confirmation because the Hub only publishes expiring high-level
targets; TinyNav/WATER and the robots retain final stop/reject authority.
After the first strict debug, repeated live episodes normally reuse the same
tracking, TinyNav, map and Foxglove processes instead of replaying the first
two states.

For normal onsite use, the repository-root `command.txt` stores copy-ready
versions of the unwrapped commands below. It is a command reference, not
another launcher: copy and run one relevant block at a time rather than
executing the entire text file. It includes calibration, debug and five
separate commands for the current `scene02-plant` campaign.

## Before the first command

The local checkout must be clean and both robot deployment roots must contain
the same committed `hub/src/focus_hub` and `hub/robot_overlay` bytes. The
scripts verify every tracked file in those two trees through the existing
SSH/tmux sessions before touching a robot process.

Required defaults:

```text
WSJ SSH pane     focus_wsj_tunnel_20260722:sensor-audit
Yunji SSH pane   focus_yunji_tunnel_20260722:sensor-audit
WSJ release      /home/nvidia/topofocus_buildmap_v2_20260723
Yunji release    /home/nyu/topofocus_buildmap_v2_20260723
Hub API          127.0.0.1:8188
GLM              127.0.0.1:31511/v1
Foxglove         ports 8765 / 8766
```

Override a deployment value only through its documented `FOCUS_*`
environment variable. A session records the resolved values; debug and live
do not silently fall back to a different root, map, transform or tunnel.

## 1. One-command board calibration and debug

Keep both robots stationary and make the existing symmetric 7 × 10 circle
board visible to both cameras:

```bash
cd /home/asus/Research/focus_realworld_workspace

SESSION_ID="scene02-plant-$(date +%Y%m%d-%H%M%S)"
bash hub/scripts/calibrate_realworld_session.sh \
  --session-id "$SESSION_ID" \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY \
  --goal-category plant
```

The command performs this sequence:

1. proves the Git tree is clean, respawns either disconnected existing
   SSH/tmux pane in place, requires a unique remote-shell probe from both
   robots, and verifies both remote deployment trees byte-identically;
2. disables Hub GOAL and starts mapping-only camera streams—no WSJ bridge,
   WATER receiver or planner command path—and opens one Foxglove preview;
3. waits for fresh WSJ and Yunji camera images, then asks the operator to
   confirm that the complete board is visible in both views and press Enter;
4. captures the first synchronized pair, runs the initial board fit and prints
   `INITIAL_BOARD_FIT_READY`;
5. asks the operator to move only the board by at least 10 cm or rotate it by
   at least 5 degrees, then press Enter a second time once the complete board
   is visible in both views;
6. captures the holdout and rejects it unless it proves independent board
   movement and cross-camera alignment;
7. writes the calibration atomically and deploys the same checked bytes to
   both robots;
8. starts calibrated read-only observation, completely fresh maps and a
   Foxglove relay bound to those exact maps;
9. writes
   `hub/runtime/sessions/<session-id>/session.json`, updates the ignored
   `current.json` pointer and runs strict no-motion VLM debug.

There are exactly two operator pauses. The fit-only result produced after the
first Enter is retained as provenance but is never deployed; only the second,
independently moved-board result can become the session calibration.

Success ends with `DEBUG_FULLSTACK_READY`. The session file binds:

- the exact Git commit;
- calibration file path, size, SHA-256, calibration ID and transform IDs;
- map directories and their start sequence boundaries;
- a per-map contract binding the Git commit, sequence boundary, transform,
  calibration, semantic backend and YOLO evidence mode;
- robot release roots, calibration paths and loopback tunnel endpoints;
- Hub, GLM, map and Foxglove tmux identities;
- generated `GOAL=false` and `GOAL=true` policy files;
- the strict debug manifest and its SHA-256.

The calibration detector and gravity-preserving solver are the existing
project implementations. The wrapper adds automatic pair selection,
independent-movement proof, atomic persistence and session binding; it does
not introduce a different calibration model.

## 2. Optional full no-motion debug without recalibration

Use this after a code change, when diagnosing the data plane, or when a new
strict debug artifact is intentionally required. A chassis-only power cycle
does not require this command:

```bash
bash hub/scripts/realworld_oneclick.sh \
  --session-file current \
  --mode debug \
  --scene-id debug-plant \
  --goal-category plant
```

The command first proves that any already-debugged session still has the same
WSJ perception/SLAM and Yunji Odin tracking processes that predated its strict debug
timestamp. It then starts a clean Hub decision epoch. It reuses a map daemon
only when its tmux command contains the session's exact Git commit,
calibration, transform and sequence boundary; otherwise it rebuilds that same
session from its immutable spool boundary. Missing or blocked map snapshots do
not prevent this recovery path: immutable session/code/debug validation runs
first, then the new daemon must produce a fresh matching generation before the
VLM may continue.

The command also replaces a stale project Foxglove relay when its map paths,
ports, semantic-overview contract or loaded relay/renderer source hash differ.
It waits until both per-robot semantic overviews and the fused overview are
actually generated; a listening port is not considered ready. If an unmanaged
process owns a required port, it fails with a clear error instead of leaving
an old picture visible. A completely fresh SegFormer map pair is allowed up
to 90 seconds to produce that first content-verified overview; subsequent
launches reuse the matching relay and normally pass immediately.

Each robot launcher must also pass
`focus-tinynav-data-plane-verification-v1` before returning ready. The
verifier receives new odometry, occupancy and router-status messages,
validates frames and nonempty known/free cells, and proves the exclusive
command topology:

```text
TinyNav controller -> v2 receiver -> guarded topic -> chassis bridge
```

WSJ debug must have no chassis subscriber. Yunji debug keeps the WATER bridge
in dry-run mode and must report an inactive command plus confirmed zero
velocity. A process, alignment file or listening port alone is not startup
success.

Debug has no stale-map or blocked-map bypass. It freezes one stable generation
of each map, requires command-capable observations with strict mapping health
received in the new Hub epoch, checks age and cross-robot skew, then runs the
real Perception/Judgment/Decision VLM while publishing HOLD only. For WSJ,
strict mapping health means the TinyNav optimizer and every IMU interval
passed even when TinyNav's all-zero odometry covariance prevents the sender
from claiming command-ready `TRACKING`. That exact fail-closed
`DEGRADED` state is valid only for freezing perception input. It cannot enable
motion: live GOAL publication still requires a fresh `READY` heartbeat from
the armed WSJ receiver and all local planner/occupancy checks.

## 3. Run one supervised physical episode

After the same session has passed debug on the same Git commit:

```bash
bash hub/scripts/realworld_oneclick.sh \
  --session-file current \
  --mode live \
  --scene-id scene02-plant \
  --episode-id scene02-plant-run01 \
  --goal-category plant \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR
```

Before enabling Hub GOAL, live records a read-only tracking-epoch probe on
each robot. The WSJ probe binds the still-running `perception` tmux process; the
Yunji probe binds `focus-yunji-odin1-driver.service`. Both process start times
must be no later than the session's strict debug timestamp. This distinguishes
a chassis-only power cycle, which is reusable, from a SLAM/Odin restart, which
is not directly reusable.

When those probes pass and both TinyNav cores are warm, the default path:

1. sends/retains a guarded stop and removes any stale receiver/bridge;
2. reuses exact session maps, GLM and Foxglove when their identities match;
3. starts a clean Hub decision epoch with no v2 decision;
4. arms WSJ and Yunji concurrently while both remain `NO_GOAL/HOLD`;
5. waits for fresh observations and `ready_for_goal=true` heartbeats.

It does not rerun the complete no-motion VLM debug, reinstall Yunji TinyNav,
rebuild matching maps, or restart a matching Foxglove relay. If the warm
non-tracking core is incomplete, it automatically prints
`FULL_DEBUG_RUNTIME_RECOVERY` and uses the slower read-only recovery. The
optional `--full-preflight` flag forces that recovery path. Neither path can
bypass a changed tracking epoch.

Only after that potentially slow startup has completed does the launcher enter
the bounded source-derived episode loop. It freezes an exact synchronized
map/source pair, advances the persistent `0,24,49,...,499` VLM state, rechecks
both runtime-readiness records and publishes one atomic pair of 8-second
expiring v2 high-level targets. Leases renew only while feedback is fresh.
Before publication, a frontier-clearance guard checks the frozen fused map.
For every active `FRONTIER_POINT`, at least one known-free cell with the
robot-specific footprint clearance must exist inside the source's unchanged
10-cell (`0.50 m`) arrival disk. A failed check holds only that robot and is
recorded in `frontier_clearance_guard.json`; the unmodified VLM output remains
in `vlm_candidate_batch.json`.

The subsequent real-world route guard reads the two frozen
`shared_world` base poses and compares the straight start-to-target segments.
If their predicted separation is below 0.9 m, or either shared pose is
unavailable, it reduces physical authority to one deterministic active robot
and holds the other. The unmodified two-robot VLM candidate is preserved as
`vlm_candidate_batch.json`; the applied decision and guard provenance are
preserved as `initial_batch.json` and `route_conflict_guard.json`. This is a
conservative execution adapter, not a change to source VLM selection and not
a certification of robot-local obstacle detours.

Yunji's deployment controller also reports when TinyNav's robot-relative
lookahead requires reverse motion. Because the deployed controller is
forward-only, the receiver immediately zeros output and rejects that frontier
leg as `LOCAL_PATH_REVERSE_REQUIRED` instead of rotating toward a backward
wall-adjacent segment. Explicit frontier path/progress rejections are
robot-local: that robot transitions to HOLD while a healthy peer retains its
existing leg. Transform, localization, e-stop, semantic and protocol failures
remain episode-wide fail-closed conditions.

This guard was added after the 2026-07-25 physical run assigned distinct
frontiers whose shared-frame routes nevertheless intersected. The observed
collision, exact pre-fix batch, video/runtime hashes and `0.0 m` geometry
replay are recorded in
[`../../audit/DUAL_ROBOT_COLLISION_20260725.md`](../../audit/DUAL_ROBOT_COLLISION_20260725.md).
The later wall-adjacent frontier evidence and exact clearance replay are
recorded in
[`../../audit/YUNJI_WALL_TURN_REJECTION_20260725.md`](../../audit/YUNJI_WALL_TURN_REJECTION_20260725.md).

At each source round boundary both robots must first acknowledge local
`HOLDING` with zero velocity; only then can the next synchronized input pair be
frozen and the next VLM round run. Frontier arrival therefore causes a replan,
not episode success.

When a `SEMANTIC_REGION` leg reports robot-local `ARRIVED`, the controller
atomically moves both robots to HOLD, waits for a post-arrival observation and
automatically preserves hash-verified terminal RGB, aligned depth, observation
metadata and map snapshots under the run directory. If no post-arrival frame
arrives before the bounded timeout, the manifest explicitly labels the latest
verifiable-frame fallback. Either form is an automatic terminal candidate, not
official SR: the model that generated the target cannot independently verify
itself.

Every exit path restores `GOAL=false`, latches WSJ navigation pause, sends a
guarded zero, removes the Go2 bridge and cancels/stops all Yunji receiver and
WATER bridge units. It deliberately leaves only camera observation, TinyNav
perception/planning cores, central maps and Foxglove running. This state has no
chassis command path and lets the next trial reuse the expensive read-only
core. One confirmation is consumed by one command; it is never persisted in
the session.

## 4. Record SR/SPL evidence immediately after a run

`ARRIVED` is not an official success. Standard SR/SPL additionally needs a
surveyed shortest collision-free path, a goal-region check and independent
terminal target evidence. The episode report now contains robot-local
start/stop poses, accumulated path length and planner STOP evidence.

Append a trial and emit an incomplete-or-complete metrics report with:

```bash
RUN_DIR=hub/runtime/oneclick_SESSION_live_SCENE_TIME
hub/.venv/bin/python hub/tools/record_realworld_trial.py \
  --episode-report "$RUN_DIR/episode_report.json" \
  --results hub/runtime/triple_ai_demo_results.json \
  --experiment-id triple-ai-lab-01 \
  --trial-index 1 \
  --termination completed \
  --robot-0-shortest-m 3.2 \
  --robot-0-shortest-evidence hub/runtime/surveys/scene01-wsj.json \
  --robot-0-reached-goal-region yes \
  --robot-0-target-verified yes \
  --robot-0-terminal-evidence "$RUN_DIR/robot-0/rgb.jpg" \
  --robot-1-shortest-m 2.8 \
  --robot-1-shortest-evidence hub/runtime/surveys/scene01-yunji.json \
  --robot-1-reached-goal-region no \
  --robot-1-target-verified no
```

For a semantic-arrival candidate, the controller's automatic terminal images
are under
`hub/runtime/oneclick_<session>_live_<scene>_<time>/<robot>/`.
An independent operator/annotator must inspect the applicable RGB before
supplying `--robot-*-target-verified yes`; do not infer that flag from the
controller's own semantic selection.

The command hashes every evidence file, rejects duplicate scene/trial
identities and atomically updates both the result set and an adjacent metrics
file. Until all four scenes × five trials exist, the metrics status remains
`incomplete` and lists the missing shape explicitly.

## Power-cycle rule

Use this decision table:

| Event | Required action |
| --- | --- |
| Go2 or WATER chassis power only; Jetson/Odin tracking stayed alive | Directly run the next live command; no board and no repeated debug |
| Hub process, local Foxglove or SSH tunnel restarted; robot tracking stayed alive | Directly run live; local components are recovered as needed |
| WSJ perception/SLAM or Yunji Odin driver restarted while the robot was certainly stationary | Create a validated stationary tracking re-anchor, then a new session |
| Robot base moved relative to the room, camera mount moved, or stationary continuity is uncertain | Run a new two-position board calibration |

`probe_tracking_epoch.py` records its source paths, process start identity,
artifact size/SHA-256 and the fact that no robot interface was used. A failed
probe stops before Hub GOAL is enabled and prints `TRACKING_EPOCH_CHANGED`.
Never edit an old session JSON to make a new transform epoch look compatible.

The fast path is structurally bounded by two parallel robot launchers rather
than two sequential full debug/live cycles. Its actual onsite duration remains
an observed quantity to measure in the next physical run; the implementation
does not claim a timing result before that measurement.

## Verification status

On 2026-07-25, session `20260725-lab05-yunjireboot4` passed strict physical
no-motion debug on commit `cdcd7e7`. Episode
`scene01-chair-run01-fastfix` then exercised fresh synchronized input, real
VLM selection, two v2 high-level targets, both robot-local planning/control
paths, physical motion, feedback and three lease renewals.

The selected frontiers were distinct, but their shared-frame routes crossed
and the platforms made physical contact. The operator stopped the run. It is
excluded from SR/SPL and documented in
[`../../audit/DUAL_ROBOT_COLLISION_20260725.md`](../../audit/DUAL_ROBOT_COLLISION_20260725.md).

The route-conflict guard described above was added afterward at commit
`b79879b`. The observed candidate replays to `0.0 m` predicted separation and
one active robot. Later session `20260725-lab12` passed a fresh two-position
board calibration and strict no-motion debug; the operator confirmed that its
new Foxglove maps looked correct. The Go2 chassis-only charging event after
that check did not restart WSJ perception/SLAM, so it does not by itself invalidate
that physical calibration. The fast-reuse orchestration described above is
locally regression-tested but still needs its first measured physical timing
run after deployment.
