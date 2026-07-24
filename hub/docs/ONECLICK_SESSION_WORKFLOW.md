# Persistent one-click physical workflow

This is the canonical operator path for a new physical placement. It replaces
the dated, hard-coded July 24 launcher. The workflow has three explicit
states:

`board calibration -> strict no-motion debug -> one bounded multi-round live episode`

The first two states are non-motion. The live state still requires one fresh,
onsite confirmation because the Hub only publishes expiring high-level
targets; TinyNav/WATER and the robots retain final stop/reject authority.

For normal onsite use, the repository-root `command.txt` stores copy-ready
versions of the unwrapped commands below. It is a command reference, not
another launcher: copy and run one relevant block at a time rather than
executing the entire text file. It includes calibration, debug and five
separate `scene01-chair` live episode commands.

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

bash hub/scripts/calibrate_realworld_session.sh \
  --session-id 20260725-lab01 \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY \
  --goal-category chair
```

The command performs this sequence:

1. proves the Git tree is clean and both remote deployment trees are
   byte-identical;
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
   `hub/runtime/sessions/20260725-lab01/session.json`, updates the ignored
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

## 2. Repeat no-motion debug without recalibration

Use this after a Hub-computer restart, code review or visualization check:

```bash
bash hub/scripts/realworld_oneclick.sh \
  --session-file current \
  --mode debug \
  --scene-id debug-chair \
  --goal-category chair
```

The command always starts a clean Hub decision epoch. It reuses a map daemon
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
  --scene-id scene01-chair \
  --episode-id scene01-chair-run01 \
  --goal-category chair \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR
```

Live startup still begins with both robot receivers read-only. It clears the
old Hub process and first establishes a new observation/history epoch with no
v2 decision present. It then starts both motion-capable receivers while the
robots remain locally in `NO_GOAL/HOLD`, and waits until both Hub
runtime-readiness records report `ready_for_goal=true` from fresh
robot-receiver heartbeats.

Only after that potentially slow startup has completed does the launcher enter
the bounded source-derived episode loop. It freezes an exact synchronized
map/source pair, advances the persistent `0,24,49,...,499` VLM state, rechecks
both runtime-readiness records and publishes one atomic pair of 8-second
expiring v2 high-level targets. Leases renew only while feedback is fresh.
Before publication, the real-world route guard reads the two frozen
`shared_world` base poses and compares the straight start-to-target segments.
If their predicted separation is below 0.9 m, or either shared pose is
unavailable, it reduces physical authority to one deterministic active robot
and holds the other. The unmodified two-robot VLM candidate is preserved as
`vlm_candidate_batch.json`; the applied decision and guard provenance are
preserved as `initial_batch.json` and `route_conflict_guard.json`. This is a
conservative execution adapter, not a change to source VLM selection and not
a certification of robot-local obstacle detours.

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
guarded zero, removes the Go2 bridge, cancels/stops the Yunji live receiver and
returns both receivers to debug. One confirmation is consumed by one command;
it is never persisted in the session.

## 4. Record SR/SPL evidence immediately after a run

`ARRIVED` is not an official success. Standard SR/SPL additionally needs a
surveyed shortest collision-free path, a goal-region check and independent
terminal target evidence. The episode report now contains robot-local
start/stop poses, accumulated path length and planner STOP evidence.

Append a trial and emit an incomplete-or-complete metrics report with:

```bash
hub/.venv/bin/python hub/tools/record_realworld_trial.py \
  --episode-report hub/runtime/oneclick_<session>_live_<scene>_<time>/episode_report.json \
  --results hub/runtime/triple_ai_demo_results.json \
  --experiment-id triple-ai-lab-01 \
  --trial-index 1 \
  --termination completed \
  --robot-0-shortest-m 3.2 \
  --robot-0-shortest-evidence hub/runtime/surveys/scene01-wsj.json \
  --robot-0-reached-goal-region yes \
  --robot-0-target-verified yes \
  --robot-0-terminal-evidence hub/runtime/terminal/scene01-run01-wsj.jpg \
  --robot-1-shortest-m 2.8 \
  --robot-1-shortest-evidence hub/runtime/surveys/scene01-yunji.json \
  --robot-1-reached-goal-region no \
  --robot-1-target-verified no
```

For a semantic-arrival candidate, the controller's automatic terminal images
are under
`hub/runtime/oneclick_<session>_live_<scene>_<time>/terminal/<robot>/`.
An independent operator/annotator must inspect the applicable RGB before
supplying `--robot-*-target-verified yes`; do not infer that flag from the
controller's own semantic selection.

The command hashes every evidence file, rejects duplicate scene/trial
identities and atomically updates both the result set and an adjacent metrics
file. Until all four scenes × five trials exist, the metrics status remains
`incomplete` and lists the missing shape explicitly.

## Power-cycle rule

A power cycle does not move a sensor mount, but it can reset a robot-local
odometry origin. A session transform is therefore reusable only when the
sensor mount, robot pose and relevant tracking origin are proven unchanged.
If that proof is unavailable, run the board-calibration command with a new
session ID. Never edit an old session JSON to make a new transform epoch look
compatible.

## Verification status

On 2026-07-24, session `20260725-lab04` passed board holdout calibration and
the strict physical no-motion debug. Its first live invocation observed fresh
dual-robot inputs and completed the real VLM in 16.089 seconds, but robot
receiver startup then aged the oldest frozen input to 89.689 seconds. The
60-second preflight correctly rejected publication with `INPUT_STALE`; only
HOLD decisions were observed and no robot command was sent.

The launcher ordering was subsequently corrected so receiver startup and
heartbeat verification precede the final input freeze. Shell syntax and the
complete local Hub regression suite pass. A physical rerun of the corrected
ordering remains unverified and the rejected attempt is not an SR/SPL trial.
