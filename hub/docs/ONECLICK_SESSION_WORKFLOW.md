# Persistent one-click physical workflow

This is the canonical operator path for a new physical placement. It replaces
the dated, hard-coded July 24 launcher. The workflow has three explicit
states:

`board calibration -> strict no-motion debug -> one supervised live episode`

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
   WATER receiver or planner command path;
3. asks for the first stable board placement;
4. asks the operator to move only the board by at least 10 cm or rotate it by
   at least 5 degrees;
5. automatically selects two fresh, synchronized camera pairs and rejects a
   holdout that does not prove independent board movement;
6. writes the calibration atomically and deploys the same checked bytes to
   both robots;
7. starts calibrated read-only observation, completely fresh maps and a
   Foxglove relay bound to those exact maps;
8. writes
   `hub/runtime/sessions/20260725-lab01/session.json`, updates the ignored
   `current.json` pointer and runs strict no-motion VLM debug.

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
old Hub process, collects new observations, freezes the exact map/source pair
and finishes the VLM/HOLD round before starting either motion-capable
receiver. The episode publishes an atomic pair of expiring v2 targets and
renews leases only while feedback is fresh.

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
  --episode-report hub/runtime/oneclick_<session>_live_<scene>_<time>/episode/episode_report.json \
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

## What remains physically unverified

The scripts, schemas and tests are locally verified. The new persistent
workflow has not yet completed a physical debug run or a valid physical
episode. Historical July 23/24 runs used the predecessor launcher and remain
excluded from SR/SPL.
