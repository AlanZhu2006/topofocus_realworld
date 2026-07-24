# V2 dual-robot physical quick start

> **Current deployment note (2026-07-24):** the normal operator path is now
> [`ONECLICK_SESSION_WORKFLOW.md`](ONECLICK_SESSION_WORKFLOW.md). Steps 1–6
> below explain the internal contract and remain useful for a new machine, but
> should not be manually substituted for the persistent launcher. The last
> predecessor calibration was `shared-board-odin1-20260723-v3`, with WSJ transform
> `wsj-tinynav-depth-20260723-powercycle-v3` and Yunji transform
> `yunji-odin1-board-20260723-powercycle-v6`. The live attempts reached both
> local planner paths but produced no official success. It is not a strict
> persistent `current` session because it predates the quantitative moved-board
> field. Use [CURRENT_STATUS.md](../../CURRENT_STATUS.md) for the gate; do not
> substitute historical placeholders below into a new session.

This is the shortest path from the working v2 synthetic chain to one
operator-supervised real episode. It deliberately postpones Foxglove styling,
map aesthetics, detector tuning and large-scale evaluation.

## Fixed authority split

```text
GLM/VLM + source-derived allocation (Hub)
  -> atomic pair of expiring shared_world high-level targets
     -> robot-0 receiver -> TinyNav POI -> TinyNav planner/controller
        -> raw /cmd_vel -> local lease gate -> guarded /cmd_vel -> Go2 bridge
     -> robot-1 receiver -> TinyNav POI -> TinyNav planner/controller
        -> raw /cmd_vel -> local lease gate -> guarded /cmd_vel
           -> WATER /api/joy_control velocity bridge
```

The Hub never sends wheel velocity. Both GOAL leases may be active at the same
time. Renewal extends a robot's local permission without resending its target.
One arrival changes that robot to HOLD while the other robot keeps its existing
leg. Expiry or loss closes only that robot's local output; an explicit
scene-level HOLD closes both.

## What is implemented versus still physical

Implemented and locally tested:

- strict v2 batch/event schema, atomic two-GOAL publication and independent
  lease renewal;
- real shadow-manifest to semantic-region/frontier batch conversion;
- WSJ TinyNav POI receiver, local occupancy reachability, `nav_done` arrival,
  and a distinct raw-to-guarded `/cmd_vel` lease gate;
- Yunji Odin-to-TinyNav adapter, fresh online occupancy, the same TinyNav
  A*/local planner/controller chain, and a guarded WATER `/api/joy_control`
  bridge;
- live receiver health heartbeats and command-capable sender metadata that
  remain disabled unless a measured `base_T_camera` artifact is loaded;
- default receiver behavior is read-only. Live output requires a robot-specific
  flag and exact operator-presence phrase.

Completed for the predecessor deployment:

1. a two-camera board fit and independently moved-board holdout;
2. measured `base_link -> camera` artifacts for both robots;
3. command-capable observations, receiver heartbeats and local planner-chain
   startup;
4. fail-closed supervised attempts through TinyNav and WATER.

The persistent-session implementation is locally tested. Still required is
one new board calibration through the canonical wrapper, its strict no-motion
full-stack result, a target outside both arrival radii, and one bounded episode
with terminal verification. The tracked 2026-07-22 board artifact remains a
historical format example, not a reusable transform.

## 1. Record the two body-camera mount transforms

Use physical measurements or a surveyed robot TF, never a nominal guess. The
tool records exactly what the operator enters and issues no robot command:

```bash
hub/.venv/bin/python hub/tools/record_base_camera_calibration.py \
  --robot-id robot-0 \
  --camera-frame camera \
  --x-m <MEASURED_FORWARD> --y-m <MEASURED_LEFT> --z-m <MEASURED_UP> \
  --roll-deg <MEASURED_OR_DERIVED> \
  --pitch-deg <MEASURED_OR_DERIVED> \
  --yaw-deg <MEASURED_OR_DERIVED> \
  --measurement-note '<HOW_AND_WHEN_MEASURED>' \
  --operator-confirmation PHYSICAL_MOUNT_VALUES_REVIEWED \
  --output hub/runtime/calibration/wsj_base_camera_<date>.json
```

Repeat for `robot-1` and the exact Odin output frame
`odin1_camera_optical_frame`. For Yunji this is the WATER chassis
`base_link -> Odin camera`, not merely Odin's internal IMU-to-camera factory
extrinsic. The receiver combines this mount with the tracked serial-specific
Odin factory calibration.

## 2. Fresh shared-frame calibration

Run the wrapper in
[`ONECLICK_SESSION_WORKFLOW.md`](ONECLICK_SESSION_WORKFLOW.md). It reuses the
existing detector/solver described in
[YUNJI_ODIN1_DEPLOYMENT.md](YUNJI_ODIN1_DEPLOYMENT.md), requires the
independently moved-board holdout, and creates new transform/calibration IDs
and map directories.

Before moving farther, keep both senders in their normal mapping-only mode and
run each receiver without its `--enable-live-*` flag. This checks the online
TinyNav frame/map alignment and writes a checksummed local alignment artifact
without publishing a POI or writing a physical velocity.

## 3. Arm command-capable observations, not motion

Restart the WSJ sender with its usual fresh shared transform plus:

```text
--enable-command-capable-observations
--activation-confirmation COMMAND_CAPABLE_OBSERVATION_ONLY
--base-camera-calibration-file <WSJ_MEASURED_ARTIFACT>
--heartbeat-hz 0
```

Restart `odin1_sender.py` with its fresh shared transform plus the same four
arguments, using the Yunji/Odin mount artifact. These switches only set
`mapping_only=false` and add measured `base_T_camera`; neither sender imports a
planner or calls a motion endpoint. The armed receivers, not the observation
senders, own the fresh command-health heartbeat.

Keep `allow_goal=false` at this stage. Confirm that new observations have the
fresh transform version, common shared calibration ID, `mapping_only=false`
and non-null `base_T_camera`.

## 4. WSJ local chain

Start the existing TinyNav semantic navigation wrapper without Go2 output:

```bash
cd /home/nvidia/twork/tinynav
bash scripts/tinynav_semantic_auto_nav.sh \
  --map <CURRENT_TINYNAV_MAP> --no-go2 --no-rviz
tmux kill-window -t tinynav_semantic_nav_auto:rviz-goal
```

Removing `rviz-goal` is mandatory: another `/mapping/cmd_pois` publisher would
bypass the versioned decision receiver. Start the receiver from this repository
and then the existing Go2 bridge on the guarded topic:

```bash
source /home/nvidia/twork/tinynav_setup.bash
export PYTHONPATH=<TOPOFOCUS_REPO>/hub/src
export FOCUS_ROBOT_TOKEN="$(< /path/to/robot-token)"
python3 -u <TOPOFOCUS_REPO>/hub/robot_overlay/v2_wsj_receiver.py \
  --calibration-file <FRESH_SHARED_BOARD_ARTIFACT> \
  --transform-version <FRESH_WSJ_TRANSFORM_VERSION> \
  --shared-frame-calibration-id <FRESH_SHARED_CALIBRATION_ID> \
  --enable-live-go2-motion \
  --operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR
```

In a separate managed tmux window:

```bash
export GO2_CMD_TOPIC=/focus_guarded_cmd_vel
export UNITREE_NET_IF=<GO2_INTERFACE>
export GO2_MAX_VX=0.20 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.50
export GO2_REMOTE_PRIORITY=true
bash /home/nvidia/twork/tinynav/scripts/run_go2_cmd_bridge.sh
```

The receiver refuses GOAL unless TinyNav owns the POI subscriber and raw
`/cmd_vel` publisher, no other POI publisher exists, no bridge subscribes
directly to raw `/cmd_vel`, and a bridge subscribes to
`/focus_guarded_cmd_vel`.

## 5. Yunji local chain

The normal launcher installs the pinned planner-only TinyNav revision
idempotently, starts the Odin adapter, online occupancy mapper, A* router,
TinyNav local planner/controller, guarded WATER bridge and v2 receiver:

```bash
FOCUS_YUNJI_SHARED_CALIBRATION_FILE=<FRESH_SHARED_BOARD_ARTIFACT> \
FOCUS_YUNJI_BASE_CAMERA_CALIBRATION=<YUNJI_ODIN_MEASURED_ARTIFACT> \
FOCUS_YUNJI_TRANSFORM_VERSION=<FRESH_YUNJI_TRANSFORM_VERSION> \
FOCUS_SHARED_CALIBRATION_ID=<FRESH_SHARED_CALIBRATION_ID> \
bash <TOPOFOCUS_REPO>/hub/robot_overlay/start_yunji_v2.sh --mode debug
```

Debug mode runs the full graph but the WATER bridge is dry-run. Live mode adds
the exact operator phrase:

```bash
# Only with the operator beside a clear, powered robot:
<same environment> \
bash <TOPOFOCUS_REPO>/hub/robot_overlay/start_yunji_v2.sh \
  --mode live \
  --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR
```

Odin `odom` is the fresh session-local TinyNav `world`: calibrated depth drives
the local planner, and the synchronized SLAM cloud drives the online occupancy
map. The Hub still emits only an expiring high-level POI. WATER is used only as
the final velocity executor and health/watchdog authority. The active path does
not call `/api/map/accessible_point_query`, `/api/make_plan`, `/api/move`, or
depend on a WATER saved map. The old `v2_yunji_receiver.py` remains in the
repository only as historical/rollback evidence and is not launched.

## 6. Publish one supervised episode

This section is an internal/reference expansion. For normal operation, use
`realworld_oneclick.sh --session-file current --mode live`; it performs these
steps, freezes exact inputs and restores debug on exit. Only after both
receivers remain healthy with no GOAL active:

1. set the exact fresh transform versions and `allow_goal=true` for both robots
   in the deployment `robots.json`;
2. restart the Hub, let both command-capable senders upload new observations,
   and let both receivers repost READY health;
3. run one fresh real VLM shadow round to create the frozen manifest;
4. run the supervised controller with a short clear scene and separated goals:

```bash
hub/.venv/bin/python hub/tools/run_v2_supervised_episode.py \
  --manifest <FRESH_SHADOW_MANIFEST> \
  --scene-id <SCENE_ID> --episode-id <EPISODE_ID> \
  --output hub/runtime/<UNIQUE_EPISODE_OUTPUT> \
  --admin-token-file <ADMIN_TOKEN_FILE> \
  --enable-live-goal-publication \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR
```

Stand beside the robots with their native local stop controls. After the run,
restore both `allow_goal` values to false. `ARRIVED` is not official success;
score SR/SPL only after the separate terminal image/region verification in
[TRIPLE_AI_REALWORLD_DEMO.md](TRIPLE_AI_REALWORLD_DEMO.md).

## Provenance note

The `/api/joy_control` limits (±0.5 m/s, ±1.0 rad/s), 0.5-second command
duration and refresh behavior are source-derived from the vendor's WATER
software API manual v1.8.7. The bridge is capped further at 0.15 m/s and
0.40 rad/s. The TinyNav source is pinned to
`AlanZhu2006/go2_tinynav@5705bb61dafb407594970ab2bc85c63fc71e0a24`;
the installer records paths, sizes and SHA-256 values. The adapter, bridge and
new Yunji chain are locally tested but physical motion remains unverified.
Runtime alignment, trajectory and terminal evidence must retain their absolute
source path, size, SHA-256 and observed/source-derived/unverified
classification.
