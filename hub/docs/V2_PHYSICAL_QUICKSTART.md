# V2 dual-robot physical quick start

> **Current deployment note (2026-07-24):** steps 1–5 below explain the
> contract and remain useful for a new machine, but they are already completed
> for the present WSJ/Yunji deployment. The current calibration is
> `shared-board-odin1-20260723-v3`, with WSJ transform
> `wsj-tinynav-depth-20260723-powercycle-v3` and Yunji transform
> `yunji-odin1-board-20260723-powercycle-v6`. The live attempts reached both
> local planner paths but produced no official success. Use
> [CURRENT_STATUS.md](../../CURRENT_STATUS.md) for current maps and gates; do
> not substitute the historical placeholders below into the active session.

This is the shortest path from the working v2 synthetic chain to one
operator-supervised real episode. It deliberately postpones Foxglove styling,
map aesthetics, detector tuning and large-scale evaluation.

## Fixed authority split

```text
GLM/VLM + source-derived allocation (Hub)
  -> atomic pair of expiring shared_world high-level targets
     -> robot-0 receiver -> TinyNav POI -> TinyNav planner/controller
        -> raw /cmd_vel -> local lease gate -> guarded /cmd_vel -> Go2 bridge
     -> robot-1 receiver -> WATER /api/move
        -> WATER move_base/local planner/controller
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
- Yunji WATER receiver using `/api/move`, `/api/move/cancel`,
  `/api/robot_status` and `/api/map/accessible_point_query`;
- live receiver health heartbeats and command-capable sender metadata that
  remain disabled unless a measured `base_T_camera` artifact is loaded;
- default receiver behavior is read-only. Live output requires a robot-specific
  flag and exact operator-presence phrase.

Completed for the current deployment:

1. a two-camera board fit and independently moved-board holdout;
2. measured `base_link -> camera` artifacts for both robots;
3. command-capable observations, receiver heartbeats and local planner-chain
   startup;
4. fail-closed supervised attempts through TinyNav and WATER.

Still required is loading the last synchronized WSJ command-floor/router
changes, passing one no-motion full-stack run from the same saved session,
choosing a target outside both arrival radii, and then completing one bounded
episode with terminal verification. The tracked 2026-07-22 board artifact
remains a historical format example, not the current transform.

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

Run the existing board workflow in
[YUNJI_ODIN1_DEPLOYMENT.md](YUNJI_ODIN1_DEPLOYMENT.md), including the
independently moved-board holdout. Use new transform/calibration IDs and new
map output directories. Do not append to the July 22 maps.

Before moving farther, keep both senders in their normal mapping-only mode and
run each receiver without its `--enable-live-*` flag. This checks TF/WATER map
alignment and writes a checksummed local alignment artifact without publishing
POI, pause, Twist, `/api/move` or `/api/move/cancel`.

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

With Odin and the command-capable sender already running:

```bash
source /opt/ros/humble/setup.bash
source /home/nyu/odin_ws/install/setup.bash
export PYTHONPATH=<TOPOFOCUS_REPO>/hub/src
export FOCUS_ROBOT_TOKEN="$(< /path/to/robot-token)"
python3 -u <TOPOFOCUS_REPO>/hub/robot_overlay/v2_yunji_receiver.py \
  --calibration-file <FRESH_SHARED_BOARD_ARTIFACT> \
  --base-camera-calibration-file <YUNJI_ODIN_MEASURED_ARTIFACT> \
  --odin-factory-calibration-file \
    <TOPOFOCUS_REPO>/hub/config/calibration/odin1_O1-P070100205_factory_20260722.json \
  --transform-version <FRESH_YUNJI_TRANSFORM_VERSION> \
  --shared-frame-calibration-id <FRESH_SHARED_CALIBRATION_ID> \
  --enable-live-water-motion \
  --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR
```

At startup the receiver aligns Odin odometry and WATER's current saved-map pose
using synchronized read-only samples. Every semantic-region or frontier goal
must also pass WATER's reachable-point query. It never uses
`/api/joy_control`.

## 6. Publish one supervised episode

Only after both receivers remain healthy with no GOAL active:

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

The WATER endpoint behavior above is source-derived from the vendor's WATER
software API manual v1.8.7. The new receiver and guard behavior is locally
tested but unverified on physical motion. Runtime alignment, trajectory and
terminal evidence must retain their absolute source path, size, SHA-256 and
observed/source-derived/unverified classification.
