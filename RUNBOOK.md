# Runbook and verification gates

## Corrected HPC reference launch

This is a reference for inspecting the original experiment, not the local robot deployment.  The upstream `running_inference.md` misses external source paths and its YOLO name does not match the checked-in `.pt` filename.

```bash
base=/scratch/jl9356/Focus_realworld
legacy=/scratch/jl9356/MCoCoNav
sif=/share/apps/images/cuda12.6.3-cudnn9.5.1-ubuntu22.04.5.sif

singularity exec --nv --overlay "$base/overlay-15GB-500K.ext3":ro "$sif" /bin/bash
source /ext3/env.sh
conda activate mcoconav
export PYTHONPATH="$base:$legacy:$legacy/habitat-lab:${PYTHONPATH:-}"
export HF_HOME="$base/hf_cache"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1

# terminal 1: VLM service
python "$base/CogVLM2/basic_demo/glm4_openai_api_demo_1gpu.py" --port 8000

# terminal 2: simulator reference only
cd "$base"
python main.py --task_config envs/habitat/configs/tasks/multi_objectnav_hm3d.yaml \
  --num_agents 2 --yolo_weights detect/yolov10m.pt --base_url http://127.0.0.1:8000/v1
```

`--num_agents` alone does not rewrite the Habitat task configuration; the YAML count and CLI count must agree.

## Local hub migration gates

| Gate | Evidence required | Current state |
| --- | --- | --- |
| G0 | local source and exact model artifacts available | **passed** (`audit/G0_LOCAL_VERIFICATION.md`) |
| G1 | fresh local environment imports Torch/CUDA, GLM server dependencies, RedNet, YOLO, CLIP | **passed** (`audit/G1_LOCAL_ENVIRONMENT.md`) |
| G2 | GLM server answers a controlled offline request locally | **passed** (`audit/G2_LOCAL_GLM_REQUEST.md`) |
| G3 | one robot replay reaches semantic-map update from recorded RGB-D/pose | **passed** (`audit/G3_LOCAL_REPLAY_MAPPING.md`) |
| G4 | two robot replays fuse into one declared coordinate frame and receive distinct decisions | live v2 board calibration plus independent moved-board holdout passed; read-only shared-map fusion is available, but distinct-decision evidence is still missing, so the full gate remains open |
| G5 | robot-side safety controller rejects stale/unsafe commands in a hardware-in-the-loop test | not implemented |

Do not claim live two-robot navigation before G4 and G5 pass.

## Reproducible commands (all local, loopback, dry-run)

```bash
cd /home/asus/Research/focus_realworld_workspace

# G0 / G1 / tests
/usr/bin/python3.10 hub/tools/g0_audit.py --workspace "$PWD" --full-hash
hub/.venv/bin/python hub/tools/g1_preflight.py --workspace "$PWD"
hub/.venv/bin/python -m pytest hub/tests -q

# G2: GLM offline service + one controlled request (two terminals)
FOCUS_GLM_PORT=31511 bash hub/scripts/run_glm_offline.sh
hub/.venv/bin/python hub/tools/g2_request.py --base-url http://127.0.0.1:31511/v1

# G3 input preparation (system python reads the robot's Berkeley-DB shelve)
/usr/bin/python3.10 hub/tools/extract_tinynav_record.py \
  --record data/robot_replays/wsj_semantic_map_record_20260717_102052 \
  --output data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted

# G3: deterministic replay mapping (runs twice, compares map hashes)
hub/.venv/bin/python hub/tools/g3_replay.py \
  --record data/robot_replays/wsj_semantic_map_record_20260717_102052 \
  --extracted data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted \
  --output data/robot_replays/g3_output_<date> --runs 2

# Single-robot full-chain rehearsal: sender -> hub API -> spool -> RedNet map
# -> frontiers -> GLM-4V choice -> decision publish -> guard dry-run POI.
# Starts and stops its own hub + GLM subprocesses; add --no-vlm to skip GLM.
hub/.venv/bin/python hub/tools/e2e_single_robot.py \
  --record data/robot_replays/wsj_semantic_map_record_20260717_102052 \
  --extracted data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted \
  --output data/robot_replays/e2e_single_<date> --stride 3

# G5 fault-injection matrix (fully local, no robot): 9 fail-closed scenarios
# over the real wire protocol, including a genuine hub SIGKILL+restart proof
# that a latched STOP survives. Evidence: audit/G5_FAULT_INJECTION.md.
hub/.venv/bin/python hub/tools/g5_fault_injection.py \
  --output data/robot_replays/g5_fault_injection_<date>

# Pseudo-dual G4 machinery rehearsal (NOT a G4 pass — one session split into
# two virtual robots, identity shared frame). Evidence in the tool's --output.
hub/.venv/bin/python hub/tools/g4_pseudo_dual.py \
  --record data/robot_replays/wsj_semantic_map_record_20260717_102052 \
  --extracted data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted \
  --output data/robot_replays/g4_pseudo_<date> --glm-url http://127.0.0.1:31511/v1

# One-click hub startup (this machine): hub API + GLM-4V + incremental
# mapping/decision pipeline daemon together, tmux session, health checks.
# First run bootstraps hub/runtime/tokens.json (printed once) and admin_token.
bash hub/scripts/focus_hub_up.sh
bash hub/scripts/focus_hub_down.sh   # stop; preserves runtime/{spool,state,tokens}

# One-click live ROS 2 rehearsal (on the robot, wsj): real perception_node +
# map_node + semantic_pointcloud_node + focus_ros_sender.py, replaying a real
# recorded bag by default (no camera/robot hardware needed); never starts
# planning/control/go2-bridge, so it cannot move the robot. --live switches
# to the real camera (operator-presence only). Evidence: audit/LIVE_ROS2_SENDER.md.
#   bash /home/nvidia/focus_sender/run_live_rehearsal.sh --base-url http://127.0.0.1:18089
#   bash /home/nvidia/focus_sender/stop_live_rehearsal.sh

# Yunji WATER second robot (on nyush-nuc): network check first, then the
# observation sender. mapping_only, dry-run command guard only — never calls
# /api/move. Pose source is live /sensors_fusion/odom (no saved-map
# dependency, HPC-faithful) since 2026-07-19, not AMCL's map-relocalized
# current_pose. Evidence: audit/YUNJI_WATER_SENDER.md.
#   bash ~/workspace/tinynav/yunji-water-robot/tools/network/prepare_yunji_usb_network_linux.sh --check-only
#   FOCUS_ROBOT_TOKEN=... python3 ~/focus_sender_yunji/yunji_sender.py \
#     --base-url http://127.0.0.1:18089 --rate-hz 1 --max-frames 8
#     # add --shared-frame-transform-file <calib.json> once a real dual-robot
#     # calibration has been run (see below); omitting it leaves poses in
#     # this robot's own local odometry frame.

# Session-start shared-frame calibration (real-machine analogue of Habitat's
# per-episode start-position reset): requires both robots' senders to have
# already uploaded at least one observation each, captured within
# --max-sync-skew-s of each other while the robots were physically
# co-located (or at a known, measured offset — see --offset-file). Pure
# read of already-spooled data; sends nothing to either robot.
hub/.venv/bin/python hub/tools/calibrate_shared_frame.py \
  --spool hub/runtime/spool --reference-robot robot-0 --other-robot robot-1 \
  --output hub/runtime/shared_frame_<date>.json \
  --transform-version shared-frame-<date>-v1 \
  --calibration-id shared-frame-<date>-v1

# Live multi-robot dashboard (this machine): run one hub_pipeline_daemon.py
# per robot with periodic snapshotting enabled, then foxglove_relay.py
# republishes each robot's latest camera frame + own incremental map and
# explicit per-robot staleness over one Foxglove
# WebSocket server. Open Foxglove, connect to ws://<this-host>:8765, and
# import hub/foxglove/dual_robot_dashboard.json. Evidence:
# audit/FOXGLOVE_DASHBOARD_20260720.md and
# audit/LIVE_MAP_RECOVERY_20260722.md and
# audit/SHARED_FRAME_V2_20260722.md.
#
# Live defaults wait for three continuous poses and a three-frame RANSAC
# ground consensus, then use 0.20m/10deg/5s keyframes, reversible log-odds
# obstacles and a 0.15-0.75m collision band. Use a NEW --out-dir for every
# fresh map session; never append across a pose/transform discontinuity.
hub/.venv/bin/python hub/tools/hub_pipeline_daemon.py \
  --spool hub/runtime/spool --robot-id robot-0 --hub-url http://127.0.0.1:8088 \
  --admin-token-file hub/runtime/admin_token --no-cascade \
  --log hub/runtime/dash_wsj.log --out-dir hub/runtime/map_out_wsj \
  --snapshot-interval-s 3.0 &
hub/.venv/bin/python hub/tools/hub_pipeline_daemon.py \
  --spool hub/runtime/spool --robot-id robot-1 --hub-url http://127.0.0.1:8088 \
  --admin-token-file hub/runtime/admin_token --no-cascade \
  --log hub/runtime/dash_yunji.log --out-dir hub/runtime/map_out_yunji \
  --snapshot-interval-s 3.0 &
hub/.venv/bin/python hub/tools/foxglove_relay.py \
  --robot robot-0:wsj:hub/runtime/map_out_wsj \
  --robot robot-1:yunji:hub/runtime/map_out_yunji \
  --port 8765

# Only after both daemons were started with the same independently verified
# --shared-frame-calibration-id may the relay add --fuse. A common frame name
# or two transform_version strings alone are insufficient. With --fuse, the
# dashboard shows /fused/geometry_map by default; /fused/semantic_map remains
# hidden until /fused/status reports real, independently checked semantic
# evidence. Re-import the JSON layout after repository layout changes.
```

The bounded live-spool parameter/RedNet diagnostics, operator-present moved
map gate, and existing board-calibration reuse sequence are documented in
`hub/docs/OFFLINE_MAP_VALIDATION.md`.

The e2e safety lane must end with the hub rejecting GOAL publishes (HTTP 409)
under the default `allow_goal=false` policy; only the explicitly-labelled
rehearsal lane (test tokens, placeholder calibration, loopback) exercises the
GOAL-to-POI path, and its output is a dry-run artifact, never a robot command.

## Local environment finding

The host has Python 3.10 and PyTorch `2.7.1+cu126` with `torch.cuda.is_available() == True` on the A6000.  `transformers`, `accelerate`, OpenAI `clip`, OpenCV, and Uvicorn are present; `bitsandbytes`, FastAPI, OpenAI client, Ultralytics, Detectron2, Habitat, Habitat-Sim, scikit-image, and yacs are not.  This supports a clean local hub environment; it is not a reason to import the HPC overlay.

Create a dedicated local environment or Docker image only after the model transfer completes.  Start with the VLM service dependencies and a robot-ingest replay, and leave Detectron2/Habitat out of the first real-robot path unless a retained module demonstrably needs them.

## First implementation order

1. Capture two short, time-synchronized robot sensor replays and calibration/transforms.
2. Build the ingest schema in `CENTRAL_DEPLOYMENT.md`; test ordering, loss, and stale-message behavior.
3. Replace Habitat observation acquisition with replay ingest while retaining a single-agent map update.
4. Introduce shared-frame map fusion, then sequential frontier allocation for two robots.
5. Send high-level target poses to robot-local safety/navigation layers.  Do not begin with direct velocity control.
6. Add VLM only after the non-VLM replay loop is deterministic and logged.
