# Local two-robot hub: required assets and boundary

> Historical design decision from the initial migration. For the current
> implemented state and reproducible commands, start with `README.md` and
> `docs/REPRODUCE.md`; this file remains as source-derived rationale.

## Decision

Do **not** use the Torch HPC Singularity image and overlay as the local production deployment.  They are a useful frozen reference for dependency recovery, but they contain an experiment environment tied to an old MCoCoNav checkout and Habitat simulation.  This local A6000 machine should run a native or purpose-built containerized hub, with the two robots communicating to it over an explicit protocol.

The required transformation is:

```text
robot 0 RGB-D + pose ─┐
robot 1 RGB-D + pose ─┼─> ingest/validation ─> per-robot semantic maps
object goal + status ─┘                           │
                                                   v
                         shared-coordinate map fusion + frontier allocation
                                                   │
                                                   v
                         VLM decision service + classical FMM planner
                                                   │
                    robot-specific safe goals/actions <─ outbound command API
```

The central hub decides at a slower planning rate.  Each robot must retain a local safety controller for emergency stop, velocity limits, obstacle braking, heartbeat loss, and final motion execution.

## Assets to keep locally

| Asset | Size | Why it is needed | Local target | Status |
| --- | ---: | --- | --- | --- |
| Focus source and configs | about 20 MB | map fusion, VLM prompts, planner logic, server code | `source/Focus_realworld/` | copied and checksum-validated |
| RedNet source | 76 KB | imported unconditionally by `LLM_Agent` | `dependencies/RedNet/` | copied and checksum-validated |
| RedNet HM3D checkpoint | 656,550,984 B | RGB-D semantic-map prediction | `artifacts/checkpoints/` | copied |
| YOLOv10m checkpoint | 33,643,667 B | object detections used by the VLM cycle | `artifacts/vision/` | copied |
| OpenAI CLIP ViT-B/32 | 353,976,522 B | `RoomSemantics` initializes it at startup | `artifacts/vision/` | copied |
| GLM-4V-9B offline cache | 27.8 GB | local OpenAI-compatible VLM decision server | `artifacts/models/hf_cache/hub/models--THUDM--glm-4v-9b/` | copied |
| Category map and priors | small | semantic-label alignment and priors | `source/Focus_realworld/data/` | copied |

The four model artifacts total roughly 29 GB on disk.  They were copied with `rsync --partial --append-verify`; the transfer completed successfully and the log is retained.

## Do not copy for the hub

| Excluded item | Remote size | Reason |
| --- | ---: | --- |
| `data/` HM3D/ObjectNav corpus | 59 GB | simulator evaluation data, not a physical robot input |
| `overlay-15GB-500K.ext3` | 16.3 GB | HPC environment snapshot; not the deployment runtime |
| CUDA SIF image | 7.5 GB | HPC base image; use it only to inspect dependency versions |
| Hugging Face CLIP ViT-L/14 cache | 1.6 GB | current `room_semantic.py` loads OpenAI CLIP ViT-B/32 instead |
| `data/objectnav_hm3d_v2.zip` | 260 MB | simulator episodes |
| full scene assets | large | no use in a live robot ingest path |

Two selected minival scenes were measured only to scope optional simulator smoke tests: 86 MB and 108 MB.  They are not needed for the hub.

## Adapter contract and implemented state

`main.py` assumes Habitat observations and writes Habitat actions.  A real hub needs an adapter with these contracts before reusing its mapping and planning code.

That adapter now exists under `hub/`. The following subsections remain the
source-derived contract; current implementation and physical-test status are
tracked in [`CURRENT_STATUS.md`](CURRENT_STATUS.md).

### Inbound message per robot

- `robot_id`, monotonically increasing `sequence`, and synchronized `timestamp`.
- RGB image, depth image, camera intrinsics, depth scale/range, and camera-to-base extrinsics.
- Pose in a declared shared world frame, heading/IMU, covariance, and transform version.
- Current goal label or goal ID, robot health, odometry status, and local safety status.

### Outbound message per robot

- decision ID, source map version, target pose or frontier ID, and expiry time;
- optional discrete planner action only for a simulation adapter;
- an explicit `STOP/HOLD` state.  The robot-side controller must be able to reject an unsafe or stale command.

### Decisions now implemented

1. Robot-local ROS adapters use authenticated HTTP to a loopback-bound Hub
   reached through the existing SSH/VPN deployment path.
2. Every observation declares a versioned shared transform and calibration ID;
   cross-robot fusion refuses mismatches.
3. The Hub sends only high-level frontier/semantic-region targets. It never
   sends motor velocity.
4. Capture time, receive time, monotonic sequence, payload hash, cross-robot
   skew, health age and expiring leases are enforced fail-closed.

Transport v2, both robot receivers and feedback are implemented. Physical
scene completion remains open; implementation is not the same as an official
navigation pass.

## What cooperation is already available

The useful part of the upstream design is centralized cooperation, not networking: each agent builds a local semantic map; the driver takes an element-wise `torch.max` fusion; a shared frontier/object/history representation is constructed; and candidates are allocated sequentially so the second robot does not receive the first robot's chosen frontier.  There is no distributed runtime, inter-machine RPC, or explicit inter-robot collision avoidance in the upstream code.
