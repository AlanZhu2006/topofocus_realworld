# Transport contract v2 for the supervised demo

> **Status update (2026-07-24): IMPLEMENTED AND PHYSICALLY EXERCISED UNDER
> SUPERVISION; NO OFFICIAL SUCCESS.** This file preserves the approved
> contract and its original design rationale despite the historical `DRAFT`
> filename. The active implementation is in
> `hub/src/focus_hub/transport_v2.py`, `v2_registry.py` and the two
> robot-local v2 receivers. Real attempts reached TinyNav/WATER and returned
> fail-closed HOLD; none may be counted in SR/SPL. Current deployment IDs and
> remaining gates are in [CURRENT_STATUS.md](../../CURRENT_STATUS.md), with
> observed details in
> [V2_ROBOT_RECEIVERS_20260723.md](../../audit/V2_ROBOT_RECEIVERS_20260723.md).

## Purpose and fixed authority boundary

This draft closes the smallest transport gap needed for the four-scene,
five-trial supervised real-world demo. It preserves the upstream navigation
intent while adapting simulator actions to physical robots:

- the Hub chooses a versioned high-level exploration point or target-semantic
  region;
- the robot transforms that intent, chooses or rejects a locally navigable
  approach, plans around live obstacles and controls velocity;
- an expiring Hub lease can permit motion but can never suppress a local HOLD,
  STOP, emergency stop or operator takeover;
- a semantic detection is model inference, not proof of success; and
- official success still needs robot-local arrival/STOP plus independent
  target verification and no operator navigation intervention.

The immutable `source/` and `dependencies/` trees are not modified. Transport
v1 observations and heartbeats remain the input path for this demo; v2 adds
only the proposed high-level decision and navigation-event messages.

## Why v1 is insufficient

Transport v1 carries one `shared_world` pose and a one-shot acknowledgement.
That is sufficient for a frontier-point dry run, but it loses two facts that
are authoritative in the upstream method:

1. after `Find_Goal`, the upstream local planner consumes the full largest
   target-semantic component rather than its centroid; and
2. `COMPLETED` does not distinguish receipt, active navigation, local arrival,
   a latched stop, rejection, operator intervention or independent success
   verification.

V2 therefore uses a target union (`FRONTIER_POINT` or `SEMANTIC_REGION`) and an
append-only navigation-event stream. V1 routes and models stay unchanged.

## Implemented routes and authentication

These routes are implemented inside `hub/`; physical receiver activation is
still gated separately:

```text
GET  /v2/robots/{robot_id}/decisions/latest
POST /v2/robots/{robot_id}/navigation-events
POST /v2/admin/decision-batches
GET  /v2/admin/robots/{robot_id}/navigation-state
```

The robot routes use the existing per-robot `X-Robot-Token`; the atomic batch
route uses `X-Admin-Token`. Production exposure still
requires a routed private network plus TLS or an equivalent authenticated
tunnel. An event is authorized only for the robot named in both the URL and
payload. Hub decision and event records are append-only and fsync'd before a
successful response.

Decision polling and event heartbeats are 2 Hz. A robot must not depend on ROS
discovery across machines. `GET` returns `204 No Content` when no v2 decision
exists; that response never grants motion and the receiver remains in or
enters local HOLD. It must not fall back to a cached expired GOAL or to v1.

## High-level decision envelope

The proposed strict schema name is `focus-high-level-decision-v2`; unknown
fields, unknown enum values and non-finite numbers are rejected. Every message
contains:

| Field | Contract |
|---|---|
| `protocol_version` | Literal `2.0`. |
| `schema_version` | Literal `focus-high-level-decision-v2`. |
| `profile` | Literal `supervised_concurrent_demo_v1`. |
| `robot_id` | Exact authenticated receiver ID. |
| `scene_id`, `episode_id` | Pre-registered experiment identities. |
| `round_index`, `source_step` | Non-negative source logical clock; source steps are `0, 24, 49, ...`. |
| `decision_batch_id` | Shared by the two robot decisions produced from one frozen round. |
| `leg_id` | Stable while one physical high-level navigation leg is renewed. |
| `decision_id` | Unique for every lease, including renewal leases. |
| `lease_sequence` | Starts at zero and increases by exactly one within a `leg_id`. |
| `mode` | `GOAL`, `HOLD` or `STOP`. |
| `coordination` | Monotonic execution epoch and the robots allowed independent GOAL leases in this batch. |
| `goal_category` | One of `chair`, `bed`, `plant`, `toilet`, `tv`, `sofa`. |
| `input_observations` | Frozen capture sequence, timestamp and payload digest for both robots. |
| `map_provenance` | Exact source map identity and coordinate contract. |
| `issued_at_ns`, `expires_at_ns` | UTC Unix nanoseconds; a GOAL/HOLD lease is at most 10 s. |
| `target` | Required only for GOAL and forbidden for HOLD/STOP. |
| `reason` | Bounded operator-readable decision reason. |

The coordination block is:

```json
{
  "execution_epoch": 6,
  "active_robot_ids": ["robot-0", "robot-1"]
}
```

`active_robot_ids` is a duplicate-free subset of the two configured robot
IDs, so zero, one or both robots may be active. A GOAL is valid only when the
receiver is in that list; a HOLD receiver must not be in it. STOP is valid
regardless of the active set. Each robot owns an independent `leg_id`, lease,
local plan, rejection and STOP state; one robot's lease renewal must never
restart or extend the other robot's motion authority.

`input_observations` records, for each robot, `sequence`, `capture_time_ns` and
the accepted observation payload SHA-256. A referenced frame must still be in
the Hub's bounded in-memory accepted-observation history, must be at most 30 s
old when the batch is published, and the two capture times may differ by at
most 5 s. It does not have to remain the newest frame while VLM inference is
running; requiring that would make a continuously uploading sender invalidate
every frozen VLM input. Current health and command readiness are checked again
from the newest observation/heartbeat independently. `map_provenance` records:

```json
{
  "map_version": 17,
  "map_snapshot_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "map_format_version": "focus-hub-central-map-v3",
  "frame_id": "shared_world",
  "resolution_m": 0.05,
  "transform_version": "board-20260722-v1",
  "shared_frame_calibration_id": "board-20260722-v1"
}
```

The snapshot hash locks the source of the Hub decision. It does not claim that
the robot-local obstacle map has identical bytes. The receiver must match the
named transform and shared-frame calibration, verify that its own referenced
observation was actually uploaded, and reject a future or unknown sequence.
For this profile, `resolution_m` is exactly the source default 0.05 m and both
target kinds carry the literal `source_goal_dilation_cells=10`; another value
requires a different reviewed profile rather than silent rescaling.
Before any v2 GOAL, that v1 observation must have `mapping_only=false`, a
measured `base_T_camera`, fresh READY health and the same transform version.
The current mapping/shadow observations do not satisfy this motion gate.

## Target union

### Frontier point

Before target semantics are found, the source decision is a frontier/history
point. The target is:

```json
{
  "kind": "FRONTIER_POINT",
  "frontier_id": "frontier-3",
  "source_goal_dilation_cells": 10,
  "pose": {
    "frame_id": "shared_world",
    "x": 1.25,
    "y": -0.40,
    "z": 0.0,
    "yaw_rad": 0.52
  }
}
```

The point is high-level intent only. The upstream FMM planner expands its
single-cell goal with a radius-10-cell disk before planning. At the required
0.05 m source resolution, this gives a 0.50 m source arrival neighborhood; it
does not turn unknown/occupied cells into traversible cells. A receiver
transforms the point and this metric neighborhood into its own map frame,
intersects it with locally traversible cells, checks distance and reachability,
and may reject it. For a planar
`shared_T_robot_map` rotation `theta`, position uses the full inverse transform
and yaw uses:

```text
yaw_robot_map = wrap_to_pi(yaw_shared_world - theta)
```

Passing shared-frame yaw through unchanged is forbidden. This is a known gap
in the current Yunji dry-run guard and is an implementation gate, not an
accepted limitation.

### Semantic region

When the source `Find_Goal` rule fires, the full component is authoritative:

```json
{
  "kind": "SEMANTIC_REGION",
  "category": "chair",
  "source_robot_id": "robot-0",
  "evidence_status": "model_inference_map_projected_unverified",
  "source_goal_dilation_cells": 10,
  "region": {
    "frame_id": "shared_world",
    "origin_xy_m": [-12.0, -12.0],
    "resolution_m": 0.05,
    "height": 480,
    "width": 480,
    "row_axis": "+y",
    "column_axis": "+x",
    "encoding": "png_u8_0_255_base64",
    "component_size_cells": 83,
    "payload_size_bytes": 612,
    "payload_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
    "payload_base64": "<base64 omitted from this example>"
  },
  "display_centroid": {
    "frame_id": "shared_world",
    "x": 0.825,
    "y": 1.175,
    "authority": "display_only"
  }
}
```

The decoded PNG must be a single-channel 2-D `uint8` image of exactly
`height x width`, with values only 0 and 255. The hash and byte size describe
the decoded PNG bytes, not the base64 text. The receiver must reject a payload
larger than 1 MiB, a side longer than 2048 cells, an empty mask, a non-binary
mask, a count other than `component_size_cells`, or any hash/shape/grid
mismatch. Inline PNG keeps the decision atomic and avoids a second artifact
download during a short lease.

Cell `(row, column)` has its centre at:

```text
x = origin_xy_m[0] + (column + 0.5) * resolution_m
y = origin_xy_m[1] + (row    + 0.5) * resolution_m
```

The mask is generated from the exact snapshot named by
`map_snapshot_sha256` using these source-derived rules:

- threshold the requested semantic channel at `> 0`;
- for `tv` only, apply one 7 x 7 binary dilation first;
- select the largest 8-connected component; a size tie selects the first
  row-major connected-component label; and
- do not replace the component with its centroid. The centroid above exists
  only for Foxglove display.

Separately, the source FMM planner expands this authoritative component with a
radius-10-cell disk before path planning. With the transmitted 0.05 m grid
that is a 0.50 m arrival neighborhood. Those added cells are navigation
candidates only: they are not target-semantic evidence and must never be
painted back into the semantic map as detections.

For the first supervised demo, a semantic region may be assigned only to its
`source_robot_id`. This matches the source behavior in which the agent whose
semantic map fires `Find_Goal` follows that goal, and avoids silently turning
another robot's model inference into ground truth.

The physical receiver keeps final navigation authority. It must transform the
entire region to its local map and either:

1. give the full region to a local region-capable planner; or
2. derive a collision-free observation/approach pose from the region and live
   obstacle map for a point-goal API such as the TinyNav POI used by both
   deployed robots.

The second path is an explicitly recorded real-world adapter because the
current robot APIs accept points. The chosen local pose, algorithm/version,
region hash, exact metric arrival radius and rejection checks are written in
the `ACCEPTED` event. The adapter first intersects the source-derived arrival
neighborhood with its live traversible map and chooses a reachable candidate;
it must not drive to an occupied semantic cell merely because that cell is in
the mask. The Hub
must not preselect a collision-free approach or command velocity. Failure to
find a locally reachable approach is `REJECTED_UNREACHABLE`.

## Lease, ordering and loss behavior

A GOAL or HOLD lease lasts no more than 10 seconds. Normal Hub operation
renews an active leg before expiry only when the newest event from that robot
is no older than 2 seconds and reports neither rejection, stop, emergency stop
nor operator intervention.

- A renewal has the same `leg_id`, byte-identical target and provenance, a new
  `decision_id`, and `lease_sequence + 1`. It extends permission; it must not
  restart the local plan or reset path-length accounting.
- Any target, category, map, transform, calibration, episode or batch change
  starts a new `leg_id` with lease sequence zero.
- A duplicate `decision_id` is idempotent only if every byte is identical.
- A skipped, repeated-with-different-content or decreasing lease sequence is
  rejected and causes local HOLD.
- If polling, authentication or time synchronization fails, or no valid
  renewal exists at expiry, the robot brings commanded velocity to zero using
  its local HOLD policy. A later valid renewal may resume only if local policy
  permits it.
- A GOAL is never valid merely because it was once accepted.

Lease validation requires UTC clock uncertainty at or below 250 ms. After
validating `expires_at_ns` once, the receiver converts the remaining interval
to a monotonic local deadline so a wall-clock adjustment cannot extend motion.

STOP is different: after authentication and robot-ID/schema validation, it is
safe to honor even if its coordinate metadata or lease is stale because it
cannot authorize motion. Once accepted, STOP is locally latched and cannot be
released by a Hub GOAL/HOLD; only the authenticated local operator reset path
may release it.

## Concurrent two-robot coordination

Every source round produces one `decision_batch_id` and one decision per
robot. The batch may contain two GOALs, one GOAL plus one HOLD, or only
HOLD/STOP. This matches the upstream execution shape: it computes every
agent's action and passes the complete action array into one environment step.

GOAL leases remain independent even when their issue times and batch IDs are
shared. A rejection, expiry, arrival or local stop on one robot affects only
that robot unless the Hub emits an explicit scene-level HOLD/STOP batch. Once
one robot reaches and independently verifies the target, the Hub ends the
episode by issuing HOLD or STOP to both according to the local reset policy.
If one robot reports `ARRIVED` while the other is still navigating, the next
atomic pair gives the arrived robot a new HOLD leg and renews the other
robot's existing GOAL leg. Changing only the coordination active set for this
reason does not restart or otherwise alter the still-active robot's target.

The Hub may use target separation and path-overlap estimates when allocating
goals, but these are coordination hints rather than collision certification.
Each robot's local obstacle/cost map, planner, controller, emergency stop and
nearby operator remain authoritative. The other robot must be treated as a
dynamic obstacle locally; if either local stack cannot do so, concurrent
motion is rejected for that run profile rather than replaced with Hub
`cmd_vel` control.

## Navigation event stream

The proposed strict schema is `focus-navigation-event-v2`. `event_id` makes a
retry idempotent only when all bytes match. Events are append-only and include:

```json
{
  "protocol_version": "2.0",
  "schema_version": "focus-navigation-event-v2",
  "robot_id": "robot-0",
  "scene_id": "scene-01",
  "episode_id": "scene-01-trial-01",
  "decision_batch_id": "batch-0003",
  "leg_id": "leg-robot-0-0002",
  "decision_id": "lease-robot-0-0002-0004",
  "lease_sequence": 4,
  "event_id": "robot-0-event-000018",
  "status": "NAVIGATING",
  "reason_code": "LOCAL_PLANNER_ACTIVE",
  "observed_at_ns": 1784772000000000000,
  "local_pose": {
    "frame_id": "wsj/map",
    "x": 0.71,
    "y": -0.24,
    "yaw_rad": 0.12
  },
  "path_length_m_from_episode_start": 1.84,
  "velocity_zero_confirmed": false,
  "terminal_observation_sequence": null,
  "resolved_local_goal": null,
  "detail": ""
}
```

Allowed `status` values and meanings are:

| Status | Meaning |
|---|---|
| `RECEIVED` | Parsed and authenticated; no motion authority accepted yet. |
| `ACCEPTED` | All local gates passed. Includes the resolved local goal and adapter version for GOAL. |
| `NAVIGATING` | Local planner is active under an unexpired lease. |
| `ARRIVED` | Local planner reports arrival and zero velocity is confirmed. This is not independent success verification. |
| `HOLDING` | HOLD/expiry/network-loss handling has brought commanded velocity to zero. |
| `STOPPED` | A latching STOP is active and zero velocity is confirmed; it does not mean target arrival. |
| `REJECTED` | No motion was authorized; `reason_code` names the gate. |
| `OPERATOR_INTERVENTION` | A person issued navigation control/takeover; the autonomous episode scores zero. |
| `LOCAL_ESTOP` | Local emergency stop engaged; the episode terminates as a failure. |

At least `RECEIVED` followed by `ACCEPTED` or `REJECTED` is emitted for each
new decision. While navigating, an event is emitted at 2 Hz; unchanged values
may be heartbeat events. `ARRIVED`, `HOLDING`, `STOPPED`, `REJECTED`,
`OPERATOR_INTERVENTION` and `LOCAL_ESTOP` are emitted immediately.

Required rejection reason codes are:

```text
PROTOCOL_MISMATCH       ROBOT_ID_MISMATCH       AUTHENTICATION_FAILED
EXPIRED                 OUT_OF_ORDER            MAP_VERSION_REGRESSION
UNKNOWN_OBSERVATION     MAP_ARTIFACT_MISMATCH   TRANSFORM_MISMATCH
CALIBRATION_MISMATCH    REGION_ARTIFACT_INVALID HEALTH_NOT_READY
DISTANCE_LIMIT          UNREACHABLE             LOCAL_PLANNER_REJECTED
LOCAL_STOP_LATCHED      UNSAFE
```

The receiver may add a bounded detail string but must not change a rejection
into acceptance based on Hub intent.

`resolved_local_goal` is null except on semantic-region GOAL acceptance. It
records its local frame/pose, region payload SHA-256 and the exact adapter
name/version, for example:

```json
{
  "frame_id": "wsj/map",
  "x": 0.92,
  "y": -0.11,
  "yaw_rad": 1.34,
  "source_region_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
  "arrival_radius_m": 0.50,
  "adapter_name": "tinynav-semantic-approach",
  "adapter_version": "1"
}
```

`path_length_m_from_episode_start` is monotonic and does not
reset on lease renewals or high-level legs. The terminal event references the
latest accepted RGB-D observation sequence so the Hub can preserve the image
that was actually available at termination.

## Verification and official SR/SPL

No robot event has status `VERIFIED`: a robot must not certify its own semantic
success. After `ARRIVED`, an evaluator writes the existing
`focus-realworld-demo-results-v1` episode record with:

- robot-local planner STOP/arrival;
- final pose inside the pre-surveyed valid stopping region;
- independent target image/operator annotation;
- intervention state;
- cumulative path length and trajectory artifact; and
- workspace-relative path, byte size, SHA-256 and provenance classification
  for trajectory, survey and terminal evidence.

`Find_Goal`, `ACCEPTED` and `ARRIVED` alone never count as success. Official
autonomous success requires every condition in
`TRIPLE_AI_REALWORLD_DEMO.md`; any operator steering/takeover marks
`OPERATOR_INTERVENTION` and scores that episode zero. Assisted demonstrations
may be reported separately but must not be mixed into autonomous SR/SPL.

## Fail-closed receiver checks

Before any GOAL reaches a local planner, the receiver checks, in order:

1. TLS/tunnel and per-robot authentication;
2. schema/protocol, robot, experiment and target-category allowlists;
3. unique ID, lease ordering, local synchronized time and expiry;
4. local health, emergency stop and STOP latch;
5. own uploaded observation identity and monotonic Hub map version;
6. exact transform version and shared-frame calibration ID;
7. target union, finite values, payload bounds, size/hash/shape and grid rules;
8. correct full position and yaw coordinate conversion;
9. local distance, occupancy, reachable-approach and planner checks; and
10. membership in the current concurrent active-robot set.

Any uncertainty becomes HOLD/REJECTED. STOP bypasses coordinate/health gates
only in the safe direction: it may latch a stop but never release one.

## Approval and activation gates

Approval of this document authorizes implementation in `hub/`; it does not by
itself authorize robot motion. The following gates remain separate and must be
recorded before changing `allow_goal`:

- **Contract approval:** passed on 2026-07-23 for the concurrent profile;
  implementation is authorized, physical motion is not.
- **Schema/API tests:** strict parsing, idempotency, durable event logging,
  expiry, renewal and two independent simultaneous GOAL leases pass locally.
- **Receiver tests:** both receivers default to dry-run and their explicit live
  paths remain physically unverified; mask corruption, transform/calibration
  mismatch, stale lease, STOP latch, command bypass and unreachable region
  fail closed locally.
- **Observation gate:** each robot proves a fresh v1
  `mapping_only=false` observation with measured body-to-camera extrinsics and
  READY local health; no mapping-only frame may authorize motion.
- **Coordinate test:** non-identity transforms prove both position and yaw;
  the current Yunji shared-yaw pass-through is fixed.
- **Robot-local no-motion test:** accepted decisions reach only a logging/mock
  adapter while both robots are physically inhibited.
- **One-robot supervised crawl:** one short, clear frontier target at minimum
  configured speed with the other robot HOLD and an operator beside it.
- **Two-robot supervised crawl:** after each local chain passes separately,
  issue two short, separated goals and prove independent renewal, rejection,
  dynamic-obstacle response and STOP feedback while the operator is present.
- **Semantic-region crawl:** one independently visible source goal proves the
  full-region adapter, local STOP and evidence capture.
- **Demo enable:** enable only the named episode/profile and restore
  `allow_goal=false` after collection.

Until every preceding gate is recorded, the authoritative runtime state is
still shadow-only.

## Provenance used to draft this contract

The hashes below identify the exact local inputs read on 2026-07-23. “Source-
derived” means the contract restates behavior visible in immutable upstream
code. “Observed” means behavior or a gap visible in the current deployment
implementation. “Unverified” means the proposed v2 behavior has not run on a
physical robot.

| Classification | Path | Bytes | SHA-256 | Use |
|---|---|---:|---|---|
| source-derived | `source/Focus_realworld/main.py` | 103808 | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` | multi-agent decision/metric flow |
| source-derived | `source/Focus_realworld/agents/vlm_agents.py` | 46500 | `992f0174d50b6959d538a418c224907156f784ffd4b35b5ef67c02da3461bee0` | `Find_Goal`, largest component and local STOP semantics |
| source-derived | `source/Focus_realworld/tasks/multi_objectnav_hm3d.yaml` | 1386 | `b4dd539bd886cd6b17c794b04fceda705577c08c684965e30ba46066c5f0c498` | source action/goal task contract |
| source-derived | `source/Focus_realworld/arguments.py` | 14140 | `66dc9a94459215d9a51d97bf8f195fd486759d7f34529c60e2a57999665a61d3` | 5 cm map resolution and 25-step decision cadence |
| observed | `hub/src/focus_hub/models.py` | 9522 | `39231a84b3d0b3f8ddc42af722503512596efb233e1be241acf0fb432c71a241` | current v1 decision/ACK limits |
| source-derived, unverified on robot | `hub/src/focus_hub/source_episode.py` | 16567 | `e5bed073f936cf05a0e884dd0107b537ce3940f316b5f2701f8cda18b8aa3a06` | current source-compatible semantic-region port |
| observed | `hub/src/focus_hub/map_snapshot.py` | 5014 | `548bbbc8cb679b2b106617cb26bfbe42c25a8d7d6aaf4346f12b26307f4b5aba` | current map/frame/calibration validation |
| observed dry-run | `hub/src/focus_hub/goal_guard.py` | 4832 | `67d69da373018202ef67c2bdc94c222aa58f4187f6e928fb451d09741a22fbd1` | WSJ point-goal guard boundary |
| observed dry-run | `hub/src/focus_hub/yunji_goal_guard.py` | 6339 | `2fc1c183af265f69e40101b2c39ad6e4f553c8aec895c3f09dd4d2f4737c95ea` | Yunji point-goal guard and yaw gap |
| source-derived experiment rule, unverified on robot | `hub/docs/TRIPLE_AI_REALWORLD_DEMO.md` | 6007 | `e675225cbff5c7c2f37b0162ce4d70332bbd916404119d086ea6bc6799c9c92a` | physical episode and success definition |
| source-derived scorer, unverified on a completed run | `hub/src/focus_hub/realworld_eval.py` | 13217 | `65738c3ed1a5c62a1313c16fca1f722cf0452c923d3e44797fba63eea29f2576` | official evidence and SR/SPL accounting |

All v2 message examples, lease timings, region transport, sequential handoff
and event semantics in this document are proposals and therefore unverified.
