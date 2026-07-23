# Transport contracts

> **Current status (2026-07-24):** v1 remains the observation-upload contract.
> The supervised-demo v2 decision/event extension is implemented and has been
> exercised through the real TinyNav/WATER paths under operator supervision.
> Those attempts failed closed and none is an official SR/SPL result. See
> [the canonical current status](../../CURRENT_STATUS.md),
> [the v2 contract](TRANSPORT_V2_DEMO_DRAFT.md), and
> [the physical evidence audit](../../audit/V2_ROBOT_RECEIVERS_20260723.md).

## V1 boundary

Robot-local ROS 2 remains responsible for camera synchronization, TinyNav pose production, local planning, obstacle avoidance, velocity control, controller priority and emergency stop. The hub receives selected observations and returns only high-level decisions.

The first transport is request/response HTTP over routed TCP. It is intentionally independent of ROS discovery. Production exposure requires TLS (reverse proxy, VPN, or equivalent) and per-robot credentials; the application defaults to `127.0.0.1`.

## Robot to hub

`POST /v1/robots/{robot_id}/observations` uses multipart form fields:

- `metadata_json`: UTF-8 JSON conforming to `ObservationMetadata`;
- `rgb`: JPEG or PNG bytes;
- `depth`: lossless 16-bit PNG, in aligned RGB pixel geometry;
- `X-Robot-Token`: the credential dedicated to that robot ID.

The hub verifies the declared byte lengths and SHA-256 values before accepting the frame. Accepted data is written to an append-only directory:

```text
runtime/spool/<robot_id>/<20-digit-sequence>/
  metadata.json
  rgb.jpg | rgb.png
  depth.png
```

No automatic deletion is performed. If the configured free-space reserve would be crossed, upload fails with HTTP 507.

Recommended initial rates:

| Stream | Rate | Rule |
|---|---:|---|
| aligned RGB-D + exact pose | 1–2 Hz | selected semantic/map keyframes only |
| health | carried on every frame | add a separate 2 Hz status channel before live execution |
| decision polling | 2 Hz | local timeout remains authoritative |
| TinyNav camera/SLAM/control | native local rate | never tunneled through the hub loop |

The current server accepts a capture age up to 3 seconds and future skew up to 250 ms. These are ingest limits, not a promise that a 3-second-old frame is suitable for control.

## Hub to robot

`GET /v1/robots/{robot_id}/decisions/latest` returns one versioned `Decision`:

- `GOAL`: target pose in `shared_world` plus optional frontier ID;
- `HOLD`: no target; stop/hold locally while retaining state;
- `STOP`: no target; robot must latch the stop until its local reset policy permits release.

All modes carry `decision_id`, `map_version`, `transform_version`, `issued_at_ns` and `expires_at_ns`. If no valid decision exists, the server synthesizes a one-second `HOLD`. The client must independently reject an expired response using its own synchronized clock; network loss must also become local HOLD/STOP.

The robot reports its disposition to:

`POST /v1/robots/{robot_id}/decisions/{decision_id}/ack`

Every publish (accepted or rejected) and every acknowledgement is durably
appended to `state_dir/decision_events.jsonl` with an fsync (2026-07-18), so
decision history survives a hub crash.

## Ordering, retry and resume

- `sequence` is monotonically increasing per robot and never resets silently.
- Retrying the same sequence is idempotent only if metadata and both payloads are identical.
- Reusing a sequence with different content or sending a lower sequence returns HTTP 409.
- Resume: `GET /v1/robots/{robot_id}/observations/latest` returns the last
  accepted sequence. A restarting sender continues from `last_sequence + 1`;
  a local counter file is only a fallback for a hub that is unreachable at
  startup. Wire-proven on the real robot (see `audit/TRANSPORT_WSJ_TEST.md`).
- Hub restart: per-robot `last_sequence`, payload digest and `map_version`
  are persisted in `state_dir/registry_state.json` and reloaded, so a
  restarted hub still rejects stale sequences. The last observation itself is
  deliberately not persisted: after a restart the hub refuses GOAL publishes
  until a fresh observation arrives (fail-closed).
- Sender retry policy (reference implementation `hub/robot_overlay/focus_sender.py`):
  exponential backoff 0.5 s doubling, 8 s cap, ≈39.5 s total budget, then a
  loud failure. Capture timestamps are re-stamped and payloads re-hashed
  across retry gaps so the hub's 3 s freshness window is honored.
- A deliberate sequence epoch reset (new `robot_id` session, wiped spool)
  still requires an explicit operator action: delete the robot's entry from
  `registry_state.json` while the hub is stopped, never silently.

## Payload selection

The v1 payload is aligned RGB plus aligned 16-bit depth because it is portable and retains enough information to replay RedNet mapping centrally. Raw infrared, IMU and full-rate odometry remain in the robot rosbag for diagnosis. A later map-delta message may reduce bandwidth, but it must not replace the replayable observation contract before G3/G4 are demonstrated.
