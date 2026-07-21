# Single-robot end-to-end chain rehearsal (dry-run)

Date: 2026-07-18 (Asia/Shanghai). Target: `asus4090`, `hub/.venv`.

Not a numbered gate: G4/G5 remain open. This run proves that every link of the
single-robot chain is implemented, runnable and fail-closed on this machine,
using only recorded real robot data and loopback services.

## Chain exercised (all over the real wire protocol)

```text
TinyNav record (real wsj data, 303 keyframes, stride 3 -> 101 frames)
  -> replay sender: depth aligned infra1->RGB frame (z-buffer reprojection),
     JPEG RGB + lossless PNG16 depth, strict metadata, SHA-256, re-stamped
     capture times, authenticated multipart HTTP
  -> hub FastAPI ingest (token, hash, size, sequence, clock, transform checks)
  -> append-only spool
  -> RedNet central semantic BEV map built from the spooled bytes
  -> frontier extraction (free/unknown boundary clusters, up to 4 candidates)
  -> GLM-4V choice over the annotated BEV (offline server, temperature 0,
     one token, string probabilities)
  -> decision publish via /v1/admin/decisions
  -> robot-side fetch via /v1/robots/robot-0/decisions/latest
  -> GoalGuard evaluation -> dry-run TinyNav POI JSON artifact -> ack POST
```

## Command

```bash
hub/.venv/bin/python hub/tools/e2e_single_robot.py \
  --record data/robot_replays/wsj_semantic_map_record_20260717_102052 \
  --extracted data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted \
  --output data/robot_replays/e2e_single_20260718 --stride 3
```

Exit 0. The runner starts and stops its own hub instances (127.0.0.1 only,
per-run random tokens) and the GLM server (15/15 shards, offline); afterwards
all ports were free and GPU memory returned to the pre-existing 976 MiB
baseline. Artifacts: `data/robot_replays/e2e_single_20260718/` (per-lane hub
and sender logs, spool, fused map npz, annotated BEV, POI JSON,
`e2e_manifest.json`).

## Observed results

Both lanes: 101/101 observations accepted (0 duplicates), identical fused maps
(19,504 explored / 14,763 obstacle cells at 5 cm), identical four frontier
candidates, and the identical GLM-4V decision — frontier **D**, string
probabilities A 0.097 / B 0.181 / C 0.232 / D 0.491, deterministic across
lanes.

Safety lane (default policy: `allow_goal=false`, `mapping_only=true` uploads):

- `POST /v1/admin/decisions` with the GOAL was **rejected 409**
  ("GOAL output is disabled for this robot") — the hub-side gate held.
- A HOLD carrying the would-be choice was published (202); the receiver
  fetched it, `GoalGuard` returned HOLD, POI artifact `{}` and the ack was
  accepted. Fail-closed end to end.

Rehearsal lane (explicit TEST policy on a second loopback hub:
`allow_goal=true`, `transform_version="e2e-test-v1"`, placeholder
`base_T_camera`, READY health, plus a <3 s heartbeat re-upload before the
publish to satisfy the registry's freshness rule):

- GOAL publish accepted 202; receiver fetched it; `GoalGuard` validated
  ID/expiry/order/map/transform/health/distance and emitted the dry-run
  legacy POI JSON with the full envelope
  (`decision_id`, `map_version`, `transform_version`, `expires_at_ns`).
- The POI JSON is a file artifact; nothing was sent to any robot.

## Recorded limitations

- `map_version` stayed 0: the pipeline does not yet advance the registry's
  map version over HTTP (needed before G4-fusion decisions matter).
- The rehearsal `base_T_camera` is an explicitly labelled placeholder, not a
  measured calibration; it exists to exercise the wire path only.
- The GLM prompt described the robot marker as orange but this run rendered it
  blue (BGR ordering slip, fixed after the run); GLM still returned a valid
  lettered choice with sensible probabilities.
- Semantic channels remain sparse on this scene (RedNet MP3D domain gap).
- `shared_T_robot_map` in the guard was identity: with one robot the shared
  frame is that robot's world. Real multi-robot operation requires the G4
  calibrated transform.

## What this does and does not prove

Proves: every module of the single-robot loop exists, runs locally, talks the
authenticated wire protocol, is deterministic on the same input, and the
default configuration refuses GOAL output at the hub while the robot-side
guard independently validates whatever it receives.

Does not prove: live network transport from the physical robot (the ROS
sender/receiver still need to run on `wsj`), two-robot shared-frame fusion
(G4) or hardware-in-loop safety (G5). `allow_goal` stays `false` in the
committed configuration.
