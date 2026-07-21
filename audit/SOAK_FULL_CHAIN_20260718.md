# Long-duration cross-machine full-chain soak (dry-run)

Date: 2026-07-18. Duration: 45.8 min continuous. All components loopback/tunnel
only, per-run random tokens, `allow_goal=false` throughout; nothing could move
a robot.

## Topology (five components, two machines, all live simultaneously)

```
wsj (Jetson): focus_sender.py --loop 18 @2 Hz  -> SSH tunnel 18091
                                               -> hub API :8288 (auth ingest -> spool)
local (4090): hub_pipeline_daemon (incremental RedNet map from spool;
              every 60 s: frontiers -> GLM-4V :31511 -> HOLD publish)
wsj (Jetson): receiver_dryrun.py polling decisions @0.5 Hz, envelope checks, acks
```

## Observed results

Sender (`metrics_soak.json`, archived on wsj):
- **5454/5454 frames accepted, 0 retries, 0 duplicates**, 1903 MiB payload,
  exact 2 Hz cadence over 2746.8 s; mean align 69.4 ms / encode 30.0 ms /
  upload 33.6 ms per frame — no drift versus the short-run numbers.

Hub daemon (`daemon.jsonl`, `map_out/soak_summary.json`):
- 5454/5454 frames mapped incrementally, **64.7 ms mean / 67.8 ms p95** per
  frame (RedNet + fusion) — hub keeps up with 2 Hz at ~13 % duty cycle;
- **47/47 decision cycles published (HTTP 202)**, GLM-4V ≈2.40 s per decision,
  stable frontier choice (D) with consistent probabilities across 45 min;
- memory flat: RSS 1475 MiB @frame 400 → 1479 MiB @frame 5454 (**+0.3 %**,
  no leak); GPU steady at 15,988 MiB total (GLM 13.8 GiB + RedNet + daemon).

Receiver (`receiver_dryrun.jsonl` on wsj):
- 729 decision transitions observed, **47/47 soak decisions received and
  acked**, every action HOLD (fail-closed posture held for the whole soak);
  the remainder are the hub's 1 s fallback-HOLD envelopes between cycles.

Decision durability: every publish and ack of the soak is fsync-logged in
`soak_20260718/state/decision_events.jsonl` (new persistence layer).

## Interpretation for deployment

- End-to-end capacity: one robot at 2 Hz consumes ~13 % of hub mapping duty
  and ~0.7 MB/s network; two robots fit with wide margin. The VLM remains the
  only multi-second stage (2.4 s), confirming the 3–4 s decision-cycle budget
  in `AL.md` §3.3.
- 45 minutes of continuous operation with zero transport errors, zero memory
  growth and deterministic VLM outputs is necessary-but-not-sufficient
  evidence for a live deployment; the same loop must next run from live ROS
  topics (sender port) before G5 work.

## Artifacts

`data/robot_replays/soak_20260718/` (spool 2.0 GiB, daemon.jsonl, glm/hub
logs, final map npz, soak_summary.json); sender/receiver logs and metrics on
wsj under `/home/nvidia/focus_sender/`.

Post-conditions: sender/receiver/proxy stopped on wsj, token removed, tunnel
closed and confirmed dead from the robot; hub, GLM stopped locally, ports
free, GPU back to the 976 MiB unrelated baseline.
