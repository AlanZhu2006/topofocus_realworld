# Yunji wall-turn frontier rejection, 2026-07-25

## Scope and classification

This record covers Scene 01 episode `trial-reanchor1-r2` under session
`lab-20260725-132014-wsjreanchor1-trial2`. It is an engineering execution
failure, not an SR/SPL sample and not evidence that the chair VLM decision
failed.

- Robot motion, poses, controller events and the frozen maps below are
  **observed**.
- The clearance replay and causal reconstruction are **source-derived** from
  those immutable artifacts and the deployed TinyNav controller.
- The corrective behavior is locally regression-tested but remains
  **physically unverified** until a new supervised episode is run.

## Preserved evidence

| Artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_lab-20260725-132014-wsjreanchor1-trial2_live_scene01_20260725_140550_667135363/episode_report.json` | 9,748 B | `37137a054e9799e534bb38ac8534225bd401907b302b20c9bb7b2f649b1679e2` | source-derived episode report over observed feedback |
| `hub/runtime/oneclick_lab-20260725-132014-wsjreanchor1-trial2_live_scene01_20260725_140550_667135363/controller_events.jsonl` | 15,912 B | `31312f20fd862052fc49fdecea39a347240e9bf9615b9d33241725fd15071a4c` | source-derived Hub controller log |
| `hub/runtime/oneclick_lab-20260725-132014-wsjreanchor1-trial2_live_scene01_20260725_140550_667135363/round_01_step_024/vlm_candidate_batch.json` | 6,465 B | `0758c5b3d570ebfed35b5214c6692758ca43322673b49604ca20a05e144f7b74` | unmodified source/VLM candidate |
| `hub/runtime/oneclick_lab-20260725-132014-wsjreanchor1-trial2_live_scene01_20260725_140550_667135363/round_01_step_024/shadow/fused_decision_map.npz` | 23,317 B | `e4c3863d15e828adc0e0f09e36d38cd08ca3ff0df7b0ea6bc1078bb3dd3d2ba0` | frozen source-derived shared map |
| `/home/nyu/.local/state/topofocus/yunji-v2-tinynav-live-20260725T060508Z.jsonl` on Yunji | 50,368 B | `0899be4efff13f0117b911e384103f60384412359ad7efa91e3498e5fb47d211` | observed robot-local receiver/controller telemetry |

The runtime directory is intentionally not committed to Git; this record
preserves its exact absolute workspace identity and hashes.

## What happened

Round 1 correctly retained two different high-level targets:

- WSJ: a `chair` semantic-region leg;
- Yunji: frontier `C` at shared-world
  `(-2.1333860812641543, 2.917351882976611)`.

The route-conflict guard reported `1.886669 m` predicted inter-route
separation, so this was not a recurrence of the earlier dual-robot crossing
collision. Yunji initially moved forward from approximately
`(-0.225, 0.072)` to `(0.943, 0.020)` in its local map. Its accumulated path
was `1.329044 m`.

At local pose about `(0.904, 0.072, -0.005 rad)`, the published path changed
from a forward segment to:

```text
first     = (0.896272, 0.076175)
lookahead = (0.696292, 0.077182)
```

The selected control segment therefore pointed backward in the robot forward
axis. The pinned controller forbids negative linear velocity, but its existing
turn rule rotated toward that backward segment. Telemetry then records yaw
changing from approximately `0` to `1.972 rad` while forward velocity fell to
zero. The fixed target remained unchanged. After `20.015 s` without at least
`0.05 m` progress, the receiver rejected the leg as
`LOCAL_PLANNER_NO_PROGRESS`, with `1.950 m` remaining.

## Frozen-map replay

The new footprint-clearance adapter was replayed against the exact candidate
and fused map above, without contacting either robot. For Yunji frontier `C`:

| Check | Result |
| --- | ---: |
| Required robot-centre clearance | `0.34 m` |
| Source arrival radius | `0.50 m` |
| Clearance at selected frontier cell | `0.10 m` |
| Safe approach cells inside arrival disk | `0` |
| Nearest `0.34 m`-clear known-free cell | `0.85 m` |

The replay therefore changes only Yunji from `GOAL` to `HOLD`; WSJ's chair
semantic leg remains `GOAL`. The original VLM batch is not edited.

## Corrective implementation

Three bounded deployment changes were made under `hub/`:

1. Before route-conflict handling, a frozen-map guard requires a
   footprint-clear known-free approach cell inside each source frontier's
   unchanged 10-cell arrival disk. It writes
   `frontier_clearance_guard.json`.
2. The deployment controller now computes the same robot-relative lookahead
   X component as pinned TinyNav. A negative component immediately zeros raw
   output and publishes `/planning/reverse_required`; Yunji's v2 receiver
   latches that status for the current authority and reports
   `LOCAL_PATH_REVERSE_REQUIRED`.
3. Explicit robot-local frontier feasibility rejections are isolated. The
   rejected robot moves to HOLD and a surviving peer keeps its existing leg;
   terminal transform, localization, e-stop, semantic and protocol failures
   still stop the coordinated episode.

No file under `source/` or `dependencies/` was changed. The complete Hub test
suite passes: `453 passed` (one third-party deprecation warning).

## Post-reboot state

After the operator power-cycled Yunji, a read-only check observed:

- host uptime about 12 minutes;
- `focus-yunji-odin1-driver.service`: `active`;
- Yunji TinyNav, v2 live receiver and WATER live bridge: `inactive`;
- WSJ v2 live receiver and Go2 live bridge: `inactive`;
- no matching manual teleoperation process.

No new motion confirmation is retained. Because the Odin tracking process
restarted, the next live attempt must first establish a valid current tracking
epoch (stationary re-anchor only if the physical stationary assumption can be
proved; otherwise a fresh board calibration), then run the no-motion check for
the new committed deployment.
