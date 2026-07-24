# Dual-robot live collision and corrective guard — 2026-07-25

## Scope and classification

This record documents the first observed concurrent physical execution of the
current WSJ/Yunji end-to-end stack, its collision termination, the paired
user-provided video evidence and the post-incident route-conflict guard.

Evidence labels used below:

- **observed:** directly present in a video, operator report, process/network
  check or robot/Hub event;
- **source-derived:** calculated by the versioned Hub/robot software from
  observed inputs;
- **unverified:** plausible but not established by the available evidence.

The run is an excluded engineering attempt. It is not an SR/SPL trial and must
not be relabelled as a success.

## Episode identity

| Field | Value |
| --- | --- |
| Session | `20260725-lab05-yunjireboot4` |
| Session Git commit | `cdcd7e70560f8bd782d83b5176bda6f5fca36780` |
| Session contract SHA-256 | `14897fc2133a8b36fe986b651848fef45dc1bce6a0d7eb2f0c357ef6e3cdbc20` |
| Calibration ID | `shared-board-odin1-20260725-lab05-yunjireboot1-v1` |
| Scene | `scene01-chair` |
| Episode | `scene01-chair-run01-fastfix` |
| Goal category | `chair` |
| Runtime directory | `hub/runtime/oneclick_20260725-lab05-yunjireboot4_live_scene01-chair_20260725_020253_667855552` |
| Controller outcome | `controller_error_TimeoutError` |
| Operator termination | `collision` |
| Metric status | excluded; `official_sr_spl_eligible=false` |

The runtime directory is intentionally ignored by Git. Its paths, sizes and
hashes are recorded below so the locally retained bytes can be verified.

## Timeline and observed behavior

1. Strict no-motion debug had passed on the same session commit.
2. The live runner froze synchronized robot inputs and the source-derived VLM
   selected different frontier IDs:
   - WSJ/`robot-0`: frontier `D`, target
     `(-1.2897033718, 4.9773252181)` in `shared_world`;
   - Yunji/`robot-1`: frontier `B`, target
     `(2.1102966282, 4.5773252181)` in `shared_world`.
3. The published decision listed both robots as active. Lease renewals 1–3
   continued while feedback was fresh.
4. The third-view recording directly shows the separated platforms drive
   toward the same central corridor and make physical contact at roughly the
   10-second point.
5. The paired Dashboard recording shows:
   - both live camera panels before contact;
   - short WSJ and Yunji trajectories converging at the lower part of the
     shared map;
   - a projected `chair` semantic region;
   - close-range occlusion of the WSJ camera after contact.
6. The operator reported: `两台机器都往前走了 但是路线上发生碰撞`.
7. The operator subsequently confirmed:
   `ROBOTS_STOPPED_AFTER_COLLISION`.
8. Hub feedback became missing and both high-level decisions were replaced by
   HOLD. The controller then exited through its error cleanup path.

The robot-reported physical-path seeds at termination were:

| Robot | Start local pose | Last local pose | Path length |
| --- | --- | --- | ---: |
| WSJ/`robot-0` | `(0.3433, -0.2018)`, yaw `1.9580` in `wsj/world` | `(0.2679, 0.6698)`, yaw `1.9580` | `1.4339869003 m` |
| Yunji/`robot-1` | `(-0.2542, 0.0737)`, yaw `-0.4751` in `yunji/world` | `(1.2151, -0.5111)`, yaw `-0.4751` | `1.5975705390 m` |

These lengths are source-derived robot navigation-event values. They are not
surveyed shortest paths and are not SPL inputs for this excluded run.

## Route interaction root cause

The source multi-agent coordinator allocates distinct frontier choices: Agent
0 selects first, its frontier is removed, and Agent 1 then selects from the
remaining candidates. The pre-incident physical adapter treated distinct
frontiers as sufficient for concurrent authority.

That assumption was false in this placement. Replaying the accepted
shared-frame starts and targets gives:

| Robot | Shared-frame start | Shared-frame target |
| --- | --- | --- |
| WSJ/`robot-0` | `(0.3362092841, -0.1997132964)` | `(-1.2897033718, 4.9773252181)` |
| Yunji/`robot-1` | `(-0.9426940507, -0.1337700123)` | `(2.1102966282, 4.5773252181)` |

The two straight start-to-target segments intersect, yielding a
source-derived minimum separation of `0.0 m`. Different goals therefore did
not imply separated routes.

The confirmed root cause is a missing inter-robot route-conflict gate before
concurrent high-level target publication. The videos confirm the physical
contact; the geometry replay explains why this exact candidate should not
have been concurrently authorized.

## Network event and causality boundary

Yunji's last Hub observation was at
`2026-07-25T02:03:29.009584754+08:00`. The Hub published the
feedback-missing HOLD at
`2026-07-25T02:03:33.965571124+08:00`. Immediately after the incident,
`10.209.85.41` did not answer ICMP or SSH.

Both the collision and Odin-host disconnection are observed. Whether the
collision caused the disconnection is **unverified**. A disturbed power
connector, cable, USB network adapter or antenna would make physical causation
plausible; an independent host/LAN outage remains plausible. Temporal
proximity alone is not treated as proof.

The host later restarted and its Odin driver returned. The new boot/tracking
epoch prevents reuse of the incident session for another live run.

## Corrective implementation

Commit `b79879bfc96805aa7e7b63cf3a8ebbfe59679730`
(`fix(runtime): serialize conflicting robot routes`) adds a real-world
execution guard without changing source VLM decisions.

Before publishing a round:

1. read each robot's frozen `shared_world` base pose;
2. preserve the original VLM batch as `vlm_candidate_batch.json`;
3. compare buffered straight start-to-target segments;
4. if minimum separation is below `0.9 m`, authorize one deterministic robot
   and HOLD the other;
5. if only one valid shared pose exists, authorize only that robot;
6. if no valid shared poses exist, HOLD both;
7. preserve the applied batch and full provenance as `initial_batch.json` and
   `route_conflict_guard.json`.

Replay of this incident produces:

```text
status: serialized_route_corridor_conflict
minimum_predicted_separation_m: 0.0
effective_active_robot_ids: [robot-0]
suppressed_robot_ids: [robot-1]
```

The implementation passed its focused tests and the complete Hub test suite
before this closeout. Its limitation is explicit: it uses conservative
straight-segment geometry and does not certify robot-local detours. It remains
physically unverified until a new calibrated session demonstrates one GOAL
plus one HOLD for a conflicting allocation.

## Runtime evidence manifest

All files below are locally retained, machine-generated runtime evidence:

| Relative path under the runtime directory | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `operator_incident.json` | 2,531 | `c52d901154b206f66f28e95727ff9a5bb0c2bc8395b177f651004edd4f491be9` | observed operator incident record |
| `episode_report.json` | 10,643 | `0276be56e218ec53951e7cc5b75014400c56b5f7399a3ce62882b7099984dbe0` | source-derived controller report |
| `scene_manifest.json` | 5,792 | `dab0a5ed1c9b89f7b388eac96d261e358d1dd8e2688545725048c926528c2666` | source-derived scene manifest |
| `scene_state.json` | 7,237 | `8140415023adc568af79bf644c16566679ec2dd4f3dc51682668a0154d6fab34` | source-derived persistent VLM state |
| `controller_events.jsonl` | 4,317 | `3216da9a5c48f9df367155853d58d27f1fbf7a329a6d621d22db062eab9c0d7a` | observed Hub navigation event log |
| `batch_001_round_0_goal.json` | 4,468 | `f3f607dbe18b80e44b8e6afbb4830764467914daff7cc06df556406d485d0f0a` | source-derived published goal batch |
| `batch_005_feedback_missing_hold.json` | 3,733 | `3ee782ccad9691385b4834509c325c6221d7c4fe05f9a95c8e2ed24a7c438050` | source-derived fail-closed HOLD |
| `batch_006_controller_error_hold.json` | 3,733 | `08c532d53d9b7edd2e2a009f24f6c76cc2eb45207a02a7377cf73411691f3f8d` | source-derived cleanup HOLD |
| `round_00_step_000/initial_batch.json` | 4,468 | `f3f607dbe18b80e44b8e6afbb4830764467914daff7cc06df556406d485d0f0a` | pre-fix applied VLM batch |

The repeated `f3f607...` hash proves that the pre-fix physical batch was the
unmodified two-active-robot VLM candidate.

## User-provided video provenance

The source masters were observed in the workspace on 2026-07-25. Capture
device metadata and clock synchronization were not independently verified.
Their content matches the reported physical event, but no timestamp was
inferred from container metadata.

| Asset | Bytes | Stream | Duration | SHA-256 |
| --- | ---: | --- | ---: | --- |
| `media/video/experiment_1_collision.mp4` | 1,640,232 | HEVC/AAC, 1280 × 720, nominal 30 fps | 12.366667 s | `1b883c310b176ed75587b5672d09a0ab14a604e214663f0c760835f1eb5ec659` |
| `media/video/experiment_1_collision_dashboard.mov` | 19,138,555 | H.264, 3420 × 1544, nominal 60 fps | 16.341667 s | `a429b838566efbd4769b1f613ba2530d1c0c972cc1cc685b72f278aac5ecffc9` |

Original masters remain ignored under `media/video/`. Public derivatives:

| Published asset | Bytes | SHA-256 |
| --- | ---: | --- |
| `media/demo/dual_robot_collision_third_view_20260725.mp4` | 2,481,165 | `8b62b5e763e529c8d2bf12caf5066d57187226de3117dc0fb3e7d7d0df3d395e` |
| `media/demo/dual_robot_collision_third_view_20260725_poster.jpg` | 199,630 | `cbb216abddf11e2089ba60afba38eac09d2b838654b8289af7f79ed72cf9417f` |
| `media/demo/dual_robot_collision_dashboard_20260725.mp4` | 284,926 | `fd8d5709fc1bf1d9568d9cf594cc37b578942ebc7b077f78307346b68780ea8f` |
| `media/demo/dual_robot_collision_dashboard_20260725_poster.jpg` | 75,948 | `cd93b77637204f85690921ea791aa0a1b4fc4fcbe95d9d52f7ea559924d80ae7` |

Derivatives were produced with the locally observed `ffmpeg`
`4.4.2-0ubuntu0.22.04.1`, H.264, `yuv420p`, fast-start MP4 and no audio.
Details are indexed in [`../media/demo/README.md`](../media/demo/README.md).

## Metric and safety conclusion

- Collision: **observed and confirmed**.
- Independent target success: **not present**.
- Semantic-region arrival: **not present**.
- Local planner stopped at a valid target: **not present**.
- Official SR/SPL eligibility: **false**.
- Post-incident operator stop: **confirmed**.
- Hub goal output after cleanup: **disabled for both robots**.
- Next motion authority: requires a fresh standing calibration/session,
  strict no-motion debug and a new onsite operator confirmation.
