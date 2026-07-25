# Real-world failure attribution protocol — 2026-07-25

## Purpose

“The robot did not reach the chair” is an outcome, not a root-cause label.
Every non-success must be classified from synchronized runtime evidence before
it is described as a VLM failure.

The project reports two separate views:

1. **SR/SPL experiment track:** episodes that passed the declared sensor,
   calibration, mapping, transport and controller eligibility gates;
2. **engineering reliability log:** preflight aborts, hardware/network faults,
   safety stops, route conflicts and unclassified attempts.

This separation prevents engineering faults from being misreported as model
reasoning failures while still preserving them as deployment evidence.

## Attribution order

Classify in this order and stop at the first supported cause:

| Class | Required evidence | SR/SPL handling |
| --- | --- | --- |
| `preflight_engineering_failure` | failure before live: invalid/stale sensor, calibration, fusion, map, localization, code/version or health gate | not an episode; exclude |
| `execution_engineering_failure` | live began, but collision, battery/hardware, network/transport, lease, local planner/controller or safety stop prevented faithful target execution | record in engineering reliability; do not call it VLM failure |
| `perception_failure` | engineering chain remained healthy, but goal-relevant YOLO/SegFormer evidence was a verified false positive/negative | eligible algorithm failure when the trial protocol was valid; subtype separately from VLM reasoning |
| `vlm_decision_failure` | engineering and perception inputs were valid, issued targets were faithfully executed, and Perception/Judgment/Decision VLM still selected the wrong direction/history/frontier until the declared budget ended | eligible algorithm failure; `SR=0`, `SPL=0` |
| `navigation_policy_failure` | valid high-level target, healthy transport and localization, but the robot-local navigation policy could not reach it without an external infrastructure fault | report separately; metric treatment follows the predeclared end-to-end protocol |
| `unclassified_failure` | media or onsite description exists, but exact episode, decision batches and controller evidence are missing | retain and exclude until evidence is bound |

No attempt may be promoted from `unclassified_failure` to a model or
engineering subtype from a filename or video alone.

## Minimum evidence for a true VLM decision failure

All of the following must be available:

- session ID, Git commit, calibration ID and transform versions;
- fresh synchronized RGB-D/pose and accepted per-robot maps;
- no calibration/fusion or localization contradiction;
- saved Perception, Judgment/FN and Decision outputs for every round;
- exact high-level targets and lease history;
- robot-local acceptance, planning and navigation-event logs showing faithful
  execution;
- an independently established target presence/location;
- a predeclared step/time/distance budget that was exhausted without success;
- no collision, hardware, battery, network, operator intervention or safety
  abort that explains the outcome first.

If YOLO/SegFormer supplied bad object evidence, the failure is perception,
even if the downstream VLM acted consistently with that bad input. If the
VLM selected a poor frontier despite valid object/context evidence, it is a
VLM decision failure.

## Current Scene 01 classification

| Record | Current attribution | Reason | Metric status |
| --- | --- | --- | --- |
| `approach_failure_1` media pair | `unclassified_failure` | no exact runtime episode/control binding | excluded pending evidence |
| `approach_failure_2` media pair | `unclassified_failure` | no exact runtime episode/control binding | excluded pending evidence |
| `scene01-chair-run01-fastfix` collision | `execution_engineering_failure/route_coordination` | different frontiers produced intersecting physical routes before the route-conflict guard existed | excluded from current SR/SPL track; retained in engineering reliability |
| `trial-05-nearwall-fix` | success; earlier terminal-classification engineering mismatch | stopped `0.321133 m` from goal while old receiver used `0.15 m`; passes declared `0.5 m` protocol | `SR=1`, source-compatible `SPL=0.864048038008398` |
| `trial-r5-01` | normal automatic success | WSJ emitted `LOCAL_PLANNER_ARRIVED` at `0.406693 m` | `SR=1`, source-compatible `SPL=0.628398923` |
| r6 fused-map preflight | `preflight_engineering_failure/calibration_fusion` | cross-map wall conflict before live | not an episode; excluded |

At this checkpoint there is **no evidence-backed Scene 01 VLM decision
failure**. This statement does not assert that the VLM is correct; it says the
available failures do not meet the evidence threshold for that attribution.

This protocol changes no file under immutable `source/` or `dependencies/`.
