# WSJ semantic-arrival stabilization — 2026-07-25

## Scope

This engineering record follows Scene 01 formal experiment 05. The operator
confirmed that WSJ physically reached the chair, so the episode remains a
formal success. This record covers only the post-run arrival-behavior analysis
and minimal implementation fix.

## Observed behavior and attribution

- WSJ (`robot-0`) completed the chair semantic leg and emitted
  `LOCAL_PLANNER_ARRIVED`.
- Yunji (`robot-1`) remained in Hub-issued HOLD. In round 0 the predicted
  route separation was `0.666262 m < 0.9 m`; in round 1 the routes
  intersected. The coordinator therefore selected WSJ and held Yunji.
- WSJ spent a long interval near the terminal region before the local planner
  asserted arrival. The Hub propagated the final ARRIVED event in about
  `0.124 s`; the delay was not in Hub message handling.

The short-route terminal waypoints lay close to the unchanged `0.5 m` arrival
boundary (`0.4507–0.4952 m`). Replanning at the rim amplified local steering
corrections. In the pinned controller, any negative raw longitudinal command
became a fixed `-0.2 m/s` reverse command, including very small negative
values.

## Minimal fix

Commit `c4b1116f524b691522c34a18dfb0d214da5011d1` applies two bounded changes:

1. semantic planning uses a `0.15 m` inward margin, so its terminal disk is
   `0.35 m` while the official ARRIVED radius remains `0.5 m`;
2. tiny negative longitudinal segments in `[-0.02, 0) m/s` are zeroed before
   the pinned controller can quantize them to `-0.2 m/s`; meaningful reverse
   requests remain rejected.

The physical launchers pass the planning margin explicitly through
`FOCUS_WSJ_SEMANTIC_TERMINAL_PLANNING_MARGIN_M=-0.15`.

Modified deployment files:

- `hub/robot_overlay/tinynav_buildmap_goal_router.py`
- `hub/robot_overlay/yunji_tinynav_cmd_vel_control.py`
- `hub/scripts/robot/launch_wsj_tinynav_receiver.sh`
- `hub/scripts/robot/launch_yunji_tinynav_receiver.sh`

No file under immutable `source/` or `dependencies/` changed.

## Verification

- 94 targeted tests passed.
- `bash hub/scripts/verify_repository.sh --tests` passed after the change.
- The formal-05 runtime report is preserved unchanged at 20,752 bytes,
  SHA-256
  `e91cbc392b692cfd2fab5dcc97d7986c3a5291eb9269ae16d771b6cae85e3d41`.

The run predates this code commit; the engineering fix is locally verified
and remains separate from the already confirmed formal-05 success record.
