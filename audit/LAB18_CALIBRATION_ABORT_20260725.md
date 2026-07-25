# Lab18 repeat-2 calibration abort and release-root repair — 2026-07-25

## Outcome

The attempted session `20260725-lab18-repeat2` did **not** produce a valid
shared-frame calibration and did **not** start a live episode. No robot motion
was authorized by this attempt.

The third calibration launch reached the fail-closed raw observation stage,
but the operator then moved the Go2 to its charging position. That invalidated
the stationary-placement premise for the next trial, so calibration was
cancelled before either an initial board fit or an independent moved-board
holdout was accepted.

The last valid physical result remains session
`20260725-lab17-nearwall-fix`, episode `trial-05-nearwall-fix`. Its calibration
is historical evidence only; it must not be used for a new run after the Go2
has been moved.

## Observed startup sequence

1. The first launch could not complete the remote release check because the
   existing WSJ SSH/tmux pane no longer had a live SSH connection. The
   connection was restored in that same established pane.
2. The second launch found a byte mismatch for
   `hub/robot_overlay/start_yunji_v2.sh` in the WSJ release root. The earlier
   per-robot deployment had updated each robot's own launcher, while the
   calibration verifier correctly requires both release roots to contain the
   complete tracked deployment set byte-for-byte.
3. The two cross-robot launchers were copied atomically through the existing
   loopback/tmux transport:

   | File | Size | Local/current SHA-256 |
   | --- | ---: | --- |
   | `hub/robot_overlay/start_yunji_v2.sh` | 15,149 B | `5eb01542b065d5411fd564819decbee68f51d1e0294ec8344c435e685ceabce6` |
   | `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 15,787 B | `c15832969df36d2cfda2ace07bb40303d78eaeeaeaba0c59ecbb5a18a5a4408e` |

4. A third launch then passed the complete byte-identical release-root
   verification on both WSJ and Yunji and started raw calibration observation.
5. The operator reported that the Go2 had been moved to charge. The WSJ
   connection also closed during this transition. The calibration process was
   stopped; no board solve, deployment, fresh-map debug or live command
   followed.

Steps 1–5 are observed terminal/operator events. The explanation in step 2
about the earlier per-robot deployment pattern is source-derived from the
verified file scope; it is not a claim that the mismatched launcher caused any
physical robot behavior.

After cancellation, a local read-only health check returned
`goal_output_enabled=false` for both robots, and no calibration or live-runner
process was present. The fail-closed Hub tmux service remained available; its
continued presence is not a motion authorization.

## Preserved local provenance

The incomplete runtime directory is intentionally ignored by Git:

`hub/runtime/calibration_sessions/20260725-lab18-repeat2/`

It contains only:

| Runtime evidence | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `repository_state.json` | 1,053 B | `9cc9c28772872b0337b66f8a1f756bac440ea48b1dcb50990a64895cadc45b19` | observed local repository state |
| `robots_raw.json` | 332 B | `afff1f723492aa8b68fd0df960435183ed17ec748402f3a847ce95a778bd89bb` | source-derived fail-closed raw robot configuration |
| `robots_debug.json` | 334 B | `eb8be6b1576fc8c0cd513864e45a6599e8372a968b11e61d12ccac6cae2f9be7` | source-derived no-motion debug configuration |

The repository state binds the attempt to commit
`b1762d15e1059281056ef1e6b4e472e9d25258e1` and records
`runtime_worktree_clean=true`. Two earlier failed attempt directories were
archived automatically under:

- `hub/runtime/calibration_sessions/failed/20260725-lab18-repeat2-20260725T014335Z-4046434/`;
- `hub/runtime/calibration_sessions/failed/20260725-lab18-repeat2-20260725T014557Z-4047958/`.

There is no `fit_only_unvalidated.json`, validated shared-frame JSON,
`shared_frame.json`, map/debug manifest or episode report for lab18. Therefore
lab18 must not be described as calibrated, debugged or metric-eligible.

## Next valid physical step

After charging:

1. return the Go2 and Yunji to their intended trial start placements and keep
   both robots stationary;
2. restore the existing WSJ SSH/tmux connection and verify both sensor/tracking
   streams;
3. use a fresh session ID and run the normal two-position board calibration;
4. accept the session only after the independent moved-board holdout, fresh
   maps/Foxglove and strict no-motion debug all pass;
5. request a new onsite motion authorization only after those gates.

A stationary tracking re-anchor is not sufficient for the present transition,
because the Go2 itself was moved to the charging position.

No file under immutable `source/` or `dependencies/` was changed.
