# WSJ calibration geometry fix — 2026-07-25

## Scope and safety

This audit used append-only Hub spool observations plus one read-only live ROS
subscription. No planner, receiver, velocity, goal, WATER, or Go2 command was
issued.

## Observed failure

The incomplete `20260725-lab05` attempt passed synchronization, center, board
movement, and gravity checks, but correctly failed its independent holdout:

- fit normal residual: `3.148036 deg`;
- holdout normal residual: `4.692630 deg` (limit `3.0 deg`);
- holdout center residual: `0.028640 m`;
- fit-to-holdout board translation: `0.336875 m`.

The WSJ calibration JPEG contains a color/depth registration mosaic with two
visible copies of the circle board. More importantly, both selected boards
were too small for repeatable planar PnP. Re-evaluation with the implemented
quality metric measured minimum axis-median adjacent-dot spacings of
`6.909 px` on WSJ and `5.278 px` on Yunji, below the new `7.0 px` gate.

## Observed input provenance

Classification: observed append-only Hub spool inputs.

| Robot/sequence | Artifact | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| WSJ 23847 | `hub/runtime/spool/robot-0/00000000000000023847/rgb.jpg` | 111562 | `561a4df9461a3b27b7c337122c4806f8a4f16487d349963d66380b18d356462d` |
| WSJ 23847 | `hub/runtime/spool/robot-0/00000000000000023847/depth.png` | 122557 | `5aac99eb7537bd4c2b1f602792ca3973d2f4d8b5688009ad74316c7fd5eb5642` |
| WSJ 23859 | `hub/runtime/spool/robot-0/00000000000000023859/rgb.jpg` | 115800 | `4b1e0ac79fc60657a436121911d41eddf6686acdc55da789ce10ce1c54fdda4f` |
| WSJ 23859 | `hub/runtime/spool/robot-0/00000000000000023859/depth.png` | 129639 | `b0bd9eb292efe6c673eff338a42afaf74dc99525d841782f15eb3c85ad6b6664` |
| Yunji 218751 | `hub/runtime/spool/robot-1/00000000000000218751/rgb.jpg` | 144222 | `bfe69e0481970558cafb500372244df8b43e9f6e5c4b4f106d5c010f571b446f` |
| Yunji 218751 | `hub/runtime/spool/robot-1/00000000000000218751/depth.png` | 213845 | `4370f3ca6dfecfff81862dcf32ac6047f77fd0569bc0d0fbca8faba3e3cd280d` |
| Yunji 218791 | `hub/runtime/spool/robot-1/00000000000000218791/rgb.jpg` | 145088 | `86c51913dd96563866277c74bacc7fbd5ab38d5bc313d1b5036e2200e97bb680` |
| Yunji 218791 | `hub/runtime/spool/robot-1/00000000000000218791/depth.png` | 214845 | `542b2505b38115ef6bf260dca1b458f6cb639e59447561f3642b4dee2baa291e` |

The pair selection files preserve metadata hashes and exact timestamps:

- `hub/runtime/calibration_sessions/20260725-lab05/fit_pair.json`;
- `hub/runtime/calibration_sessions/20260725-lab05/holdout_pair.json`;
- `hub/runtime/calibration_sessions/20260725-lab05/fit_only_unvalidated.json`.

## Read-only live check

Classification: observed live console result; no image artifact was retained.

The existing WSJ SSH/tmux session subscribed to exactly one
`/camera/camera/infra1/image_rect_raw` frame in memory. It reported `mono8`,
`848x480`, one complete 10x7 grid, and `7.261 px` minimum axis-median spacing.
No topic was published.

## Implemented source-derived correction

- WSJ calibration observation and calibration preview now use native rectified
  `infra1`, TinyNav keyframe depth, TinyNav intrinsics, and TinyNav camera pose
  in the same optical geometry. The formal RGB-D semantic mapping launchers
  are unchanged.
- The formal debug/live observation entry point verifies that its preview
  process subscribes to `/camera/camera/color/image_raw`; a leftover grayscale
  calibration preview is stopped and replaced before formal observation.
- Live board selection and final calibration reject complete but undersized
  grids below `7.0 px` with `BOARD_TOO_SMALL`.
- The wrapper reports that condition immediately, asks the operator to move
  the board closer, and bounds ordinary pair acquisition to 60 seconds.

Classification: source-derived implementation, awaiting a new physical
moved-board holdout. Repository verification passed all 400 tests after the
change.
