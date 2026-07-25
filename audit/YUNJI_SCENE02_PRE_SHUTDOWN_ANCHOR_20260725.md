# Yunji Scene 02 pre-shutdown anchor — 2026-07-25

The operator planned to power Yunji off without moving its chassis. A
read-only anchor was captured before shutdown so the next tracking epoch can
be validated by stationary re-anchoring rather than a new board calibration.

## Observed anchor

- Shared-frame base pose:
  `x=0.6457808209 m`, `y=-0.2708059225 m`,
  `yaw=1.5890845515 rad` (`91.048°`).
- Stable pre-restart sequence range: `261785–261789`.
- Maximum translation variation: `0.000231615 m`.
- Maximum rotation variation: `0.115074°`.
- Yunji boot ID:
  `da83840f-3f1b-4ebf-b7bb-7912e8ebfd98`.
- Odin driver PID: `3780`, active since
  `2026-07-25 19:08:19 CST`.
- Calibration source:
  `hub/runtime/calibration_sessions/scene02-plant-20260725-201949/shared_frame.json`.
- Calibration ID:
  `shared-board-odin1-scene02-plant-20260725-201949-v1`.
- Transform version:
  `yunji-odin1-board-scene02-plant-20260725-201949-v1`.

The complete local anchor is preserved at
`hub/runtime/shutdown_anchors/yunji_scene02_20260725_pre_shutdown.json`,
4,230 bytes, SHA-256
`b3f73658100c4a3e7e794a8cc2069d2d2337e3c06757b4add5b8222589d81d72`.
The runtime file records the RGB, depth and metadata hashes for all five
append-only spool observations.

## Safety and reuse condition

The capture contacted no robot interface and issued no command. It is
**observed pre-shutdown evidence**, not proof that the robot remains stationary
after power loss.

After reboot, stationary re-anchoring may be used only if the operator confirms
that the chassis and camera mount were not moved during power-off. Fresh
post-restart stable observations must pass the existing translation and
orientation residual gates. If that validation fails, a new board calibration
is required.

