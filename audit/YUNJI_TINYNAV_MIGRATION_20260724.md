# Yunji online TinyNav migration

Date: 2026-07-24 CST

Status: **implemented and locally tested; robot-side debug and physical motion
unverified**

## Approved authority split

```text
Hub expiring high-level target
  -> robot-local online occupancy + A* goal router
  -> TinyNav local planner + trajectory controller
  -> raw /cmd_vel
  -> v2 lease/freshness graph gate
  -> /focus_guarded_cmd_vel
  -> WATER short-lived /api/joy_control
```

The active launcher is
`hub/robot_overlay/start_yunji_v2.sh`. It does not launch the historical
`v2_yunji_receiver.py` and does not call WATER saved-map, accessible-point,
make-plan or move endpoints.

## Source provenance

### TinyNav planner/controller

- repository:
  `git@github.com:AlanZhu2006/go2_tinynav.git`
- pinned commit:
  `5705bb61dafb407594970ab2bc85c63fc71e0a24`
- classification: **observed in a clean local shallow checkout**

| Relative path | Size | SHA-256 |
| --- | ---: | --- |
| `tinynav/core/planning_node.py` | 32,331 B | `1d78d6204508a3cec880eb6899980fc77850fc5b262bf1266f0e15ba43c7dc0e` |
| `tinynav/platforms/cmd_vel_control.py` | 15,083 B | `40519ebb1c9845e0a112f55f0a1ef5790280153ebaf198ff5122103e1372c50b` |
| `tinynav/core/math_utils.py` | 10,674 B | `067bcc799b35d68850c4c90d54d579935fe9b7fffe84ea29865b33a9d825c787` |

`install_yunji_tinynav_runtime.sh` verifies these bytes and writes an
on-robot provenance JSON. It installs only planner/controller Python
dependencies; it does not download model weights, simulator scenes or
datasets.

### WATER velocity contract

- observed robot-local source:
  `/home/nyu/workspace/tinynav/yunji-water-robot/docs/vendor/yunji_water_development_guide.md`
- size: 9,591 B
- SHA-256:
  `22d5bfb7fd722933af72e98da50968dc36baad476533ffac335baf2dd97eaf55`
- classification: **source-derived vendor API summary**

That source records `/api/joy_control`, linear range ±0.5 m/s, angular range
±1.0 rad/s, an approximately 0.5-second command duration and refresh above
2 Hz. The deployment further limits output to 0.15 m/s and 0.40 rad/s at
5 Hz with a 0.30-second input watchdog.

### Yunji footprint and camera offset

- existing robot-local source:
  `/home/nyu/workspace/tinynav/yunji-water-robot/tools/navigation/yunji_reachable_server.py`
- size: 48,197 B
- SHA-256:
  `6fcc3ca31630069dc9186b2e5620b8dd0f213a17d31ec5f64861f589279f2548`
- recorded circumscribed body radius: 0.283 m
- classification: **source-derived from the earlier robot-local deployment**

The 0.23 m forward and 0.36 m upward Odin offsets were supplied by the
operator. TinyNav's planar robot-center correction uses the 0.23 m forward
component. The complete orientation/translation remains in the versioned
`base_link_T_odin1_camera_optical_frame` calibration artifact.

## New deployment files

- `odin1_tinynav_adapter.py`: synchronized calibrated depth, camera odometry,
  raw world cloud and camera pose; no command publisher.
- `run_yunji_tinynav_planner.py`: injects the documented Yunji circle and
  camera offset into the pinned unmodified planner module.
- `water_cmd_vel_bridge.py`: dry-run by default, explicit live phrase,
  WATER-health gate, stale-input zero, reconnect handling and shutdown zeros.
- `run_yunji_tinynav_component.sh`: common ROS/runtime environment.
- `start_yunji_v2.sh`: idempotent debug/live systemd orchestration.

The existing v2 receiver was generalized to canonical `robot-0`/`robot-1`,
external Odin odometry health and optional local platform bridge health. WSJ's
existing flags/defaults remain compatible.

## Verification boundary

Observed locally:

- Python compilation;
- shell syntax;
- quaternion conversion;
- Odin sender regression tests;
- velocity clamping, unsupported-axis rejection, stale-input zero, WATER
  health rejection and newline-terminated API encoding;
- receiver and operator-launcher regression tests.

Not yet observed:

- pinned runtime installation on Yunji;
- live Odin ROS topics through the new adapter;
- occupancy growth and TinyNav trajectory on Yunji;
- zero-only debug bridge status against the real WATER service;
- any physical motion through this chain.

Therefore this change does not create an SR/SPL sample and must first pass the
no-motion `realworld_oneclick.sh --mode debug` gate.
