# WSJ post-reboot observation readiness — 2026-07-22

## Outcome and safety scope

WSJ was reachable again after its 2026-07-22 14:19:04 UTC reboot. Read-only
inspection found no TinyNav, sender, preview, planner, `cmd_vel`, Unitree bridge
or other robot process. The existing local reverse-tunnel tmux pane was reused
and reconnected to WSJ; Yunji was intentionally left offline as reported by
the operator.

No ROS/camera stack was started because the operator had left the physical
area. The only remote state change was restoration of the already-audited USB
power persistence files and deployment of tomorrow's observation-only scripts.
No planner, Hub receiver or actuator path was installed or started.

## Reboot finding

The reboot invalidated the prior TinyNav odometry origin. Therefore
`wsj-tinynav-depth-20260722-session-v1` and
`shared-board-odin1-20260722-v1` cannot be reused. The local waiting WSJ map
daemon was stopped before it received any post-reboot frame, and its empty
runtime directory now contains `INVALID_AFTER_WSJ_REBOOT_20260722.md`.

Yunji is also expected to start a new odometry origin after power-on. A fresh
fit pair plus independently moved-board holdout and a new calibration ID are
mandatory before cross-robot fusion or target computation.

## USB persistence repair

After reboot, the D435i enumerated as `8086:0b3a` and
`usbfs_memory_mb=1000`, but camera `power/control` had reverted to `auto`.
The udev rule and usbfs unit already matched the repository exactly; the power
helper and systemd unit were missing. The tracked host installer restored them
without launching ROS.

Observed final state:

- `usbfs-memory-fix.service`: enabled, active;
- `focus-realsense-power.service`: enabled, active/exited, result success;
- Genesys hub `05e3:0625`: `power/control=on`;
- D435i `8086:0b3a`: `power/control=on`;
- no known planner/control process.

| Installed artifact | Size (B) | SHA-256 | Status |
| --- | ---: | --- | --- |
| `/usr/local/sbin/focus-set-realsense-power` | 1,058 | `39b43f54035b5ce182b43a037109741d214ac7892cf15d51fed7dda7fe413d46` | tracked deployment, observed installed |
| `/etc/systemd/system/focus-realsense-power.service` | 251 | `144f742473b99bf65bec4757a99f3fac8ada34c370ac0d3d17f06ee1e11316dd` | tracked deployment, observed installed |

The deploy source is retained under
`/home/nvidia/focus_sender/host_config_topofocus_20260722/`.

## Tomorrow's WSJ mapping-only entry point

The deployed `start_wsj_mapping_session.sh` composes the already-reviewed
camera/perception launcher with the TinyNav-native synchronized mapping sender
and raw Foxglove preview. It requires an explicit unique post-reboot transform
version, refuses any known planner/control process, verifies both loopback
tunnels, and waits for a real Hub sequence advance before reporting success.
The sender remains hard-coded `mapping_only=true` and has no base extrinsic or
decision receiver.

The remote environment file is
`/home/nvidia/focus_sender/go2_20260723.env` (mode 0600, no token inside). The
existing `.token` remains separate. Previous `focus_ros_sender.py` and
`wsj_camera_preview.py` files were preserved as
`*.bak_pre_20260723_mapping_session` before deployment.

| Remote/local identical artifact | Size (B) | SHA-256 |
| --- | ---: | --- |
| `start_go2_observation.sh` | 5,610 | `890625365b5e9906d2c05fe37ca72282aef301b134c3ada24de81d502573f974` |
| `start_wsj_mapping_session.sh` | 5,143 | `41e5d636a9ee6ccb9155609e48a0d2ff7b9ddba45de8b3667d2b29bda8d3d946` |
| `focus_ros_sender.py` | 37,212 | `6195a575ad132660c088a3c981c39f59515a843a5ca660d8950908f6349e7a70` |
| `wsj_camera_preview.py` | 4,862 | `6b8ce46bf70cd87ed267c46344a559fc521fc27cb2a96f83fe57900ec3914316` |
| `verify_go2.sh` | 4,459 | `a6589f0ecd80facaca5068bf6adec3189378ac7877a3b0af5b6a1d3b83f57e53` |
| `go2_20260723.env` | 644 | `c91edca3516f1c99260d4e7bf9ffed688143a14538e08aaf3d6df31b38466800` |

The verifier initially exposed a worktree-specific false failure because it
required `.git` to be a directory. It now recognizes Git worktrees and accepts
the exact clean live-tested recovery commit
`29f26bc058886ff450f02cdc0d6e9977e1c57010` only when
`perception_node.py` also matches SHA-256
`3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3`.
The final remote hardware verification passed every check without starting the
camera or robot.

## On-site command

Only with an operator physically present:

```bash
ssh wsj
cd /home/nvidia/focus_sender
bash verify_go2.sh --hardware \
  --tinynav-root /home/nvidia/focus_sender/tinynav_imu_fix_worktree_20260721
bash start_wsj_mapping_session.sh \
  --env /home/nvidia/focus_sender/go2_20260723.env \
  --transform-version wsj-tinynav-depth-20260723-session-v1
```

This brings real observations to the Hub and Foxglove but cannot move WSJ.
