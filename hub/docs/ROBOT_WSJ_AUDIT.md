# `wsj` TinyNav read-only audit

Observed on 2026-07-18 from `/home/nvidia/twork/tinynav`; no remote source was changed.

## Platform

- host: `tegra-ubuntu`, NVIDIA Jetson Orin NX;
- ROS 2 Humble, Python 3.10-oriented workspace;
- reachable private WLAN address observed; the ephemeral site address is
  deliberately omitted from the public repository;
- NTP reported synchronized, timezone UTC;
- repository commit `933fce54ae65e775a1262c346180341f5657c0e4` (`complete deployment`);
- the repository has modified and untracked semantic-mapping work, so it must not be reset or overwritten.

No TinyNav runtime nodes or tmux sessions were active during the audit.

## Observation path available on the robot

The aligned semantic path uses:

- `/camera/camera/color/image_raw`;
- `/camera/camera/aligned_depth_to_color/image_raw`;
- `/camera/camera/aligned_depth_to_color/camera_info`;
- exact posed output `/semantic_mapping/camera_pose`;
- TinyNav visual pose `/slam/odometry_visual` and high-rate pose `/slam/odometry`;
- TF and TF-static for RealSense extrinsics and map alignment.

The audited semantic package already implements synchronized RGB-D geometry, sparse occupancy voxels, free-space carving, semantic voxels and aligned occupancy/semantic BEV persistence. Its docs report live and replay validation, but also record pose jumps and intentionally limited fusion in one long run. This evidence is useful for constructing a sender; it is not evidence of two-robot shared-frame fusion.

## Existing goal-to-motion path

```text
/mapping/cmd_pois (std_msgs/String JSON in robot map coordinates)
  -> map_node relocalization + global path
  -> /control/target_pose
  -> planning_node local 3-D occupancy/ESDF trajectory selection
  -> /planning/trajectory_path
  -> cmd_vel_control
  -> /cmd_vel
  -> go2_cmd_bridge
  -> Unitree SportClient.Move()
```

The POI JSON currently consumes only positions; it has no decision ID, map version or expiry. A new robot-side guard must validate the hub envelope before reducing it to this legacy input.

## Existing local safeguards

- planning refuses to publish a trajectory when all candidates collide;
- command control slows/stops on stale path and honors `/nav/paused`;
- Go2 bridge clamps velocity, drops stale `/cmd_vel` after 0.35 seconds, gives the handheld remote priority, and sends stop/release on timeout or shutdown;
- navigation wrappers can start without Go2 actuation.

These are valuable layers, but the current docs explicitly state there is no calibrated Go2 `base_link -> camera_link` transform. The planner uses an approximate 0.2 m camera offset rather than a measured 6-DoF body calibration. There is also no observed versioned/expiring high-level command guard. G5 is therefore not passed.

## Integration point

The future sender should synchronize aligned RGB, aligned depth, CameraInfo and the exact `/semantic_mapping/camera_pose` stamp, compose the configured `T_shared_world_robot_map`, and upload selected keyframes. The future receiver must be dry-run by default; after G5 it may transform a fresh `GOAL` into robot-map coordinates and publish a guarded `/mapping/cmd_pois` payload. It must publish HOLD/pause on expiry, disconnect, transform mismatch, stale localization or unsafe health.
