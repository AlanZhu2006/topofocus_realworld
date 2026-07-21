# Changelog

## Unreleased

- Initialize reproducible Git management for the local Hub and WSJ Go2 deployment.
- Pin and sanitize the WSJ TinyNav source delta, including the three IMU scheduling fixes.
- Add Go2 bootstrap, USB reliability, read-only preflight, observation-only launch and native BuildMap save workflows.
- Preserve and memory-bound the concurrent real-depth free-space ray-marking update; add five focused unit tests.
- Exclude credentials, models, recordings, maps, virtual environments and runtime state from Git.
- Add machine-readable source/artifact manifests and clean-clone verification.
- Prevent stale first poses from fixing live map bounds; add startup pose and
  three-frame RANSAC ground consensus gates.
- Add live keyframe filtering, pose-discontinuity latching, and reversible
  free/occupied log-odds evidence while preserving upstream replay mode.
- Make map frame/calibration metadata explicit and require a common verified
  calibration ID before Foxglove fusion.
- Retain the latest camera message for Foxglove reconnects and replace the
  unverified fused dashboard panel with two centered per-robot maps.
