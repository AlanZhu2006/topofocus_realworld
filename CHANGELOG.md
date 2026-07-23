# Changelog

## Unreleased

### Dual-robot physical stack (2026-07-22 through 2026-07-24)

- Replace Yunji's interim RealSense lane with the observed Odin1
  `O1-P070100205` RGB/SLAM-cloud/odometry path and preserve its factory
  calibration and driver patch provenance.
- Add measured body-camera calibration records, shared-board fit/holdout,
  gravity-preserving alignment and explicit calibration IDs.
- Add goal-scoped YOLO semantic BEV reinforcement, pixel-region overlays,
  category labels, trajectory/pose/frontier visualization and shared Foxglove
  overviews.
- Add source-derived Perception, Judgment/FN and Decision VLM stages, shared
  directional memory, sequential two-agent allocation and continuous
  non-motion scene state.
- Implement transport v2 atomic two-robot decisions, semantic-region targets,
  per-leg lease renewal, navigation events and SR/SPL report structures.
- Implement fail-closed WSJ and Yunji receivers. WSJ uses online BuildMap A*,
  TinyNav control and a guarded Unitree bridge; Yunji uses bounded WATER
  high-level move/cancel.
- Add two-mode real-world one-click startup, strict synchronized-input
  preflight, frozen VLM provenance, automatic dual HOLD and debug restoration.
- Align WSJ sender/receiver IMU thresholds and make receiver heartbeat health
  authoritative in both v1 and v2 registries.
- Persist exact Go2 bridge commands and identify retry3's rotation-only
  `wz=-0.200` output.
- Raise staged WSJ nonzero command floors to `0.15 m/s` and `0.30 rad/s`
  without changing the `0.20 m/s` and `0.50 rad/s` hard maxima.
- Isolate router odometry and occupancy callbacks while retaining the
  one-second stale-input fail-closed threshold.
- Preserve three failed engineering attempts as excluded evidence; no official
  SR/SPL result is claimed.
- Synchronize the final retry3-fix snapshot to both versioned robot deployment
  roots with one independently verified archive hash.

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
- Split Foxglove geometry from semantic overlays, reduce evidence before
  assigning colors, and add current-camera/trail/legend status channels.
- Add bounded live-spool occupancy sweeps, raw-vs-thresholded RedNet
  diagnostics, and a read-only operator moved-map acceptance gate.
- Reuse the existing board-calibration flow while recording a common
  calibration ID and input provenance hashes.
