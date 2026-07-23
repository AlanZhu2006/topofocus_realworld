# Changelog

## Unreleased

### Dual-robot physical stack (2026-07-22 through 2026-07-24)

- Replace the dated, hard-coded v12 launcher with a persistent physical-session
  contract that binds Git, calibration, transforms, spool boundaries, maps,
  remote roots, generated policies and managed process identities.
- Add one-command board calibration using the existing detector/solver, fresh
  synchronized pair selection, quantitative independent-board movement
  holdout, checksummed robot deployment, fresh maps and strict no-motion
  debug.
- Make debug/live verify both remote code trees, start a clean Hub epoch,
  reject stale/blocked/torn inputs, atomically freeze accepted map/camera
  generations and replace mismatched managed Foxglove relays.
- Bind every map directory to a separate code/session/sequence/transform/
  calibration/backend contract and allow one-click to reconstruct a missing
  or blocked map before applying strict VLM freshness gates.
- Arm motion receivers only after a frozen HOLD-only VLM round; preserve
  fail-closed cleanup to mapping-only Hub policy and robot-local stop/reject
  authority.
- Add explicit real-world trial capture with local start/stop poses,
  accumulated path, planner STOP evidence, surveyed shortest paths and hashed
  independent terminal evidence for incomplete/complete 4 × 5 SR/SPL reports.
- Preserve an observed ARRIVED event across the following coordination HOLD
  so the successful robot's metric seed cannot be overwritten.
- Remove predecessor calibration/transform fallbacks from both robot v2
  launchers; a session identity is now mandatory.
- Prevent Hub admin-token expansion in tmux metadata and make generated token
  printing opt-in.
- Audit staged and untracked files in the repository verifier, including
  Python/shell/JSON/YAML syntax, whitespace, secret and size checks.
- Add user-provided physical-platform, calibration-board and semantic-map
  showcase assets with byte provenance and a future failed-demo index.
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
