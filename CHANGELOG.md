# Changelog

## Unreleased

### Scene 02 Standard SPL and Formal 02 replacement media (2026-07-28)

- Record the operator-provided independently measured Scene 02 shortest
  feasible path `L≈7 m` and finalize Standard SPL for formal experiment 03:
  `S*L/max(L,P) = 0.8376688690258246`. Scene 02's mean Standard SPL (failures
  at zero) is now `0.27922295634194155`, replacing the prior "pending" state
  across the README, `CURRENT_STATUS.md` and both progress manifests.
- Publish the replacement Formal 02 run's third-view and Dashboard masters
  (reusing the original `experiment_2_failure_2.*` filenames with replaced
  content) byte-for-byte through Git LFS, with H.264 derivatives and a
  time-lapsed preview GIF pair matching the Formal 01/03 pipeline; update the
  README, media manifest, audit doc and both progress manifests so Formal 02
  shows its own evidence instead of the superseded run's.

### Scene 02 formal experiment 02 replacement (2026-07-28)

- Replace the previous Formal 02 designation with `yunji-single-02` and
  archive it as `FAILURE` (`Success=0`, `SR=0`, both SPL variants zero).
  Yunji travelled `7.425951 m` through 13 completed non-semantic frontier
  rounds without a plant arrival; an operator-observed branch explored away
  from the target, and the two-interval progress guard ended the run in
  `failed_cross_round_no_progress_holding`.
- Classify the result as
  `navigation_policy_failure / algorithmic_exploration_failure`, not an
  engineering-chain failure. Preserve the superseded record's exact commit,
  file hashes and Git blobs, and exclude its media from the replacement run.

### Scene 02 formal experiment 03 (2026-07-28)

- Archive Scene 02 formal experiment 03 as `SUCCESS` (`Success=1`, `SR=1`,
  source-compatible `SPL=0.865191906`): an operator-scoped Yunji-only live
  run (WSJ forced HOLD, no live motion authority) explored, switched to the
  plant semantic region in round 6 and emitted `LOCAL_PLANNER_ARRIVED` in
  round 10, with the plant visible in the terminal RGB; the operator
  confirmed the physical success afterward. Standard SPL remains pending an
  independent Scene 02 shortest-feasible-path measurement.
- Publish the user-provided third-view and Dashboard masters byte-for-byte
  through Git LFS, with H.264 derivatives and a time-lapsed README preview
  GIF pair (matching the Scene 01/formal-01/02 naming and encoding
  convention); update the Scene 02 section of the main README (now `SR=1/3`,
  mean source-compatible `SPL=0.288397302`) and record the attempt in
  `manifests/realworld_experiment_progress.json`.

### Scene 02 formal experiments 01-02 (2026-07-28)

- Archive Scene 02 formal experiment 01 as `FAILURE` (`Success=0`, `SR=0`,
  both SPL variants zero): one assigned frontier was rejected as locally
  unreachable, and the remaining semantic-navigation leg terminated before
  arrival. Attribution is `navigation_policy_failure`, not VLM — the plant
  semantic region was correctly identified and assigned.
- Apply a local-navigation stale-route recovery timing fix, covered by
  regression tests; not yet re-verified against a live physical run.
- Archive Scene 02 formal experiment 02 as `FAILURE`: both robots' frontiers
  were rejected as locally unreachable before any plant semantic region was
  found, and the episode timed out waiting for a fresh synchronized round
  after Yunji's map was blocked by ground-plane drift. Attribution is also
  `navigation_policy_failure`, not VLM.
- Publish the user-provided third-view and Dashboard masters for both runs
  byte-for-byte through Git LFS, with H.264 derivatives and time-lapsed
  README preview GIFs (matching the Scene 01 naming/encoding convention);
  update the Scene 02 section of the main README and record both attempts
  in `manifests/realworld_experiment_progress.json`.

### Calibration reliability (2026-07-27)

- Stop treating WSJ's sparse `/slam/keyframe_depth` and
  `/slam/keyframe_odom` tuple as a continuous sensor heartbeat. Continuous
  recovery now uses processed depth and visual odometry, while the unchanged
  synchronized keyframe sender and Hub sequence advance remain mandatory.
- Interrupt timed-out remote foreground jobs before calibration/one-click
  cleanup; recover and probe the existing SSH/tmux pane if a job does not
  release it.
- Fail immediately when Yunji's NUC-to-WATER Ethernet has no carrier, recover
  the existing MAC-bound `Yunji-Robot` profile when carrier is present, and
  require an actual Hub observation sequence advance before calibration
  reports the Yunji sender ready.
- Decouple WSJ's formal observation sender from motion-selected keyframes:
  synchronize continuous depth/intrinsics/visual odometry, select the nearest
  timestamp from a bounded 90-frame color history behind a strict `50 ms`
  skew gate, retain calibrated RGB-to-depth reprojection, and restore the
  bounded startup gate to `15 seconds`.

### Scene 02 plant preparation (2026-07-25)

- Prepare `scene02-plant` as the next five-trial formal campaign without
  issuing any physical command.
- Bind the operator command sheet to `plant`, fresh calibration/debug and
  episode IDs `scene02-plant-run01` through `scene02-plant-run05`.
- Record the existing end-to-end plant category support with checksummed
  provenance and keep Standard SPL unset until a Scene 02 shortest feasible
  path is independently measured.

### Scene 01 results and evidence (2026-07-25)

- Unify operator-labelled `success_1` through `success_5` as Scene 01 formal
  experiments 01–05, all successful, with exact runtime/action/media bindings.
- Add the previously omitted formal-02 runtime record
  `trial-reanchor1-r1`, its automatic WSJ `LOCAL_PLANNER_ARRIVED` and onsite
  success annotation; archive formal-05 `scene01-chair-run05` as automatic
  WSJ arrival with operator-confirmed physical success.
- Update the five-run aggregate to `SR=5/5=1.0`, mean source-compatible
  `SPL=0.7260879584850242` and mean Standard
  `SPL=0.7809932415154623`, consistently using the independently measured
  Scene 01 reference `L≈3.25 m`.
- Publish five standardized H.264 third-view/Dashboard pairs plus five
  third-view and five Dashboard animated previews that play directly in the
  main README; preserve every user-provided master through Git LFS.
- Replace the long status-style README with a compact paper-repository layout:
  quantitative results, inline rollouts, one-line dual-robot action analysis
  and per-run metrics.
- Separate all collision, preflight and development-attempt records into the
  Scene 01 engineering-debug index while retaining their original evidence.
- Archive `trial-wallfix-imudebounce-r1` as operator-designated formal
  experiment 04 success: automatic Yunji `LOCAL_PLANNER_ARRIVED`, `SR=1`,
  source-compatible `SPL=0.956360614325575`, and checksummed terminal
  evidence.
- Record the operator-confirmed independent shortest-feasible-path measurement
  `L≈3.25 m` for formal experiment 04 and its official standard
  `SPL=1.0`; the surveyed standard track is now `SR=1/1=1.0` with mean
  standard `SPL=1.0`.
- Archive formal experiment 05 preparation as a strict no-motion preflight
  abort: WSJ raw D435i streams were live but calibrated camera/odometry output
  was frozen; no live, GOAL, episode, motion or SR/SPL row was created.
  Record both robots as powered down for charging and calibration as deferred.
- Commit the three remaining `success_2`/`success_4` source masters
  byte-for-byte through Git LFS, extending the Scene 01 source-video inventory
  from 12 to 15 while preserving their unverified runtime-binding boundary.
- Update the current README metric table to three eligible samples,
  `SR=3/3=1.0`, mean source-compatible `SPL=0.816269`, independently measured
  Formal-4 `L≈3.25 m`, and official standard `SPL=1.0`.
- Split the wide Scene 01 result table into a compact metric table and a
  fixed-width video gallery; add H.264/`yuv420p`/fast-start derivatives and
  inspected posters for all three unbound candidate masters.
- Make board calibration recover a disconnected existing WSJ/Yunji SSH tmux
  pane in place and require a unique remote-shell response before release
  verification, avoiding the previous 180-second wait on a dead pane.
- Make the WSJ controller reload graceful and wait for the old Fast DDS
  publisher to disappear before respawn; reject an unresolved/`UNKNOWN`
  publisher identity instead of timing out later in strict no-motion debug.
- Preserve the `0.50 m` semantic ARRIVED contract while planning WSJ routes
  `0.15 m` inside it, and suppress sub-2-cm negative trajectory jitter before
  TinyNav can quantize it into a fixed `-0.2 m/s` reverse pulse.
- Record the operator clarification that the manually terminal-annotated
  `trial-05-nearwall-fix` is formal experiment 03; preserve its existing
  `SR=1` and source-compatible `SPL=0.864048038008398`.
- Update the three-sample evidence-bound physical track to `SR=3/3=1.0` and
  mean source-compatible `SPL=0.816269191777991`; formal experiment 04 is the
  one sample with an independently measured path and official standard SPL.
- Preserve the operator's byte-identical formal-experiment-03 master-video
  renames and do not infer a video binding for formal experiment 04.
- Record `trial-r5-01` as the second normal 0.5 m physical-protocol success:
  automatic WSJ `LOCAL_PLANNER_ARRIVED`, `SR=1`, source-compatible
  `SPL=0.628398923`.
- Update the two-sample physical track to `SR=2/2=1.0` and mean
  source-compatible `SPL=0.746223480504199`; keep standard SPL unavailable
  without independently surveyed shortest paths.
- Publish the paired r5 third-view and Foxglove Dashboard masters through Git
  LFS, plus browser-playable H.264 derivatives and terminal posters.
- Preserve r6 as a calibration/fusion preflight abort after the operator found
  a cross-map wall conflict; no live target, robot command, episode or metric
  entry was created.
- Define evidence-based failure attribution so engineering, perception, VLM
  decision, navigation-policy and unclassified failures are not conflated.

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
- Prevent interval-only stationary keyframes from counting as independent
  multi-view semantic votes; preserve their geometry refresh.
- Render calibrated base-link pose/heading and trajectory when the measured
  camera mount is present, with an explicit camera fallback for historical
  snapshots.
- Reset the operator trajectory on a pose-discontinuity latch so visualization
  cannot connect two incompatible coordinate frames with a false path.
- Add deterministic annotation collision avoidance and a read-only,
  checksum-manifested semantic-overview exporter matching the Foxglove Image
  topics.
- Make one-click reject a stale Foxglove process by loaded-source hash and
  require both robot overviews plus the fused overview to be ready.
- Stop rewriting unchanged map snapshots and report observation/map-content
  age from capture timestamps instead of treating file mtime as freshness.
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
