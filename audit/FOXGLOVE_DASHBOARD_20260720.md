# Live multi-robot Foxglove dashboard — 2026-07-20

## Request and constraint

Asked for a Foxglove-style layout to simultaneously view wsj's and Yunji's
sensors plus the incremental 2D semantic map, "considering the delay." Given
a choice between a real Foxglove Studio setup and a custom hub web
dashboard, the user chose real Foxglove.

## Researched first: Foxglove's actual multi-source model

Before designing anything, checked Foxglove's own docs/source (a dedicated
research pass, not assumed): a single Foxglove layout drives from exactly
**one live connection** — there is no way to point one layout at wsj's ROS 2
topics, Yunji's ROS 1 topics, and the hub's own map state as three
simultaneous live sources. This is architectural, not a paid-tier
restriction. Confirmed panel-schema facts used below by inspecting a real
`foxglove-sdk` 0.25.3 install directly (`help()` on its message classes) and
a real exported Foxglove layout JSON (`dagar/foxglove-studio`, a public fork
that still carries the studio-base panel source and a sample layout,
`packages/studio-base/src/dataSources/SampleNuscenesLayout.json`) — not
guessed: panel type identifiers, the `configById`/mosaic-tree `layout`
schema, and that the Log panel's internal type string is still `"RosOut"`
(kept for backwards compatibility) all came from reading real source/JSON,
not from memory.

**Architecture chosen**: the hub becomes the single aggregation point.
`hub/tools/foxglove_relay.py` is a pure-Python process (using the official
`foxglove-sdk` PyPI package, no ROS dependency at all — confirmed the hub
machine has no `rclpy` installed) that runs its own `foxglove.sdk.v1`
WebSocket server. Foxglove makes exactly one connection, to this relay.

## Why the relay reads snapshot files, not live ROS topics

Considered having the relay open its own ROS connections to both robots
directly (an `rclpy` client to wsj, `roslibpy` to Yunji, mirroring what the
existing senders already do). Rejected this for two concrete reasons:

1. **Duplicate GPU cost**: RedNet segmentation already runs once per frame
   inside `hub_pipeline_daemon.py`, at ~65-85 ms/frame per the earlier
   soak numbers. A second, independent `SpoolMappingPipeline` instance in
   the relay would run RedNet a second time on the same frames for no
   reason.
2. **Correctness risk, not just waste**: `SpoolMappingPipeline`'s map
   origin/extent is fixed from whichever observation it happens to process
   *first* (see `hub_pipeline_daemon.py`'s `mapper_init`). A relay-owned
   pipeline started at a different moment than the daemon's would very
   likely compute a *different* origin for what's supposed to be "the same"
   map — the dashboard would show a map that looks similar but isn't
   actually spatially registered the same way as the one driving decisions.

Instead: `hub_pipeline_daemon.py` gained an opt-in `--snapshot-interval-s`
flag. On that cadence it writes `central_map.npz`/`map_summary.json` (via
the pipeline's existing `.save()` method, unchanged), a JPEG of the latest
RGB frame, and a small `live_status.json` (`written_at_ns`,
`last_capture_time_ns`, `last_sent_time_ns`, `frames_total`,
`last_camera_xy_m`). This is additive and off by default (0 = disabled) —
existing soak/e2e callers of the daemon are unaffected. The relay polls
these files. **This means the dashboard shows exactly the same map state
that (would) drive decisions** — a snapshot of it, not an independent
recomputation — and the snapshot's own staleness is trivially computable
from its own timestamps.

## What "considering the delay" means concretely

Every poll cycle, the relay publishes a `foxglove.Log` message per robot on
`/{name}/status` reporting: snapshot age (`now - written_at_ns`), camera
capture age (`now - last_capture_time_ns`), and `frames_total`. If snapshot
age exceeds 30s, the log level is bumped to `Warning` and the message is
suffixed `-- STALE`. This is deliberate: wsj and Yunji update on very
different cadences (and neither is fused into a single shared-frame map —
see below), so showing two panels side by side without any staleness signal
would actively mislead an operator into assuming synchrony that doesn't
exist. The Log/`RosOut` panel in the saved layout aggregates both robots'
status topics together (no `topicToRender` set, so it doesn't filter to one
source), letting an operator watch both ages in one place.

## Deliberately NOT a fused map

The 3D panels show each robot's own per-robot incremental semantic map
separately (`/wsj/semantic_map`, `/yunji/semantic_map`), not a merged one.
Real cross-robot fusion needs a physically-verified shared-frame calibration
(G4), which has not happened yet (`research-state.yaml`:
`shared_frame_fusion: not_implemented`). Showing a fused map here would
imply a capability that doesn't exist; this dashboard reflects the system's
actual current state honestly, including this real gap.

## Implementation

- `hub/tools/hub_pipeline_daemon.py`: added `--snapshot-interval-s` (default
  `0.0`, opt-in) and `write_live_snapshot()`. Tracks `last_metadata` per
  processed observation to source the staleness timestamps. `cv2` added as
  a top-level import (was previously only imported inside `frontiers.py`).
- `hub/tools/foxglove_relay.py` (new): polls one or more
  `ROBOT_ID:NAME:SNAPSHOT_DIR` sources (repeatable `--robot` flag), and for
  each publishes `foxglove.CompressedImage` (from `latest_rgb.jpg`, read as
  raw bytes, not re-encoded again), `foxglove.Grid` (colorized from
  `central_map.npz` using the exact same obstacle/explored/per-category-
  argmax coloring rule as `frontiers.render_semantic_decision_map`'s
  background, imported from there — `_category_palette` — so dashboard
  colors match any saved decision-map PNGs), and `foxglove.Log` (staleness,
  described above). Does not talk to either robot directly; no new ROS
  bridge, no new trust/auth surface on wsj or Yunji.
- `hub/foxglove/dual_robot_dashboard.json` (new): a hand-authored Foxglove
  layout (not exported from a running app, since this environment has no
  GUI) — two `Image` panels (`/wsj/camera`, `/yunji/camera`) on top, two `3D`
  panels (`/wsj/semantic_map`, `/yunji/semantic_map`) plus one `RosOut` (Log)
  panel showing both status topics on the bottom.

## Verification performed (and what could NOT be verified)

This environment is headless — no way to run the actual Foxglove desktop/web
app to confirm the layout renders correctly. What *was* verified, for real:

- `write_live_snapshot()` exercised against the real 30-frame wsj spool from
  the same-day deployment test (`audit/WSJ_ODOMETRY_DEPLOYMENT_20260720.md`):
  ran `hub_pipeline_daemon.py --snapshot-interval-s 2.0` against it,
  confirmed `central_map.npz`, `latest_rgb.jpg`, `live_status.json`, and
  `map_summary.json` were all written with correct, real content (30 frames
  processed, real camera_xy).
- `foxglove_relay.py` run against that real snapshot directory for multiple
  poll cycles with no exceptions.
- **Real protocol-level verification, not just "it starts"**: connected a
  raw Python `websockets` client (the correct subprotocol,
  `foxglove.sdk.v1`, was found by `strings`-inspecting the compiled
  `foxglove` extension module after an initial guess — `foxglove.websocket.v1`
  — was rejected with `HTTP 400`). Confirmed: real `serverInfo` handshake;
  real channel advertisement (`/wsj/semantic_map` -> `foxglove.Grid`,
  `/wsj/camera` -> `foxglove.CompressedImage`, `/wsj/status` -> `foxglove.Log`,
  correct schema names); sent a real `subscribe` message and received real
  binary Message Data frames (opcode 1) back on schedule. This is
  wire-protocol-level proof the server is correct, not just that the process
  doesn't crash.
- The layout JSON's schema (panel type strings, `configById` keys, the
  `first`/`second`/`direction`/`splitPercentage` mosaic-tree format, and the
  `RosOut` panel's actual type identifier) was cross-checked against a real
  panel implementation and a real sample layout file from a public Foxglove
  Studio source fork, not authored from memory.
- **NOT verified**: that the layout actually renders sensibly when opened in
  a real Foxglove app (panel camera framing, whether the 3D panels need a
  manual "reset view" since no `/tf` transform tree is published, whether
  `RosOut`'s `nameFilter`/nameing behavior displays both robots' names
  cleanly). No dual-robot live run (both `hub_pipeline_daemon.py` instances
  + the relay + both real sender rehearsals, simultaneously) has been run
  either — this was validated against one robot's already-spooled data, not
  a live two-robot session end to end.

## Known limitations, stated plainly

- Not a fused map (see above) — two independent per-robot views.
- Update cadence is bounded by `--snapshot-interval-s` (a poll/snapshot
  design, not a true push), on top of RedNet's own per-frame processing time
  — the dashboard is always showing something a few seconds old, by design,
  and says so via the Log panel rather than hiding it.
- No `/tf` transform tree is published, so the 3D panels' camera framing is
  unmanaged; an operator will likely need to manually frame the view once
  per panel after opening the layout.
- Running two `hub_pipeline_daemon.py` instances (one per robot) concurrently
  on the hub machine roughly doubles the RedNet/GPU load of running one —
  not measured in this pass (no live dual-robot run happened), flagged as an
  open question for a real dual-robot session.

## Follow-up, same day: real dual-robot live session found and fixed four real bugs

Ran the dashboard for real with both robots live (wsj `--live` real camera,
Yunji continuous) instead of just the earlier single-robot spool replay.
User feedback and direct investigation surfaced four genuine, previously-
undiscovered bugs — recorded honestly, not glossed over:

**1. `calibrationTopic: ""` stalls the Image panel forever ("waiting for
calibration").** Read Foxglove's own source (`ImageMode.ts`,
`#fallbackCameraModelActive`): only `undefined`/`null` are treated as "no
calibration selected" (`== undefined`, loose equality); an empty string is
neither, so the panel waits for real `CameraInfo` messages on topic `""`
that will never arrive. Fixed by omitting the key entirely from both Image
panel configs in `hub/foxglove/dual_robot_dashboard.json`.

**2. Camera freshness was accidentally coupled to the map's slow save
cadence.** `hub_pipeline_daemon.py`'s `write_live_snapshot()` wrote
`latest_rgb.jpg` and `central_map.npz` together, gated by the same
`--snapshot-interval-s` (3s in this session) — so the camera panel only
ever showed a frame that old, regardless of how often the relay polled.
Split into `write_camera_snapshot()` (called every processed frame, cheap)
and `write_map_snapshot()` (kept on the slow interval, expensive). This
alone cut the observed camera lag from ~3-5s to ~1s (sender fetch rate).

**3. wsj's camera was structurally blocked behind SLAM relocalization for
no good reason.** `run_live_rehearsal.sh`'s sender window only starts after
`/semantic_mapping/camera_pose` appears, which needs `map_node.py` to
successfully relocalize against a pre-built map — real camera images are
available from the very first pipeline stage, long before that. Since the
`--live` switch (per user request) hit exactly this: relocalization never
converged (`not enough similar embeddings to relocalize, 0`) against the
old recorded map, wsj's Foxglove panel stayed frozen on the last bag-replay
frame indefinitely. Fixed architecturally, not by working around
relocalization: pushed the camera path out of the whole SLAM/mapping
dependency chain. New `hub/robot_overlay/wsj_camera_preview.py` subscribes
to only the raw color topic (no sync, no pose, no map, no perception_node/
map_node/pointcloud needed at all) and POSTs JPEG frames straight to a new
`foxglove_relay.py` HTTP push endpoint the instant they're captured — true
push, not polled, and completely decoupled from whether relocalization ever
succeeds. `foxglove_relay.py` gained `POST /camera/{name}`
(`X-Robot-Token`-authenticated against `hub/runtime/tokens.json`, same
tokens the main wire protocol uses) via an embedded FastAPI/uvicorn app
running alongside the Foxglove WebSocket server (on a separate
`--preview-port`, since the Foxglove SDK's server owns its own port fully).
The map polling loop moved to a background thread so it keeps running
independently of the new HTTP server thread. Verified via the raw protocol
client: frames arriving every ~0.2s (5 Hz), confirmed independent of
`map_node`'s relocalization state.

**4. Two more bugs found rolling the same fix out to Yunji, both fixed
immediately:**
  - A first attempt used a *second, independent* rosbridge poller
    (`yunji_camera_preview.py`, now deleted) alongside the existing
    `yunji_sender.py`, which already fetches the same RGB topic every
    cycle. Running both concurrently doubled the load on Yunji's rosbridge
    server for the same data and caused real fetch timeouts under load.
    Reverted that design: `yunji_sender.py` itself gained an opt-in
    `--camera-preview-url`/`--camera-preview-token` pair that pushes its
    *already-fetched* frame to the relay right after encoding — one fetch,
    two destinations, no contention.
  - Both preview scripts initially used a persistent `requests.Session()`.
    Over the SSH reverse tunnel bridging robot -> hub, an idle keep-alive
    connection between the ~1s gaps was silently dropped somewhere in the
    tunnel, then every subsequent push on that session failed with
    `RemoteDisconnected`. This project already has one documented run-in
    with SSH tunnel keep-alive unpredictability (`audit/TRANSPORT_WSJ_TEST.md`)
    — same class of bug, different transport. Fixed by using a fresh
    `requests.post()` (with an explicit `Connection: close` header) per
    push instead of a session; confirmed via repeated `curl` calls over the
    same tunnel (100% success with fresh connections) vs. the session-based
    failures.

**A fifth, separate, more serious bug was found and fixed along the way**:
the atomic-write fix applied earlier the same day to `pipeline.py`'s
`save()` (temp file + `os.replace()`, to prevent the relay from reading a
half-written `central_map.npz`) had a real defect —
`numpy.savez_compressed` silently *appends* `.npz` to any path that
doesn't already end with it, so the temp path `central_map.npz.tmp` was
actually written as `central_map.npz.tmp.npz`, and the following
`os.replace()` then raised `FileNotFoundError` looking for a file that was
never created. **This crashed both live `hub_pipeline_daemon.py` processes
for real** (wsj's and Yunji's, within about a minute of each other) partway
through this session — not caught earlier because `save()` had zero unit
test coverage despite two real bugs in it the same day. Fixed by naming the
temp file `central_map.tmp.npz` (ends in `.npz`, so numpy doesn't touch
it). Added `hub/tests/test_pipeline_save.py` (2 tests: a loadable-npz-with-
no-stray-files check, and a repeated-save-doesn't-collide check) — this is
new, previously-nonexistent coverage for a method that had caused two real
production incidents in one session. Test suite: 104 -> 106.

All four (well, five) fixes verified against the real, live, two-robot
session (not just unit tests): both daemons restarted cleanly and stayed
up, both camera channels confirmed flowing via a raw WebSocket protocol
client (not just "the process didn't crash" — actual binary Message Data
frames observed on `/wsj/camera`, `/yunji/camera`, `/wsj/semantic_map`,
`/yunji/semantic_map`, `/wsj/status`, `/yunji/status`), full test suite
green throughout.

## Follow-up, same day: Yunji camera is still choppier than wsj — root
## cause found (camera hardware, not the client), a "fix" attempted and
## reverted after it made things worse

After the fixes above, Yunji's camera still felt noticeably choppier than
wsj's. Investigated properly rather than assuming the earlier fixes weren't
enough.

**Hypothesis 1 (wrong, but reasonable): per-fetch connection overhead.**
`yunji_sender.py`'s RGB fetch uses `fetch_one_topic`'s one-shot pattern
(connect, subscribe, wait for one message, disconnect) — deliberately
chosen elsewhere in this file so a stuck fetch just times out instead of
wedging a long-lived connection (see that function's own docstring). A
live run showed the full cycle (status + odom + RGB + depth + encode +
upload + preview push) averaging ~1.9s/frame (548 frames / 1057s wall
time), and a standalone measurement of `fetch_rgb_frame` alone showed ~1s.
Reasonable theory: reconnecting from scratch for every single frame is
where the time goes, same as wsj's fix (that one really did have a
per-frame-blocking problem, see the section above).

**Built a fix matching that theory**: `RgbStreamThread`, a genuinely
persistent rosbridge subscription (connect once, subscribe once, keep
reading `publish` messages as they arrive) running as a background thread
in `yunji_sender.py`, mirroring wsj's async ROS 2 subscription model. The
main loop would read a thread-safe cache instead of fetching RGB itself,
and the thread would push to the camera preview endpoint the instant each
frame arrived.

**Deployed it and it was worse, not better** — frames arrived every ~5s
instead of faster. Diagnosed directly rather than assuming the new code
was buggy: an isolated, minimal script (same `_WebSocket` class, same
persistent-subscribe pattern, nothing else running) measured the RGB
topic's real arrival times over a clean 15s window: `1.6s, 6.64s, 11.67s,
16.73s` — a **consistent ~5.0s period**, independent of any of this
project's code. A second, completely unrelated camera on the same robot
(`/uvc_camera_imu/image_raw`, the VIO tracking camera, aimed at the
ceiling) was checked as a control and showed the *same* ~5s cadence over
an 8s window (`0.25s, 5.25s`) — two different cameras with matching rate
ceilings is too consistent to be coincidence, and points at a shared,
robot-side publish-rate limit (driver config, CPU budget, or a shared
image_transport node) rather than anything on the client/network side.
**There was no client-side latency left to fix.**

Worse, the persistent thread introduced a *new* problem: with the RGB
subscription now held open continuously in a separate thread, the main
loop's own one-shot fetches for other topics (odom, depth) started
timing out — contention between the persistent connection and the main
loop's connections to the same rosbridge server, the same class of bug as
the earlier "two independent pollers" mistake, just shifted to a different
pair of topics instead of eliminated.

**Reverted cleanly**: removed `RgbStreamThread`/`LatestRgbFrame` entirely,
restored the original one-shot `fetch_rgb_frame` call in the main loop and
the inline preview push after it (identical to the working version from
earlier the same day). Confirmed the reverted version runs clean (2488
frames sent, `live_status.json` fresh, no errors) and the test suite is
unaffected (`RgbStreamThread` never had dedicated unit tests, so nothing
to remove there either).

**Actual conclusion**: Yunji's camera choppiness relative to wsj is real
and now understood, but is a robot/firmware-level publish-rate limitation
(~5s per frame on this specific camera pipeline), not a defect in the
sender, the relay, or the network path. Nothing on the client side can
make this camera update faster than the robot itself publishes it. Not
investigated further in this pass: whether the vendor's own tooling
exposes a way to reconfigure the camera driver's publish rate (would need
vendor documentation or SSH access to the chassis itself, neither
available), or whether `extra_camera` (seen in the topic list, currently
not responding — see the earlier same-day camera-height investigation)
might have a different, faster publish rate if it can be gotten working.

## Follow-up, same day: lidar-augmented occupancy considered and declined; camera preview resolution bug found and fixed

Asked whether ~5s camera updates are enough for "real-time" 2D semantic
mapping, and whether Yunji's laser scanners (`/front_laser/scan_filtered`
etc., already running at ~15 Hz for its own AMCL/Cartographer nav) should
be added to speed up the occupancy channel. Answered honestly rather than
just implementing: lidar can only ever help the *obstacle/occupancy*
channel — semantic categories (chair/person/tv/...) have no lidar analogue
and would remain camera-gated regardless — and more importantly, the
original HPC/Habitat source has no lidar sensor at all, so wiring one in
would be a deliberate, explicit departure from the standing HPC-fidelity
directive, not a free win. Presented this as a real fork with a real
tradeoff rather than deciding unilaterally; user chose to hold off and
keep fidelity for now. No code changed for this part.

Separately, "camera清晰度也很低" (the camera preview looks low-resolution)
turned out to be a real, distinct bug, not the same 5s-rate issue: the
camera preview push in `yunji_sender.py` was reusing `rgb_bytes`, which is
`rgb_registered` -- the raw 640x360 RGB frame *reprojected down onto the
depth grid's 160x120 resolution* so it has pixel-for-pixel correspondence
with depth for the semantic mapping pipeline. That reprojection is
necessary and correct for the actual hub upload, but was an unnecessary
16x pixel-count quality loss for a viewing-only preview that has no need
to match depth's resolution at all. Fixed by encoding `rgb_full` (the
native, un-reprojected 640x360 frame, already available in scope) into a
separate JPEG for the preview push, leaving the mapping upload's
`rgb_bytes` completely unchanged. Verified for real: pulled an actual
frame back out of the live WebSocket channel after redeploying, parsed its
JPEG SOF0 marker directly (not just trusting the byte count) to confirm
640x360, and visually inspected it (sharp carpet-texture and chair-wheel
detail clearly visible, a real quality difference from the blurry
160x120 that was being sent before).
