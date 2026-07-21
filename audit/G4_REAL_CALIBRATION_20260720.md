# G4: first real dual-robot shared-frame calibration — 2026-07-20

## What this is

The first real run of `hub/tools/calibrate_shared_frame.py` against two
physically co-located, live robots — not a synthetic/pseudo-dual rehearsal.
Operator placed both wsj and Yunji stationary with a shared visual reference
(a water cup) in front of both cameras, then asked for calibration to be
attempted.

Two real, unrelated bugs were found and fixed along the way to get here —
both are genuine wins independent of calibration itself, documented in full
below.

## Blocker #1: wsj had no fresh spooled data at all

Checked before attempting anything (per this project's standing practice of
not trusting stale state): wsj's most recent spooled observation was
**8670 seconds (~2.4 hours) old** — from the bag-replay session before the
day's earlier `--live` switch. `run_live_rehearsal.sh`'s "sender" tmux
window had never started; the launcher was still waiting on
`/semantic_mapping/camera_pose` (needs `map_node`'s relocalization to
succeed) as its readiness gate before starting `focus_ros_sender.py` —
which never happened (relocalization was failing all day, documented
earlier). Yunji's spool, by contrast, was fresh (1.5s old).

**Root cause, and why it's a real bug, not just "relocalization is
stuck"**: `focus_ros_sender.py`'s pose source was switched away from
`/semantic_mapping/camera_pose` to `/slam/keyframe_odom` on 2026-07-19,
specifically *because* the new topic doesn't need relocalization (see
`audit/LIVE_ROS2_SENDER.md`). But `run_live_rehearsal.sh`'s startup gate
was never updated to match — it kept waiting on the old, now-irrelevant
topic. Confirmed live: `/slam/keyframe_odom` was already publishing fine,
in real time, independent of whatever `map_node` was doing.

**Fixed**: changed the wait condition in `run_live_rehearsal.sh` from
`/semantic_mapping/camera_pose` to `/slam/keyframe_odom`, matching what the
sender actually subscribes to. Deployed the fix, then manually started the
sender window (the underlying perception/maploc/pointcloud nodes were
already running from earlier in the day, no need to restart the whole
rehearsal). It started immediately and began sending real frames within
seconds: `sent 1 (seq=74, ack=accepted, pose_skew=33.4ms, upload=53.9ms)`.
This unblocks wsj's entire real observation pipeline going forward, not
just this one calibration attempt.

## The calibration itself

Once both robots had fresh spooled data (wsj 3.3s old, Yunji 0.5s old, well
under the tool's 5s default `--max-sync-skew-s`), ran:

```
hub/.venv/bin/python hub/tools/calibrate_shared_frame.py \
  --spool hub/runtime/spool --reference-robot robot-0 --other-robot robot-1 \
  --output hub/runtime/calibration/shared_frame_20260720.json \
  --transform-version shared-frame-20260720-v1
```

Result: `sync skew between latest observations: 3.713s`. Wrote
`shared_frame_20260720.json`.

**Sanity-checked before trusting it**, not blind acceptance:

- No NaN/Inf anywhere in the output matrix.
- The 3x3 rotation submatrix is a genuine rotation: `det(R) = 1.0000011`,
  `R @ R.T ≈ identity` (checked numerically).
- Translation magnitude is ~22.8m — large, but expected and not a red
  flag: it reflects how far apart each robot's own arbitrary odometry
  origin is from the other's in their own local coordinate systems, not
  how far apart the robots physically are right now.

**One real observation flagged, not resolved**: wsj's own camera
translation at the sync instant, in wsj's own local frame, has z=2.753m —
unusually high for a quadruped's camera height. Could be a harmless
artifact of `/slam/keyframe_odom`'s world frame not being floor-referenced
(episode-reset semantics don't require z=0 grounding either), or could
indicate real SLAM drift. No independent ground truth exists to
distinguish these, so this is recorded as an open question, not resolved
here.

## Method caveat, stated plainly

This used the "coincident" assumption (no `--offset-file`): it treats both
robots' cameras as being at the exact same 3D point and orientation at the
sync instant. The cup was a shared *visual* reference for the operator to
confirm rough co-location, not a measured input to the algorithm — the
tool itself has no vision/fiducial-detection step. Any real gap between
where the two cameras physically sat becomes calibration error. A more
precise calibration would need a real measured offset (e.g. AprilTag-based,
discussed and deferred earlier the same day) — not attempted here.

## Applied to Yunji's sender

```
python3 yunji_sender.py ... --shared-frame-transform-file shared_frame_20260720.json \
  --transform-version shared-frame-20260720-v1
```

Verified live: spooled Yunji observations now carry
`transform_version=shared-frame-20260720-v1`, and the transformed pose
matrix's leading values closely match wsj's own reference-frame pose from
the same sync instant — expected for two robots that were stationary and
co-located when the transform was computed. wsj needs no corresponding
flag (it's the reference; `shared_world` is defined as its own frame by
convention).

## Blocker #2 (found investigating a separate correction, same session): `depth_registered` was mislabeled, not actually registered

User provided direct evidence that `/camera_front_up/depth_registered/image_raw`
(this sender's previous default `--depth-topic`) is not really registered:
`depth_align=false`, `color_depth_synchronization=false`, no `camera_info`
published for it, and — the clinching evidence — pixel-by-pixel comparison
against the genuinely raw `/camera_front_up/depth/image_raw` showed **100%
identical values**, just uint16 millimetres cast to float32 (still
millimetres despite the `32FC1` tag implying REP103 metres).

**Verified independently before changing anything** (per this project's
standing practice, not taking the report at face value): live-fetched
`/camera_front_up/depth/image_raw` directly — confirmed `16UC1`, 160x120,
matching. Live-fetched `/camera_front_up/depth/camera_info` — its `K`
matrix (`[95.5086, 0, 77.4977, 0, 95.5086, 60.6648, 0, 0, 1]`) is **byte-
identical** to this sender's hardcoded `K_DEPTH`, confirming `K_DEPTH` was
always the real raw-depth camera's own intrinsics (presumably sourced
during the earlier `/tf`-based extrinsics investigation), not intrinsics
for a topic that doesn't even publish a `camera_info`.

**Net effect: nothing was actually broken.** The mm-vs-m unit conversion
was already correctly handled (`depth_m = depth_mm / 1000.0`, already
present with a comment noting the empirical confirmation). The RGB-depth
reprojection (`reproject_rgb_onto_depth_grid`) was already real per-pixel
backproject/reproject geometry using the (confirmed-identity) depth-color
extrinsic, not a naive resize — exactly the "adapter layer" the user
described as the correct approach already existed. The only actual issue
was that the *topic name* being subscribed to claimed a registration that
was never really applied.

**Fixed anyway, for robustness**: switched `--depth-topic` default to the
honestly-named raw `/camera_front_up/depth/image_raw`, and made
`fetch_depth_frame` dispatch on the message's own `encoding` field
(`16UC1` -> uint16, else float32) rather than hardcoding one dtype, so it
keeps working correctly against either topic. Rationale: depending on a
topic whose name lies about what it is (registered) is fragile — a future
firmware update could someday actually implement real registration on that
topic, silently changing its behavior out from under an assumption baked
into the code that it's still raw. Verified live after redeploying: fresh
spooled observation shows correct `intrinsics` matching `K_DEPTH` exactly,
`depth_scale_m=0.001`, no crash, `verify_intrinsics_match_frame_size`
still passes.

## Verification summary

- Test suite: 106 passed throughout (no new tests added for this pass —
  no new pure logic was introduced, only topic/gate corrections and a
  live calibration run).
- Both robots confirmed live and healthy after all changes: wsj sending
  real frames every ~2s cycle (`focus_ros_sender.py`, unbounded), Yunji
  sending every ~1s cycle with the calibration and corrected depth topic
  both applied.

## What this does and does not prove

**Proves**: the full G4 calibration mechanism (`calibrate_shared_frame.py`,
`apply_shared_frame_transform`, `--shared-frame-transform-file`) works
end to end against two real, live, physically co-located robots for the
first time — not just synthetic data or a pseudo-dual single-session
rehearsal. Also proves wsj's real observation pipeline (not just the
camera-only preview) can run live, once the launcher's stale readiness
gate is fixed.

**Does not prove**: that this specific calibration is *accurate* — the
coincident assumption's real-world error is unmeasured (no independent
ground truth), and the flagged wsj z=2.753m anomaly is unresolved.
`fusion.py`'s actual map-fusion logic has not yet been run against this
real transform (only the pseudo-dual rehearsal has exercised that code
path so far). `allow_goal` remains `false`; this is still mapping-only,
dry-run throughout.

## Follow-up, same day: real cross-robot fusion wired in and live-verified

User asked why the dashboard still showed two separate maps after
calibration. Correct question — calibration alone doesn't fuse anything;
it only computes the transform. Nothing in the live pipeline had ever
actually merged two robots' grids together outside the pseudo-dual
rehearsal. Wired real fusion in properly:

**Gap found**: `fuse_grids` (existing, upstream's element-wise max rule)
requires both grids to already share an identical origin/resolution/shape
— by design, per its own docstring ("this module only implements the
machinery that runs after [establishing the frame]"). But each
`hub_pipeline_daemon.py` instance still independently picks its own map
bounding box from wherever its own robot happened to start (even though,
post-calibration, both robots' *poses* now land in the same physical
shared_world frame) — so the two live grids are not `fuse_grids`'s
precondition, just different-sized/offset windows into the same frame.

**Added** `align_and_fuse_grids` to `focus_hub/fusion.py`: computes the
union bounding box across all input grids, places each robot's own grid
into a same-shaped canvas at the correct integer pixel offset (a plain
slice assignment — no resampling needed, since both grids already share
resolution and axis alignment; only origin differs), then reuses the
existing `fuse_grids` for the actual max-fusion. Kept `fuse_grids` itself
completely untouched (existing tests/contract intact) — this is a new,
separate function for the real differently-origined case, not a
modification of the strict-precondition one. 4 new unit tests: identical-
origin case matches plain `fuse_grids` exactly, disjoint origins place
each robot in its own non-contaminating region, overlapping origins take
the real max (not last-write-wins), and a malformed-input rejection.

**Wired into `foxglove_relay.py`**: new `--fuse` flag starts a
`fusion_poll_loop` background thread that reads every robot's latest
`central_map.npz`, calls `align_and_fuse_grids`, and publishes the result
to a new `/fused/semantic_map` channel (plus `/fused/status` reporting
shape/origin or a clear "waiting for all robots" / "fusion failed this
cycle" message — never silent). Runs on its own, slower cadence
(`--fuse-interval-s`, default 8s) since it does strictly more work than a
single-robot map poll. `build_grid_message` was refactored (no behavior
change) into `load_grid_npz` + `grid_to_message` so the fusion loop and
the existing per-robot loop share the same RGBA-encoding/message-building
code, not a duplicate copy.

**Live-verified for real**, not just unit tests: restarted the relay with
`--fuse`, confirmed via a raw WebSocket protocol client that
`/fused/semantic_map` and `/fused/status` both appear in the channel
advertisement and deliver real binary data. The status channel's real
message reported `fused 2 robots: shape=(17, 841, 658),
origin=(-12.81, -12.54)` — a real union bounding box, larger than either
robot's own square map, computed from the two robots' real, live,
independently-chosen origins (`wsj: (-12.81, -12.54)`, `yunji: (-5.94,
3.49)`). Rendered the actual fused grid to a PNG and inspected it visually:
two distinct, non-contaminating explored/obstacle regions (wsj's larger,
Yunji's smaller, including a real yellow chair-category cell in wsj's
region) placed correctly within one shared canvas — not overlapping
incorrectly, consistent with the robots' real physical offset.

Added a third 3D panel (`/fused/semantic_map`) to
`hub/foxglove/dual_robot_dashboard.json` alongside the two existing
per-robot panels — kept the per-robot panels rather than replacing them,
since they remain useful for debugging which robot contributed what to
the fusion.

Test suite: 106 -> 110 (4 new `align_and_fuse_grids` tests).

**What this does and does not prove**: proves the real fusion mechanism
works end to end against two genuinely live, calibrated robots for the
first time, with a real (not synthetic) union grid and real semantic
content from both. Does **not** prove the calibration itself is precise
(same caveats as the section above — coincident assumption, unresolved
wsj height anomaly) — a fusion computed from an imprecise calibration will
look plausible while still being subtly misaligned; visual inspection at
this map's resolution/downsampling cannot rule that out. Also does not
change anything about `allow_goal` or actuation — this is purely an
additional, read-only visualization channel.

## Follow-up, same day: "still no map at all" — two real likely causes fixed

User reported no map visible in any panel after the fusion work above,
despite the relay/daemons all confirmed healthy on the backend (fresh
`live_status.json`, valid `central_map.npz`, real data flowing over the
WebSocket protocol per the section above). Root-caused two real, concrete
problems on the *rendering* side rather than assuming the backend was
still broken:

1. **Camera framing was pointed at empty space.** Every 3D panel's default
   `cameraState.target` was `[0, 0, 0]` — a leftover placeholder, never
   corrected against real data. Computed each panel's real current map
   center directly from the live `central_map.npz` files: wsj's center
   happens to be near the origin (`(0.19, 0.46)`, so its panel may have
   looked fine), but Yunji's is at `(7.06, 16.49)` — about 18 m away — and
   the fused map's union center is at `(3.64, 8.49)`. A panel aimed at
   `(0,0,0)` with `distance: 30` would show empty space for both of those,
   not the actual data. Updated `dual_robot_dashboard.json`'s three map
   panels to target their real current centers and widened `distance` (45
   for the per-robot panels, 65 for the larger fused union) for margin.
   Honest caveat: these are point-in-time values from when this was fixed
   — as the robots move and the maps grow, the real centers will drift
   away from these fixed numbers again; this is a one-time correction, not
   a tracking mechanism.

2. **No coordinate frame was ever registered.** Every Grid/map message
   this relay publishes uses `frame_id="shared_world"`, but nothing had
   ever published anything establishing that frame in a transform tree
   (no `/tf`-equivalent at all). Checked Foxglove's own `TransformTree.ts`
   source for whether an unregistered frame blocks rendering entirely —
   inconclusive (frames appear to be created lazily on reference, which
   argues against this being a hard blocker, but this environment has no
   way to confirm against the real running app). Added `frame_tree_loop`
   to `foxglove_relay.py`: publishes a periodic identity `FrameTransform`
   (`world -> shared_world`) via a new `/tf` channel. Stated plainly: this
   is defense in depth for a genuinely unverified risk, not a confirmed
   fix — it costs nothing to publish and is standard practice regardless
   (a real deployment would normally have at least one static transform),
   so it's included rather than left as an open question with no
   mitigation attempted.

Verified what can be verified from here: the new `/tf` channel appears in
the real channel advertisement with the correct `foxglove.FrameTransform`
schema and delivers real binary data when subscribed to, alongside all the
previously-verified channels. **Cannot verify actual on-screen rendering**
in a real Foxglove app from this environment — if the map is still not
visible after re-importing the updated layout, the next step would need
someone with the real app open to report exactly what they see (blank
panel vs. an error message vs. wrong-looking data), since that's the one
piece of information this environment structurally cannot produce itself.

Test suite: 110 passed throughout (no new pure logic, only channel/layout
additions).

## Follow-up, same day: "I moved but the map didn't change" — a real, distinct bug (map bounds locked to a coordinate frame that jumped mid-session)

User moved Yunji and reported no visible map change. This is not the
rendering-side issue above -- root-caused to something more fundamental:

**The jump**: applying the G4 calibration to Yunji's sender
(`--shared-frame-transform-file`, done earlier the same day) causes every
subsequently-published pose to be transformed from Yunji's own local
odometry frame into wsj's shared_world frame -- a real, correct, but
*discontinuous* jump in reported position (Yunji's own raw coordinates
were around `(7, 16)`; after the transform, `(-0.6, -5.4)`, matching wsj's
own frame as expected).

**The bug**: Yunji's `hub_pipeline_daemon.py` had been running continuously
since well before that calibration was applied to the sender -- only the
*sender* was restarted, not the daemon. A daemon's map extent is fixed
once, from whichever observation it processes *first*
(`hub_pipeline_daemon.py`'s `mapper_init`: a fixed bounding box around the
first observation's position, never revisited). That bounding box was
locked in around Yunji's pre-calibration position, in Yunji's own local
frame. Checked directly: post-calibration, Yunji's real current position
(`-0.64, -5.42`) was **outside** that fixed map's bounds
(`x=[-5.9, 20.1], y=[3.5, 29.5]`) by ~9m on the y-axis. Every observation
uploaded since the calibration jump was landing off the edge of the
already-allocated grid array -- silently unable to update the map, exactly
matching "I moved, the map didn't change."

**Fix**: added `--start-after-sequence` to `hub_pipeline_daemon.py` (skip
spooled observations at or before a given sequence number before
processing begins) -- a real, reusable option, not a one-off hack: any
time a sender's pose source changes mid-run (a calibration applied, a
pose-topic switch, etc.) while its daemon keeps running, the same class of
bug would recur without a way to restart the daemon cleanly past the
stale, differently-framed history. Restarted Yunji's daemon with
`--start-after-sequence` set to the spool's current max sequence at
restart time, and cleared the stale `central_map.npz`/`live_status.json`
snapshot files so the relay wouldn't keep showing frozen pre-restart data
in the gap.

**Verified for real**: Yunji's current position is now inside its fresh
map's bounds (`x=[-13.6, 12.4], y=[-18.4, 7.6]`, generous margin around
`(-0.64, -5.42)`). The fusion loop picked up the corrected map immediately
and produced a new, sensible union shape. Test suite: 110 passed
throughout (one new CLI option, no new pure logic requiring dedicated
tests beyond what already exercises the daemon's argument handling).

**Real, disclosed cost of this fix**: Yunji's map restarted from empty
(frames_total reset to a low number) -- all previously-accumulated
exploration history from before the restart is gone from the live map
(though the raw spool data itself is untouched, so it could be replayed
offline later if ever needed). This was the correct tradeoff here (stale,
wrong-framed data is worse than no data), but it's a real, one-time cost,
not free.

wsj was not affected by this specific bug -- it's the calibration
*reference* robot, so its own pose frame never jumped; checked and
confirmed its current position remains within its original map bounds.
