# Live map contract

`hub/tools/hub_pipeline_daemon.py` is the incremental RGB-D map used by the
Foxglove dashboard and dry-run decision pipeline. It is separate from
TinyNav's finalized native BuildMap artifact.

## Startup

The daemon does not choose a fixed map extent from one observation. It waits
for three consecutive poses that satisfy all of these defaults:

- adjacent translation no greater than 2.0 m;
- adjacent rotation no greater than 90 degrees;
- adjacent capture interval no greater than 10 s;
- one transform version and one parent frame.

The median startup position centers a 26 m map. A three-frame deterministic
RANSAC consensus estimates the ground plane. If no consistent near-horizontal
ground is visible, startup remains blocked. An operator may explicitly choose
`--ground-mode camera-height --camera-height <metres>` only when that physical
height is measured for the current posture.

## Incremental geometry

The live defaults accept a mapping keyframe after 0.20 m translation, 10
degrees rotation, or 5 s. A translation above 2.0 m or rotation above 90
degrees between adjacent observations latches the map as halted; it does not
resume irreversible fusion into a potentially different pose frame. Start a
new output directory/session after investigating the discontinuity.

Obstacle geometry uses one frame-level update per cell:

- traversed free cell: log odds -0.40;
- height-filtered endpoint: log odds +0.85;
- clamp: [-4, 4];
- occupied threshold: probability >0.70;
- minimum supporting keyframes: 2;
- collision band: 0.15–0.75 m above estimated ground.

Explored and semantic channels preserve the upstream maximum-fusion contract.
Semantic projection uses its own 0.25–1.50 m height band.

## Snapshot and fusion

Every new `central_map.npz` contains at least:

- grid, origin, resolution, floor height and floor source;
- frame ID and robot transform version;
- optional shared-frame calibration ID;
- obstacle fusion mode, height band and hit threshold;
- `map_format_version=focus-hub-central-map-v2`.

Foxglove refuses legacy snapshots without frame/transform metadata unless
`--allow-legacy-maps` is explicitly given for an unverified per-robot view.
Fusion never accepts that override. `--fuse` requires every input map to name
the same non-empty `shared_frame_calibration_id` and frame ID. Once that
contract passes, the relay publishes both `/fused/geometry_map` and
`/fused/semantic_map`. The geometry view is the operator default because it
does not turn an empty or low-quality semantic layer into apparent object
evidence.

## Dashboard interpretation

- `/<name>/geometry_map` is the default dashboard layer:
  dark gray/translucent is unknown, light gray/white is explored free space,
  and near-black is current obstacle evidence;
- `/<name>/semantic_map` is a separate combined layer and is hidden by default
  while the live semantic-quality gate remains open;
- `/<name>/map_pose` draws a red current camera XY and a blue relay-lifetime
  camera trail. It is not a calibrated body footprint or heading;
- `/fused/geometry_map` is the large shared-frame panel by default, with both
  robots' map-pose trails overlaid;
- `/fused/semantic_map` is present but hidden. Enable it only after
  `/fused/status` reports non-zero semantic evidence and the relevant target
  has been independently checked. Geometry occupancy alone does not prove a
  semantic class such as `chair`;
- evidence is max-reduced before categorical color assignment. The previous
  RGBA block average could invent irregular blended colors which were not
  semantic classes.

Foxglove does not update an already imported local layout when this repository
file changes. Re-import `hub/foxglove/dual_robot_dashboard.json` after a layout
revision.

A stationary depth camera naturally produces one fan-shaped observed sector.
It cannot yield a complete room outline. Movement tests require an operator at
the robot and remain separate from Hub software deployment.

The relay retains the latest JPEG and republishes it with its original
timestamp for late/reconnected Foxglove subscribers. Status age is based only
on real camera pushes and the actual map file mtime, so this retention does not
claim a stale image is fresh.

Offline parameter replay, RedNet confidence diagnosis, the operator-present
moved-map acceptance gate and reuse of the existing board calibration scripts
are documented in [OFFLINE_MAP_VALIDATION.md](OFFLINE_MAP_VALIDATION.md).
