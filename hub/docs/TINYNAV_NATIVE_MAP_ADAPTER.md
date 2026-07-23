# TinyNav BuildMap native occupancy adapter

> This file describes the original finalized-map adapter. The current WSJ
> physical lane uses online BuildMap plus
> `tinynav_buildmap_goal_router.py`; see
> [CURRENT_STATUS.md](../../CURRENT_STATUS.md). The offline adapter remains a
> reproducibility and rollback path.

## Approved boundary

The robot remains authoritative and this path is mapping-only. It consumes a
**finalized** TinyNav `BuildMapNode` directory only after
`/benchmark/data_saved=true`; it sends no command, decision, or velocity data
to the robot. TinyNav source and dependencies remain immutable. Deployment and
adapter code lives under `hub/`.

The first gate is file-based and offline. A future network uploader requires a
separately versioned transport contract; this adapter does not silently add one.

## Input contract

Required files, emitted by TinyNav `BuildMapNode.save_mapping()`:

- `occupancy_grid.npy`: non-empty 3-D integer array indexed `[x,y,z]`, with
  exactly `0=unknown`, `1=free`, `2=occupied`;
- `occupancy_meta.npy`: four finite values
  `[origin_x_m, origin_y_m, origin_z_m, resolution_m]`, with positive
  resolution.

The adapter records the resolved source path, byte size, and SHA-256 of both
files. Loading uses `allow_pickle=False` and rejects unexpected shape, dtype,
state values, or metadata.

## Projection contract

TinyNav itself produces its 2-D image with `max(occupancy_grid, axis=2)`, so
the adapter preserves that exact precedence: any occupied voxel makes the XY
cell occupied; otherwise any free voxel makes it known-free. Native arrays are
`[x,y]`; Hub/Foxglove arrays are `[row=y,column=x]`, so the plane is transposed.

Hub channels are:

- channel 0: native 2-D state equals occupied;
- channel 1: native 2-D state is free or occupied;
- channels 2–16: zero. Semantic labels are not invented by this adapter.

Output `central_map.npz` also records `frame_id`, `transform_version`, native
origin, resolution, and source kind. A session-local BuildMap world must use a
session-local frame name and must not be fused with another robot until a
physical shared-frame calibration is applied.

## Lifecycle

1. Run `BuildMapNode` from a fresh output directory.
   `robot_overlay/run_tinynav_buildmap_live.py` is the live-topic wrapper; it
   bypasses the source file's mandatory BagPlayer and refuses any existing
   output path before TinyNav can remove scratch database files.
2. Stop it through `/benchmark/stop`, not by invalidating the ROS context.
3. Wait for `/benchmark/data_saved=true` and verify the required files.
4. Transfer/copy only through an explicitly approved mechanism, preserving
   hashes.
5. Run `tools/import_tinynav_occupancy.py` into a new output directory.
6. Validate the independent Foxglove topic before replacing any current map.
