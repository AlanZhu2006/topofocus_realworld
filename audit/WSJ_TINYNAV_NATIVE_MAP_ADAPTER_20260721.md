# WSJ TinyNav native BuildMap occupancy adapter — 2026-07-21

## Scope and safety

This gate replaces no running map and starts no planner, velocity, Unitree
sport/motion, or actuator path. Hub `/healthz` reported
`goal_output_enabled.robot-0=false`. TinyNav `source/` and `dependencies/`
remain immutable; all new implementation is under `hub/`.

The first transport boundary is deliberately file-based and offline. A native
map is eligible only after TinyNav publishes `/benchmark/data_saved=true`.
No native-map network uploader was introduced. The approved file and frame
contract is recorded in `hub/docs/TINYNAV_NATIVE_MAP_ADAPTER.md`.

## Source-derived native contract

The deployed WSJ source
`/home/nvidia/twork/tinynav/tinynav/core/build_map_node.py` was observed as
45,365 B, SHA-256
`4fcac8a38e870981e202bdcf133ab1bad3bbe58b0fa46c5950fba17e1eaae82e`.
Direct source inspection established:

- `generate_occupancy_map()` raycasts every saved optimized-pose/depth pair
  with ground filtering;
- local grids are accumulated additively, retaining both free and occupied
  evidence rather than using the Hub mapper's irreversible maximum;
- finalized `occupancy_grid.npy` is `[x,y,z]` uint8 with
  `0=unknown, 1=free, 2=occupied`;
- TinyNav's own 2-D export is `max(grid, axis=2)`;
- `occupancy_meta.npy` is `[origin_x, origin_y, origin_z, resolution]`;
- the occupancy/SDF files are generated inside `save_mapping()`, not
  continuously published as a live ROS occupancy topic;
- online operation publishes the refined pose-graph trajectory, local map,
  and visualization products while occupancy generation remains a final-save
  stage.

This corrects the shorthand description "live 2-D BuildMap map": BuildMapNode
does live keyframe collection, loop closure, and pose-graph refinement, but its
navigable occupancy artifact is finalized on stop.

## Adapter implementation

`hub/src/focus_hub/tinynav_occupancy.py` validates the two native arrays with
`allow_pickle=False`, preserves TinyNav's Z-collapse exactly, and transposes
native `[x,y]` into Hub `[row=y,column=x]`. Hub channel 0 is occupied, channel
1 is known (free or occupied), and semantic channels remain zero. It records
the resolved source paths, byte sizes, hashes, native origin, resolution,
session frame, and transform version.

`hub/tools/import_tinynav_occupancy.py` writes a new snapshot only and refuses
to overwrite one. `hub/robot_overlay/run_tinynav_buildmap_live.py` bypasses
the source script's mandatory BagPlayer, refuses an existing output path, and
exits successfully only after TinyNav's existing `/benchmark/stop` callback
sets `_save_completed`; an interrupt does not silently claim a finalized map.

`hub/tools/foxglove_relay.py` now reads an optional `frame_id` from a snapshot
(old snapshots default to `shared_world`). Fusion additionally refuses maps
whose frame ids differ.

## Tests and real-artifact result

All 137 Hub tests passed, including seven new native-adapter tests covering
axis placement, state precedence, metadata, hashes, overwrite refusal, and
fail-closed validation.

The immutable 2026-07-17 real WSJ BuildMap record was converted:

- native volume: `[155,117,23]`, origin
  `[-10.1000004,-5.6999998,-1.1000000]` m, resolution 0.1 m;
- Hub grid: `[17,117,155]` in frame
  `wsj_tinynav_buildmap_20260717_world`;
- 12,642 unknown, 1,969 free, and 3,524 occupied XY cells;
- 5,493 explored cells total and zero invented semantic cells.

Reconstructing TinyNav's grayscale convention from the adapted Hub channels
and transposing back produced a byte-for-byte pixel match with the native
`occupancy_2d_image.png`: **0 mismatched pixels**.

An independent relay was started in local tmux session
`focus_buildmap_native_relay`, WebSocket port 8775 and preview/health port
8776. It does not replace the existing port-8765 relay and does not enable
fusion. A real Foxglove SDK WebSocket subscription advertised
`/wsj_buildmap_native/semantic_map` as `foxglove.Grid` and received a 72,731 B
binary map message.

## Provenance

| Artifact | Size | SHA-256 | Status |
|---|---:|---|---|
| `hub/src/focus_hub/tinynav_occupancy.py` | 6,825 B | `97b1e313fa495db250728852e9a1e6cde5fd557729174e2fada3c4b08b609d7e` | implemented/tested |
| `hub/tools/import_tinynav_occupancy.py` | 1,676 B | `13cd141ee83b4a076fe90f2867e4a7218e8dd2151d2d3f049e24f6efb4a82f44` | implemented/tested |
| `hub/tests/test_tinynav_occupancy.py` | 3,598 B | `889d764c89179600d509e5d523cb856524ffeb55614143ef86f5e3fd335e72fe` | 7/7 passed |
| `hub/robot_overlay/run_tinynav_buildmap_live.py` | 2,851 B | `98a0bc0adb20af38da48aac1b8fe43b2c6275bbde513de8d049a0386a4f9f204` | live static-save gate passed |
| `hub/docs/TINYNAV_NATIVE_MAP_ADAPTER.md` | 2,551 B | `41f2206aee175b66936cfbc992189cfbfcd7766efc94c9f4097522a816a23214` | approved contract |
| `hub/tools/foxglove_relay.py` | 22,966 B | `6a61d1b3bafdd977392a2bea51c3617c6089250079010838179d753220d9ab9b` | tested/live relay |
| Native `occupancy_grid.npy` | 417,233 B | `e2e629bd83ae623634e1dbf8c25f6e0d5df2ff7ba00f44c955bfb31720edb982` | observed input |
| Native `occupancy_meta.npy` | 144 B | `62e4eb3c6313c6cbf4945f7058eda4d30957267a19ac486e3821515de5892cf0` | observed input |
| Native `occupancy_2d_image.png` | 1,724 B | `668dbcdf75e7252929ee1fc31550ff5bd945490a3ae580106e24c3a476a83d8c` | independent reference |
| Adapted `central_map.npz` | 5,142 B | `b0b9fdec17c37fccce2632c58aaa58cef80a5703341d3715a164474b44131da3` | observed output |
| Adapted `map_summary.json` | 1,365 B | `68162ec4ad7b121c1a463605df4cfa308ec2d07c806b8b19c913b1b9285e9e3b` | observed output |

## Live static save gate — PASS

The first launch attempt was not delivered because WSJ went offline and then
rebooted. After the reboot, the previously transferred wrapper inode was
present but its content was 0 B, consistent with power loss before file data
was durable. It was redeployed through a checked temporary pathname, atomically
renamed, `sync -f`'d, and reverified at 2,851 B with the expected SHA-256 before
execution.

With the robot stationary, the wrapper instantiated `BuildMapNode` directly
against the live ROS topics. It accumulated 16 keyframes over 76.056 s, added
loop constraints, and completed online Ceres refinements. The optimized
trajectory accumulated 9.60 mm of XY path but ended only 1.20 mm from its
start. Hub goal output remained disabled and no actuator process existed.

Publishing `{data: true}` once on `/benchmark/stop` exercised TinyNav's native
save callback while ROS was healthy. Observed results:

- `/benchmark/data_saved` published `true`;
- final pose-graph solve, TF publish, occupancy raycast, SDF calculation, and
  all file writes completed;
- `occupancy_grid.npy`, `occupancy_meta.npy`, `occupancy_2d_image.png`,
  `sdf_map.npy`, `poses.npy`, intrinsics/extrinsics, RGB/infra videos, depth,
  feature and embedding stores all exist;
- occupancy volume `[102,102,22]` contains 218,861 unknown, 9,119 free, and
  908 occupied voxels; its 2-D projection contains 8,979 unknown, 1,039 free,
  and 386 occupied cells;
- the saved PNG exactly matches the source algorithm's projected states (zero
  mismatched pixels);
- the wrapper logged `BuildMap finalized` and exited with no BuildMap process
  left behind. Perception remained `optimizer_status=ok` with zero invalid IMU
  reanchors at the post-save checkpoint.

The complete 36 MiB finalized record was copied read-only to
`data/robot_replays/wsj_buildmap_native_adapter_gate_20260721_2111`; every
remote/local payload hash matched before local write permissions were removed.
The adapter then produced
`hub/runtime/map_out_wsj_buildmap_native_static_20260721_2111`:

| Artifact | Size | SHA-256 | Status |
|---|---:|---|---|
| Live native `occupancy_grid.npy` | 229,016 B | `164c34e0350dd561fb77f66eeb220d9fbd35921fbccba48b96b88b6bbbaea974` | observed/copied/hash-matched |
| Live native `occupancy_meta.npy` | 144 B | `afd9b9001ccfabb33b9772a6149ba5568ffad3be769a2148b5b576a29327a980` | observed/copied/hash-matched |
| Live native `occupancy_2d_image.png` | 589 B | `dfbb83d96e1c042e867d3b6be1424b23d3ffcb147abfa7b65e5c39e6426de4f7` | observed/copied/hash-matched |
| Live native `poses.npy` | 3,145 B | `75820501345808df827008f64452391d1f18318278a4bf865254ef2fa4766f45` | observed/copied/hash-matched |
| Adapted live `central_map.npz` | 2,933 B | `ff161df2348ba25db9905175f1270ec1fbf65ac59a829bd837095dcc26c542c2` | tested |
| Adapted live `map_summary.json` | 1,384 B | `8e6574c0634da6a0a0e81973ddb6576034281039130c4bb0583f2be2855a40c8` | tested |

The independent port-8775 relay now publishes both the older moved reference
and this live static gate. A real subscription received a 41,813 B
`/wsj_buildmap_static/semantic_map` Grid message. Neither map is fused.

## Remaining gates

- Run a measured, controlled moved path in a fresh BuildMap output directory,
  save through the same native callback, and compare its optimized path and
  occupancy to the physical route.
- After reboot, the RealSense device appeared as `power/control=auto`; even
  after setting it to `on` before launch, driver binding changed it back to
  `auto`. It was manually re-pinned to `on` for this gate. The current udev
  rule therefore does not yet provide the claimed reboot/bind persistence and
  needs an interface-bind-aware follow-up before unattended use.
