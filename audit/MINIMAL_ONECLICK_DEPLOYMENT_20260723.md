# Minimal one-click deployment — 2026-07-23

## Scope

This record covers the minimal two-mode real-world entry point requested for
the current WSJ + Yunji session.  No file under `source/` or `dependencies/`
was modified.  Robot-side deployment was installed into new release roots:

- WSJ: `/home/nvidia/topofocus_buildmap_v2_20260723`
- Yunji: `/home/nyu/topofocus_buildmap_v2_20260723`

The old release roots were not overwritten.

## Transport and deployed inputs

The initial bundle contained `hub/src/focus_hub`, the required
`hub/robot_overlay` entry points, and the Odin1 factory calibration.  It was
transferred through the already-running SSH/tmux sessions, not a new SSH
session.

- Local temporary source:
  `/tmp/topofocus-v2-minimal.fER6xo/topofocus_buildmap_v2_20260723.tar.gz`
- Size: `162224` bytes
- SHA-256:
  `ab98eddbe4e00c2d7b71d6a093c9380f1aee290ffc8f42b8a7ec337d30b4ce7c`
- WSJ result: observed same size and SHA-256 before extraction
- Yunji result: observed same size and SHA-256 before extraction

Live smoke testing exposed small launcher/packaging defects.  The following
files were then transferred individually and their remote SHA-256 values were
observed to match the final local files:

| Final local source | Size (bytes) | SHA-256 |
|---|---:|---|
| `hub/robot_overlay/start_wsj_command_observation.sh` | 4948 | `11323b5f46a45cd46aee0ec3b3eeb0cb2e4a786a58c797aead3dcee6e2396c27` |
| `hub/robot_overlay/start_go2_buildmap.sh` | 2428 | `ba71b627174baf06b97fc3612aa9a3f134a4afa6f140796298a2e933a15e752e` |
| `hub/robot_overlay/v2_wsj_receiver.py` | 43421 | `6725369578825fdabf87ae093e304557ddc3b62ad17568e8295038dd258bfd47` |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 5352 | `89b127b7daf52abfab7632f1c8c9d54a97428a671804c45f1f4ff3eb76fe869c` |
| `hub/robot_overlay/start_yunji_v2.sh` | 5405 | `f20eab776820c4ff69f306437f9f8935db7cd1075f81c93edacf1ff990b28e65` |
| `hub/robot_overlay/yunji_sender.py` | 65704 | `4455b70106aefcb9b2415f79e2dd63e448ea7010a5baf6b0abf342a305c87242` |

The local-only entry point is
`hub/scripts/realworld_oneclick.sh`, size `9212` bytes, SHA-256
`fdd53dcd981ad5299dd18bdfc1f0b40de93b399154665cc4704b81835db9148c`.

## Observed debug smoke result

Command:

```bash
bash hub/scripts/realworld_oneclick.sh \
  --mode debug \
  --goal-category chair \
  --scene-id debug-chair-20260723
```

Observed result:

- status: `DEBUG_FULLSTACK_READY`
- Hub: `127.0.0.1:8188`, both robot GOAL policies `false`
- GLM endpoint: `127.0.0.1:31511`
- Foxglove WebSocket/preview: ports `8765` / `8766`
- WSJ windows alive: camera, perception, Hub sender, native BuildMap,
  online occupancy, planning, goal router, controller, and read-only v2
  receiver
- WSJ Go2 bridge: no matching process and no tmux window observed
- Yunji services alive: Odin driver, command-capable observation sender, and
  read-only v2 receiver
- VLM result: robot-0 selected a `chair` semantic mask; robot-1 selected
  remaining frontier `C`
- Hub publication: both robot decisions were observed as `HOLD`, never `GOAL`

Frozen manifest:

- Path:
  `hub/runtime/oneclick_debug_debug-chair-20260723_20260723_190510/shadow/shadow_manifest.json`
- Size: `18861` bytes
- SHA-256:
  `1b8bbdc1b60d1609ad97521c1a0476a77a6ea06e77f208f332028beb48568117`
- Status: observed `complete_shadow_only`

The debug run explicitly used `allow_blocked_shadow_input=true` and
`allow_stale_shadow_input=true`.  Those exceptions are restricted to the
non-command shadow path, preserved in the manifest, and are not added to live
mode.

Final local validation after the smoke-test fixes:

- shell syntax: passed for the one-click and WSJ/Yunji launchers
- Python bytecode compilation: passed for both receivers, the online router
  and mapping wrapper, and the Odin/Yunji senders
- focused regression suite: `49 passed in 0.92s`
- temporary local and remote transfer files: removed after checksum and
  extraction verification

## Provenance classification

- Observed: bundle hashes on both remotes; running processes/services; Hub
  GOAL policies; debug VLM output; HOLD responses; no WSJ Go2 bridge; WSJ
  `eth0` state.
- Source-derived: online WSJ `world_T_world` identity alignment and the
  high-level BuildMap/POI routing contract.
- Unverified for live motion: physical goal execution, path tracking,
  arrival, cancel, and dual-robot SR/SPL episode completion.

## Remaining live blockers observed at handoff

- WSJ `eth0` is `DOWN`, so the guarded Go2 bridge cannot be armed.
- WSJ command readiness is `HEALTH_NOT_READY`; current `/slam/data` reports
  legacy/incomplete optimizer evidence (`Infinity` errors and no validated IMU
  interval report).
- The frozen WSJ central map used by shadow mode is blocked for ground-plane
  drift and stale.  Live mode continues to reject this input.

Therefore the debug full stack is ready and repeatable, while live motion is
intentionally still fail-closed.
