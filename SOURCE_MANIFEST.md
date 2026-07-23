# Source and artifact manifest

## Remote provenance

- Source: `/scratch/jl9356/Focus_realworld` on Torch HPC (`ssh torch`).
- Source inventory at audit: 102 GB, 3,041 files, 1,087 directories, 33 symlinks.
- Remote environment reference: Apptainer 1.5.2, shared CUDA 12.6.3 SIF, and `overlay-15GB-500K.ext3`.
- Local artifact transfer is through a loopback-only temporary rsync service over the already-shared SSH session.  It is read-only and will be removed after completion.

## Copied and validated

`rsync -rcni` reported no output for the copied core source, selected metadata, RedNet source, and Habitat source.  That is a byte-identical validation of the selected files.

| Local path | Contents |
| --- | --- |
| `source/Focus_realworld/` | upstream code, configs, docs, small data metadata; no datasets, checkpoints, or HF cache |
| `dependencies/RedNet/` | code imported by `agents/vlm_agents.py` |
| `dependencies/habitat-lab/` | source/configs from modified MCoCoNav checkout; no `.git` history |
| `source/Focus_realworld/data/datasets/objectnav_hm3d_v2/minival/` | two selected episode files, 30 total episodes, metadata only |

All 135 copied Python files passed AST parsing.

Selected upstream SHA-256 values:

| File | SHA-256 |
| --- | --- |
| `running_inference.md` | `c295cc0d4b97dc7d6532a6a949a22764a1a801e241d7f110febe44a2a14123d7` |
| `README.md` | `d7e3eed94e1da1e5d8c95d196c79b3aa2f6839b00e0bdafe312766a8fdd01c00` |
| `main.py` | `0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674` |
| `arguments.py` | `66dc9a94459215d9a51d97bf8f195fd486759d7f34529c60e2a57999665a61d3` |
| task YAML | `b4dd539bd886cd6b17c794b04fceda705577c08c684965e30ba46066c5f0c498` |
| GLM server | `991bb7a288a69c36f2ad3999f63e5908d8044b9d6bc2adef71c8f432a58526f6` |

## Explicit external requirements found

The bundle is not self-contained.

- `main.py` imports `RedNet.RedNet_model` unconditionally, but no `RedNet` package exists within the source bundle.
- Habitat resolves from `/scratch/jl9356/MCoCoNav/habitat-lab` in the documented HPC environment.
- `room_semantic.py` calls `clip.load("ViT-B/32")`.  The required checkpoint was found at `/home/jl9356/.cache/clip/ViT-B-32.pt`, not inside the project or its HF cache.
- The GLM offline cache is the Hugging Face layout `hf_cache/hub/models--THUDM--glm-4v-9b/` (reference `3376fea6e54db68587a89bf1ac27a6889bafb867`), mirrored under `artifacts/models/hf_cache/`.
- Documented startup must make the source, MCoCoNav root, and Habitat source visible in `PYTHONPATH`.

The upstream `requirements.txt` is incomplete; the overlay is the actual dependency lock, but is intentionally not copied to the hub.

## Deliberate exclusions

No large simulator corpus, SIF, overlay, or unused HF CLIP-L/14 cache is copied.  See [CENTRAL_DEPLOYMENT.md](CENTRAL_DEPLOYMENT.md) for rationale.

## Target-machine verification (2026-07-18)

The prepared workspace was copied to `/home/asus/Research/focus_realworld_workspace`. Content-checking rsync dry-runs for `source/`, `dependencies/`, and `artifacts/` produced no differences. The target then read and hashed every GLM cache blob and the three standalone checkpoints; see [audit/G0_LOCAL_VERIFICATION.md](audit/G0_LOCAL_VERIFICATION.md). New files under `hub/` are target-specific adapter code and are not upstream source.

## Git snapshot boundary (2026-07-21)

The immutable, non-generated `source/` and `dependencies/` snapshot is now
covered by 248 per-file SHA-256 entries in
`manifests/source-files.sha256`. Two relative dataset symlinks are recorded
separately in `manifests/source-symlinks.txt`. Python bytecode, notebook
checkpoints and empty dataset directory skeletons are not versioned.

This Git import did not edit source or dependency content. The sole upstream
CRLF file has an explicit `.gitattributes` rule so a clean checkout reproduces
its original bytes and passes the manifest check.

Large model identities moved to the machine-readable
`manifests/artifacts.json`; the files themselves remain ignored.

## Yunji Odin1 external driver delta (2026-07-22)

The Odin driver is not copied into `source/` or `dependencies/`. The observed
external source is `https://github.com/manifoldsdk/odin_ros_driver.git`, tag
`v0.13.0`, commit `13aa528b1da581e2168ac858f8b144f0b4438a7a`. Its exact
four-file Yunji working-tree delta is preserved as a 10,346-byte patch at
`hub/robot_overlay/odin1_snapshot/` with SHA-256
`2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806`.
The serial-specific factory calibration and vendor binary SDK remain external;
their observed identities are recorded in
`hub/config/calibration/odin1_O1-P070100205_factory_20260722.json` and
`audit/YUNJI_ODIN1_INTEGRATION_20260722.md`.

## Current robot deployment snapshot (2026-07-24)

The current code-availability snapshot is the persistent-session
implementation at Git object
`90dd8fe43dad16515017fe4fd9bd017e02277bf6`. It was synchronized without
starting or restarting robot processes to:

- `/home/nvidia/topofocus_buildmap_v2_20260723`
- `/home/nyu/topofocus_buildmap_v2_20260723`

The `git archive <object> hub` transfer contained 326 entries, was 2,133,790
bytes and had
SHA-256
`4298f048591ca8b6a7cfa9d9aa3fe3ba34058965329f32bfba827af72f2a097f`.
Both robot computers independently observed that hash before extraction and
matched a 175-record critical runtime manifest afterward. The manifest was
23,560 bytes with SHA-256
`bc16cbaa3337b1e27237de64e88d5e0c94cc7e81e64365f95d625f86142bb6bf`.
All 196 Python files in the archive parsed and all 39 shell files passed
`bash -n` on both robots. `source/`, `dependencies/`, runtime maps,
environments, caches and credentials were excluded. See
[`audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md`](audit/REPOSITORY_AND_ONECLICK_AUDIT_20260724.md).

This transfer proves byte availability, not that a process loaded the new
files. The observed final process state was WSJ receiver 0 / Go2 bridge 0 and
Yunji receiver 0 / live and debug services inactive. The persistent
calibration/debug/live workflow remains physically unverified.

The superseded retry3 archive remains preserved as historical evidence in
[`audit/DUAL_ROBOT_CODE_SYNC_20260723.md`](audit/DUAL_ROBOT_CODE_SYNC_20260723.md):
392 entries, 2,371,165 bytes, SHA-256
`e1b9001fb188a3890037f5e33927d25afa44473fb50a6b8c40b61a6e123b1b72`.

## Runtime-only calibration and evidence boundary

Physical-session calibration, map and episode artifacts remain outside Git.
The last predecessor identifiers were:

- shared calibration ID `shared-board-odin1-20260723-v3`;
- WSJ transform `wsj-tinynav-depth-20260723-powercycle-v3`;
- Yunji transform `yunji-odin1-board-20260723-powercycle-v6`;
- current map directories under ignored `hub/runtime/`;
- robot-local measured calibration JSON files under
  `~/.local/state/topofocus/calibration/`;
- exact receiver and bridge logs under robot-local
  `~/.local/state/topofocus/`.

Their observed paths, sizes and checksums are recorded in
[`CURRENT_STATUS.md`](CURRENT_STATUS.md) and the dated audit records. They must
not be reconstructed from an undocumented assumption or committed as
credentials/runtime state. A strict persistent `current` pointer is generated
only by `calibrate_realworld_session.sh`; the legacy v3 artifact is not
silently promoted.
