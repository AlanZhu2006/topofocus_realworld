# Repository and workspace organization — 2026-07-29

## Outcome

The local repository and physical-runtime workspace were consolidated without
deleting experiment evidence:

- the detached Scene 03 repair line was anchored as
  `agent/scene03-runtime-recovery-20260729` and merged with the two newer
  `main` platform-figure commits;
- five previously untracked platform-figure source/PDF assets were validated
  and committed;
- `source/` and `dependencies/` remained unchanged;
- all 55 completed calibrations and all 163 real-world session contracts
  passed schema, artifact-size and artifact-hash validation;
- 30 calibration attempts that never produced `shared_frame.json` were moved
  intact under `hub/runtime/calibration_sessions/failed/`;
- the stale current-session pointer was archived and intentionally not
  replaced, so the next physical run cannot silently reuse it;
- three complete interrupted spool writes were hash-validated, restored to
  their canonical sequence directories and successfully decoded;
- two incomplete spool writes were retained under quarantine;
- temporary transfer and preview files under `/tmp` were removed after their
  committed or reproducible replacements were confirmed.

This organization issued no physical robot command.

## Git state and provenance

The observed starting point was detached commit
`b7884cc75369ff684282937e64a2bebe02247a8f`. It contained 27 commits not in the
then-current local `main`; local `main` and the cached `origin/main` both
pointed to `a25432305b2166c261fe24a374434624261e93e4`, which contained two
platform-figure commits absent from the detached line.

The two lines merged cleanly at
`821d6f3` because the Scene 03 line did not change the platform media. The five
additional platform artifacts were committed at
`3123abbaf7f4b1f8fe9e2a8effc401a150a47e81`. Their observed identities are:

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `media/image/platforms_figure1_annotated_crop.svg` | 321 | `1482c19da510531f8f650bd1cd33ae5438cd7deb3693dbf88efd60e9dd63c614` | locally generated SVG view over committed source image |
| `media/image/platforms_figure1_clean.pdf` | 5,687,832 | `8c51019af77b08f2d1568dbc15faecbb8e5c03ed41dd8bb741674dc7ac515a54` | locally generated one-page PDF |
| `media/image/platforms_figure1_clean.svg` | 1,888 | `23df1849c7c38d5a7b720e35fbdf5e6744c656d6ec813ed749dbffcf3dad1bb4` | locally generated SVG source |
| `media/image/platforms_figure1_original.svg` | 312 | `9745f3950a045947a30ad3975b35928d45dc6e227bf5c8478ad2375ce00526ea` | locally generated SVG view over committed source image |
| `media/image/platforms_figure1_three_versions.pdf` | 15,563,089 | `462f2685f9fe0d061eb43b90ae75e04853420072479c956c413cf990198182ba` | locally generated three-page PDF |

Both PDFs passed `pdfinfo` parsing (one and three pages respectively), and all
three SVG files passed XML validation.

An authenticated refresh of the remote refs was attempted but returned
`Repository not found`; consequently, remote state newer than the cached
`origin/main=a254323` remains unverified. No push was attempted.

## Runtime safety boundary

The Hub tmux session was stopped and ports `8188`, `8765` and `8766` were
released before runtime files were moved. WSJ retained only its
camera/perception observation sender; no WSJ command bridge, planner or
velocity-control process was observed. The existing Yunji SSH session had
timed out, so Yunji process state was not remotely re-observed; local Hub goal
output was already disabled before shutdown.

The GLM service was deliberately left warm on loopback
`127.0.0.1:31511` to avoid another model-load delay. It has no robot command
interface. At the checkpoint it used 31,205 MiB of 49,140 MiB GPU memory.

## Calibration and session organization

All 55 directories remaining directly under
`hub/runtime/calibration_sessions/` contain a non-empty
`shared_frame.json`. All 163 `hub/runtime/sessions/*/session.json` files
validated against their recorded calibration and generated robot-config
artifacts. Of those sessions, 107 retain strict-debug evidence and 56 do not;
that distinction was preserved.

The previous pointer was moved from `hub/runtime/sessions/current.json` to:

`hub/runtime/sessions/pointers/current-invalidated-scene03-plant-long-20260729-173359-wsjreanchor1-20260729T112720Z.json`

Its observed identity is 204 bytes,
SHA-256 `1ef32ec41d8160db47837b1463f61e0cd1ee065760a37d6f3309d0f591506e9c`.
There is intentionally no current pointer after organization.

The associated read-only map diagnosis remains at
`hub/runtime/diagnostics/scene03-wsjreanchor1-mapcheck-20260729T1748`.
It contains eight files totaling 125,437 bytes; the deterministic tree
manifest SHA-256 is
`c381948b8b9e82ddbfa70e2d3fc3eb51790fad8ee558aff7a01fc822c203cb19`.

### Incomplete-calibration archive manifest

Every row below is classified as an observed incomplete local calibration
attempt: no `shared_frame.json` was produced and no physical command was
issued by this archival operation. The destination is
`hub/runtime/calibration_sessions/failed/<ID>-workspace-archive-20260729T112720Z`.
The tree hash covers sorted relative path, file size and file SHA-256 tuples.

| ID | Files | Bytes | Tree SHA-256 |
| --- | ---: | ---: | --- |
| `20260724-lab01` | 2 | 634 | `ee6e9d15d493b8f76d44ff20739e1285a26c387c00ca0f0aa3611730e9647c48` |
| `20260724-lab02` | 2 | 634 | `ac9ca8b38862e5af767b56c35875329ae858d5be8b490ad7da2b56f43defba0e` |
| `20260724-lab03` | 2 | 634 | `ff118c32ae3c23aeef1358ef3d729876403d9429f4f66f743f4dbb67e9515e42` |
| `20260724-lab04` | 2 | 634 | `5c0c3004a513d9c9e7c2b86722bab42db9c0d500e67fe5897d5d0821da79ef42` |
| `20260724-lab05` | 2 | 634 | `280bf632bbfd26e9d4dc8faced3715c7b674582a30e80c183b489707c3188345` |
| `20260724-lab06` | 2 | 634 | `f093d691caa085a23c24e2b31d761934d6cb8bdb49da534e6a12f1f55bc47aa7` |
| `20260725-lab02` | 6 | 12,139 | `9820957f7702fb98821d0c5abf9e26a493702553d5fccf3df1c6e3fa1fa9ac50` |
| `20260725-lab03` | 3 | 1,705 | `8fcb5c5f3de257294313ff005d180219bebcc53aa3f3ec56816680888b01af5a` |
| `20260725-lab06` | 4 | 2,550 | `4904004b026fbc8647d9610d08861917bdb7ae778f9077d4e8016c7be601cae6` |
| `20260725-lab07` | 7 | 13,383 | `048c4b547e97c4ac9b1076b050ef51b70a33f0f7c92cfde38508355af5d6ed36` |
| `20260725-lab09` | 3 | 1,101 | `24fe0b396e7036497ee0ad09c7a5ebfb7c64d30458faa61a2869a813054b2b25` |
| `20260725-lab10` | 3 | 1,101 | `b1fe23f7feac60d0364f6e434021e563a3e80e22ed03b8c12ed8737a3de0b571` |
| `20260725-lab11` | 3 | 1,101 | `e35ad51fd745358abd2424d9c03b85226cd9e908eb6265f16971a292fa7b0bf7` |
| `20260725-lab13` | 3 | 1,101 | `7d4e775fe8125038131eea4ea28b8ba258349dd2cf7f667abaf95a4fb95c6799` |
| `20260725-lab14` | 4 | 2,526 | `2d7299a9b99f04e5bde9f486925e2f174b28d6dce608dc0134d59a1b70182271` |
| `20260725-lab18-repeat2` | 3 | 1,719 | `afb2b8934b9e062dded962166fc1bc00c47ac30459de47cf7d5f349b57a70553` |
| `20260725-lab20-scene01-run02-recal` | 3 | 1,339 | `3c3be1f57da37559457b850c0e6225f264593b1a1848039241e23b37281a1316` |
| `20260725-lab25` | 3 | 1,702 | `1b9a1c8e5bd520bc761ec1406a4341289d6e4d325e13e7d56b5c70431a13ac05` |
| `lab-20260725-131322` | 3 | 1,121 | `df86948bf457890e73bd2dff15fdbd6d8c34d93bc41fdc61f02ea9fa566383ff` |
| `lab-20260725-131655` | 3 | 1,121 | `18aa0ad022ed962c415dc73b682b344e731e7809dcbe92560e290cc48ed0d1b4` |
| `scene02-plant-20260725-201311` | 3 | 1,161 | `768977da122c4e8001d5c582c9d6c3f31967275d7e22b6c531aa3b5c17a17e9b` |
| `scene02-plant-20260727-143913` | 3 | 1,161 | `c2864d6957c401b0b3d7eeaec88ad6608774f66524955128969573247d19a90c` |
| `scene02-plant-20260727-144138` | 3 | 1,161 | `0c21c7c15049473eb8640a1333dd82c03a54b887a0fa64043b6e11bb8b934a54` |
| `scene02-plant-20260727-145443` | 3 | 1,161 | `398b915b2cade62f183b8ec5cbe2ce331e3facc1d9d53f459abced09c0d3c801` |
| `scene02-plant-20260728-043817-recal1` | 4 | 4,040 | `556052de25b2724ebea35b2b23be403eae553250e3063ee301564c941134cc79` |
| `scene02-plant-20260728-061806-recal3` | 4 | 3,068 | `bc201f4826a463301776294b924679491d9ce6170b15107876071e7e46f15309` |
| `scene03-plant-long-20260728-112729-recalibration` | 7 | 13,554 | `0e4096d0dab76b7e4bb9a2c18ffbb54324e1b18603ebe7e4556c4b19c1f9248f` |
| `scene03-plant-long-20260728-121028-recalibration` | 3 | 1,237 | `2099c5241c48ec833de351d7c08c3ab8b3fe5091906a7f86c18a042744caf797` |
| `scene03-plant-long-20260728-1818-recalibration3` | 4 | 7,057 | `74deea04e0a731d00d14ac0dfca542ed4a422878ed52e742c2dafb16e830aaa6` |
| `scene03-plant-long-20260729-175508-recalibration5` | 3 | 1,515 | `58d57c49438f4f05ac1b2e3d2e95c154b5442f5b560f4a846dbd605acaf29dda` |

## Spool organization

The append-only spool was retained. Its post-organization inventory is:

| Robot | Valid sequence directories | Bytes |
| --- | ---: | ---: |
| `robot-0` | 113,446 | 29,271,402,845 |
| `robot-1` | 344,547 | 106,277,473,214 |

Three hidden interrupted writes contained valid metadata and byte-identical
RGB/depth payload hashes. They were restored as sequences `robot-0/78365`,
`robot-1/283573` and `robot-1/313617`. Each restored frame passed the normal
RGB/depth decoder and yielded the recorded image dimensions.

Two non-recoverable interrupted writes were moved, not deleted, to
`hub/runtime/spool/quarantine/20260729T112720Z`. They total 198,736 bytes and
have deterministic tree SHA-256
`0b6b8206a3099984ab66ed29112cee3ab3f08644a9106d8be1aeb7660eaee34f`.
One contains only RGB/depth and the other a zero-byte RGB; neither has
metadata, so neither is a protocol-valid observation.

## Repository validation

The repository verifier completed successfully after one stale test
expectation was aligned with the intentional generic mapping-only readiness
marker introduced for both board calibration and stationary re-anchor:

- source manifest and immutable snapshot checks passed;
- 369 Python files passed AST parsing;
- 27 JSON and 75 YAML files parsed;
- shell syntax, forbidden-path, large-file/LFS, whitespace and credential
  checks passed;
- all 677 Hub tests passed;
- README has 54 local references and none are missing;
- Git LFS reports no staged or missing objects.

The five apparent hash differences found while scanning historical manifests
are explicitly versioned historical identities: two point to the superseded
Scene 02 Formal 02 record and three point to mutable code/command paths at the
older Scene 02 preparation snapshot. No referenced artifact is missing.

## Storage boundary

Observed local usage after organization:

| Area | Size |
| --- | ---: |
| `hub/runtime/spool` | 129 GiB |
| `.git` | 1.7 GiB |
| `.git/lfs` | 1.3 GiB |
| `media` | 1.5 GiB |
| `hub/runtime/calibration_sessions` | 3.1 MiB |
| `hub/runtime/sessions` | 2.9 MiB |

The filesystem had 307 GiB free (83% used). The 129 GiB spool remains the
dominant candidate for a future checksum-backed cold archive, but no valid
spool frame was deleted or relocated in this pass.
