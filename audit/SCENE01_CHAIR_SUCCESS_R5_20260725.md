# Scene 01 chair automatic success r5 — 2026-07-25

## Outcome

Session `20260725-lab19-scene01-8ca1d52-yunjireboot1-r5`, scene
`scene01-chair`, episode `trial-r5-01` is the second normal success in the
separate `physical_0p5m_protocol` track.

WSJ (`robot-0`) emitted `LOCAL_PLANNER_ARRIVED` for a semantic-region target,
stopped, and triggered the automatic terminal-evidence bundle. Its stop pose
was `0.406692832069 m` from the selected chair semantic goal, inside the
declared `0.5 m` physical radius.

| Metric | Value |
| --- | ---: |
| SR | `1.0` |
| Actual WSJ path `P` | `3.8507920422516904 m` |
| Start-to-stop displacement `D` | `2.419833573 m` |
| Source-compatible SPL | `D / max(D, P) = 0.628398923` |
| Standard SPL | unavailable |

Standard SPL remains unavailable because no independently surveyed shortest
collision-free path `L` was recorded before the run. No value is inferred
after the fact.

Together with `trial-05-nearwall-fix`, the physical 0.5 m track now contains
two normal successes: `SR=2/2=1.0` and mean source-compatible
`SPL=0.746223480504199`.

## Runtime evidence

The automatic episode report records:

- start pose `(-1.048071261143531, 0.2622543944229162)`;
- semantic goal `(-1.3887036027715443, 3.0674017103544813)`;
- arrival pose `(-1.3167335325004346, 2.6671275934781966)`;
- navigation status `ARRIVED`;
- reason code `LOCAL_PLANNER_ARRIVED`;
- local planner stopped: `true`;
- both robots held after arrival: `true`;
- automatic terminal candidate complete: `true`.

The frozen WSJ decision RGB at sequence `27693` contains the chair. The
automatic post-arrival WSJ RGB faces the wall and is therefore retained as a
timing-aligned terminal observation, not used alone to identify the target.
The paired external-view recording shows the white chair and both robots at
the end of the run.

The later r6 cross-map alignment rejection does not turn this completed result
into a VLM failure: r5's arrival distance and path are in WSJ's robot-local
navigation frame, and the external view establishes physical chair proximity.
The fused Dashboard is visualization context only and is not used as an
independent distance measurement for this metric row.

## Media publication

The two user-provided masters remain byte-for-byte unchanged. Browser-friendly
copies are H.264 High, `yuv420p`, fast-start MP4 without audio.

| Evidence | Bytes | Duration/source time | SHA-256 | Classification |
| --- | ---: | ---: | --- | --- |
| `media/video/third_view/experiment_1/experiment_1_success_1.mp4` | 8,511,523 | 74.100 s | `3d67f19f48601dcd239b950689c391b96231010bc2c6fdea07351f0ff12c8bd6` | observed user-provided external-view master; Git LFS |
| `media/demo/scene01_success_2_third_view_20260725.mp4` | 10,359,239 | 74.100 s | `0469c894b80035a30cdc5d8fb4142577c7bebf53ace00f71528147046ff35675` | source-derived H.264 public copy |
| `media/demo/scene01_success_2_third_view_20260725_poster.jpg` | 188,224 | 73.0 s | `7611b39fae911b2def21018067240881b3910d04e949c4ec822e67fa01f339ac` | source-derived terminal still |
| `media/video/dashboard/experiment_1/experiment_1_success_1_dashboard.mov` | 45,502,070 | 74.955 s | `4668756e7d7ad7e2dd18eab85d65de5c8243ef3c9b281afcc6aace1f942291dd` | observed user-provided Dashboard master; Git LFS |
| `media/demo/scene01_success_2_dashboard_20260725.mp4` | 1,175,482 | 74.900 s | `d30636dad6e9218b91f966c493b50eca9b82229bf3d9be4f45e6e25f0755cbf5` | source-derived H.264 public copy |
| `media/demo/scene01_success_2_dashboard_20260725_poster.jpg` | 77,100 | 73.0 s | `15d9c77a8c525036cf7e3adb78c454e53391e1067656b5f2d8e3c160d244e13d` | source-derived Dashboard still |

The observed local encoder was `ffmpeg` 4.4.2 with `libx264`. The external
view used CRF 25 at 1280 × 720. The Dashboard used CRF 26 scaled to
1280 × 578.

## Runtime provenance

Runtime files remain ignored by Git; their paths, sizes and hashes make the
committed result auditable.

| Runtime evidence | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/trial_annotations/scene01-chair_trial-r5-01_success.json` | 2,960 | `bdcbadce63237de14804e4d27d99849ff0af7b4732c18572588dcd7d1eccf223` | observed onsite success record joined with source-derived runtime evidence |
| `hub/runtime/oneclick_20260725-lab19-scene01-8ca1d52-yunjireboot1-r5_live_scene01-chair_20260725_122623_471712554/episode_report.json` | 20,686 | `e25e6853c4272be698b3049a3fec11a9f296c1d8d7b55397a82e5631b6c264c4` | source-derived from observed robot feedback |
| `hub/runtime/oneclick_20260725-lab19-scene01-8ca1d52-yunjireboot1-r5_live_scene01-chair_20260725_122623_471712554/terminal/terminal_evidence.json` | 10,605 | `64bd6c7d3ac26c5b245d08cc8627a9d9e45209babd7eda0fb2a6714c55610c62` | source-derived terminal evidence index |
| `hub/runtime/spool/robot-0/00000000000000027693/rgb.jpg` | 84,007 | `7c9f08f8e0fc8541c6166f0fd53ae2040f746ee7908d5b93fbe89ab5f2a97d9a` | observed frozen decision RGB |
| terminal `robot-0/rgb.jpg` | 91,042 | `2df63768a7f582e64d2476014d0f90c75c7ce4cfe638bdb6aa7141247d3ade8e` | observed post-arrival RGB |

This record changes no file under immutable `source/` or `dependencies/`.
