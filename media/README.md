# System architecture

`image/system_architecture.png` is a source-derived visualization generated
from `hub/config/deployments/realworld_dual_robot_v1.json` and manually checked
against its Hub/Robot 0/Robot 1 authority flow.

| Path | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `image/system_architecture.png` | `1,124,950 B` | `333baf2ab81cb1c24acebf89737b12442abf84cee8ed1980e85cc7c6b7dd63df` | source-derived generated visualization |

# Scene 01 media

Scene 01 contains five formal successful real-robot experiments. User-provided
master videos are byte-preserved under `media/video/**` through Git LFS;
browser-ready H.264 files and inline GIF previews are under `media/demo/`.
The explored semantic map is
[`experiment_1_map.png`](image/experiment_1_map.png).

| Run | Third-view master | Dashboard master | README previews |
| --- | --- | --- | --- |
| Formal 01 | [`experiment_1_success_1.mp4`](video/third_view/experiment_1/experiment_1_success_1.mp4) | [`experiment_1_success_1_dashboard.mov`](video/dashboard/experiment_1/experiment_1_success_1_dashboard.mov) | [Third view](demo/scene01_formal_01_preview.gif) · [Dashboard](demo/scene01_formal_01_dashboard.gif) |
| Formal 02 | [`experiment_1_success_2.mp4`](video/third_view/experiment_1/experiment_1_success_2.mp4) | [`experiment_1_success_2_dashboard.mov`](video/dashboard/experiment_1/experiment_1_success_2_dashboard.mov) | [Third view](demo/scene01_formal_02_preview.gif) · [Dashboard](demo/scene01_formal_02_dashboard.gif) |
| Formal 03 | [`experiment_1_success_3.mp4`](video/third_view/experiment_1/experiment_1_success_3.mp4) | [`experiment_1_success_3_dashboard.mov`](video/dashboard/experiment_1/experiment_1_success_3_dashboard.mov) | [Third view](demo/scene01_formal_03_preview.gif) · [Dashboard](demo/scene01_formal_03_dashboard.gif) |
| Formal 04 | [`experiment_1_success_4.mp4`](video/third_view/experiment_1/experiment_1_success_4.mp4) | [`experiment_1_success_4_dashboard.mov`](video/dashboard/experiment_1/experiment_1_success_4_dashboard.mov) | [Third view](demo/scene01_formal_04_preview.gif) · [Dashboard](demo/scene01_formal_04_dashboard.gif) |
| Formal 05 | [`experiment_1_success_5.mp4`](video/third_view/experiment_1/experiment_1_success_5.mp4) | [`experiment_1_success_5_dashboard.mov`](video/dashboard/experiment_1/experiment_1_success_5_dashboard.mov) | [Third view](demo/scene01_formal_05_preview.gif) · [Dashboard](demo/scene01_formal_05_dashboard.gif) |

Formal 05 display media covers the beginning of the run through physical
arrival. Metric path length remains the complete runtime odometry value.

Exact source paths, sizes, durations, SHA-256 values, evidence classes,
runtime bindings and derivative identities are recorded in
[`scene01_chair_formal_experiments_20260725.json`](../manifests/scene01_chair_formal_experiments_20260725.json).

Development recordings are retained separately as engineering-process
evidence and indexed in
[`SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md`](../audit/SCENE01_ENGINEERING_DEBUG_INDEX_20260725.md).

# Scene 02 media

Scene 02 contains five archived formal episodes: two failures and three
operator-confirmed successes. User-provided master videos are
byte-preserved under `media/video/**` through Git LFS; browser-ready H.264
files and inline GIF previews are under `media/demo/`.
The explored semantic map is
[`experiment_2_map.png`](image/experiment_2_map.png).

| Run | Third-view master | Dashboard master | README previews |
| --- | --- | --- | --- |
| Formal 01 · FAILURE | [`experiment_2_failure_1.mp4`](video/third_view/experiment_2/experiment_2_failure_1.mp4) | [`experiment_2_failure_1_dashboard.mov`](video/dashboard/experiment_2/experiment_2_failure_1_dashboard.mov) | [Third view](demo/scene02_formal_01_preview.gif) · [Dashboard](demo/scene02_formal_01_dashboard.gif) |
| Formal 02 · FAILURE | [`experiment_2_failure_2.mp4`](video/third_view/experiment_2/experiment_2_failure_2.mp4) | [`experiment_2_failure_2_dashboard.mov`](video/dashboard/experiment_2/experiment_2_failure_2_dashboard.mov) | [Third view](demo/scene02_formal_02_preview.gif) · [Dashboard](demo/scene02_formal_02_dashboard.gif) |
| Formal 03 · SUCCESS | [`experiment_2_success_1.mp4`](video/third_view/experiment_2/experiment_2_success_1.mp4) | [`experiment_2_success_1_dashboard.mov`](video/dashboard/experiment_2/experiment_2_success_1_dashboard.mov) | [Third view](demo/scene02_formal_03_preview.gif) · [Dashboard](demo/scene02_formal_03_dashboard.gif) |
| Formal 04 · SUCCESS | [`experiment_2_success_2.mp4`](video/third_view/experiment_2/experiment_2_success_2.mp4) | [`experiment_2_success_2_dashboard.mov`](video/dashboard/experiment_2/experiment_2_success_2_dashboard.mov) | [Third view](demo/scene02_formal_04_preview.gif) · [Dashboard](demo/scene02_formal_04_dashboard.gif) |
| Formal 05 · SUCCESS | [`experiment_2_success_3.mp4`](video/third_view/experiment_2/experiment_2_success_3.mp4) | [`experiment_2_success_3_dashboard.mov`](video/dashboard/experiment_2/experiment_2_success_3_dashboard.mov) | [Third view](demo/scene02_formal_05_preview.gif) · [Dashboard](demo/scene02_formal_05_dashboard.gif) |

Formal 02 was replaced: the operator designated a later Robot 1-only run
(algorithmic exploration failure, 13 rounds without a plant semantic
arrival) as the official formal experiment 02. Its third-view and Dashboard
masters reuse the original `experiment_2_failure_2.*` filenames with
replaced content — the originally published Formal 02 media belonged to
the superseded run and remains recoverable in Git history at commit
`dcc8812b027c40fad2716b8a097e45d226d46686`, but is no longer current
evidence.

README previews time-lapse the entire master into an ~8 s / 8 fps loop,
matching the Scene 01 formal-experiment GIF convention. Exact source paths,
sizes, durations, SHA-256 values and evidence classes are recorded in
[`scene02_plant_formal_experiment_01_failure_20260728.json`](../manifests/scene02_plant_formal_experiment_01_failure_20260728.json) /
[`SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md`](../audit/SCENE02_PLANT_FORMAL_EXPERIMENT_01_FAILURE_20260728.md),
[`scene02_plant_formal_experiment_02_failure_20260728.json`](../manifests/scene02_plant_formal_experiment_02_failure_20260728.json) /
[`SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md`](../audit/SCENE02_PLANT_FORMAL_EXPERIMENT_02_FAILURE_20260728.md)
and
[`scene02_plant_formal_experiment_03_success_20260728.json`](../manifests/scene02_plant_formal_experiment_03_success_20260728.json) /
[`SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md`](../audit/SCENE02_PLANT_FORMAL_EXPERIMENT_03_SUCCESS_20260728.md).
Formal 04/05 and the unified campaign totals are recorded in
[`scene02_plant_formal_experiments_20260728.json`](../manifests/scene02_plant_formal_experiments_20260728.json).

# Scene 03 media

Scene 03 contains five archived long-range cooperative episodes: two
time-limit failures and three operator-confirmed successes. User-provided
master videos are byte-preserved under `media/video/**` through Git LFS;
browser-ready H.264 files and inline GIF previews are under `media/demo/`.
The explored semantic map, including the Robot 0 and Robot 1 trajectories, is
[`experiment_3_map.png`](image/experiment_3_map.png).

| Run | Third-view master | Dashboard master | README previews |
| --- | --- | --- | --- |
| Formal 01 · FAILURE | [`experiment_3_failure_1.mp4`](video/third_view/experiment_3/experiment_3_failure_1.mp4) | [`experiment_3_failure_1_dashboard.mov`](video/dashboard/experiment_3/experiment_3_failure_1_dashboard.mov) | [Third view](demo/scene03_formal_01_preview.gif) · [Dashboard](demo/scene03_formal_01_dashboard.gif) |
| Formal 02 · FAILURE | [`experiment_3_failure_2.mp4`](video/third_view/experiment_3/experiment_3_failure_2.mp4) | [`experiment_3_failure_2_dashboard.mov`](video/dashboard/experiment_3/experiment_3_failure_2_dashboard.mov) | [Third view](demo/scene03_formal_02_preview.gif) · [Dashboard](demo/scene03_formal_02_dashboard.gif) |
| Formal 03 · SUCCESS | [`experiment_3_success_1.mp4`](video/third_view/experiment_3/experiment_3_success_1.mp4) | [`experiment_3_success_1_dashboard.mov`](video/dashboard/experiment_3/experiment_3_success_1_dashboard.mov) | [Third view](demo/scene03_formal_03_preview.gif) · [Dashboard](demo/scene03_formal_03_dashboard.gif) |
| Formal 04 · SUCCESS | [`experiment_3_success_2.mp4`](video/third_view/experiment_3/experiment_3_success_2.mp4) | [`experiment_3_success_2_dashboard.mov`](video/dashboard/experiment_3/experiment_3_success_2_dashboard.mov) | [Third view](demo/scene03_formal_04_preview.gif) · [Dashboard](demo/scene03_formal_04_dashboard.gif) |
| Formal 05 · SUCCESS | [`experiment_3_success_3.mp4`](video/third_view/experiment_3/experiment_3_success_3.mp4) | [`experiment_3_success_3_dashboard.mov`](video/dashboard/experiment_3/experiment_3_success_3_dashboard.mov) | [Third view](demo/scene03_formal_05_preview.gif) · [Dashboard](demo/scene03_formal_05_dashboard.gif) |

README previews time-lapse each complete display clip into a 64-frame,
8 fps loop. Exact source paths, sizes, durations, SHA-256 values, evidence
classes, both robot trajectories and SR/SPL values are recorded in
[`scene03_plant_formal_experiments_20260731.json`](../manifests/scene03_plant_formal_experiments_20260731.json)
and its five linked per-run manifests.

# Scene 04 media

Scene 04 currently contains four archived failures. The media-backed Formal 01
and Formal 04 records are listed below; Formal 02 and Formal 03 are bound to
their runtime and metric evidence in the campaign manifest. User-provided masters are
byte-preserved through Git LFS; browser-ready H.264 files and 64-frame inline
GIF previews are under `media/demo/`.

| Run | Third-view master | Dashboard master | README previews |
| --- | --- | --- | --- |
| Formal 01 · FAILURE | [`experiment_4_failure_1.mp4`](video/third_view/experiment_4/experiment_4_failure_1.mp4) | [`experiment_4_failure_1_dashboard.mov`](video/dashboard/experiment_4/experiment_4_failure_1_dashboard.mov) | [Third view](demo/scene04_formal_01_preview.gif) · [Dashboard](demo/scene04_formal_01_dashboard.gif) |
| Formal 04 · FAILURE | Historical Git LFS object at `7eabe3d` | Historical Git LFS object at `7eabe3d` | [Third view](demo/scene04_formal_04_preview.gif) · [Dashboard](demo/scene04_formal_04_dashboard.gif) |

Exact runtime binding, paths, durations and hashes are recorded in
the linked
[Formal 01](../manifests/scene04_plant_formal_experiment_01_failure_20260801.json)
,
[Formal 02](../manifests/scene04_plant_formal_experiment_02_failure_20260801.json)
,
[Formal 03](../manifests/scene04_plant_formal_experiment_03_failure_20260801.json)
and
[Formal 04](../manifests/scene04_plant_formal_experiment_04_failure_20260731.json)
records. The current `failure_1` masters are the newly uploaded Formal 01
files; the previously committed Formal 04 masters remain recoverable at
commit `7eabe3d`, while its browser-ready derivatives remain at their stable
public paths.
