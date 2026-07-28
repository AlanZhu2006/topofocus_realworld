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

Formal 02 was replaced: the operator designated a later Yunji-only run
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
