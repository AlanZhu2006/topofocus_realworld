# Workspace media provenance

These files are the project README's physical-experiment showcase assets.
Their bytes were observed in the workspace on 2026-07-24 and 2026-07-25.
Capture-device and EXIF metadata were not independently verified.

All media files currently present under `media/`, including the original
Scene 01 videos under `media/video/`, are committed; source masters use Git
LFS. H.264,
`yuv420p`, fast-start derivatives under `media/demo/` are retained because
the original HEVC/MOV masters may not play inline in every browser.

## Images

| Path | Size (bytes) | Dimensions | SHA-256 | Classification |
| --- | ---: | --- | --- | --- |
| `image/calibration.jpg` | 325176 | 1707 × 1280 | `687a87750b3867eecde9cd4edd6f66105e585d32316f6c13536fc6166ff2d909` | observed user-provided calibration-board/setup photo; capture metadata unverified |
| `image/example.png` | 10736 | 686 × 599 | `ecdc053d57ddf23dbd5fc80cc3f5692c96b381cd74245f16da75aa7ebddb5360` | observed user-provided semantic-map reference; renamed byte-for-byte from repository-root `example.png` |
| `image/experiment_1_map.png` | 67105 | 820 × 942 | `7b9cbfa20b0b09c1dba33f24ae23b8cc64817d52eb48222613e829ceac1e867b` | observed user-provided Scene 01 map screenshot; semantic pixels remain unverified model inference |
| `image/showcase_1.jpg` | 355320 | 1280 × 1707 | `a54993201184799d8d77b948cb7d0b139ff6de6fbdff41f03d1b00f519459ee1` | observed user-provided two-robot setup photo; capture metadata unverified |
| `image/showcase_2.jpg` | 863185 | 2486 × 1280 | `0d433920549fd073a15e22c0ae6f420584abd5dc5bfacee07d953b44b251586e` | observed user-provided two-robot setup photo; capture metadata unverified |

These images must not be used as ground-truth terminal evidence for SR/SPL
unless a trial record separately identifies and hashes the exact evidence
file under its own collection protocol.

## Complete committed Scene 01 source-video inventory

| Source master | Size (bytes) | Video stream | Duration | SHA-256 | Classification |
| --- | ---: | --- | ---: | --- | --- |
| `video/third_view/experiment_1/experiment_1.mp4` | 6295795 | HEVC Main, 720 × 1280, 30 fps | 52.500000 s | `134bd28556f1d675b27413e2a9b66db17174aac82e20338b407e8f968c0051b2` | observed initial Scene 01 third view; exact episode/outcome unverified |
| `video/dashboard/experiment_1/experiment_1_dashboard.mov` | 75272522 | H.264 Main, 3644 × 2194, 60 fps | 40.906667 s | `d1c66431fbb2255b976ad70874ade9e0672219613560d3cf49ce38c650f5dafe` | observed initial Scene 01 Dashboard; exact episode/outcome unverified |
| `video/third_view/experiment_1/experiment_1_collision.mp4` | 1640232 | HEVC Main, 1280 × 720, 30 fps | 12.366667 s | `1b883c310b176ed75587b5672d09a0ab14a604e214663f0c760835f1eb5ec659` | observed and explicitly bound collision third view |
| `video/dashboard/experiment_1/experiment_1_collision_dashboard.mov` | 19138555 | H.264 Main, 3420 × 1544, 60 fps | 16.341667 s | `a429b838566efbd4769b1f613ba2530d1c0c972cc1cc685b72f278aac5ecffc9` | observed and explicitly bound collision Dashboard |
| `video/third_view/experiment_1/experiment_1_approach_failure_1.mp4` | 9212106 | HEVC Main, 1280 × 720, 30 fps | 80.733333 s | `46502195c9a5e80f5f26550c5b4970a33d08649c19497b51840e2a5e01d11d10` | observed user-labelled Scene 01 approach failure 1; exact runtime identity unverified |
| `video/dashboard/experiment_1/experiment_1_approach_failure_1_dashboard.mov` | 78882292 | H.264 High, 2392 × 1080, 30 fps | 154.135000 s | `77c09b2f4bedaf4114e81fde3c06f5b854b4b248ed74569c15698f61fcbc884b` | observed user-labelled Scene 01 approach failure 1 Dashboard; exact runtime identity unverified |
| `video/third_view/experiment_1/experiment_1_approach_failure_2.mp4` | 14118922 | HEVC Main, 1280 × 720, 30 fps | 116.466667 s | `2d6c8904f472f2591848e59bc94fdee9d503403c27ab012a14423ff01604a8e6` | observed user-labelled Scene 01 approach failure 2; exact runtime identity unverified |
| `video/dashboard/experiment_1/experiment_1_approach_failure_2.mov` | 77686245 | H.264 High, 2392 × 1080, 30 fps | 118.399000 s | `056a45564220138b85f2863642ef9e13fa3c197230b82739b893d322bc72b442` | observed user-labelled Scene 01 approach failure 2 Dashboard; exact runtime identity unverified |
| `video/third_view/experiment_1/experiment_1_success_3.mp4` | 9749574 | HEVC Main, 544 × 960, 30 fps | 141.733333 s | `ab611dde043c78abbbe618a8f2ce95f1f5113b4f6b0df8466038abc1ebafb44c` | observed formal-experiment-03 success third view; byte-identical rename from `experiment_1_success.mp4`; bound to the 0.5 m physical-protocol track |
| `video/dashboard/experiment_1/experiment_1_success_3_dashboard.mov` | 80244552 | H.264 High, 2392 × 1080, 30 fps | 125.829000 s | `ad0e99c4ebb56e90a5923af636f0269b118349e3353c05511b3e21c5020fae34` | observed formal-experiment-03 success Dashboard; byte-identical rename from `experiment_1_success_dashboard.mov`; bound to the 0.5 m physical-protocol track |
| `video/third_view/experiment_1/experiment_1_success_1.mp4` | 8511523 | HEVC Main, 1280 × 720, nominal 30 fps | 74.100000 s | `3d67f19f48601dcd239b950689c391b96231010bc2c6fdea07351f0ff12c8bd6` | observed second success third view bound to `trial-r5-01` |
| `video/dashboard/experiment_1/experiment_1_success_1_dashboard.mov` | 45502070 | H.264 High, 2392 × 1080, 30 fps | 74.955000 s | `4668756e7d7ad7e2dd18eab85d65de5c8243ef3c9b281afcc6aace1f942291dd` | observed second success Dashboard bound to `trial-r5-01` |
| `video/third_view/experiment_1/experiment_1_success_2.mp4` | 2403359 | HEVC Main, 1280 × 720, nominal 29.947 fps | 18.900000 s | `145c7a334816d845a757ef714f0cbe74a0983ab7b728bf2c05e3e61551a2069e` | observed user-labelled `success_2` third-view candidate; exact runtime/episode binding unverified |
| `video/dashboard/experiment_1/experiment_1_success_2_dashboard.mov` | 17640290 | H.264 High, 2392 × 1080, 30 fps | 28.733333 s | `4e060ba8fe006a27113e1c3aa5ea0fddbc9d7e58812959b74ff34a69d492231d` | observed user-labelled `success_2` Dashboard candidate; exact runtime/episode binding unverified |
| `video/dashboard/experiment_1/experiment_1_success_4_dashboard.mov` | 41979329 | H.264 High, 2392 × 1080, 30 fps | 67.663000 s | `dd3518091413d7267e50b896364071882321c8a3802fd8c35d8efdb9e7c6a633` | observed user-labelled `success_4` Dashboard candidate; exact formal-experiment-04 binding unverified |

The `failure_1`, `failure_2` and `success` labels above come from the
user-provided filenames and onsite classification. A filename alone does not
establish a controller reason code, automatic terminal or standard
metric eligibility. The two approach failures remain excluded from SR/SPL
until exact runtime evidence is bound. The success record has the separate
evidence described below.

The three newly committed `success_2`/`success_4` candidates are likewise
filename-labelled evidence, not episode bindings. The two `success_2` files
form a same-label third-view/Dashboard candidate pair; the `success_4` file is
Dashboard-only. All three are committed byte-for-byte through Git LFS and
remain excluded from metric evidence until an exact runtime association is
established.

## Bound Scene 01 successes

The success masters are bound to session `20260725-lab17-nearwall-fix`,
episode `trial-05-nearwall-fix`. Yunji stopped `0.321133 m` from its selected
semantic approach point. Under the project's `0.5 m` physical terminal radius,
Scene 01 passes normally. It contributes `SR=1` and source-compatible
`SPL=0.864048` to the `physical_0p5m_protocol` track; standard pre-surveyed
SPL remains unavailable.

The original automatic outcome remains a failure under the old `0.15 m`
arrival threshold. Public derivatives, exact classification and arithmetic
are indexed in [`demo/README.md`](demo/README.md) and
[`../audit/SCENE01_CHAIR_SUCCESS_20260725.md`](../audit/SCENE01_CHAIR_SUCCESS_20260725.md).

The second success masters are bound to session
`20260725-lab19-scene01-8ca1d52-yunjireboot1-r5`, episode `trial-r5-01`.
WSJ automatically emitted `LOCAL_PLANNER_ARRIVED` at `0.406693 m` from the
chair semantic goal. It contributes `SR=1` and source-compatible
`SPL=0.628398923`. Exact provenance is in
[`../audit/SCENE01_CHAIR_SUCCESS_R5_20260725.md`](../audit/SCENE01_CHAIR_SUCCESS_R5_20260725.md).

Formal experiment 04,
`20260725-lab21-wallfix-imudebounce-3a2d953 /
trial-wallfix-imudebounce-r1`, is bound through runtime and terminal evidence,
not through any of the three newly committed media candidates. Yunji
automatically emitted `LOCAL_PLANNER_ARRIVED`; the run contributes `SR=1` and
source-compatible `SPL=0.956360614325575`. The operator independently measured
the shortest feasible path as approximately `3.25 m`, giving official
standard `SPL=1.0`.

The three recorded physical-protocol samples therefore have `SR=3/3=1.0` and
mean source-compatible `SPL=0.816269191777991`. The surveyed standard track
contains formal experiment 04 alone: `SR=1/1=1.0`, mean standard `SPL=1.0`.
Exact provenance is in
[`../audit/SCENE01_CHAIR_FORMAL_EXPERIMENT_04_SUCCESS_20260725.md`](../audit/SCENE01_CHAIR_FORMAL_EXPERIMENT_04_SUCCESS_20260725.md).

## Bound collision

The collision masters are bound to session `20260725-lab05-yunjireboot4`,
episode `scene01-chair-run01-fastfix`. The third-view recording directly
shows physical contact. The paired Dashboard supplies visualization context
but does not independently establish semantic target truth or network
causality. The episode is excluded from SR/SPL.

All source masters remain byte-for-byte unchanged. Public H.264 derivatives,
posters and their derivation parameters are indexed in
[`demo/README.md`](demo/README.md). The complete publication inventory is
preserved in
[`../audit/SCENE01_MEDIA_PUBLICATION_20260725.md`](../audit/SCENE01_MEDIA_PUBLICATION_20260725.md).

The two user-labelled approach failures still lack an exact runtime binding.
They remain `unclassified_failure` rather than being called VLM failures.
The project-wide distinction between engineering, perception and VLM
decision failures is defined in
[`../audit/FAILURE_ATTRIBUTION_PROTOCOL_20260725.md`](../audit/FAILURE_ATTRIBUTION_PROTOCOL_20260725.md).
