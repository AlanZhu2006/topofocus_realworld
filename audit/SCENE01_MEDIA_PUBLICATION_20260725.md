# Scene 01 media publication — 2026-07-25

## Outcome

All media files currently present in the workspace are now included in the
repository publication scope. This adds the complete current Scene 01 source
set through Git LFS under `media/video/` rather than publishing only
size-bounded derivatives.

The Scene 01 homepage presentation contains:

| Display row | Source classification | Metric status |
| --- | --- | --- |
| Failure 1 | user-labelled `approach_failure_1`; exact runtime binding and controller reason unverified | excluded |
| Failure 2 | user-labelled `approach_failure_2`; exact runtime binding and controller reason unverified | excluded |
| Success | bound to `20260725-lab17-nearwall-fix / trial-05-nearwall-fix`; normal pass at the 0.5 m physical radius | `physical_0p5m_protocol`: `SR=1`, source-compatible `SPL=0.864048038008398` |

This table does not convert the two filenames into automatic controller
results. It also does not rewrite the success episode's original
`LOCAL_PLANNER_PATH_STALE` report under the old 0.15 m threshold.

## Committed source masters

The ten source files are observed user-provided assets. They remain
byte-for-byte unchanged.

| Source master | Bytes | Duration | SHA-256 | Classification |
| --- | ---: | ---: | --- | --- |
| `media/video/third_view/experiment_1/experiment_1.mp4` | 6,295,795 | 52.500000 s | `134bd28556f1d675b27413e2a9b66db17174aac82e20338b407e8f968c0051b2` | initial Scene 01 third view; exact outcome unverified |
| `media/video/dashboard/experiment_1/experiment_1_dashboard.mov` | 75,272,522 | 40.906667 s | `d1c66431fbb2255b976ad70874ade9e0672219613560d3cf49ce38c650f5dafe` | initial Scene 01 Dashboard; exact outcome unverified |
| `media/video/third_view/experiment_1/experiment_1_collision.mp4` | 1,640,232 | 12.366667 s | `1b883c310b176ed75587b5672d09a0ab14a604e214663f0c760835f1eb5ec659` | bound collision third view |
| `media/video/dashboard/experiment_1/experiment_1_collision_dashboard.mov` | 19,138,555 | 16.341667 s | `a429b838566efbd4769b1f613ba2530d1c0c972cc1cc685b72f278aac5ecffc9` | bound collision Dashboard |
| `media/video/third_view/experiment_1/experiment_1_approach_failure_1.mp4` | 9,212,106 | 80.733333 s | `46502195c9a5e80f5f26550c5b4970a33d08649c19497b51840e2a5e01d11d10` | user-labelled approach failure 1 |
| `media/video/dashboard/experiment_1/experiment_1_approach_failure_1_dashboard.mov` | 78,882,292 | 154.135000 s | `77c09b2f4bedaf4114e81fde3c06f5b854b4b248ed74569c15698f61fcbc884b` | user-labelled approach failure 1 Dashboard |
| `media/video/third_view/experiment_1/experiment_1_approach_failure_2.mp4` | 14,118,922 | 116.466667 s | `2d6c8904f472f2591848e59bc94fdee9d503403c27ab012a14423ff01604a8e6` | user-labelled approach failure 2 |
| `media/video/dashboard/experiment_1/experiment_1_approach_failure_2.mov` | 77,686,245 | 118.399000 s | `056a45564220138b85f2863642ef9e13fa3c197230b82739b893d322bc72b442` | user-labelled approach failure 2 Dashboard |
| `media/video/third_view/experiment_1/experiment_1_success.mp4` | 9,749,574 | 141.733333 s | `ab611dde043c78abbbe618a8f2ce95f1f5113b4f6b0df8466038abc1ebafb44c` | bound near-chair success third view |
| `media/video/dashboard/experiment_1/experiment_1_success_dashboard.mov` | 80,244,552 | 125.829000 s | `ad0e99c4ebb56e90a5923af636f0269b118349e3353c05511b3e21c5020fae34` | bound near-chair success Dashboard |

The largest source master is 80,244,552 bytes. No source file in this
publication set reaches 100,000,000 bytes.

## New web-playable failure derivatives

HEVC third views and large MOV Dashboards were converted to H.264,
`yuv420p`, fast-start MP4 without audio. Source masters were not modified.

| Published asset | Bytes | Duration/source time | SHA-256 | Classification |
| --- | ---: | ---: | --- | --- |
| `media/demo/scene01_failure_1_third_view_20260725.mp4` | 10,816,593 | 80.734 s | `0ac7702650891f1ba39bf29d61759b3e8762ab04dd1158e5bccce4f93f348a0d` | source-derived H.264 |
| `media/demo/scene01_failure_1_third_view_20260725_poster.jpg` | 72,618 | 75 s | `0fd8772f285663077857a05fcb151face1d34e39822a22682adfdbabee4fbea5` | source-derived still |
| `media/demo/scene01_failure_1_dashboard_20260725.mp4` | 1,528,819 | 154.067 s | `672185fa44bd58fbbc7e4c06feb011272878b5bdf3f6cf37a7132c0e132f18bd` | source-derived H.264 |
| `media/demo/scene01_failure_1_dashboard_20260725_poster.jpg` | 38,961 | 148 s | `eeb4faf4d43796b5a61aa33f6dab55309946ab0c393288cd2bd3a5f376aa9854` | source-derived still |
| `media/demo/scene01_failure_2_third_view_20260725.mp4` | 14,416,450 | 116.467 s | `381e73976f6ce3e08607e6a00053baf740b4d6b9a7b2e9351eba4f4d81c3eee3` | source-derived H.264 |
| `media/demo/scene01_failure_2_third_view_20260725_poster.jpg` | 74,517 | 110 s | `5aa2c67de2d63afc1a0e13b2985cd54a3e44b44b586d0ee3d5ff2746a1419ef7` | source-derived still |
| `media/demo/scene01_failure_2_dashboard_20260725.mp4` | 1,303,005 | 118.334 s | `f4fcffcdd776ed650e7d5654201061e1e477604c47c298fb882508e1a408a0e3` | source-derived H.264 |
| `media/demo/scene01_failure_2_dashboard_20260725_poster.jpg` | 38,882 | 112 s | `f02505185e21fbc9bb755c3f32693bcdab9fba48b92dfe429b56482134f1c0b6` | source-derived still |

The observed local encoder is `ffmpeg` 4.4.2 with `libx264`. Third-view
derivatives use CRF 25 at their source dimensions. Dashboard derivatives use
CRF 26 and are scaled to 1280 pixels wide.

## Completeness boundary

- Every file currently returned by `find media -type f` is in the Git
  publication scope after this change.
- Three older 2026-07-24 derivative entries name source masters that were
  observed then but are no longer present in the workspace. Their existing
  derivatives and recorded source hashes remain; missing bytes are not
  invented.
- Runtime episode directories, maps, tokens and robot-local logs remain
  outside Git.
- No file under immutable `source/` or `dependencies/` was changed.

## Later 2026-07-25 addendum: second success pair

Two later user-provided masters extend the original ten-file inventory to
twelve files:

| Source master | Bytes | Duration | SHA-256 | Classification |
| --- | ---: | ---: | --- | --- |
| `media/video/third_view/experiment_1/experiment_1_success_1.mp4` | 8,511,523 | 74.100 s | `3d67f19f48601dcd239b950689c391b96231010bc2c6fdea07351f0ff12c8bd6` | observed external view bound to `trial-r5-01` |
| `media/video/dashboard/experiment_1/experiment_1_success_1_dashboard.mov` | 45,502,070 | 74.955 s | `4668756e7d7ad7e2dd18eab85d65de5c8243ef3c9b281afcc6aace1f942291dd` | observed Dashboard bound to `trial-r5-01` |

The corresponding public H.264 assets are:

| Published asset | Bytes | Duration/source time | SHA-256 |
| --- | ---: | ---: | --- |
| `media/demo/scene01_success_2_third_view_20260725.mp4` | 10,359,239 | 74.100 s | `0469c894b80035a30cdc5d8fb4142577c7bebf53ace00f71528147046ff35675` |
| `media/demo/scene01_success_2_third_view_20260725_poster.jpg` | 188,224 | 73.0 s | `7611b39fae911b2def21018067240881b3910d04e949c4ec822e67fa01f339ac` |
| `media/demo/scene01_success_2_dashboard_20260725.mp4` | 1,175,482 | 74.900 s | `d30636dad6e9218b91f966c493b50eca9b82229bf3d9be4f45e6e25f0755cbf5` |
| `media/demo/scene01_success_2_dashboard_20260725_poster.jpg` | 77,100 | 73.0 s | `15d9c77a8c525036cf7e3adb78c454e53391e1067656b5f2d8e3c160d244e13d` |

`trial-r5-01` normally passed the 0.5 m physical protocol after WSJ emitted
`LOCAL_PLANNER_ARRIVED` at `0.406692832069 m`. Its metric row is
`SR=1`, source-compatible `SPL=0.628398923`, standard SPL unavailable.
Together with the first success, the current two-sample physical track is
`SR=2/2=1.0` with mean source-compatible
`SPL=0.746223480504199`.

The two earlier approach-failure media pairs remain unclassified because they
lack exact runtime bindings. This addendum does not relabel them as VLM
failures. See
[`FAILURE_ATTRIBUTION_PROTOCOL_20260725.md`](FAILURE_ATTRIBUTION_PROTOCOL_20260725.md)
and
[`SCENE01_CHAIR_SUCCESS_R5_20260725.md`](SCENE01_CHAIR_SUCCESS_R5_20260725.md).
