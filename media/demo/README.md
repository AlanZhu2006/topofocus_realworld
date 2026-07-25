# Physical demo index

Physical demo videos are indexed here after upload. Every entry must record:

- public file path and SHA-256;
- scene ID, episode ID and capture date;
- relevant Git commit and real-world session ID;
- observed outcome and failure/termination reason;
- whether the run is excluded, incomplete or eligible for SR/SPL;
- links to the corresponding episode report and dated audit when publishable.

Known July 23/24 engineering attempts (`official-run01` and retry1–retry3)
will be labelled as failed/excluded demos. Uploading their videos does not
change their metric status.

## Published Scene 01 successes

### Near-chair Yunji run — 2026-07-25

These recordings are explicitly bound to:

- session: `20260725-lab17-nearwall-fix`;
- session Git commit:
  `cbe6d9b02f50ded3607037c46abb0230d1639ebc`;
- scene: `scene01-chair`;
- episode: `trial-05-nearwall-fix`;
- automatic outcome: `failed_robot_or_controller_holding` with Yunji
  `LOCAL_PLANNER_PATH_STALE` under the old `0.15 m` arrival radius;
- outcome: normal success under the declared `0.5 m` physical terminal
  radius;
- metric track: `physical_0p5m_protocol`;
- metric result: `SR=1`, source-compatible `SPL=0.864048038008398`,
  standard SPL unavailable without a pre-surveyed shortest path.

The external video shows Yunji drive toward the white chair and stop beside
it while WSJ remains at the start area. Robot-log geometry places Yunji
`0.321133366707683 m` from its selected semantic approach point. The original
automatic report is preserved rather than rewritten.

#### External-view success record

- Published video:
  `scene01_success_third_view_20260725.mp4`
- Poster:
  `scene01_success_third_view_20260725_poster.jpg`
- Original user-provided master:
  `media/video/third_view/experiment_1/experiment_1_success.mp4`,
  9,749,574 bytes, HEVC/AAC, 544 × 960, nominal 30 fps,
  141.733333 seconds, SHA-256
  `ab611dde043c78abbbe618a8f2ce95f1f5113b4f6b0df8466038abc1ebafb44c`;
  committed byte-for-byte.
- Published derivative: 15,972,419 bytes, H.264, `yuv420p`, 544 × 960,
  30 fps, 141.734 seconds, SHA-256
  `62390579a390451b442562029c2d385729d282696605bb347dcbe2d64f11af1c`.
- Poster: 138,572 bytes, 544 × 960, sampled at 141.0 seconds, SHA-256
  `27340abea65bdb2706aff8254c5257c915bc29aa03d336eaa849f0e5022702bc`.

#### Dashboard success record

- Published video:
  `scene01_success_dashboard_20260725.mp4`
- Poster:
  `scene01_success_dashboard_20260725_poster.jpg`
- Original user-provided master:
  `media/video/dashboard/experiment_1/experiment_1_success_dashboard.mov`,
  80,244,552 bytes, H.264/AAC, 2392 × 1080, 30 fps, 125.829 seconds,
  SHA-256
  `ad0e99c4ebb56e90a5923af636f0269b118349e3353c05511b3e21c5020fae34`;
  committed byte-for-byte.
- Published derivative: 1,365,229 bytes, H.264, `yuv420p`, 1280 × 578,
  30 fps, 125.767 seconds, SHA-256
  `a88716cc94600859eb23e87ccfa131133259389e0bbdc46c6b5cd4746d4df4a5`.
- Poster: 82,157 bytes, 1280 × 578, sampled at 123.0 seconds, SHA-256
  `7d76b00a4e793cd9d88887ba24d7c906ba9fa6b01d0716950ea12a1ed95741bc`.

Both derivatives were produced with the locally observed `ffmpeg` 4.4.2,
`libx264`, `yuv420p`, fast-start MP4 and no audio. Exact runtime evidence and
the separation between the 0.5 m physical-protocol track and the
pre-surveyed standard track are in
[`../../audit/SCENE01_CHAIR_SUCCESS_20260725.md`](../../audit/SCENE01_CHAIR_SUCCESS_20260725.md).

### Automatic WSJ arrival r5 — 2026-07-25

These recordings are explicitly bound to:

- session: `20260725-lab19-scene01-8ca1d52-yunjireboot1-r5`;
- session Git commit:
  `8ca1d528b5e1bdc6e029f63031330250a4c962a9`;
- scene: `scene01-chair`;
- episode: `trial-r5-01`;
- automatic result: WSJ `LOCAL_PLANNER_ARRIVED`;
- goal distance: `0.406692832069 m` under the declared `0.5 m` radius;
- metric result: `SR=1`, source-compatible `SPL=0.628398923`;
- standard SPL: unavailable without a pre-surveyed shortest path.

The external view shows the white chair and both robots at the end of the
run. The Dashboard records both cameras, the per-robot and fused maps,
trajectories and chair semantic region. The frozen WSJ decision RGB contains
the chair; the later automatic terminal RGB faces the wall and is not used
alone as target identity evidence.

#### External-view automatic-success record

- Original master:
  `media/video/third_view/experiment_1/experiment_1_success_1.mp4`,
  8,511,523 bytes, HEVC/AAC, 1280 × 720, nominal 30 fps,
  74.100 seconds, SHA-256
  `3d67f19f48601dcd239b950689c391b96231010bc2c6fdea07351f0ff12c8bd6`.
- Published derivative:
  `scene01_success_2_third_view_20260725.mp4`, 10,359,239 bytes,
  H.264 High, `yuv420p`, 1280 × 720, 30 fps, 74.100 seconds, SHA-256
  `0469c894b80035a30cdc5d8fb4142577c7bebf53ace00f71528147046ff35675`.
- Poster:
  `scene01_success_2_third_view_20260725_poster.jpg`, 188,224 bytes,
  sampled at 73.0 seconds, SHA-256
  `7611b39fae911b2def21018067240881b3910d04e949c4ec822e67fa01f339ac`.

#### Dashboard automatic-success record

- Original master:
  `media/video/dashboard/experiment_1/experiment_1_success_1_dashboard.mov`,
  45,502,070 bytes, H.264/AAC, 2392 × 1080, 30 fps,
  74.955 seconds, SHA-256
  `4668756e7d7ad7e2dd18eab85d65de5c8243ef3c9b281afcc6aace1f942291dd`.
- Published derivative:
  `scene01_success_2_dashboard_20260725.mp4`, 1,175,482 bytes,
  H.264 High, `yuv420p`, 1280 × 578, 30 fps, 74.900 seconds, SHA-256
  `d30636dad6e9218b91f966c493b50eca9b82229bf3d9be4f45e6e25f0755cbf5`.
- Poster:
  `scene01_success_2_dashboard_20260725_poster.jpg`, 77,100 bytes,
  sampled at 73.0 seconds, SHA-256
  `15d9c77a8c525036cf7e3adb78c454e53391e1067656b5f2d8e3c160d244e13d`.

The external-view derivative used CRF 25 at source dimensions. The Dashboard
used CRF 26 scaled to 1280 pixels wide. Both were generated with the locally
observed `ffmpeg` 4.4.2, `libx264`, `yuv420p`, fast-start MP4 and no audio.
Exact runtime evidence is in
[`../../audit/SCENE01_CHAIR_SUCCESS_R5_20260725.md`](../../audit/SCENE01_CHAIR_SUCCESS_R5_20260725.md).

## Published failed demos

### Scene 01 approach failure 1 — 2026-07-25

The filenames and onsite description classify this pair as the first
approach failure in Scene 01. The exact runtime session/episode and controller
reason code have not been independently bound, so this recording is an
`unclassified_failure`, is excluded from SR/SPL and must not be described as
a VLM failure.

- Third-view master:
  `media/video/third_view/experiment_1/experiment_1_approach_failure_1.mp4`,
  9,212,106 bytes, HEVC Main, 1280 × 720, 30 fps, 80.733333 seconds,
  SHA-256
  `46502195c9a5e80f5f26550c5b4970a33d08649c19497b51840e2a5e01d11d10`.
- Published third-view derivative:
  `scene01_failure_1_third_view_20260725.mp4`, 10,816,593 bytes,
  H.264 High, `yuv420p`, 1280 × 720, 30 fps, 80.734 seconds, SHA-256
  `0ac7702650891f1ba39bf29d61759b3e8762ab04dd1158e5bccce4f93f348a0d`.
- Third-view poster:
  `scene01_failure_1_third_view_20260725_poster.jpg`, 72,618 bytes,
  sampled at 75 seconds, SHA-256
  `0fd8772f285663077857a05fcb151face1d34e39822a22682adfdbabee4fbea5`.
- Dashboard master:
  `media/video/dashboard/experiment_1/experiment_1_approach_failure_1_dashboard.mov`,
  78,882,292 bytes, H.264 High, 2392 × 1080, 30 fps, 154.135 seconds,
  SHA-256
  `77c09b2f4bedaf4114e81fde3c06f5b854b4b248ed74569c15698f61fcbc884b`.
- Published Dashboard derivative:
  `scene01_failure_1_dashboard_20260725.mp4`, 1,528,819 bytes,
  H.264 High, `yuv420p`, 1280 × 578, 30 fps, 154.067 seconds, SHA-256
  `672185fa44bd58fbbc7e4c06feb011272878b5bdf3f6cf37a7132c0e132f18bd`.
- Dashboard poster:
  `scene01_failure_1_dashboard_20260725_poster.jpg`, 38,961 bytes,
  sampled at 148 seconds, SHA-256
  `eeb4faf4d43796b5a61aa33f6dab55309946ab0c393288cd2bd3a5f376aa9854`.

### Scene 01 approach failure 2 — 2026-07-25

The filenames and onsite description classify this pair as the second
approach failure in Scene 01. The third-view terminal frame shows Yunji closer
to the chair region than in failure 1, but no automatic terminal or exact
runtime binding is established by the media alone. It is an
`unclassified_failure`, is excluded from SR/SPL and must not be described as
a VLM failure.

- Third-view master:
  `media/video/third_view/experiment_1/experiment_1_approach_failure_2.mp4`,
  14,118,922 bytes, HEVC Main, 1280 × 720, 30 fps, 116.466667 seconds,
  SHA-256
  `2d6c8904f472f2591848e59bc94fdee9d503403c27ab012a14423ff01604a8e6`.
- Published third-view derivative:
  `scene01_failure_2_third_view_20260725.mp4`, 14,416,450 bytes,
  H.264 High, `yuv420p`, 1280 × 720, 30 fps, 116.467 seconds, SHA-256
  `381e73976f6ce3e08607e6a00053baf740b4d6b9a7b2e9351eba4f4d81c3eee3`.
- Third-view poster:
  `scene01_failure_2_third_view_20260725_poster.jpg`, 74,517 bytes,
  sampled at 110 seconds, SHA-256
  `5aa2c67de2d63afc1a0e13b2985cd54a3e44b44b586d0ee3d5ff2746a1419ef7`.
- Dashboard master:
  `media/video/dashboard/experiment_1/experiment_1_approach_failure_2.mov`,
  77,686,245 bytes, H.264 High, 2392 × 1080, 30 fps, 118.399 seconds,
  SHA-256
  `056a45564220138b85f2863642ef9e13fa3c197230b82739b893d322bc72b442`.
- Published Dashboard derivative:
  `scene01_failure_2_dashboard_20260725.mp4`, 1,303,005 bytes,
  H.264 High, `yuv420p`, 1280 × 578, 30 fps, 118.334 seconds, SHA-256
  `f4fcffcdd776ed650e7d5654201061e1e477604c47c298fb882508e1a408a0e3`.
- Dashboard poster:
  `scene01_failure_2_dashboard_20260725_poster.jpg`, 38,882 bytes,
  sampled at 112 seconds, SHA-256
  `f02505185e21fbc9bb755c3f32693bcdab9fba48b92dfe429b56482134f1c0b6`.

Both failure pairs keep their source masters byte-for-byte. The public
derivatives were produced with the locally observed `ffmpeg` 4.4.2,
`libx264`, `yuv420p`, fast-start MP4 and no audio. Third views use CRF 25;
Dashboards are scaled to 1280 pixels wide and use CRF 26.

### Bound dual-robot collision — 2026-07-25

These two user-provided recordings are explicitly bound to:

- session: `20260725-lab05-yunjireboot4`;
- session Git commit:
  `cdcd7e70560f8bd782d83b5176bda6f5fca36780`;
- scene: `scene01-chair`;
- episode: `scene01-chair-run01-fastfix`;
- operator termination: `collision`;
- metric status: **excluded**, `official_sr_spl_eligible=false`.

The landscape third-view video directly shows WSJ Go2 and Yunji travel from
separated positions toward the same central corridor and make physical
contact. The Dashboard recording shows the paired live camera/map view, short
trajectories converging, close-range WSJ-camera occlusion and a projected
`chair` region. The semantic region is model inference, not independent
target truth.

#### Third-view collision record

- Published video:
  `dual_robot_collision_third_view_20260725.mp4`
- Poster:
  `dual_robot_collision_third_view_20260725_poster.jpg`
- Original user-provided master:
  `media/video/third_view/experiment_1/experiment_1_collision.mp4`,
  1,640,232 bytes, HEVC/AAC,
  1280 × 720, nominal 30 fps, 12.366667 seconds, SHA-256
  `1b883c310b176ed75587b5672d09a0ab14a604e214663f0c760835f1eb5ec659`;
  committed byte-for-byte.
- Published derivative: 2,481,165 bytes, H.264, `yuv420p`, 1280 × 720,
  30 fps, no audio, 12.367 seconds, SHA-256
  `8b62b5e763e529c8d2bf12caf5066d57187226de3117dc0fb3e7d7d0df3d395e`.
- Poster: 199,630 bytes, 1280 × 720, SHA-256
  `cbb216abddf11e2089ba60afba38eac09d2b838654b8289af7f79ed72cf9417f`.

#### Dashboard collision record

- Published video:
  `dual_robot_collision_dashboard_20260725.mp4`
- Poster:
  `dual_robot_collision_dashboard_20260725_poster.jpg`
- Original user-provided master:
  `media/video/dashboard/experiment_1/experiment_1_collision_dashboard.mov`,
  19,138,555 bytes, H.264, 3420 × 1544, nominal 60 fps,
  16.341667 seconds, SHA-256
  `a429b838566efbd4769b1f613ba2530d1c0c972cc1cc685b72f278aac5ecffc9`;
  committed byte-for-byte.
- Published derivative: 284,926 bytes, H.264, `yuv420p`, 1280 × 578,
  30 fps, no audio, 16.334 seconds, SHA-256
  `fd8d5709fc1bf1d9568d9cf594cc37b578942ebc7b077f78307346b68780ea8f`.
- Poster: 75,948 bytes, 1280 × 578, SHA-256
  `cd93b77637204f85690921ea791aa0a1b4fc4fcbe95d9d52f7ea559924d80ae7`.

Both public derivatives were produced with the locally observed `ffmpeg`
4.4.2 using `libx264`, `yuv420p`, fast-start MP4 and no audio. The third-view
master was normalized to 30 fps at CRF 24; the Dashboard master was scaled to
1280 pixels wide, normalized to 30 fps and encoded at CRF 26. Posters were
sampled at 10 and 12 seconds respectively. Source masters were not modified.

The corresponding runtime hashes, event sequence, geometry replay,
network-causality boundary and corrective route guard are in
[`../../audit/DUAL_ROBOT_COLLISION_20260725.md`](../../audit/DUAL_ROBOT_COLLISION_20260725.md).

### Third-view two-robot engineering clips

Both clips were observed as user-provided files in the workspace on
2026-07-24. They show WSJ Go2 and Yunji together from an external laboratory
view. Capture time, episode/session identity, navigation outcome and failure
reason are not inferred from video pixels alone.

#### Clip 1 — landscape

- Published video:
  `third_view_failure_1_20260724.mp4`
- Poster:
  `third_view_failure_1_20260724_poster.jpg`
- Metric status: **excluded engineering demonstration**, not an official
  episode.
- Original user-provided master:
  `media/video/third_view_failure_1.mp4`, 1,296,348 bytes, HEVC Main/AAC,
  1280 × 720, nominal 30 fps, 10.167 seconds, SHA-256
  `19d3e8958ab27fa7a1381d16c00842a113f64a051d0b4651217b48f3687718e2`;
  observed on 2026-07-24 but not present in the current workspace; its
  published derivative and source hash preserve provenance.
- Published derivative: 1,712,294 bytes, H.264 High, `yuv420p`, 1280 × 720,
  30 fps, no audio, 10.167 seconds, SHA-256
  `98fd321a207fd099c741338309cc0c313e0dbdab3ba98ad1438853aa377b82a5`.
- Poster: 174,685 bytes, 1280 × 720, SHA-256
  `f7fc9b6425e2ac3e8f2a611a687916ad2a2fb730486914c0668ed6b1d3962ade`.

#### Clip 2 — portrait

- Published video:
  `third_view_failure_2_20260724.mp4`
- Poster:
  `third_view_failure_2_20260724_poster.jpg`
- Metric status: **excluded engineering demonstration**, not an official
  episode.
- Original user-provided master:
  `media/video/third_view_failure_2.mp4`, 767,264 bytes, HEVC Main/AAC,
  720 × 1280, nominal 30 fps, 4.833 seconds, SHA-256
  `4c51517313c01e6310e170368c0c3336027523569d03467f12b6ea9c77fbe456`;
  observed on 2026-07-24 but not present in the current workspace; its
  published derivative and source hash preserve provenance.
- Published derivative: 1,182,484 bytes, H.264 High, `yuv420p`, 720 × 1280,
  30 fps, no audio, 4.834 seconds, SHA-256
  `5cb70e13fe4a6ac53102a09b0f4ad874efd385021fc1e43a567c5a4ac4f316e1`.
- Poster: 194,849 bytes, 720 × 1280, SHA-256
  `a3d2796b08be9416653df85eec5f509e94431c4e8feef9d1a7355372d7623cb2`.

Both public derivatives were produced with the locally observed `ffmpeg`
4.4.2 using the source video stream, normalized 30 fps, `libx264` CRF 24,
`yuv420p`, fast-start MP4 and no audio. Source masters were not modified.
Posters were sampled at 2.5 seconds and 2.0 seconds respectively.

### Early Foxglove dashboard map failure

- Published video:
  `dashboard_failure_20260724.mp4`
- Poster:
  `dashboard_failure_20260724_poster.jpg`
- Observed content: both live camera panels update while the early 2-D
  occupancy/semantic views contain ray-like and irregular regions.
- Metric status: **excluded demonstration**, not an official episode.
- Episode/session identity: unverified from the video alone; no identity is
  inferred.
- Related engineering evidence:
  [`../../audit/LIVE_MAP_RECOVERY_20260722.md`](../../audit/LIVE_MAP_RECOVERY_20260722.md)
- Original user-provided master:
  `media/video/dashboard_failure.mov`, 63,604,095 bytes,
  SHA-256
  `302fa28afcaf67f47189a7a099d310b82ee3f8e05b38afc5a4a155e8c8c4fe9f`;
  observed on 2026-07-24 but not present in the current workspace; its
  published derivative and source hash preserve provenance.
- Published derivative: 499,397 bytes, H.264, 1280 × 770, 60 fps,
  24.084 seconds, SHA-256
  `17c7678ff8a268dd9b54036a45dcaacbfd18c36e2478907c6605aa0944cca598`.
- Poster: 107,204 bytes, 1280 × 770, SHA-256
  `dd1abc37f8309614d2b7b7d2eb31cec5cc93fc57b3c882e33a938dfea2b9c111`.
- Derivation: observed local `ffmpeg` 4.4.2 transcode using
  `scale=min(1280,iw):-2`, `libx264`, CRF 26, `yuv420p`, no audio; poster
  sampled at 5 seconds. The source master was not modified.
