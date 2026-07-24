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

## Published failed demos

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
  `media/video/experiment_1_collision.mp4`, 1,640,232 bytes, HEVC/AAC,
  1280 × 720, nominal 30 fps, 12.366667 seconds, SHA-256
  `1b883c310b176ed75587b5672d09a0ab14a604e214663f0c760835f1eb5ec659`;
  retained locally and ignored.
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
  `media/video/experiment_1_collision_dashboard.mov`, 19,138,555 bytes,
  H.264, 3420 × 1544, nominal 60 fps, 16.341667 seconds, SHA-256
  `a429b838566efbd4769b1f613ba2530d1c0c972cc1cc685b72f278aac5ecffc9`;
  retained locally and ignored.
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
  retained locally and ignored.
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
  retained locally and ignored.
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
  retained locally and ignored because it exceeds the repository's 50 MiB
  audit bound.
- Published derivative: 499,397 bytes, H.264, 1280 × 770, 60 fps,
  24.084 seconds, SHA-256
  `17c7678ff8a268dd9b54036a45dcaacbfd18c36e2478907c6605aa0944cca598`.
- Poster: 107,204 bytes, 1280 × 770, SHA-256
  `dd1abc37f8309614d2b7b7d2eb31cec5cc93fc57b3c882e33a938dfea2b9c111`.
- Derivation: observed local `ffmpeg` 4.4.2 transcode using
  `scale=min(1280,iw):-2`, `libx264`, CRF 26, `yuv420p`, no audio; poster
  sampled at 5 seconds. The source master was not modified.
