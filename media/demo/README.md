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
