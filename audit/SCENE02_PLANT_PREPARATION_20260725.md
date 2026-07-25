# Scene 02 · Plant preparation — 2026-07-25

Scene 02 is prepared as `scene02-plant`, target `plant`, with five formal
episode IDs `scene02-plant-run01` through `scene02-plant-run05`. This
preparation was strictly no-motion: no GOAL, robot command or episode was
created.

## Prepared contract

| Item | Prepared value |
| --- | --- |
| Target | HPC ObjectNav `plant` |
| Formal trials | `5` |
| Success radius | `0.5 m` |
| Source-compatible SPL | `S × D / max(D, P)` |
| Standard SPL | `S × L / max(L, P)` |
| Scene 02 shortest feasible path `L` | independently measure before metric finalization |
| Per-trial media | third view + Dashboard masters, MP4 derivatives and GIF previews |

The operator command sheet now binds calibration, strict debug, all five live
episode IDs and recovery to `plant`. Its SR/SPL example deliberately contains
empty shortest-path placeholders, so Scene 01's `3.25 m` cannot be copied into
Scene 02 accidentally.

## Static plant path

| Path | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/src/focus_hub/transport_v2.py` | 14,817 | `44d261be718fb7155b3a0b20b6d6264c45d50bcb708ab381da2a03ee38032cd6` | source-derived transport support |
| `hub/src/focus_hub/source_episode.py` | 16,567 | `e5bed073f936cf05a0e884dd0107b537ce3940f316b5f2701f8cda18b8aa3a06` | source-derived ObjectNav selection |
| `hub/src/focus_hub/segformer_ade20k.py` | 10,122 | `71b958f0713d1854e0479cdffa3cebd30a0b387f9de83697c830923ddf272d5f` | source-derived plant/flower/pot semantic mapping |
| `hub/src/focus_hub/semantic_yolo.py` | 8,593 | `90870d54a72795ed042ad4ef7a0e3917b978153dc4dcaecd4f0f73f8e06c2f6a` | source-derived `potted plant` mapping |
| `hub/scripts/calibrate_realworld_session.sh` | 27,855 | `c676d5e341d72185b805768f0db3119d42ca74533e021ad738e055bc9fa55de0` | source-derived plant-capable no-motion calibration |
| `hub/scripts/realworld_oneclick.sh` | 34,002 | `f7cf8b2dd9b25ffa72a4d4861568e2bd82a6515033d39fcfcd72416861c04aa8` | source-derived plant-capable debug/live path |
| `command.txt` | 10,645 | `6f2cd46878f9b97dc2cb32340ce7f6dc6629ca504be23d4002e4eacdff822d36` | prepared Scene 02 operator commands |

`plant` is an allowed v2 goal, the source episode reads its semantic channel,
SegFormer maps plant/flower/pot to MP3D channel 15, and YOLO maps
`potted plant` to the HM3D `plant` category.

## Onsite gate

The current pointer still names the archived Scene 01 session
`20260725-lab22-formal05-recalibration-ddsfix`; its fresh-map contract is
`chair`, so it is not reusable for Scene 02.

On the final Scene 02 placement:

1. independently measure the shortest feasible path to the valid plant goal
   region and preserve its evidence;
2. keep both robots stationary, show the complete calibration board to both
   cameras and run section 1 of `command.txt`;
3. accept the session only after `DEBUG_FULLSTACK_READY` and a visually
   consistent plant-bound Foxglove view;
4. obtain a fresh onsite live confirmation for each formal episode.

The machine-readable preparation record is
[`../manifests/scene02_plant_preparation_20260725.json`](../manifests/scene02_plant_preparation_20260725.json);
the prepared measurement record is
[`../manifests/scene02_plant_shortest_feasible_path_20260725.json`](../manifests/scene02_plant_shortest_feasible_path_20260725.json).
