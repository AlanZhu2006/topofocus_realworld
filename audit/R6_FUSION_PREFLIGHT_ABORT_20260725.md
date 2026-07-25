# r6 fused-map preflight abort — 2026-07-25

## Outcome

Session `20260725-lab19-scene01-8ca1d52-yunjireboot1-r6` passed the automated
strict no-motion debug, but the operator then observed that the WSJ marker in
the fused Foxglove view was close to or inside a wall. The run was stopped
before live goal publication.

This is a calibration/fusion preflight failure:

- no live runner was started;
- no high-level target was published;
- no robot command path was authorized;
- no robot moved;
- no episode was created;
- the attempt is excluded from SR/SPL and is not a VLM failure.

Both robots were subsequently powered down for charging. No earlier operator
motion authorization remains valid.

## Read-only diagnosis

A frozen export preserved the two per-robot maps and status records. At the
WSJ base pose `(-1.8277348192, 0.3600512602)`:

- nearest occupied cell edge in WSJ's own map: `3.1313448390 m`;
- nearest occupied cell edge in Yunji's map: `0.2241184622 m`.

The wall conflict is therefore introduced by cross-map alignment, not by
WSJ's own occupancy map placing its base in an obstacle.

The r5 debug snapshot and the r6 pre-live snapshot used the same shared
calibration ID but reported substantially different relative geometry:

| Snapshot | WSJ base `(x, y, heading)` | Yunji base `(x, y, heading)` | Separation | Heading delta |
| --- | --- | --- | ---: | ---: |
| r5 accepted debug | `(-1.048670, 0.263534, 87.411°)` | `(-0.543905, -0.261489, 104.678°)` | `0.728311 m` | `17.267746°` |
| r6 pre-live | `(-1.827735, 0.360051, 79.057°)` | `(-0.680845, -0.314117, 110.089°)` | `1.330361 m` | `31.032278°` |

The relative separation changed by `0.602050 m` and the relative heading by
about `13.7645°`. Combined with the observed fused-wall conflict, this is
sufficient to reject calibration reuse. It does not identify which single
sensor transform drifted; that remains unverified until a new board
calibration is collected.

## Provenance

| Evidence | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| r6 session contract | 4,286 | `2adf2940beea91a3d2889eb66a5a709df2813041a76dc52ba3f41fc0c5a567df` | source-derived local runtime contract; Git-ignored |
| r6 strict debug shadow manifest | 18,833 | `e55447da56308c35c2a8fbb165fe983c071f31fd82e9f1bbe813de3aa78d077e` | observed strict no-motion debug evidence; Git-ignored |
| frozen export manifest | 4,795 | `1a3fa416d3050f0178921a73e7e71d7afc31bd78dff06a63db877217a4fccead` | source-derived read-only export index; Git-ignored |
| frozen WSJ map | 24,467 | `74264986548e15f70ade0e8eca1e87e3d662a1c864c24747e956e6a24e031c55` | source/model-derived map input; Git-ignored |
| frozen WSJ status | 4,424 | `95c8368265ba9d7568d3a54b7aba78450146a11d710b709a17f7d496c4e04ea8` | observed pose/status input; Git-ignored |
| frozen Yunji map | 24,133 | `766764152dadc22b4b1be27bf6960dd3407631805a78a063b984fe176a7c879a` | source/model-derived map input; Git-ignored |
| frozen Yunji status | 4,460 | `3107b7e0a70e484cf2b559eb0e003ba5cc814d9817faee924c45a3b2945d47ba` | observed pose/status input; Git-ignored |
| fused overview | 25,715 | `43e0e514b5387bf78de0ae72968e610b28e94a537a6df2653723b1c9136ecd1d` | source/model-derived operator visualization; Git-ignored |
| r5 accepted WSJ status | 4,801 | `dcb84c3104e0a1c7c284da4f0289ce7b766f65ff68670736d9ead12e2ec04714` | earlier observed comparison input; Git-ignored |
| r5 accepted Yunji status | 4,464 | `95991d6ad2cf8581743a017338c7d69be2a2322c7ee6f2dfcf2abb9ebf99504d` | earlier observed comparison input; Git-ignored |

The read-only nearest-obstacle calculation used `grid[0] > 0.5`, each map's
recorded origin and `0.05 m` resolution, and Euclidean distance to occupied
cell boundaries.

## Next valid gate

After charging and restoring both sensor computers:

1. keep both robot bodies stationary at the intended start poses;
2. run a new full two-position board calibration with a new session ID;
3. rebuild both online maps;
4. inspect WSJ, Yunji and fused overviews before live;
5. reject the session if either base appears in the other map's occupied
   region;
6. only then obtain a new onsite motion authorization.

This record changes no file under immutable `source/` or `dependencies/`.
