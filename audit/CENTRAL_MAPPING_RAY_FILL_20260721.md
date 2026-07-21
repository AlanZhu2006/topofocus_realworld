# Central mapper free-space ray fill — 2026-07-21

## Motivation

A concurrent workspace change addressed the reported black radial gaps in the
Hub semantic map. Endpoint-only explored marking can leave gaps with sparse,
invalid or range-limited real depth. The new path marks only the explored/free
channel along each camera-to-return ray; obstacle and semantic channels still
come only from measured endpoints.

This is a practical real-sensor deviation from the immutable Habitat source,
not an upstream behavior claim.

## Integration review

The initial vectorized implementation passed three focused tests, but an
**observed** synthetic dense-frame benchmark at 848x480 allocated every ray
sample at once:

```text
elapsed=2.59 s
max_rss_kb=900004
explored_cells=3680
```

The integration was changed to process at most 8192 endpoints per chunk and to
accumulate cell counts with `numpy.bincount`. A second **observed** run with
identical inputs produced:

```text
elapsed=0.41 s
max_rss_kb=210608
explored_cells=3680
```

These timings include Python process startup and are a regression comparison,
not a robot throughput guarantee.

## Tests

Five focused tests now cover:

1. cells between camera and endpoint become explored but not obstacles;
2. `ray_trace_steps=0` preserves endpoint-only behavior;
3. cells beyond an endpoint remain unknown;
4. an endpoint outside map bounds still marks the segment crossing the map;
5. chunk size 1 and chunk size 1000 produce byte-identical grids.

The complete Hub suite passed 142/142 after integration.

## Remaining gate

The implementation and synthetic performance are observed locally. A new real
Yunji/WSJ replay and visual Foxglove comparison are still **unverified**; do not
claim the original on-screen artifact is closed until that run is captured.
