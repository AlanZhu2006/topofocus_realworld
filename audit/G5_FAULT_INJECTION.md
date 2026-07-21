# G5 fault-injection evidence layer (NOT a G5 pass)

Date: 2026-07-18. This is explicitly **not** G5. G5 requires hardware-in-the-loop
with a physical robot, a measured `base_T_camera`, and proof that real
actuation is rejected under fault conditions. Neither this machine nor this
run touches a robot. What this document records is the fault-injection
*evidence layer* G5 will need — that the hub and the robot-side `GoalGuard`
fail closed, over the real wire protocol and (for two scenarios) through a
genuinely killed and restarted process, for every fault category the handoff
doc names: expired, out-of-order, wrong transform, wrong map, network
disconnect, unsafe health, distance.

## Two layers of coverage

1. **Unit tests** (fast, deterministic, exhaustive over internal state):
   - `hub/tests/test_registry.py` — 11 tests, including 7 new fault-injection
     cases added today: future-clock observation rejected, duplicate-sequence
     content conflict is HTTP 409, publish of a future-issued decision
     rejected, publish of an already-expired decision rejected, publish with
     a stale `map_version` rejected, GOAL blocked when the robot's health is
     stale, GOAL blocked when the decision's `transform_version` disagrees
     with the accepted observation's.
   - `hub/tests/test_goal_guard.py` — 12 tests, including 8 new cases:
     STOP latches even with a wrong `transform_version` (safety-critical:
     STOP must never be blockable by a stale calibration), decision addressed
     to another `robot_id` rejected, out-of-order `issued_at_ns` rejected,
     `map_version` regression rejected, e-stop engaged blocks GOAL, degraded
     localization blocks GOAL, goal beyond the configured max distance
     rejected, HOLD passes through accepted.
   - Full suite: `hub/.venv/bin/python -m pytest hub/tests -q` → **49 passed**
     (up from 34 before this work).

2. **Wire-level fault matrix** (`hub/tools/g5_fault_injection.py`) — what unit
   tests structurally cannot show: the full hub-over-real-HTTP round trip, and
   genuine process kill/restart behavior. Runs two real local hub instances
   (loopback, random per-run tokens: one `allow_goal=false` — the shipped
   default — one explicitly-labelled `allow_goal=true` TEST instance so a
   GOAL can reach the guard to be rejected *there*).

## Result: 9/9 PASS

```bash
hub/.venv/bin/python hub/tools/g5_fault_injection.py \
  --output data/robot_replays/g5_fault_injection_20260718
```

| # | Scenario | Expected | Observed |
| - | --- | --- | --- |
| 1 | hub: GOAL blocked by `allow_goal=false` policy | HTTP 409 | 409 |
| 2 | hub: already-expired decision rejected at publish | HTTP 422 | 422 |
| 3 | hub: observation with clock 10 s in the future rejected | HTTP 422 | 422 |
| 4 | guard: decision that expired in transit (fetched valid, evaluated after expiry) | REJECTED_EXPIRED | REJECTED_EXPIRED |
| 5 | guard: hub-issued decision evaluated against the wrong local calibration | REJECTED_TRANSFORM | REJECTED_TRANSFORM |
| 6 | guard: local e-stop overrides an otherwise-valid hub GOAL | REJECTED_HEALTH | REJECTED_HEALTH |
| 7 | guard: goal beyond the local max-distance safety limit | REJECTED_UNSAFE | REJECTED_UNSAFE |
| 8 | transport: hub completely unreachable (connection refused) | local fail-closed, no crash | fail-closed, no crash |
| 9 | transport: **STOP latch survives a real SIGKILL + restart of the hub process** | REJECTED_UNSAFE | REJECTED_UNSAFE |

Raw output: `data/robot_replays/g5_fault_injection_20260718/matrix.json`
(also per-hub logs and spool for scenario replay).

Scenario 9 detail, because it is the strongest safety proof here: a STOP
decision is fetched over real HTTP and latches the guard; the `allowed`
hub process is then genuinely `SIGKILL`ed (its in-memory registry state,
including that STOP decision, is gone — nothing is persisted about a STOP
by design) and a fresh hub process is started on the same port/config. The
restarted hub, knowing nothing of the prior STOP, serves its own benign
fallback HOLD. The robot-side guard — the same in-memory object the whole
time — still returns `REJECTED_UNSAFE` for that HOLD, because the STOP latch
is intentionally not clearable by network recovery or a hub restart, only by
`GoalGuard.local_operator_reset_stop()` called from an authenticated local
operator path.

## What is deliberately not covered here

- Malformed/corrupted decision payloads from a compromised or buggy hub
  (would need a stub server returning invalid JSON/schema; the strict Pydantic
  models already reject anything that doesn't parse, but that path isn't
  exercised end-to-end here).
- `map_version` *regression* specifically as an over-the-wire race (the
  registry only ever serves one current decision per robot and requires an
  exact `map_version` match to publish, so a true wire-level regression race
  needs two hubs or concurrent publishers; the regression rule itself is unit
  tested in `test_goal_guard.py`).
- Any of this running against real ROS 2 topics or the `wsj` receiver
  overlay — this run is entirely local-hub, local-guard.
- Real actuation rejection (no `cmd_vel`, no Go2 bridge, no hardware).

## Post-conditions

Both fault-injection hub instances (including the one that was SIGKILLed and
restarted) were stopped at the end of the run; `ss -tlnp` confirms ports
8390/8391 free. No GPU or robot involvement in this run.
