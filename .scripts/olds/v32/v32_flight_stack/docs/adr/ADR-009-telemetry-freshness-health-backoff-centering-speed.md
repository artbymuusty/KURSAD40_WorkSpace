# ADR-009 — Telemetry Freshness, Health Correctness, Pursuit Backoff, and the First Centering Speed Step

**Status:** Accepted
**Date:** 2026-08-16
**Amends:** ADR-008 (B0 freshness regression, B1 validation hook), ADR-004 §10 (health model)
**Baseline:** the V1 nominal run of 2026-08-16 22:55 (`mission_f822d252b30e`) and the V2 forced-failure run of 23:06 (`mission_62f5f9f4f77f`)

## Context

ADR-008's V1 run was the first full end-to-end success: Görev 2 + Görev 3, landing 0.48 m from the checkpoint. Its instrumentation then made four separate problems measurable for the first time — three defects and one tuning gap.

### Measured baseline (V1, 21 centering pursuits)

| Metric | V1 |
|---|---|
| `CENTERING_STEP` / setpoint rate | 9.35–9.87 Hz (design 10 Hz) |
| budget spent per 150-attempt pursuit | 15.76–15.86 s (design 15 s) |
| vision frames | 3645 / 389.4 s = 9.36 Hz, **zero** gaps > 1.5 s |
| telemetry streams | position / position_velocity_ned / attitude_euler 9.95–10.1 Hz |
| landing distance | 0.48 m |

Pacing was correct. Convergence was not: `|ey|` decayed with **τ ≈ 17–23 s** against a 15 s budget, so no first-lock pursuit ever converged. Targets were captured only because 5–6 cumulative pursuits walked the error down, and pursuit 6 **stalled outright** at `|ey| = 0.0146` (≈7 px, 0.20 m ground) — where the control law asks for `0.0146 × 0.3 × 2.0 = 0.009 m/s`, i.e. 9 mm/s, which PX4 cannot act on against drift.

### The three defects

**D1 — the B0 cache serves stale data silently (regression I introduced).** ADR-008 B0 replaced blocking per-call subscriptions with cache reads and lost the one useful property of the old design: a dead channel used to block or raise. On 2026-08-16 at 23:10:49 the MAVSDK channel wedged; `get_global_position()` / `get_velocity_ned()` returned the same frozen sample for **66.8 s**, and `goto_global_position_and_wait()` flew its full 60 s timeout computing distance and speed from numbers that had stopped changing. `land()` then timed out too. `_StreamCache.age_s()` existed; nothing consulted it.

**D2 — HealthMonitor feeds itself, so DOWN is unreachable.** `HEALTH_STATE_CHANGED` carries `subsystem=<the subsystem it describes>`, `on_event()` counts any event from a registered subsystem as a heartbeat, and `check()` publishes to the same bus `on_event` is subscribed to. A dead subsystem therefore refreshes its own liveness every tick. Reproduced in isolation: with the pre-fix code and a regular 1 Hz supervisor tick, a subsystem that has been silent for 11 s reads **`HEALTHY` at every single tick**. Live on 23:10–23:11 the timing was less regular so it oscillated HEALTHY↔DEGRADED↔STALE instead — either way it **never reached DOWN** during 66.8 s of a completely dead link. "Going silent IS the signal" (ADR-004 §10) was unreachable for every registered subsystem.

This also corrects part of ADR-008's A1 analysis: the `MavsdkBackendBase` flapping in the 21:04 log was attributed solely to heartbeat starvation in the target-lost branch. That starvation was real, but the *never-settling* pattern is this self-feed.

**D3 — a failed pursuit re-engages instantly, and ADR-008's validation hook made that pathological.** The `KURSAD_FORCE_CENTERING_FAILURE` hook returned `False` before the loop began, so every failure was free. The search loop then re-engaged the same still-track-ready target on the very next iteration: **585 pursuits and 601 Mission resumes in 3.5 minutes** (~3 PX4 pause/resume cycles per second) until PX4 stopped answering and `pause_mission()` timed out — taking the simulator with it. The route also never advanced past waypoint 3/4, because the Mission spent more time paused than flying. Two separate faults: the instrument distorted pacing, and the search policy had no backoff of its own.

## Decision

### D1 — Freshness is part of the getter contract

`TelemetryStale(RuntimeError)` is defined at the contract boundary ([`i_flight_backend.py`](../../core/interfaces/i_flight_backend.py)). Every cached getter — `get_global_position`, `get_velocity_ned`, `get_position_ned`, `get_yaw_deg`, `get_flight_mode` — routes through one `_fresh()` guard and **raises rather than returning a stale value or `None`**. Raise was chosen over `None` so a caller cannot accidentally propagate a missing reading into arithmetic; the contract is "if you got a value, it is current."

- `TELEMETRY_STALE_AFTER_S = 1.0` (10× the 10 Hz stream period), `TELEMETRY_STALE_AFTER_FLIGHT_MODE_S = 3.0` (change-driven, so quiet is normal).
- `TELEMETRY_STALE` is published **once per stale episode**, not per call — a 10 Hz loop reading a dead cache would otherwise bury the timeline in identical CRITICALs.
- Consumers (`go_to_and_center`, `goto_global_position_and_wait`, `climb_to_altitude`) stop commanding immediately via `_abort_on_stale()` and return `False`, which the existing chain already treats as non-convergence → abort → return-to-start → land. Deliberately **no** final zero-velocity setpoint: with the link down it cannot arrive, and pretending otherwise only delays the fallback.
- The return leg is closed-loop navigation, so `master_fsm` waits `TELEMETRY_RECOVERY_WAIT_S = 5.0` for the link to come back before giving up and landing in place — a transient hiccup should not cost the return, and a dead link should not cost 60 s.

### D2 — The monitor ignores its own events

`_SELF_EMITTED_CODES = {"HEALTH_STATE_CHANGED"}`, skipped in `on_event()`. Health recovery still works on a *real* heartbeat; only the self-referential path is cut. DOWN already renders red (`_HEALTH_COLOR["DOWN"] == COL_BAD`), now reachable.

### D3 — Faithful failure instrument, plus real backoff

- The instant-fail hook is **deleted**. `CENTERING_LATERAL_TIMEOUT_ENV` (`KURSAD_CENTERING_LATERAL_TIMEOUT_S`) shortens the budget instead, so a failure costs real wall-clock time and pacing stays honest.
- `CENTERING_MAX_ATTEMPTS_PER_TARGET = 3` and `CENTERING_RETRY_COOLDOWN_S = 10.0`, enforced in the orchestrator's search policy (not the control loop — "how often do we chase this shape" is a search question). A shape in cooldown or over the cap is excluded from candidates; detection continues normally, only the pursuit backs off. At the cap it publishes `TARGET_SEEN_BUT_NOT_CENTERED` and searching continues for the other shape, because "I can see it and cannot centre on it" is a materially different outcome from "I never saw it" and the operator must be able to tell them apart afterwards.

### S1 — First centering speed step (conservative, measured)

Exactly three changes, no more:

| Change | From → To | Rationale from V1 |
|---|---|---|
| `kp_vertical` (gz) | 0.3 → 0.5 | matches `kp_horizontal`; `ey` normalizes by the **shorter** half-axis (480 vs 640 px), so forward was both weaker-gained and larger-normalized — the axis that never closed in any pursuit |
| `MAX_CENTERING_SPEED_M_S` | 2.0 → 3.0 | τ ≈ 17–23 s against a 15 s budget |
| `CENTERING_MIN_CMD_SPEED_M_S` | new, 0.15 | removes the 0.009 m/s dead zone that stalled pursuit 6 |

The floor applies to the **lateral axes only**; altitude keeps pure proportional control (its 0.3 m tolerance is 20× looser and never reached the dead zone). Inside tolerance the command is exactly **0**. Tolerances, the 15 s budget, HSV streak params, `kp_horizontal` and `kp_altitude` are unchanged.

`kp_vertical` was briefly reverted to 0.4 after V1′ showed oscillation, then **restored to 0.5** (2026-08-17) once the V1′ data showed the oscillation was on the X axis and caused by the floor, not by this gain — see S2. `real_system.yaml` deliberately stays at 0.3: a real-vehicle gain must not move on a simulator measurement.

### S2 — The floor must scale with altitude

S1's floor was one speed for every altitude, but the tolerance is **angular** (normalized pixels), so its ground equivalent shrinks with altitude. V1′ measured the consequence directly: both 0.30 m payload steps failed to converge and one diverged (`|ey|` 0.0042 → 0.1396), with 0.66 zero-crossings/second.

```
floor(alt) = min(CENTERING_MIN_CMD_SPEED_M_S,
                 CENTERING_FLOOR_TOL_FRACTION × ground_tolerance_m(alt) / OFFBOARD_SETPOINT_INTERVAL_S)
```

`CENTERING_FLOOR_TOL_FRACTION = 0.5` — one iteration may cross at most half the tolerance band, so the vehicle can never be commanded straight across it and out the other side. Per-axis, because x and y normalize by different half-axes (640 px vs 480 px). Inside tolerance: still exactly 0.

| alt | tol_x (ground) | floor_x | step_x | ratio | tol_y | floor_y | step_y | ratio |
|---|---|---|---|---|---|---|---|---|
| 15.00 m | 0.1778 m | 0.1500 | 0.01500 | 0.084 | 0.1333 m | 0.1500 | 0.01500 | 0.112 |
| 10.00 m | 0.1185 m | 0.1500 | 0.01500 | 0.127 | 0.0889 m | 0.1500 | 0.01500 | 0.169 |
| 5.00 m | 0.0593 m | 0.1500 | 0.01500 | 0.253 | 0.0444 m | 0.1500 | 0.01500 | 0.337 |
| 0.45 m | 0.0053 m | 0.0267 | 0.00267 | **0.500** | 0.0040 m | 0.0200 | 0.00200 | **0.500** |
| 0.30 m | 0.0036 m | 0.0178 | 0.00178 | **0.500** | 0.0027 m | 0.0133 | 0.00133 | **0.500** |

S1's flat-floor ratios for comparison: 15 m 0.084, **0.45 m 2.81, 0.30 m 4.22**. At and above 5 m the absolute 0.15 m/s cap still binds, so S1's high-altitude gain is untouched. Measured steady-state effect:

| alt | S2 crossings | S1 crossings |
|---|---|---|
| 15 / 10 / 5 m | 0 | 0 (identical) |
| 0.45 m | **0** | 27 |
| 0.30 m | **0** | 43 |

**A note on the test harness.** The first version of the S2 closed-loop test integrated the commanded velocity perfectly and let `go_to_and_center()` exit on the first in-band sample — it passed with S1's flat floor too, i.e. it proved nothing. It was rewritten to add first-order velocity lag (τ = 0.3 s, the reason a coarse floor overshoots at all) and to measure **steady-state** behaviour without the early exit. Only then did it discriminate.

Payload release altitude was separately revised 0.30 m → **0.45 m** (operator, 2026-08-17); `GOREV3_DESCENT_ALTITUDE_M` stays 0.30 m because it also fixes the physical pickup geometry.

### PX4 stability — Mission resume spacing and confirmation

PX4 SITL died three times on 2026-08-16/17. The navigator's own `Executing Mission` lines (extracted from the ULogs) show one variable tracking the outcome:

| session | resumes | min spacing | outcome |
|---|---|---|---|
| V1′ `10_17_23.ulg` | 4 | **14.39 s** | survived; clean landing + disarm |
| V2′ `10_28_08.ulg` | 4 | **5.04 s** | 4th resume produced **no** `Executing Mission` → HOLD → PX4 stopped logging 2.5 min later |
| V2 `20_06_33.ulg` | 450 | **0.11 s** | wedged; `pause_mission()` timed out; PX4 died |

No failsafe, no mission rejection, no error of any kind appears in any of the three logs — PX4 accepted every `start_mission()` and simply did not act on the last one. Two defects in `_resume_mission_route()` explain it:

1. **No spacing.** Resumes went out as fast as pursuits failed. Note the per-target `CENTERING_RETRY_COOLDOWN_S` does not bound this: with two shapes alternating, the *global* resume rate is up to 2× the per-target cooldown, which is why V2′ managed 5.04 s despite a 10 s cooldown.
2. **No confirmation.** `_start_uploaded_mission()` has always polled until PX4 reports MISSION before proceeding. This path fired `start_mission()` and published `MISSION_ROUTE_RESUMED` immediately. When PX4 ignored the 4th resume, nothing noticed: the route froze at 3/4, `is_mission_finished()` could never become True, and the search span until the watchdog.

Mitigation (implemented — confined to the resume helper, five tests): `MISSION_RESUME_MIN_INTERVAL_S = 15.0` (15 s because 14.39 s is the smallest spacing *observed to survive* — evidence, not tuning) and a MISSION-mode confirm with one retry, escalating to a CRITICAL `MISSION_RESUME_NOT_CONFIRMED` rather than claiming success. Deliberately not an abort: a stuck Mission is already covered by `MISSION_TIMEOUT`.

**Not implemented, needs a decision:** resuming via `set_current_mission_item(current_index)` + `start_mission()` rather than bare `start_mission()`. That is the more robust form, but it needs the backend to expose the live mission index, which is outside the resume helper.

## Consequences

- A wedged link now costs ~1 s of guard time instead of a full navigation timeout, and the mission falls into land-in-place with `TELEMETRY_STALE` on the record instead of appearing to navigate.
- Any getter can now raise mid-loop. Every call site in the centering/return paths handles it; the search loop deliberately lets it propagate, which routes to A2 row 1 (Görev 2 exception → return-to-start → land).
- A genuinely dead subsystem reaches DOWN and stays there. Some previously-noisy DEGRADED/STALE flapping will disappear from logs — that flapping was an artefact, not signal.
- Pursuits are capped at 3 per shape with 10 s spacing, so a target that cannot be centred costs at most ~45 s and one `TARGET_SEEN_BUT_NOT_CENTERED` instead of an unbounded pause/resume storm.
- Faster gains and a command floor introduce overshoot risk. The operator's instruction stands: **if V1′ shows oscillation or overshoot, revert `kp_vertical` to 0.4 and report; nothing beyond S1 without explicit go.**

## Validation

V1″ (2026-08-17 14:07, `mission_a4985ab21eb2`), V3, and a live D1 proof, all against PX4 SITL with the operator's QGC route (byte-identical to backup, ADR-007 accepted).

**S1+S2 centering — τ improved 3–4× against the V1 baseline:**

| | V1 (baseline) | V1′ (S1) | V1″ (S1+S2) |
|---|---|---|---|
| τ on \|ey\|, first lock | 17–23 s | 9.4 / 17.5 s | **5.3 / 6.0 s** |
| τ, subsequent lock | — | 3.5 / 5.2 s | **3.0 s** |
| converged inside 15 s budget | never | yes | yes (74/150 = 7.7 s) |
| `CENTERING_STEP` rate | 9.35–9.87 Hz | 9.37–9.55 Hz | 9.42–9.63 Hz |
| vision | 9.36 Hz, no gaps | 9.36 Hz, no gaps | 9.40 Hz, no gaps |

**x-axis question — closed.** V1′ showed dx zero-crossings consistently above dy at every altitude. With S2 the asymmetry is gone: dx 10 (0.25/s) vs dy 10 (0.25/s), **ratio 1.00**. The asymmetry was the altitude-independent floor, not `kp_horizontal`. No x-axis tuning applied or needed.

**PX4 stability — the mitigation holds.** V1″ made 9 resumes at a minimum observed spacing of **16.0 s** (target ≥15 s), with **0** `MISSION_RESUME_NOT_CONFIRMED`. One resume did fail to confirm and the retry recovered it — the exact silent failure that killed V2′, now caught. **PX4 survived every run of the session**, the first day it has.

Cost: 4 throttled resumes totalling **35.8 s**, i.e. **17.8%** of the 201.7 s mission. The cost is not only time — the vehicle keeps flying the route while waiting, so a target near the start can be left behind. In V1″ `MAVI_ALTIGEN` hit the 3-attempt cap and was abandoned (`TARGET_SEEN_BUT_NOT_CENTERED`), so only one target was recorded and the run took the search-incomplete path.

**B2 return, on the path that started all of this.** Search-incomplete → `RETURNING_TO_START_FINISH (dist=99.0 m)` → arrived → landed **0.83 m** from the checkpoint. This is A2 row 2, the exact 2026-08-16 failure that landed 39.8 m away, exercised live for the first time.

**D1 live proof.** `kill -STOP` on the PX4 binary during flight: freeze at 14:19:12.476, `TELEMETRY_STALE (position 1.1s)` at 14:19:13.532 — **1.06 s**, matching `TELEMETRY_STALE_AFTER_S = 1.0`. The mission aborted on stale inputs instead of commanding on them, the bounded recovery wait saw the link return, and the return leg then flew and landed **0.7 m** from the checkpoint.

**Open after this ADR:**
- `switch_to_offboard()` failures: 0 in V1′, **4 in V1″**, and one that cost V3 its return leg (`OFFBOARD_UNAVAILABLE_FOR_RETURN`). Correlates with the resume throttle: pursuits now cluster immediately after a resume, so `pause_mission()` lands ~1 s after `start_mission()` and PX4 declines OFFBOARD. This now blocks the B2 return, which is the whole point of B2.
- V3 (Ctrl-C) remains **unproven**: the SIGINT never reached `main_gz.py`'s handler (no `Ctrl-C -- stopping mission` in the log); the mission ended naturally instead.
- PX4 stays in `flight_mode=LAND` after landing and refuses to re-arm; three runs died on this. Worked around in the test harness by commanding `hold()` first (2 s vs 3+ min of waiting), not in mission code.

167 → **177 unit tests pass**, including 23 ADR-009 guards.
