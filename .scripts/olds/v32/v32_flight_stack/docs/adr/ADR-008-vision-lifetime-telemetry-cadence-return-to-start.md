# ADR-008 — Vision Lifetime, Telemetry Cadence, and Return-to-Start/Finish

**Status:** Accepted
**Date:** 2026-08-16
**Supersedes:** the `_precision_control_active` detector-exclusivity mechanism introduced 2026-08-13; the report-only `MISSION_TIMEOUT` watchdog behaviour from ADR-004 §18
**Amends:** ADR-004 (§9.1 heartbeat ownership, §10 health model, §18 watchdogs), ADR-007 (point 10, shutdown deadline)

## Context

The 21:04 run on 2026-08-16 (`mission_db212c28c813`) was the first end-to-end success of the ADR-007 autonomous-start flow on macOS: self takeoff → checkpoint → operator route → search → `KIRMIZI_UCGEN` detected → OFFBOARD → centering converged → GPS recorded. It then failed in three distinct ways, which a read-only diagnosis (PART A) traced to three independent root causes.

### Observed

| Symptom | Evidence |
|---|---|
| `HEALTH vision=DOWN` for the whole of both centering windows | `VISION_FRAME_PROCESSED` gaps of **77.3 s** and **85.2 s**, bracketing `CENTERING_STARTED`/`CENTERING_*` to within ~30 ms |
| Camera window kept showing live frames *and* the `KIRMIZI_UCGEN` box | `_frame_grab_loop` is ungated and republished a **frozen** `_latest_detections` for 82 s |
| First centering lost the target on all 150 attempts | 150 consecutive `Merkezleme sirasinda hedef kayboldu!` |
| "15 s" centering budgets took 77.2 s and 82.0 s | `BLOCKING_STATE_CLEARED … after 77.2s / 82.0s` against an advertised `timeout_s: 5.0` |
| Landed at WP4 (north end), not the start/finish checkpoint (south) | `MISSION_FAILED → LANDING` with `flight.land()`; last recorded position **39.8 m** from the checkpoint, plus three further waypoints flown after |

### Root causes

**RC1 — the detection loop was muted during every pursuit.** `_detection_loop` was gated on `_precision_control_active`, set right after `switch_to_offboard()` and cleared only in `_resume_mission_route()`. The gate also suppressed the `VISION_FRAME_PROCESSED` publish, which *is* the vision heartbeat.

**RC2 — health DOWN was arithmetically guaranteed.** `VISION_HEARTBEAT_INTERVAL_S = 1/10 = 0.1 s` × `HEALTH_GRACE_MULTIPLIER = 3.0` → DOWN after 0.3 s of silence. Muting the loop could not produce any other outcome.

**RC3 — every telemetry getter cost ~1 s.** `get_global_position()` opened a fresh `async for pos in drone.telemetry.position()` per call and returned its first pushed value; PX4's default position rate is 1 Hz. The centering loop calls it once per iteration, so a loop designed for `OFFBOARD_SETPOINT_INTERVAL_S = 0.1 s` ran at ~1 Hz. Proof: **81 `VEHICLE_TELEMETRY` events at exactly 1.000 s spacing across the 82.0 s window** — one per iteration. Consequences: setpoints at 1 Hz (2× past PX4's ~500 ms Offboard timeout, i.e. the Hold↔Offboard bounce), and a 150-attempt budget stretched from 15 s to 150 s.

**RC4 — the exclusivity flag protected a streak that could not hold anyway.** `HSVContourDetector` commits a shape only after `HSV_STREAK_FRAMES = 3` detections within `HSV_STREAK_DIST_PX = 60 px`. At the 1 Hz rate RC3 imposed, consecutive frames are ~1 s apart and the target moves far more than 60 px, so the streak reset every iteration — 150/150 misses. The mechanism defending detector coherence was destroying it.

**RC5 — one terminal path of eight returned home.** Only Görev 3 completing successfully reached `Gorev3FinishPhase`, the sole consumer of `MissionCheckpoint`. Everything else called `MasterMissionController._safe_land()` → `flight.land()` (descend in place). Two paths did not land at all: `MISSION_TIMEOUT`'s `on_fire` only relabelled the phase, and Ctrl-C/window-close cancelled the mission task — `CancelledError` is a `BaseException`, so `except Exception` never saw it and `_safe_land()` was skipped entirely.

## Decision

### B0 — One background watcher per telemetry stream

Subscribe once at `connect()`, cache the latest value, serve every getter from cache. Applies to the four streams the centering and return-navigation paths read: `position`, `position_velocity_ned` (position *and* velocity), `flight_mode`, `attitude_euler`. Request `TELEMETRY_STREAM_RATE_HZ = 10.0` for the three that support `set_rate_*`; `flight_mode` has none (PX4 pushes it on change, the right cadence for a mode field).

- **The heartbeat moves to the producer.** `VEHICLE_TELEMETRY` is published by the position watcher, not by whoever calls `get_global_position()`. ADR-004 §9.1 called it "the orchestrator tick"; it was always the *position stream's* tick, and binding it to the caller is what let `go_to_and_center()`'s target-lost branch — which never called the getter — starve it into a DEGRADED↔STALE flap. Throttled to `TELEMETRY_HEARTBEAT_PUBLISH_INTERVAL_S = 0.5 s`.
- **Achieved rates are measured, not assumed.** `TELEMETRY_STREAM_RATES` is published every 10 s with observed Hz per stream, and a rejected `set_rate_*` publishes `TELEMETRY_RATE_REJECTED` and degrades that one stream rather than failing the connection. A rate PX4 accepts but does not honour must never look identical to one it does — that is exactly how the 1 Hz position stream hid for so long.
- **A bounded first-sample wait** guards the `CHECKPOINT_SAVE` read from recording the `(0,0,0)` cache-miss placeholder as the start/finish position.

*Rejected:* keeping per-call subscription and raising only the position rate. It would have fixed the measured symptom while leaving `flight_mode`, `attitude_euler` and `position_velocity_ned` paying a full round-trip per call — and `flight_mode` is the worst of them, being change-driven: a fresh subscription can block until the *next* mode change, which is precisely when `switch_to_offboard()` polls it in a tight loop.

### B1 — One detection loop, for the whole of Görev 2

`_detection_loop` is the only caller of `detector.detect()` and runs unconditionally for all of `run()` (search → centering → payload → return). Everything else consumes a shared `DetectionFeed`. This inverts the 2026-08-13 fix: rather than making the loop yield the detector, we remove every competing consumer, so the streak stays coherent **by construction** and there is no flag that can get stuck `True`. (It could: once `_search_complete` permanently disabled `_resume_mission_route()`, nothing would ever have cleared it, and vision would have stayed DOWN through the entire payload phase.)

`DetectionFeed` makes freshness part of the contract. `get()`/`detections()` refuse any sample older than `DETECTION_STALE_AFTER_S = 0.5 s`, so a consumer cannot mistake "the loop went quiet" for "the loop says there is no target". `latest()` still exposes the stale sample for diagnostics. A failing detector publishes **nothing**, letting the feed age out rather than keeping the last good sample alive.

Also in scope, all consequences of the same root cause:

- **The frozen overlay.** `_frame_grab_loop` publishes only fresh detections; a stale feed draws no boxes and the dashboard says `VISION FEED STALE … detections not drawn`. This was the single most misleading thing about the run — it made a dead pipeline look like a working one.
- **The display-only lie.** `WAITING_CENTERING_CONVERGENCE` now reports `CenteringController.budget_s()` — the real `max_attempts × OFFBOARD_SETPOINT_INTERVAL_S` — instead of `CENTERING_CONVERGENCE_TIMEOUT_S = 5.0`, whose comment ("matches CenteringController's own 30×0.1s budget") had been stale for two revisions. Nothing enforces it either way; `set_blocking` is display-only. That is precisely why it must not lie.
- **The target-lost cadence.** That branch slept 0.5 s — slower than the control loop in the one situation that most needs speed, holding setpoints at 2× PX4's Offboard timeout and stretching the 15 s lateral budget to 75 s of wall clock. Now `OFFBOARD_SETPOINT_INTERVAL_S`, like every other iteration. **The attempt budget (150) is unchanged**; only the wall-clock cost of spending it returns to the designed 15 s.
- **The detection busy-spin** (long-standing, previously deferred). The failure path `continue`d past the loop's only `await asyncio.sleep()`. With a camera whose `get_frame()` raises without awaiting — what `GzCameraSource` does before its first frame, and what the 21:04:32 "art arda 30. hata" burst was — the loop had *no yield point at all* and starved every other coroutine on the event loop. The sleep now sits outside the `try`, so both paths pay it. This is the item B1 was asked to absorb; it is fixed here.
- **Instrumentation.** Each iteration publishes `CENTERING_STEP` (attempt/max, dx/dy px, normalized error, altitude error, setpoint sent, target seen, feed stale/age) and echoes a throttled INFO line every 0.5 s. Previously the loop emitted four strings total, so an 82 s centering was indistinguishable from a hung one.

**Unchanged, deliberately:** every gain, tolerance, attempt budget, HSV streak parameter, and the payload sequence. `PayloadReleaseService._verify_marker()` changed *source* (feed instead of its own `detect()`) and nothing else — what is checked, when, and Görev 2 Rapor Bölüm 13's "informational only, never gates mission flow" rule all stand.

**Görev 3 is out of scope.** Its phases still call `detect()` directly, which remains correct: the detection loop's lifetime is `Gorev2Orchestrator.run()`, so during Görev 3 they are again the only consumer. Unifying that is future work, not a live defect.

### B2 — Return to start/finish on every terminal path

One implementation, `MasterMissionController._return_to_start_finish_and_land()`, reached by every route: fly to the recorded checkpoint at `MISSION_ALTITUDE_M` via `goto_global_position_and_wait()`, hold `RETURN_TO_START_FINISH_HOLD_S = 2 s`, then land. It logs `RETURNING_TO_START_FINISH (dist=… m)`, sets the `RETURNING_TO_START_FINISH` blocking reason, and publishes arrival distance.

| A2 row | Path | Before | After |
|---|---|---|---|
| 1 | Görev 2 raises | land in place | return → land |
| 2 | Görev 2 → `MISSION_FAILED` | land in place | return → land |
| 3 | Görev 3 raises | land in place | return → land |
| 4 | Görev 3 returns False | land in place | return → land |
| 5 | Görev 3 succeeds | return (via `Gorev3FinishPhase`) → land | unchanged in effect — already there, so the return converges immediately |
| 6 | `MISSION_TIMEOUT` (600 s) | phase relabel only | **abort** → return → land |
| 7 | Ctrl-C / `q` / window close | **nothing — vehicle left airborne** | return → land, bounded |
| 8 | Centering/offboard/GPS-save failure | resumes route (not terminal) | unchanged |

Mechanics:

- The mission body runs as its own task so both abort sources converge on one `except asyncio.CancelledError` handler: `request_abort()` cancels it directly (row 6), and a Ctrl-C cancel of `run()` propagates into it through the await (row 7). `CancelledError` handling is now explicit rather than accidentally excluded.
- Aborts are bounded by `ABORT_RETURN_DEADLINE_S = 45 s`; on overrun, land immediately. `main_gz.py`'s mission-thread join rises from 15 s to `ABORT_RETURN_DEADLINE_S + 15 s` — a shorter join would abandon the thread mid-return and reintroduce the very failure this removes.
- Offboard is acquired first when the vehicle is not already in it (a failed pursuit leaves it in HOLD; an abort may land mid-MISSION), since `goto_global_position_and_wait()` streams Offboard setpoints.
- Pressing `q` no longer breaks the paint loop: the vehicle is still airborne flying its return leg, which is exactly when an operator needs to watch it. Closing the window still breaks (nothing to paint into); the bounded join still waits for the landing.
- **The one permitted exception** is a failure before `CHECKPOINT_SAVE` — `checkpoint.is_saved()` is false, there is no recorded position, and Görev 2 Rapor Bölüm 4.2 forbids inventing one. Land in place and publish `RETURN_TO_START_FINISH_SKIPPED` with the reason.
- PX4 failsafes (battery, link loss) remain PX4's. Nothing here intercepts them.
- A failed return never costs the landing: navigation errors are caught, published as `RETURN_TO_START_FINISH_FAILED`, and the landing proceeds regardless.

`MISSION_TIMEOUT` acting is not a nicety. `GOREV2_MAX_FLIGHT_DURATION_S = 600` is Şartname Bölüm 5.6's **mandatory** 10-minute limit; a watchdog that only renames a phase does not enforce it.

## Consequences

- Vision health now reflects the vision pipeline rather than the mission phase. Any future DOWN during flight is a real fault, not an artefact.
- Centering runs at its designed 10 Hz, so setpoints stay well inside PX4's Offboard timeout and the HSV streak has coherent, closely-spaced frames to work with — the two preconditions the tuned gains and tolerances always assumed.
- Wall-clock centering timeouts drop ~10× (150 attempts now cost 15 s, not 150 s). Intended: it restores the configured budget rather than changing it.
- The event log grows a `CENTERING_STEP` per control iteration (DEBUG). Deliberate — it is the record that makes a centering failure diagnosable without another flight.
- `CenteringController` and `PayloadReleaseService` take a `DetectionFeed` instead of an `IDetector`; the three entrypoints construct one feed at the composition root. `Gorev2Orchestrator` keeps the detector, since it owns the only loop that uses one.
- `tests/test_detector_exclusivity.py` asserts the inverse of what it used to. Rewritten, not deleted: the race it guarded is real, and the new invariant (one consumer, never paused, freshness enforced) is what now prevents it.

## Validation

`V1` nominal, `V2` forced centering failure (`KURSAD_FORCE_CENTERING_FAILURE=1`, which short-circuits before the loop and touches no gain, tolerance or budget), `V3` Ctrl-C mid-mission. See the PHASE 6 report for timelines, landing distances, and measured stream rates.
