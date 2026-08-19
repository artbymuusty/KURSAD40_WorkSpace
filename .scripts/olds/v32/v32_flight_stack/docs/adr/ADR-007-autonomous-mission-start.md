# ADR-007 — Autonomous mission start after self-takeoff

**Status:** Accepted / Implemented (2026-08-16).
**Supersedes:** the 2026-08-13 "mission_gz supervisor model" operator-start rule.

## 1. Superseded decision (quoted verbatim)

`gorev2_orchestrator._wait_for_operator_mission_start()` previously stated:

> "BUG FIX (operator revision, 2026-08-13, "mission_gz supervisor model"):
> this system must NEVER issue the MAVLink command that starts Mission mode
> -- starting the mission, exactly like uploading its route, is exclusively
> the operator's action in QGroundControl. `_run_inner()` previously called
> `flight.start_mission()` itself right after confirming a route was present,
> silently doing what only the operator's own Start Mission button is
> supposed to do. This instead blocks, polling `get_flight_mode()`, until
> Mission mode is externally observed active."

and `parameters.py`:

> "starting Mission mode is exclusively the operator's action in
> QGroundControl, same as the route upload itself -- this system only waits
> and observes."

**What survives:** route DEFINITION and upload remain exclusively the
operator's job in QGroundControl. This system still never generates or
uploads a route (`test_missing_operator_route_fails_loudly_...` continues to
enforce that). **What changes:** only the start command.

## 2. Why

Observed 2026-08-16: after the stack's own arm + takeoff to 15 m the vehicle
sat in `HOLD`, polling for `MISSION`, and the operator's QGC action did not
result in Mission mode. The 300 s `OPERATOR_MISSION_START_TIMEOUT` then fired
and the orchestrator executed its safe-landing path — the "RTL" that ended
several runs. A human-paced handoff in the middle of an otherwise autonomous
sequence is also unnecessary: the vehicle is already airborne and stable at
mission altitude, with a validated route on board.

## 3. Decision — flow

```
connect → route validated → arm → takeoff(MISSION_ALTITUDE_M=15)
   → altitude reached → checkpoint (current lat/lon/alt) saved
   → hold MISSION_START_HOLD_S = 3.0s (flight heartbeat published throughout)
   → [if seq0 is NAV_TAKEOFF] set_current_mission_item(start_index)
   → start_mission()
   → confirm flight_mode == "MISSION" within MISSION_MODE_CONFIRM_TIMEOUT_S = 10s
   → existing search phase, unchanged
```

The heartbeat (`get_global_position()`) is published during both the hold and
the confirmation wait; without it HealthMonitor ages `MavsdkBackendBase` into
`STALE` (`health.py:70-76`, interval 1.0 s) purely because the old operator
wait polled only `get_flight_mode()`.

## 4. Route validation rule

The uploaded route is accepted only if:

1. item count **>= 2**;
2. every item is `NAV_WAYPOINT` (16), except that `NAV_TAKEOFF` (22) is
   permitted **only at seq 0**;
3. **no** `NAV_LAND` (21) and **no** `NAV_RETURN_TO_LAUNCH` (20) anywhere.

On violation the run refuses to start, naming the offending `seq` and command,
and exits through the existing failure/safe path — before arming, not
mid-flight. The route is **never modified**; validation and
`set_current_mission_item()` only.

**Start index:** `1` when seq 0 is `NAV_TAKEOFF` (this system already performed
its own takeoff, so re-running the item against an airborne vehicle is
avoided), otherwise `0`.

**Why land/RTL are rejected rather than tolerated:** the search loop
(`gorev2_orchestrator.py:421-423`) runs while `not is_mission_finished()`, and
treats a route that finishes before both targets are found as
`MISSION_FAILED` (`search_incomplete_mission_finished`). A route ending in
`NAV_LAND` would additionally fly the vehicle into a landing while the
Offboard payload phase still expects to be airborne. Refusing such a route up
front converts a confusing mid-air failure into a clear pre-flight error.

## 5. Consequences

- `_wait_for_operator_mission_start()` and the use of
  `OPERATOR_MISSION_START_TIMEOUT_S` are removed (the constant remains in
  `parameters.py`, marked superseded, for reference).
- Dashboard blocking reason `WAITING_OPERATOR_MISSION_START` →
  `STARTING_UPLOADED_MISSION`.
- `start_mission()` is now called once at startup, so it is no longer a unique
  marker for "route resumed" — `test_mission_route_resume.py` was updated to
  distinguish the startup call (#1) from the resume (#2).
- `MISSION_START_HOLD_S` is a wall-clock delay inside `run()`; it is read
  through the parameters module so tests can zero it (`tests/conftest.py`),
  with its behaviour pinned by a dedicated test.
- Shutdown hardening: the mission runtime thread is a daemon with a bounded
  15 s join, so a stalled teardown can no longer keep the process alive
  holding `udp:14540`/`tcp:50051`.
