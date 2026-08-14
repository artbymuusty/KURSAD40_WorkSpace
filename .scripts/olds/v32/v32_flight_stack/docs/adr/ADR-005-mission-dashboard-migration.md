# ADR-005: Mission Dashboard — Archaeology & Migration Plan

**Status:** Investigation complete, migration plan proposed — no implementation authorized by this document
**Owner:** Principal Software Architect
**Companion to:** ADR-004 (Mission Operations Center)
**Applies to:** `v32_flight_stack` (Görev 2 + Görev 3)

---

## 0. Field findings (read this first)

Three things are true simultaneously, and conflating them will misdirect the fix:

1. **The dashboard was not deleted.** A dashboard exists in every version, V30 through the current `v32_flight_stack`, and is constructor-wired into all three live entrypoints (`main_real.py`, `main_gz.py`, `main_dual.py`) today.
2. **What actually regressed is architectural, not presence/absence.** V30/V31's dashboard ran on a dedicated thread behind a bounded queue with crash isolation from the flight loop. `v32_flight_stack`'s `Dashboard` runs synchronously, unisolated, inside the mission's own coroutine — a step backward from a more mature design that already existed and was not carried forward.
3. **The specific symptom reported — "mission starts, drone takes off, dashboard never opens" — has a precise, already-identified root cause that is neither of the above.** It is the still-open `upload_mission()` empty-mission bug from earlier in this engagement (ADR/session finding, not yet applied): it causes `Gorev2Orchestrator.run()`'s main loop — the only place `dashboard.update()` is ever called — to never execute at all. Takeoff happens before that loop; the dashboard window is created lazily on the loop's first iteration, which never comes.

Fixing the dashboard-open symptom today requires fixing `upload_mission()`, not the dashboard. Fixing the dashboard *architecture* — restoring V30/V31's isolation guarantees inside `v32_flight_stack` — is a separate, still-necessary migration, detailed below.

---

## 1. Investigation method

Comparative, evidence-first, no assumptions:

- `diff` between `debug_view.py` across `v31_3rd_mission/` and `v32/` (byte-level).
- `diff` between `main.py` across `v31_3rd_mission/` and `v32/` (byte-level).
- `grep` for `dashboard|viewer|imshow|debug_view|EventBus|event_bus` across every version directory and `v32_flight_stack`.
- Direct read of `mission_types.py`, `debug_view.py`, `main.py` (flat lineage) and `core/telemetry/dashboard.py`, `gorev2_orchestrator.py`, `gorev3_orchestrator.py`, `master_fsm.py`, `main_gz.py`, `main_real.py`, `main_dual.py` (`v32_flight_stack`).
- `python -c "import ast; ast.parse(...)"` against `v32/main.py` to verify a suspected syntax defect.
- `cat` of the three `run_mission_v32_*.sh` launchers to determine which codebase the operator actually runs.
- `git log --follow` / `git log -- .scripts/olds` to check for commit-level attribution.

**Git history disclosure:** `.scripts/olds/` (V30 through `v32_flight_stack`) entered this repository in a single commit — `c5eb734e5a`, *"Initial import: modified PX4 (Kursad edition) + scripts,"* 2025-11-27. There is no incremental commit history showing the V30→V31→V32 evolution or the moment the dashboard was reduced; it was developed outside this repository's version control before that snapshot. **Item 2 of the requested deliverables ("which commits/refactors caused it") cannot be answered from commit history — no such history exists.** The answer below is instead reconstructed from structural evidence: comparing the actual file trees, which is conclusive on its own.

---

## 2. Where the dashboard existed — V30 → V31 → V32(flat) lineage

| Module | Role | Evidence |
|---|---|---|
| `mission_types.py` | `Event` (dataclass), `EventBus` (pub/sub, per-subscriber try/except isolation), `event_bus` (module-level singleton), `UISnapshot` (**frozen** dataclass — vision, mission, flight, servo, diagnostics, mission-map fields) | present, near-identical, in `v30/`, `v31/`, `v31_2nd_mission/`, `v31_3rd_mission/`, `v32/` |
| `debug_view.py` | `DebugView` — pure rendering: camera feed (~70% width) + a 9-panel dark-theme dashboard (MISSION / VEHICLE / VISION / SERVO / PAYLOAD / HEALTH / DIAGNOSTICS / TIMELINE / MISSION MAP with auto-fit NED plot, flight trail, drop markers, target markers). `UIWorker` — owns the window lifecycle. | **byte-identical** between `v31_3rd_mission/debug_view.py` and `v32/debug_view.py` (`diff` exit code 0) |
| `main.py` | Constructs `ui_worker = UIWorker()`; calls `ui_worker.start()` immediately after mission-core init — i.e. automatically, the instant the mission process launches, no operator action. Every 33 Hz flight-loop tick builds a `UISnapshot` and calls `ui_worker.update(snapshot)`. `ui_worker.stop()` in the `finally:` shutdown block. | `main.py:127-129` (construct+start), `:260-304` (per-tick snapshot+update), `:317` (stop) — line numbers stable across `v31_3rd_mission` → `v32` |

### 2.1 `UIWorker`'s isolation properties (`debug_view.py:400-482`) — the part that mattered most

```python
class UIWorker:
    """Dedicated thread for rendering OpenCV UI to prevent blocking the flight loop."""
    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def update(self, snapshot: UISnapshot):
        # ... this runs directly on the main mission loop's thread, so it
        # must never raise: an unhandled exception here would take the
        # mission down with it, not just the dashboard.
        try:
            self.snapshot_queue.put(snapshot, block=False)
        except queue.Full:
            self.dropped_snapshots += 1
            pass  # Drop old snapshot to maintain 33Hz flight loop performance

    def _run(self):
        while self.running:
            try:
                snapshot = self.snapshot_queue.get(timeout=0.05)
                self.debug_view.draw_dashboard(snapshot, ...)
            except queue.Empty:
                pass
```

All actual `cv2` work (`namedWindow`, `imshow`, `waitKey`) happens inside `_run()`, on the dedicated thread. The flight loop's only interaction with it is a non-blocking, bounded-queue `put()` — the exact "mission runtime must never block on the Ops Center" property ADR-004 (§1, §13) proposed as new. **It already existed. It is what got lost.**

Additionally, `main.py`'s own per-tick call site wraps the entire snapshot-construction-and-update block in its own `try/except`, with the explicit comment: *"a bug in snapshot construction or rendering can never take the mission loop down with it — mission logic must never depend on the UI."* Two independent layers of crash isolation, both intentional, both documented in-line.

---

## 3. What "V32" means at runtime — the fork the investigation had to resolve first

Two codebases both call themselves V32 and coexist on disk:

| | `.scripts/olds/v32/*.py` (flat) | `.scripts/olds/v32/v32_flight_stack/` (package) |
|---|---|---|
| Structure | Single-directory, procedural `main.py` + siblings — direct continuation of the V30/V31 lineage | Ground-up rewrite: `core/`, `gz_system/`, `real_system/`, `dual_system/`, `tests/`, its own `pyproject.toml` |
| Mission logic | `MissionManager` (`mission.py`), state-string driven | `Gorev2Orchestrator`, `Gorev2DurumMachine`, explicit-ish async flow |
| Dashboard | `UIWorker`/`DebugView` — full 9-panel, threaded, byte-identical to V31 | `Dashboard` (`core/telemetry/dashboard.py`) — HUD only, synchronous |
| **Actually launched by** | *(no script found that runs this)* | `run_mission_v32_real.sh`, `run_mission_v32_gz.sh`, `run_mission_v32_dual.sh` — all three `exec` into `v32_flight_stack/{real,gz,dual}_system/main_*.py` |
| Runnable right now? | **No** — `v32/main.py:52` contains `prev_time = time.time()1`, a hard `SyntaxError` (verified with `ast.parse`); the whole module fails to import regardless of which code path would execute | Yes |

**Conclusion:** the flat `v32/` directory (currently open in the editor) is not the live system. It is either abandoned mid-edit or superseded debris left alongside the real rewrite. Every finding about "the current V32" in the rest of this report refers to `v32_flight_stack`, because that is what the operator's `RUN MISSION` action actually executes.

---

## 4. Feature and property comparison — what was ported, what wasn't

| Property | V30/V31/V32(flat) — `UIWorker`/`DebugView` | `v32_flight_stack` — `Dashboard` | Verdict |
|---|---|---|---|
| Event bus (`EventBus`/`Event`/`event_bus`) | Yes — module-level pub/sub, isolated per-subscriber | **Absent entirely** — zero matches for `EventBus`/`event_bus` anywhere in `v32_flight_stack` | Dropped |
| Structured snapshot type (`UISnapshot`) | Yes — frozen dataclass, ~25 typed fields across vision/mission/flight/servo/diagnostics | No — `Dashboard.update()` takes 6 loose keyword args (`frame`, `detections`, `state_text`, `current_alt`, `payload_1_released`, `payload_2_released`) | Reduced |
| Rendering thread | Dedicated `threading.Thread(daemon=True)` | None — runs inline in the calling coroutine | Dropped |
| Backpressure | Bounded `queue.Queue(maxsize=1)`, drop-oldest | None — every tick calls `cv2.imshow` directly | Dropped |
| Crash isolation | Two layers: `UIWorker.update()`'s own try/except *and* `main.py`'s call-site try/except | **None** — `Gorev2Orchestrator.run()` calls `self.dashboard.update(...)` with no surrounding try/except | Dropped — see §6, this is the dangerous one |
| Panel set | 9 panels: Mission, Vehicle, Vision, Servo, Payload, Health, Diagnostics, Timeline, Mission Map (auto-fit NED plot, flight trail, drop/target markers) | 1 HUD strip: state text, altitude, 2 payload flags, plus bounding boxes on the raw frame | Reduced |
| Auto-launch on mission start | Yes, unconditional (`ui_worker.start()` right after core init) | Structurally yes (constructed and injected before `orchestrator.run()`), but see §0.3 for why it currently doesn't appear | Present in principle, blocked by an unrelated bug |
| Carries through Görev 3 | N/A (V30/V31 had no Görev 3 concept) | `Gorev3Orchestrator` **does** accept and use an optional `dashboard` param (`gorev3_orchestrator.py:16,23,26-29`) | Present at the class level |

---

## 5. Root cause of the reported symptom (traced, not inferred)

Full call chain, `v32_flight_stack/core/mission/gorev2_orchestrator.py`:

1. `run()` connects, arms, takes off to `MISSION_ALTITUDE_M` — **this is the part the operator observes working** ("drone takes off").
2. `upload_mission(waypoints)` is called with 5 real waypoints from `_generate_square_mission()`, but the backend implementation (`mavsdk_common/mavsdk_backend_base.py:80-84`) discards the argument and uploads an empty `MissionPlan([])` — a defect this engagement already identified and has not yet applied a fix for.
3. `start_mission()` starts the empty mission.
4. `while not await self.flight.is_mission_finished() and not gorev2_tamamlandi:` — for an empty mission, `mission_progress()` reports `current == total == 0` on the very first check, so `is_mission_finished()` returns `True` immediately.
5. **The loop body never executes.** `self.dashboard.update(...)` (line ~94-101) lives entirely inside that loop. It is never called. `cv2.namedWindow`/`cv2.imshow` are never reached. No window opens.

This is falsifiable: applying the `upload_mission()` fix already proposed in this engagement is sufficient, on its own, to make the dashboard window appear — independent of anything else in this report.

---

## 6. Secondary, latent regression (not yet observed, but live)

Once §5 is fixed and the loop begins running, `self.dashboard.update(frame=..., detections=..., ...)` executes on every tick with **no surrounding exception handling**, unlike both isolation layers V30/V31 had. `cv2.imshow`/`cv2.waitKey` are not guaranteed to succeed in every deployment environment (headless companion computer with no `$DISPLAY`, a missing Qt platform plugin, a remote SSH session without X forwarding). If they raise, the exception propagates out of `Dashboard.update()`, out of `Gorev2Orchestrator.run()`, and **terminates the entire mission coroutine — mid-flight.**

This directly contradicts the requirement restated in this task: *"The Mission Dashboard is NOT a mission controller... it must never make flight decisions... it is an observer."* Today, a rendering failure is not just an observer going dark — it can end the mission. This is the most safety-relevant single finding in this report and should be treated as the top migration priority once §5 is fixed.

---

## 7. Adjacent finding — Görev 3 or "Mission Complete" transition is currently unreachable regardless

`MasterMissionController` (`core/mission/master_fsm.py`) is the only component that sequences `Gorev2Orchestrator.run()` → `Gorev3Orchestrator.run()` → `flight.land()`, i.e. the only path by which "dashboard remains active through the whole mission… then transitions into Mission Complete" could occur end-to-end. A repository-wide search shows `MasterMissionController` is referenced **only inside the file that defines it** — none of the three live entrypoints (`main_gz.py`, `main_real.py`, `main_dual.py`) construct or run it. Each entrypoint calls `asyncio.run(orchestrator.run())` directly on a bare `Gorev2Orchestrator`.

This means: today, a mission ends when `Gorev2Orchestrator.run()` returns, with no explicit `MISSION_COMPLETE` transition, no Görev 3, and no graceful dashboard handoff — the process either keeps running with a static last frame or exits, depending on what follows in `main()` (nothing does, currently — `main()` returns after `asyncio.run(...)`). This is independent of the dashboard investigation but blocks part of the "required behavior" (graceful termination / Mission Complete mode) from being satisfiable until `MasterMissionController` is actually wired into an entrypoint — a decision this report flags but does not resolve, since it is a mission-sequencing question, not a dashboard one.

---

## 8. Target architecture — restoring V30/V31's guarantees inside `v32_flight_stack`

This is not "copy `UIWorker` back in." `v32_flight_stack` has a real interface-driven architecture V30/V31 didn't (`IFlightBackend`, `ICameraSource`, `IPayloadActuator`, dependency injection throughout `main_gz.py`/`main_real.py`/`main_dual.py`). The migration must fit that shape, not override it.

**What is preserved from V30/V31 (proven, not experimental):**
- A pub/sub event model — this is exactly `EventBus`/`Event`, and is the same mechanism ADR-004 (§12) specified as new; here it is correctly understood as *restoring* a pattern that already shipped and worked, not inventing one.
- A dedicated rendering thread, fed through a bounded, drop-oldest queue — `UIWorker`'s core technique, directly reusable.
- Two-layer crash isolation: the publish call site never raises, and the render thread's own failures never reach the mission coroutine.
- A structured, typed snapshot (`UISnapshot`) as the single object the renderer consumes — richer than `Dashboard.update()`'s six loose kwargs.

**What must change to fit `v32_flight_stack`, not be reintroduced as-is:**
- `UISnapshot` was built for `MissionManager`'s string-based state model (V30/V31). It must be re-derived from `v32_flight_stack`'s actual objects: `IFlightBackend` telemetry, `TargetValidator`/`TargetSelector` state, `PayloadInterlock`, `PositionStore`, and — if ADR-004 is adopted — `MissionSnapshot` (ADR-004 §6) rather than being a parallel, competing data model. **These two documents should converge on one snapshot type, not define two.**
- `event_bus` was a bare module-level global (`mission_types.py:150`) — acceptable in a single-process procedural script, a coupling smell in the interface-driven `v32_flight_stack`. It should become an injected dependency (constructor parameter, same pattern already used for `flight`/`camera`/`detector`/`actuator`), consistent with ADR-004 §14's "minimal-diff integration" principle and with how `Dashboard` is already injected today.
- The renderer (`DebugView`'s 9-panel layout) is reusable presentation logic almost as-is — it only consumes a snapshot object and draws; porting it means changing what populates the snapshot, not how the panels draw.
- `Gorev3Orchestrator` already accepts a `dashboard` param — the new dashboard object should be accepted the same way, so no new coupling is invented there; it is really `MasterMissionController` (§7) that needs to exist as a live entrypoint for that wiring to matter.

---

## 9. Event flow (target state)

```
RUN MISSION (operator runs run_mission_v32_*.sh)
  → main() builds flight/camera/actuator/detector via existing DI pattern
  → main() constructs the dashboard's publisher + render-thread pair
  → render thread .start()'d — UNCONDITIONALLY, before orchestrator.run()
      (matches V30/V31's "automatic, no operator action" behavior, and
       matches Dashboard's already-correct current construction point)
  → orchestrator (Gorev2Orchestrator, then Gorev3Orchestrator via
    MasterMissionController once §7 is resolved) runs; every tick:
      - build/derive a snapshot from live subsystem state
      - publish it — non-blocking, bounded queue, drop-oldest
  → render thread consumes independently, draws, shows
  → on MasterMissionController reaching MISSION_COMPLETE / FAILED / ABORTED:
      - a terminal snapshot is published (state banner reflects it)
      - render thread stays alive briefly in "Mission Complete" mode,
        then .stop() is called from the same finally: block pattern
        main.py already uses today
```

---

## 10. Launch sequence & runtime lifecycle

| Step | Owner | Blocking to mission? |
|---|---|---|
| Construct flight/camera/actuator/detector (existing DI) | `main_*.py` | n/a, already sequential |
| Construct dashboard renderer + publisher | `main_*.py` | No — construction only, no I/O |
| Start render thread | `main_*.py`, before `orchestrator.run()` | No — thread starts independently |
| Per-tick publish | `Gorev2Orchestrator.run()` / `Gorev3Orchestrator.run()` | **No** — must be a bounded, non-blocking `put()`, never a direct `cv2` call on this thread |
| Render + display | dedicated thread | No — isolated by construction |
| Mission terminal state | `MasterMissionController` (once wired, §7) | No — one more publish, then... |
| Shutdown | `main_*.py` `finally:` | Render thread `.stop()`, joined with a timeout, window destroyed |

---

## 11. Required changes (descriptive — no implementation here)

1. **Fix `upload_mission()`** (already scoped in this engagement, independent of this report) — required before any dashboard change can even be observed working.
2. **Wrap `Gorev2Orchestrator`'s and `Gorev3Orchestrator`'s dashboard-update call sites in isolation** — either restore thread+queue (preferred, matches proven V30/V31 design) or, as a stopgap, wrap the existing synchronous call in try/except. The stopgap alone does not fix the async-loop-blocking problem ADR-004 §1 already flagged; only the thread+queue restores full parity.
3. **Introduce a snapshot type** derived from `v32_flight_stack`'s real subsystems, converging with ADR-004's `MissionSnapshot` rather than reintroducing V30/V31's `UISnapshot` verbatim.
4. **Introduce an injected event/publish mechanism**, replacing the bare `event_bus` global with a constructor-injected dependency, consistent with existing DI in `main_gz.py`/`main_real.py`/`main_dual.py`.
5. **Port `DebugView`'s panel-drawing logic**, retargeted to read the new snapshot type instead of the old one.
6. **Resolve the `MasterMissionController` gap (§7)** — a prerequisite for "dashboard remains active through Mission Complete," but is a mission-sequencing decision, not a dashboard change; flagged here, not decided here.
7. **Delete or fix the flat `v32/` directory's `SyntaxError`** — not required for the dashboard fix, but currently a landmine for anyone who runs `v32/main.py` expecting it to be live; recommend explicit archival (e.g. rename to make clear it predates `v32_flight_stack` and is not maintained) so it stops being mistakable for the current system, which is exactly the confusion this investigation had to resolve first.

---

## 12. Migration plan

| Phase | Scope | Depends on |
|---|---|---|
| 0 | Apply the `upload_mission()` fix | — |
| 1 | Add try/except isolation around existing synchronous dashboard calls (stopgap safety net) | Phase 0, to be able to observe it |
| 2 | Introduce injected publisher + dedicated render thread + bounded queue, replacing the synchronous call sites | Phase 1 |
| 3 | Introduce the converged snapshot type; port `DebugView`'s panels to read from it | Phase 2 |
| 4 | Resolve `MasterMissionController` wiring so Mission Complete / Görev 3 transition is reachable and the dashboard survives it | Independent, can run parallel to 2-3 |
| 5 | Archive the flat `v32/` directory unambiguously | Independent, no code risk |

Each phase is independently testable and revertible; none requires the others to be complete to ship, aside from the stated dependencies.

---

## 13. Risk analysis

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Dashboard rendering failure aborts a live mission (§6) | Confirmed live, not yet observed only because §5 blocks the code path entirely | High — mid-flight mission loss | Phase 1/2, priority after Phase 0 |
| Reintroducing `UISnapshot`/`event_bus` verbatim creates two competing observability data models (this ADR + ADR-004) | Medium, if migrated without cross-referencing ADR-004 | Medium — duplicated, drifting schemas | Explicit convergence directive in §8 |
| Render thread contention with the asyncio event loop (GIL, frame-copy cost at 33 Hz-equivalent tick rates) | Low-medium, needs field measurement | Low-medium — dropped frames only, not mission-blocking, given bounded-queue design | Keep `maxsize=1` drop-oldest semantics from V30/V31, measure `dropped_snapshot_count` |
| `MasterMissionController` wiring (§7/§14 Phase 4) touches mission sequencing, not just the dashboard | Certain, by definition | Scope creep if bundled into "fix the dashboard" | Kept as an explicitly separate, flagged decision, not bundled into Phases 0-3 |
| Flat `v32/` directory is mistaken for live code again by a future contributor | Already happened once (this session's initial framing) | Low direct risk, high confusion cost | Phase 5 |

---

## 14. Compatibility analysis

- **Backward compatibility with V30/V31:** not a goal — no external consumer depends on the flat lineage's `UISnapshot`/`event_bus` shapes; they are not imported by anything outside their own directory.
- **Compatibility with `v32_flight_stack`'s existing DI pattern:** required and achievable — every subsystem already receives its dependencies through constructor injection in `main_gz.py`/`main_real.py`/`main_dual.py`; the dashboard publisher slots into the same pattern without altering it.
- **Compatibility with `Gorev3Orchestrator`'s existing `dashboard` parameter:** already compatible — no signature change needed there, only what gets passed in.
- **Compatibility with ADR-004:** requires the explicit convergence noted in §8/§13 — building this migration without referencing ADR-004's `MissionSnapshot`/EventBus design would produce two parallel, divergent observability systems in the same codebase.
- **Compatibility with the three run modes (`gz`, `real`, `dual`):** `DualFlightBackend`-style fan-out already exists for flight/camera/actuator; the dashboard publisher should follow the same single-instance-shared-across-both-backends pattern already used there, not be duplicated per backend.

---

## 15. Acceptance checklist

This document moves from "proposed" to "accepted" when every row below is either resolved or explicitly deferred with a named owner:

- [ ] §5 root cause fix applied and verified (dashboard opens on a real run)
- [ ] §6 crash isolation in place (a forced `cv2` failure no longer aborts the mission)
- [ ] §8 snapshot convergence decision made (reuse ADR-004's `MissionSnapshot`, or an explicit documented reason not to)
- [ ] §7 `MasterMissionController` wiring decision made (even if the decision is "not yet, Görev 3 stays out of scope this cycle")
- [ ] §11.7 flat `v32/` directory archived or fixed
