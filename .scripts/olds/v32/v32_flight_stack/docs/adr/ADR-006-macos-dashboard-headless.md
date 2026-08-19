# ADR-006 — Mission Operations Center painting on macOS

**Status:** **Implemented** (2026-08-16). Supersedes the interim headless
decision below; constrains ADR-005 §3/§8 on macOS only.

**Implemented design:** §4's "future fix" was built instead of the headless
stopgap. `main_gz.py` runs the mission coroutine on a `MissionRuntime` worker
thread (`_run_with_main_thread_gui()`, `_run()` itself unmodified); the main
thread drains composed frames from `core/telemetry/paint_bridge.py` and calls
`imshow`/`waitKey` at ~30 Hz. `MissionOpsDashboard` still owns state,
composition and lifecycle on its own thread and merely `publish()`es the
finished image. Linux/Windows are unchanged (`asyncio.run(_run(...))`, dashboard
paints on its own thread).

**Validated 2026-08-16 on macOS 26.5.2 / Apple M4:** dashboard window opens on
the main thread (confirmed via `sample`: `cv2.waitKey` → Cocoa/AppKit on the
Main thread stack), live frames with detections (`MAVI_ALTIGEN`), zero headless
lines, zero cv2 exceptions, paint loop **9.7–9.8 FPS** (equal to the
dashboard's `refresh_hz=10.0` — the loop services Cocoa at 30 Hz but paints
only newly composed frames), and mission cadences unchanged with painting
active: `detect()` ~3.07 s, MISSION_START poll ~5.0 s.
**Date:** 2026-08-16
**Context:** Ubuntu → macOS Apple Silicon port.

## 1. The constraint triangle

Three requirements that are individually reasonable and jointly unsatisfiable
on macOS as `main_gz.py` is currently structured:

1. **Cocoa** requires every `cv2` GUI call (`namedWindow`/`imshow`/`waitKey`)
   to run on the process's **main thread**. Off-thread calls raise
   `error: Unknown C++ exception from OpenCV code` — reproduced directly:
   main thread OK, worker thread raises.
2. **ADR-005 §3** requires all `cv2` work to happen on the dashboard's
   **dedicated thread**, and its §8 table forbids *"a direct `cv2` call on
   this thread"* for the mission loop — that isolation is precisely what
   ADR-005 was written to restore.
3. **`main_gz.py:143`** runs the mission via `asyncio.run(_run(...))`, and
   `_run()` does `await master.run()`. **The mission coroutine therefore runs
   on the main thread.**

Given (3), "main thread" and "mission thread" are the same thread, so
satisfying (1) necessarily violates (2).

## 2. Decision

On `sys.platform == "darwin"`, `MissionOpsDashboard` skips cv2 GUI setup
entirely and runs headless **deliberately**, logging one WARNING at startup:

```
[DASHBOARD] headless on macOS: Cocoa requires GUI on main thread, ADR-005
forbids cv2 on the mission thread; use ./run_just_cam for live video
```

Everything else in the dashboard is unchanged: state, snapshot composition,
`FrameChannel` queue and lifecycle all still run, and frames still flow to the
detection loop. Only the paint call is skipped. Live video on macOS is served
by `./run_just_cam` → `gz_system/camera_viewer.py`, which drives the canonical
`GzCameraSource` (`camera_service_manager` + `CameraClient`) in a standalone
process that owns its own main thread — no mission competing for it.

The non-darwin path is unchanged, except that the render-failure fallback now
logs at **ERROR** with the exception text and thread name instead of WARNING;
an unexpected display failure should never again be indistinguishable from a
working panel.

## 3. Rejected alternative (for the record)

Marshalling `imshow`/`waitKey` into an asyncio task on `main_gz.py`'s loop
would satisfy Cocoa, but puts `cv2` on the mission thread — the exact thing
ADR-005 §8 forbids and calls the regression that "got lost". Rejected.

## 4. Future fix (NOT implemented here)

**Run the mission coroutine on a worker thread and reserve the main thread for
the GUI.** That satisfies Cocoa and ADR-005 simultaneously: the mission would
no longer own the main thread, so GUI calls there would not be "on the mission
thread". It requires restructuring `main_gz.py`'s entrypoint (and the
`main_real.py`/`main_dual.py` equivalents) and re-validating shutdown/signal
handling, so it is deliberately out of scope for the platform port and left as
the recorded next step.
