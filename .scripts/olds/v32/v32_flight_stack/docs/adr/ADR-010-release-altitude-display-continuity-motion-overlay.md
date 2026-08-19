# ADR-010 — Release-Altitude Guarantee, Display Continuity, Setpoint Smoothing, and the Contour Overlay

**Status:** Accepted
**Date:** 2026-08-17
**Amends:** ADR-008 B1 (vision lifetime — corrected scope), ADR-009 S1/S2 (setpoint stage, gains untouched)
**Baseline:** the V1‴ nominal run of 2026-08-17 14:49 (`mission_81cfefe66ad7`) — 301.0 s, both targets ≤2 attempts, 0 offboard-switch failures, landed 0.5 m from the checkpoint

Earlier phases of this ADR (R1–R4: retry-in-place, robust resume, launcher anchoring, signal handling) are recorded in the R-section at the end; P1–P5 below are the 2026-08-17 revision.

## Context

V1‴ was accepted: the mission works end to end. What it also did, for the first time, was make the *payload drop itself* measurable — and three things fell out of that measurement that no amount of tuning could fix.

### P1 — release altitude was decided by which shape it was

Per centering call, the altitude of the last committed detection before a sustained loss:

| target | commanded | last committed | first miss | seen/lost | released at |
|---|---|---|---|---|---|
| `MAVI_ALTIGEN` | 0.45 m | **1.63 m** | 1.47 m | 29 / 171 | **1.587 m** |
| `KIRMIZI_UCGEN` | 0.45 m | **0.47 m** | 0.48 m | 151 / 49 | **0.407 m** |

Both were commanded to 0.45 m. The hexagon stopped being detectable at ~1.6 m, the target-lost branch commanded zero velocity to *hold*, and the descent simply stopped — the servo fired 1.14 m high. The triangle happened to stay visible almost to the release altitude and landed close. Nothing in the flight log said any of this at the time; it only came out of post-run analysis.

The loss is **geometric, not a threshold**. `HSVContourDetector` approximates the hexagon contour with a *single* `eps` and requires exactly 6 convex vertices; once the shape clips the frame edge its contour is no longer a hexagon and no `eps` sweep recovers it. The triangle survives lower because it is smaller on screen and is tried over 6 `eps` values. Detector gates are explicitly out of scope, so the fix is not to make vision work lower — it is to **stop requiring vision** below the altitude where it demonstrably stops committing.

### P3 — the display and Görev 3's vision both died at a phase boundary

```
VISION_FRAME_PROCESSED: 2185 events, 9.35 Hz, worst gap 0.13 s, gaps >1.5 s: 0
last event: t = 236.3 s
GOREV2_COMPLETE -> GOREV3_START: t = 236.3 s
```

The feed was *perfectly* healthy for as long as it existed. Both loops lived in `Gorev2Orchestrator.run()` and were cancelled in its `finally`, giving them Görev 2's lifetime instead of the mission's. For the final 64.7 s of a 301 s flight there were no frames at all: the dashboard froze on its last image, and Görev 3's own 0.30 m descent centering got **0 committed detections in 200 attempts** (first miss already at 3.03 m) and could only time out. Compounding it, `Gorev3PickupPhase` held the real detector and called `detect()` itself — the second-caller problem ADR-008 B1 exists to prevent.

ADR-008 B1's invariant ("exactly one `detect()` caller, running unconditionally") was right. It was scoped to the wrong lifetime.

### P4 — the motion problem is a step change, not the gains

From 1647 `CENTERING_STEP` setpoints at 10 Hz:

| quantity | value |
|---|---|
| \|dv\| per 0.1 s tick | mean 0.035, p95 0.150 m/s |
| ticks with \|dv\| > 0.5 m/s | **9**, worst **2.50 m/s in one tick** |
| where those 9 occur | alt 9.99 / 5.24 / 5.00 m — the first tick of each new descent step |

Actual airframe response (ULog `vehicle_local_position`, Offboard windows only): horizontal accel mean 0.63, p95 5.08, p99 7.85, **max 9.65 m/s²**.

Steady-state control is already smooth. What is not smooth is the boundary: a fresh `go_to_and_center()` starts with a large error and jumps from the previous call's terminal zero straight to max speed in a single tick.

### P5 — the bounding box stopped being informative exactly where it mattered

Divergence between the two candidate centre definitions, by altitude:

| alt | n | mean sep | max | mean bbox width |
|---|---|---|---|---|
| 15.00 m | 426 | 3.51 px | 6.18 px | 103 px |
| 5.00 m | 259 | 11.26 px | 15.51 px | 168 px |
| **0.45 m** | 180 | **77.99 px** | **163.50 px** | **814 px** |

At the release altitude the shape spans 814 px of a 1280 px frame and the two "centres" disagree by up to 12× the tolerance band. The moment centre is the unstable one: a clipped blob's moments are computed over the *visible* part only, so the centre slides toward whatever is still in view while the true centre has not moved.

## Decision

### P1 — hybrid descent (`LOW_ALT_VISION_LIMIT_M = 2.0 m`)

Below the limit, losing the target is **expected** and no longer stops the descent:

1. Every tick with a committed detection freezes a **measured** target ground position — vehicle GPS plus the pixel offset back-projected through the camera intrinsics and rotated by yaw. Only committed samples; never an extrapolation.
2. When the detector stops committing *below* the limit, the descent continues on that frozen estimate to the release altitude. "Open loop" is true only of **vision**: position stays closed-loop against live GPS, so drift is corrected the whole way down. What stops improving is the target's estimated position, which stopped improving the moment vision stopped.
3. Above the limit nothing changes. A target lost at 10 m means something is actually wrong, and descending blind there would be unsafe — the vehicle still holds.
4. While the target *is* still committed below the limit, the controller steers on the **bounding-box centre** rather than the moment centre.

2.0 m sits just above the highest observed loss (1.63 m) with margin. `LOW_ALT_OPEN_LOOP_DESCENT` logs the frozen estimate, the altitude vision was lost at, and the altitude achieved; `PAYLOAD_RELEASE_ALTITUDE` reports the drop against the 0.45 ± 0.05 m band.

This is a change of *measurement and gating*. The P-law, `kp_horizontal` / `kp_vertical` / `kp_altitude`, both tolerances and the ADR-009 S1/S2 floors are untouched.

Because the target is generally **not** detectable at 0.45 m, the release offset is reported as `best_known_offset_cm` together with `best_known_offset_alt_m` — the last committed measurement and the altitude it was taken at. Reporting the number without that altitude would imply a precision it does not have.

### P2 — payload observability

`PAYLOAD_STATE` carries active payload index, target, current/target altitude, descent step, whether vision is committed, last known offset, and release altitude with its band verdict. The dashboard renders a PAYLOAD panel from it, green once released. The aggregator **merges** partial updates rather than replacing state, because a descent step carries no release altitude and a release carries no descent step.

`vision_committed = false` is drawn dim, not red: below the vision limit it is the expected condition that hands the descent to the open-loop path.

### P3 — mission-lifetime vision (`core/detection/vision_runtime.py`)

Both loops move to a `VisionRuntime` owned by the composition root, started before the master FSM and stopped after it returns. Görev 3 consumes the same `DetectionFeed` as Görev 2; where it must satisfy the `IDetector` interface (the visibility strategy), it receives a `FeedDetector` adapter that answers from the feed and **ignores the frame it is handed** — deliberately, because the frame a caller happens to hold is not the frame the streak logic was advanced on.

`Gorev2Orchestrator` never creates a pipeline. It accepts an optional `vision_runtime` and scopes it only if one is handed in — which the production root does not do, and tests exercising Görev 2 alone do. Either way there is exactly one `detect()` caller.

### P4 — setpoint-stage smoothing

Two limits applied to the **final setpoint only**, after the control law has decided what it wants:

- **Rate limit** `SETPOINT_MAX_DELTA_V_M_S = 0.30` per 0.1 s tick (3.0 m/s² commanded — well under the 5.08 m/s² the airframe already sustains at p95). The worst measured 2.50 m/s jump becomes a ~0.8 s ramp.
- **Distance-scaled cap** `v_max(d) = clamp(1.0·d, 0.20, 3.0)`, `d` = ground distance to target.

Both are **shape-preserving**: they never reverse a sign, never raise a command the law made small, and never push when the law says stop. An explicit zero is exempt from the rate limit — ramping a stop down would mean coasting past the target right after convergence. The DOWN axis is exempt from the distance cap, since `d` is horizontal and goes to zero once centred.

All setpoints leave the controller through one method, so no branch can bypass the limit, and `CENTERING_STEP` now reports what was **actually commanded** rather than what was requested.

### P5 — contour overlay

`Detection` carries an optional `contour_px`. `HSVContourDetector` supplies the `approxPolyDP` polygon its own vertex-count gate accepted, so a triangle renders as a green triangle and a hexagon as a green hexagon. Bounding rectangles are removed entirely; the class label and confidence sit just above the topmost contour vertex. Kept unchanged: the black lock line, the `d=` label, the tolerance ellipse, the crosshair.

### Detector adapter contract (for the incoming YOLO model)

A detector produces `Detection` values into `DetectionFeed` and nothing downstream knows which detector ran. The contract is exactly: `shape_type` (one of the four class names), `confidence` (0–1), `center_px` (the centre source the controller steers on — HSV supplies the contour-moment centre, and below `LOW_ALT_VISION_LIMIT_M` the controller substitutes the bbox centre itself), `bbox_px` (always present, `(x1, y1, x2, y2)`), `contour_px` (optional polygon; a YOLO adapter supplies its segmentation polygon if it has one and leaves it `None` if not, in which case the overlay strokes the bbox in the same green), and `rotation_deg` (rectangles only). A YOLO detector therefore drops in as a `VisionRuntime` constructor argument and can be A/B tested against HSV without touching mission logic, because mission code consumes the feed and never the detector. The one property an adapter must preserve is that `center_px` is stable under frame-edge clipping, since that is what the low-altitude centre-source switch exists to guarantee.

## Consequences

- Release altitude becomes a mission-controlled quantity instead of a per-shape accident, and an out-of-band drop is loud rather than invisible.
- The vehicle descends the last ~1.5 m on a frozen estimate. Lateral accuracy over that stretch is bounded by the drift accumulated since the last commit, not by vision — accepted deliberately: a drop from the commanded altitude at a slightly stale lateral position beats a drop from 1.59 m.
- Vision, dashboard and Görev 3 detection now share one producer for the whole flight; the single-owner invariant is unchanged, only its lifetime.
- Motion limits are additive to ADR-009: S1/S2 set the *floor*, P4 sets the *ceiling and slew rate*. Neither touches the gains.

## Validation

210 tests pass (21 new). The P1 regression test is discriminating: with the open-loop branch disabled it fails with exactly the V1‴ symptom — 200 attempts of "hedef kayboldu", timeout, altitude held.

### V1⁗ (2026-08-17 15:51, `mission_106256fe426a`)

**P3 — solved.** 2328 frames at 9.26 Hz, worst gap **0.21 s**, **0 gaps > 1.5 s**, first frame t=2.6 s, last frame t=254.1 s = the end of the mission. The V1‴ cliff at t=236.3 s is gone. Dashboard paint loop held 9.6–9.9 FPS across 48 samples, so the worst stall is ~0.10 s against the 0.5 s bar.

**P4 — smoothness solved, τ budget MISSED.** Separating accelerations from commanded stops (a stop is braking, not jerk toward the target, and is exempt by design):

| accelerating ticks | V1‴ | V1⁗ |
|---|---|---|
| p95 \|dv\| | 0.150 | 0.254 m/s |
| max \|dv\| | **2.500** | **0.367 m/s** |
| ticks > 0.5 m/s | **7** | **0** |

Actual (ULog, Offboard windows): horizontal accel p95 **5.08 → 4.14**, p99 7.85 → 7.14, max **9.65 → 8.53 m/s²**; speed p95 7.08 → 4.18 m/s.

But τ on the four pursuits common to both runs regressed **beyond the +20% budget**:

| pursuit | V1‴ | V1⁗ |
|---|---|---|
| MAVI 15 m | 5.7 | 11.8 s (+105%) |
| KIRMIZI 15 m | 7.8 | 9.4 s (+21%) |
| MAVI 10 m | 16.8 | 18.3 s (+9%) |
| MAVI 5 m | 6.2 | 13.4 s (+117%) |
| **mean** | **9.1** | **13.2 s (+44.9%)** |

An earlier V1⁗ attempt gave +10.4% on the same four, so run-to-run variance is large, but both runs are slower and the two-run mean is **+27.7%** — over budget either way.

Cause is the **distance-scaled cap**, not the rate limit. `v_max(d) = 1.0·d` compounds with proportional control in exactly the regime where the tail is already long: at d = 1 m the cap is 1.0 m/s and at d = 0.3 m it is 0.3 m/s, on top of a P-law that is itself shrinking the command. The rate limit alone is what removed the 2.50 m/s steps (0 accelerating ticks > 0.5) and produced the ULog improvement. Recommended remedy, **not applied**: raise `SETPOINT_SPEED_PER_DISTANCE` to ~2.0 /s or drop the distance cap entirely and keep only `SETPOINT_MAX_DELTA_V_M_S`.

Separately, commanded stops > 0.5 m/s rose 2 → 4. All four were single-frame target dropouts (`target_seen` True→False) hard-braking from ~1.3 m/s. The "explicit zero is immediate" exemption was designed for *convergence* stops; it also fires on *transient loss*. Worth revisiting, not changed here.

V1⁗ also *converged* six pursuits against V1‴'s four: `KIRMIZI_UCGEN` at 10 m and at **0.45 m** both converged where they previously timed out — the low-altitude bbox-centre switch working.

**P1 — solved.** The open-loop descent engaged exactly as designed. Vision-loss altitude is now measured three times and is stable: **1.63 / 1.64 / 1.67 m**, which is what the 2.0 m gate was sized against.

The first V1⁗ attempt exposed a defect in this ADR's own implementation: the exit condition was `ALTITUDE_CONVERGENCE_TOLERANCE_M` (0.30 m), sized for staged approach steps. It carried payload 1 from the V1‴ stall of 1.587 m down to 0.744 m and then stopped, because 0.744 − 0.45 = 0.294 m fell just inside 0.30 — released **out of band**. Corrected to exit on `PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M` (0.05 m) with a `LOW_ALT_OPEN_LOOP_MIN_DESCENT_M_S = 0.12` floor, since pure proportional control asymptotes against a band that tight.

Final result — **both payloads in band**:

| payload | released at | error | vision at release | offset |
|---|---|---|---|---|
| `MAVI_ALTIGEN` | **0.481 m** | 0.031 | false (open-loop from 1.67 m) | best-known 2.06 cm @ 4.98 m |
| `KIRMIZI_UCGEN` | **0.471 m** | 0.021 | **true** | **live 16.95 cm** |

against V1‴'s 1.587 m and 0.407 m. `KIRMIZI_UCGEN` is the first release in any run with the target still committed at release altitude, so its offset is a **live** measurement rather than a last-known one — the bbox-centre switch is what kept it committed that low. The <5 cm goal is **not met** on the live measurement (16.95 cm); the only sub-5 cm figure (2.06 cm) is a best-known value measured at 4.98 m and must not be read as release accuracy.

**Görev 3 — a false pass exposed.** In V1‴ Görev 3 reported all four phases complete while the feed published **zero** frames for its entire duration: `Gorev3PickupPhase` held the real detector and ran its own `detect()` calls, bypassing the dead feed. On the shared feed the true commit rate for `KIRMIZI_DIKDORTGEN` from the Görev 3 vantage is **2 frames in 186** (1.1%), and the acquisition window is `GOREV3_PICKUP_ALIGN_MAX_ATTEMPTS = 30` × 0.1 s = **3.0 s** — roughly a 1-in-4 chance of catching a commit. V1‴ passed by hammering the detector at a higher rate than the shared loop provides, which also corrupted the streak state it was sharing.

This is a real outcome regression (Görev 3 no longer completes) produced by removing a brute-force path Görev 3 was silently depending on. The single-owner design is not the defect; the 3.0 s window against a 1.1% commit rate is. Left unchanged pending direction, since widening it is a Görev 3 timing change outside P1–P5.

---

## Phase 12 (2026-08-17) — P4 finalized, Görev 3 diagnosed, release offset explained

### Q1 — the distance cap is removed

Phase 11 shipped two limits; only one was earning its place. The rate limit alone accounted for the entire smoothness gain (accelerating ticks > 0.5 m/s: 7 → 0, max |dv| 2.50 → 0.367 m/s, actual horizontal accel p95 5.08 → 4.14 m/s²), while the distance cap `v_max(d) = clamp(1.0·d, 0.20, 3.0)` cost τ 9.1 → 13.2 s (+44.9%) against a +20% budget by compounding with proportional control in exactly the regime where the convergence tail is longest. Removed; `SETPOINT_MAX_DELTA_V_M_S` stays.

Transient-loss braking is also fixed. All four >0.5 m/s commanded stops in V1⁗ were **single-frame** dropouts hard-braking from ~1.3 m/s, after which the vehicle re-accelerated from standstill when the target reappeared on the next frame. Target loss now decelerates under the same rate limit for `TARGET_LOSS_GRACE_FRAMES = 5` before zeroing; a **convergence** stop keeps the immediate-zero exemption, because coasting past a target just declared centred is the post-lock drift we measure.

**Verified — V1⁵ (2026-08-17 16:32, `mission_20078152f6f9`), 240.7 s.**

| metric | V1‴ baseline | V1⁗ (with cap) | **V1⁵ (Q1)** | budget |
|---|---|---|---|---|
| τ, 4 common pursuits | 9.1 s | 13.2 s (+44.9%) | **9.7 s (+6.2%)** | ≤ +20% |
| accelerating max \|dv\| | 2.500 | 0.367 | **0.340 m/s** | — |
| accelerating ticks > 0.5 m/s | 7 | 0 | **0** | — |
| commanded stops > 0.5 m/s | 2 | 4 | **0** | — |
| actual horiz accel p95 | 5.08 | 4.14 | **4.50 m/s²** | — |
| actual horiz accel max | 9.65 | 8.53 | **8.73 m/s²** | — |

τ comes in at **+6.2%**, comfortably inside budget, and V1⁵ converged **seven** pursuits against V1‴'s four. Commanded smoothness is equal or better than Phase 11 (max |dv| 0.340 vs 0.367; zero >0.5 ticks in both the accelerating and stopping categories, where V1⁗ had four hard brakes). Actual accel p95 sits at 4.50 vs Phase 11's 4.14 — ~9% higher, the honest cost of removing the cap — but still well below the 5.08 baseline. That is the intended trade: 0.36 m/s² of p95 accel bought back 38 percentage points of convergence time.

`lost_frames` telemetry confirms the grace mechanism: 7 loss frames across the run, max 6 consecutive — i.e. at least one genuine loss correctly ran past the 5-frame grace into a hard stop, while the transient ones decelerated.

**Q1 ACCEPTED (operator, 2026-08-17).** τ +6.2% against a +20% budget, 0 accelerating ticks > 0.5 m/s, 0 hard-brake stops. The distance-scaled cap is removed for good and `SETPOINT_MAX_DELTA_V_M_S` + `TARGET_LOSS_GRACE_FRAMES` are the shipped configuration.

### Q2 — Görev 3's target does not exist in the simulated world

`Tools/simulation/gz/worlds/default.sdf` includes exactly two ground models:

```
<uri>model://blue_hexagon</uri>
<uri>model://red_triangle</uri>
```

There is **no red rectangle and no blue rectangle anywhere in the world**. `KIRMIZI_DIKDORTGEN` — the target Görev 3's pickup phase waits for, and the marker Görev 2's `_verify_marker` looks for — cannot legitimately be detected at any altitude, from any vantage, for any gate setting.

So the Phase 11 framing ("which HSV gate fails at the pickup vantage") had no valid answer: the 160 `KIRMIZI_DIKDORTGEN` and 103 `MAVI_DIKDORTGEN` detections logged across the run are **false positives** on rendered-scene artifacts, not weak detections of a real target. Clean synthetic triangles and hexagons at four sizes, clipped and unclipped, produce **zero** spurious rectangles, so the source is scene-specific (lighting, shadow, the released payload cylinders — `payload_cyl_red` / `payload_cyl_blue` exist as models — or ground texture) rather than the shape geometry itself.

The gates are recorded for completeness, all in `core/detection/hsv_contour_detector.py::_detect_rectangle`:

| gate | line | value |
|---|---|---|
| min contour area | :114 | `HSV_MIN_AREA_RECT_BASE = 400` × area_scale |
| vertex count + convexity | :119 | exactly 4, convex, over `eps ∈ linspace(0.02, 0.06, 5)` |
| min side length | :121 | 8 px |
| colour fill fraction | :132 | `HSV_COLOR_FRAC_RECT = 0.40` |
| streak commit | :150 | `HSV_STREAK_FRAMES = 3` within `HSV_STREAK_DIST_PX = 60` |

`GOREV3_PICKUP_ALIGN_MAX_ATTEMPTS` was raised 30 → 80 (3.0 s → 8.0 s) as instructed. It did not help — V1⁵ failed Görev 3 Faz 1 identically, which is the predicted outcome and further evidence for the diagnosis. A longer window only buys more chances to latch a false positive, which is worse than failing, because Görev 3 would then align to a phantom.

**This is a world/vantage matter, not a detector one.** The smallest correct fix is to add a red rectangle (and a blue rectangle for the Görev 2 verification markers) to `default.sdf` at the Görev 3 pickup location, sized like the existing 1 m shapes. Until then Görev 3 cannot be validated in simulation and its Phase-11-and-earlier "successes" should be treated as unverified.

### Q3 — the release offset is a fixed geometric constant, not drift

The 16.95 cm at the `KIRMIZI_UCGEN` release decomposes cleanly:

| contributor | value |
|---|---|
| last centering lock error | **0.13 cm** (converged) |
| drift during low-altitude descent | ±8 cm oscillation, converged at release |
| lateral motion during the final 0.12 m/s descent | contained in the above |
| **bbox-vs-moment centre choice** | **13.1 cm** |

`center_sep_cm` is **constant across every altitude** — 13.69 cm at 15 m, 13.07 cm at 0.5 m — while the same quantity grows 4.9 → 133.9 px. Constant in centimetres is the signature of a fixed *physical* offset, not a clipping instability. The red triangle is a 1 m equilateral (`triangle_red_1m_2cm.stl`): height 0.866 m, and the centroid-to-bbox-centre distance is h/6 = **14.4 cm predicted against 13.1 cm measured**.

This **corrects the Phase 11 P1 rationale.** Phase 11 attributed the growing pixel divergence to the moment centre destabilising as the shape clips the frame, and switched to the bbox centre below `LOW_ALT_VISION_LIMIT_M` on that basis. The centimetre figure was in the Phase 11 table and was even remarked on, but the wrong conclusion was drawn from it: the moment centre was never unstable. The divergence is the triangle's centroid/bbox geometry, and it is present at 15 m just as much as at 0.45 m.

The consequence is that the bbox switch introduced a **systematic ~13 cm error on the triangle** — the vehicle centres correctly on the bounding-box centre while the payload should land on the centroid. The hexagon is centrosymmetric, so centroid ≡ bbox centre and it is unaffected: its best-known offset was 2.06 cm.

**Proposed (not applied): drop `LOW_ALT_BBOX_CENTER` and steer on the moment centre at all altitudes.** Predicted effect: removes ~13 cm of the 16.95 cm, leaving **~3–4 cm — inside the < 5 cm goal**. Risk to validate: `KIRMIZI_UCGEN` converged at 0.45 m in V1⁗ where it previously timed out, and it is not established whether that came from the bbox switch or from the other Phase 11 changes; the alternative is to keep the bbox centre for control and apply the polygon-centroid correction (available from `contour_px`) at the release measurement only.

### OFFBOARD_SWITCH_FAILED

Both V1⁗ runs show the same shape: `MISSION_AUTHORITY_RELEASED (target_pause)` → Offboard requested immediately → 3.0 s timeout → retry 0.4–1.0 s later succeeds.

```
t= 24.7 MISSION_AUTHORITY_RELEASED {"reason": "target_pause"}
t= 27.8 OFFBOARD_SWITCH_FAILED     {"timeout_s": 3.0}
t= 28.2 MISSION_STARTED_ONBOARD
t= 31.5 OFFBOARD_SWITCH_CONFIRMED
```

It is a race: PX4 has not finished leaving MISSION when Offboard is requested. `OFFBOARD_AFTER_RESUME_SETTLE_S = 2.0` already exists for the Offboard→Mission→Offboard *resume* path but is not applied to this pause→Offboard path. **Yes, the same settle should apply here** — it costs ~2 s per target engagement (~4 s per mission) against a 3.0 s timeout plus a full retry cycle when it misses. Not applied; it is a mission-timing change outside Q1–Q3.

---

## R-section (2026-08-17, earlier phase)

**R1 — retry in place.** A failed pursuit used to fly away during its cooldown and lose the target entirely (68.3 s fly-away, zero detections on the retry). The vehicle now holds position for the cooldown. V1‴: both targets converged on attempt 2 with the target still visible (`d = 0.92 m` / `1.17 m`); MAVI converged at 55/150 = 5.5 s against a 5.3 s prediction from the τ extrapolation.

**R2 — robust resume.** `set_current_mission_item` + `start_mission`, spaced by `MISSION_RESUME_MIN_INTERVAL_S = 6.0`, with mode confirmation. V1‴: resumes dropped 9 → 1, `MISSION_RESUME_NOT_CONFIRMED` = 0, `OFFBOARD_SWITCH_FAILED` 4 → **0**. Only one resume occurred, so no spacing was measurable and 6.0 stands.

**R3 — launcher.** `pkill` patterns anchored (`bin/px4$`, `gz sim`) after unanchored patterns killed the caller's own helper shells; added a `clear_land_mode.py` step, since PX4 parks in `LAND` after a mission and refuses to arm while every individual pre-arm check passes.

**R4 — signal handling.** A process started with `&` from a non-interactive shell inherits `SIGINT = SIG_IGN`, so Python never installs `default_int_handler` and `KeyboardInterrupt` can never be raised — the abort path was unreachable on any background-launched mission. Own SIGINT/SIGTERM handlers are installed explicitly. Confirmed live (V3, 2026-08-17 15:12): handler entered **6 ms** after the signal, returned from 13.9 m to 0.6 m from the checkpoint in 5.5 s, landed, process exit at **+6.76 s**, 0 residual processes, PX4 survived.
