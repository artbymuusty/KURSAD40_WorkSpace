# ADR-011 — Simulated Payload Physics: World-Loaded Bodies, DetachableJoints, and a Release That Is Confirmed

**Status:** Accepted
**Date:** 2026-08-17
**Supersedes:** the `PayloadDropSystem` spawn-on-release mechanism
**Scope:** `Tools/simulation/gz/worlds/default.sdf`, `Tools/simulation/gz/models/x500_mono_cam_down/model.sdf`, `gz_system/gz_payload_actuator.py`, `gz_system/gz_pose_monitor.py`, `core/mission/payload_release.py`, `core/config/parameters.py`, `real_system/config/real_system.yaml`. The mission-side release API, `PayloadInterlock` and the drop sequencing are unchanged.

## Context

Every payload drop this project has ever run in simulation was a lie, in three successive ways. Each one was invisible until the previous one was fixed.

### 1. The payload fell through the world

`PayloadDropSystem` released a payload by **spawning a brand-new model** through the world `create` service. Measured on 2026-08-17: a body spawned that way gets no reliable collision pairs in this gz-sim 8.15 build. A real mission drop was logged at **z = −0.72 and still falling** three seconds later, and a payload-equivalent probe fell through bare ground at every offset tested, while an identical probe *loaded with the world* rested on the ground perfectly.

So there was never a payload sitting on a target. There was never anything to photograph, and the "verification marker" step was inspecting a world in which the payload had left through the floor.

### 2. Release was fire-and-forget

Fixing (1) exposed (2). The actuator published a detach message and immediately reported success; `PAYLOAD_RELEASED`, the QGC status text and the log banner all fired off nothing more than "a message was sent". On the first nominal flight after the DetachableJoint change, payload 2 was measured **still attached 2 s after the servo** and came to rest **4.9 m past the triangle**, and every operator channel reported a clean release.

### 3. The payload was mounted through the landing gear

And fixing (2) exposed (3), which turned out to be the real cause of the 4.9 m miss. The mount at y = ±0.20 m put the payload's volume straight through the x500's landing skid:

| body | z range | \|y\| range |
|---|---|---|
| payload | 0.000 – 0.050 | 0.0875 – 0.3125 |
| landing skid | 0.0136 – 0.0286 | 0.1245 – 0.1395 |

The skid is **entirely inside the payload**. Nothing pushed them apart while the joint held, because DART suppresses collision between joint-connected bodies — but the instant the joint was removed, the solver saw a deeply interpenetrating pair and **ejected** the payload. That is the 4.9 m flight, and it is also why payload 1 came to rest at z = 0.156 (a 0.30 m slab standing on its long edge: 0.006 surface + 0.150 half-side) instead of lying flat at 0.031.

The transport was a contributing, secondary factor: a fresh `gz topic -p` process was measured at **0.91 s** from launch to delivery (n=5, σ < 0.005 s), every time, because it advertises and publishes in the same breath and gz-transport is a slow joiner.

## Decision

**Payloads exist from world load and are held to the vehicle by DetachableJoints.** Two `payload_red` / `payload_blue` models are declared in `default.sdf`; two `gz-sim-detachable-joint-system` plugins in the vehicle model bind them to `base_link` and free them on `/payload/detach/{red,blue}`. This keeps genuine drop physics — the payload is an ordinary dynamic body that falls, bounces and settles — while guaranteeing collision pairs, because both bodies exist before the simulation starts rather than being created mid-run.

**Mount points clear the airframe.** y = ±0.28 m (clearance needs ≥ 0.1395 + 0.1125 = 0.252), z = 0.025 m so the body starts exactly at rest instead of dropping 3.5 cm while the joint forms. Sides follow the existing, unchanged servo→colour mapping in `gz_payload_actuator.py`: `release_payload_at_mavi_altigen() → "red"` (RIGHT, −y), `release_payload_at_kirmizi_ucgen() → "blue"` (LEFT, +y).

**A release is not believed until the body is seen to leave.** `GzPoseMonitor` holds one long-lived subscription to `dynamic_pose/info`, started before takeoff so the ~2 s discovery cost is paid during setup. After publishing, the actuator polls vehicle-relative drop at 20 Hz for 0.5 s; on failure it republishes once and polls again. An unconfirmed release makes `release_and_verify` **hold at release altitude instead of climbing**, at CRITICAL, until separation is confirmed or `PAYLOAD_DETACH_HOLD_MAX_S` expires. The detach itself is published five times over 0.25 s, which costs nothing and closes the slow-joiner window.

**A landing is scored against the shape it was aimed at.** `settled_above_ground` (z > −0.05) only ever answered "did it fall through the world"; it passed the 4.9 m miss. `settled_on_target` requires |z − 0.031| < 0.03 **and** horizontal offset < 0.5 m from the target centre, and the payload's tilt is reported so an edge landing is an observation rather than an inference.

**Colours are true red and true blue** (operator requirement). Separation from the arena targets is by **size**, not hue: at 15 m a 0.30 × 0.225 m prism projects to 10.8 × 8.1 px = 87 px², against `HSV_MIN_AREA_TRI_BASE` 390 and `HSV_MIN_AREA_RECT_BASE` 400 — a 4.5× margin, reached only below ~7 m. The cross-colour mapping helps independently: the red payload is dropped on the blue hexagon and the blue payload on the red triangle, so a released payload never lies on a same-colour target.

**Mass is 0.15 kg per payload in simulation; the real payload is 1.05 kg.** The sim x500 is a 2.0 kg airframe (2.114 kg all-up), not the real KURSAD40 aircraft. Two real payloads would add 2.1 kg — a 99% increase — and push thrust-to-weight and `MPC_THR_HOVER` far outside the tuning this SITL model ships with, so every flight-dynamics observation would describe an aircraft nobody is building. The real figure is recorded in `real_system/config/real_system.yaml` with this reasoning, so it is carried forward rather than lost.

## Consequences

### Accepted cost: a side-mounted payload lands where its mount is

This is the honest headline. The mission centres the **camera** on the target, and the payload hangs 0.28 m to the side, so a perfectly centred release still puts the body 0.28 m from the target centre (it was 0.20 m before, and that error was always there — it was simply invisible while the payload was falling through the floor). It is now the dominant term in the release offset, larger than everything Phase 12 Q3 was measuring.

There is no geometry that removes it: two payloads cannot both be under the centreline. Mounting them fore/aft at y = 0 would reduce it to 0.16 m but abandons the left/right 180° servo the real aircraft uses. **Proposed, not applied:** offset the aim point by the mount vector before descending, so the *payload* is centred on the target rather than the camera. That is a mission-logic change and is out of this ADR's scope.

### Verified

**Drop table** — payload teleported to the target centre at each height and released; rest pose read from `dynamic_pose/info`. Expected rest z = 0.031.

| payload | target | 0.30 m | 0.45 m | 0.80 m | 1.20 m |
|---|---|---|---|---|---|
| `payload_red` | hexagon (0, 15) | 0.026 | 0.030 | 0.027 | 0.028 |
| `payload_blue` | triangle (0, 40) | 0.026 | 0.030 | 0.027 | 0.028 |

All 8: flat (tilt 0.0°), horizontal offset 0.0 cm, none below ground, no tunnelling at 1.2 m, and neither target ever appeared in `dynamic_pose/info` — i.e. the shapes never moved.

**Nominal flights** (two, 2026-08-17 20:13 and 20:20):

| run | payload | release alt | detach latency | rest (x, y, z) | offset | tilt | on target |
|---|---|---|---|---|---|---|---|
| 1 | 1 red → hexagon | 0.475 m ✓ | (not measured) | 0.243, 14.748, 0.029 | 35.0 cm | 0.1° | yes |
| 1 | 2 blue → triangle | 0.485 m ✓ | (not measured) | −0.270, 39.742, 0.026 | 37.3 cm | 0.0° | yes |
| 2 | 1 red → hexagon | 0.484 m ✓ | 1.142 s | 0.240, 14.736, 0.027 | 35.7 cm | 0.0° | yes |
| 2 | 2 blue → triangle | **0.564 m ✗** | 1.169 s | −0.337, 39.999, 0.023 | 33.7 cm | 0.0° | yes |

P3 continuity held in both (2553 / 2401 frames, 9.35 / 9.33 Hz, worst gap 0.13 / 0.14 s, **0 gaps > 1.5 s**, covering 2.6→275.7 s and 2.5→259.8 s of 275.8 s and 259.9 s missions). 0 CRITICAL/ERROR events, 0 `OFFBOARD_SWITCH_FAILED`, returns 0.3 m and 0.5 m, PX4 survived both.

**Mass effect**, all-up 2.114 → 2.414 kg (+14.2%):

| | before | after |
|---|---|---|
| hover throttle (measured, 15 m steady) | 0.7402 ± 0.0140 | 0.8020 ± 0.0236 |
| thrust-to-weight (4 × 8.549 N) | 1.650 | 1.445 |
| takeoff to 15 m | 11.8 s | 11.5 s |

The throttle ratio 1.084 matches √(mass ratio) = 1.069 to within 1.4%. Takeoff time is unchanged because the climb is rate-limited, not thrust-limited.

**Detector**, commits per frame while at ≥ 14 m:

| run | payloads | MAVI_ALTIGEN | KIRMIZI_UCGEN | MAVI_DIKDORTGEN | KIRMIZI_DIKDORTGEN |
|---|---|---|---|---|---|
| ×3 baseline | amber/magenta | 0.451–0.492 | 0.321–0.329 | 0.046–0.077 | 0.032–0.069 |
| ×2 | **true red/blue** | 0.403, 0.410 | 0.287, 0.290 | 0.083, 0.074 | 0.075, 0.056 |

Rectangle rates stay inside the pre-change band, so the true-coloured prisms add **no false `KIRMIZI_DIKDORTGEN` / `MAVI_DIKDORTGEN` commits** — those detections pre-date the colour change and come from the targets' own contours occasionally fitting four convex vertices. Target commit rates are down 10–13%, consistently across both runs and outside the baseline spread; see below.

### A1 — the release band applies on every path (accepted, verified)

`go_to_and_center` takes a per-call `alt_tolerance_m`. Staged approach steps keep `ALTITUDE_CONVERGENCE_TOLERANCE_M` = 0.30 m; the **final** step is handed `PAYLOAD_RELEASE_ALTITUDE_TOLERANCE_M` = 0.05 m, so the guarantee no longer depends on whether vision happened to survive the descent.

Four releases across the two flights that ran the corrected build: **0.48, 0.45, 0.471, 0.474 m — all in band**, on both the vision-guided and open-loop paths.

### A2 — payload-centred aim (implemented, goal NOT met)

The aim point becomes the payload's position in frame rather than the frame centre. No heading rotation is needed for the vision path — the camera is bolted to the airframe, so image axes are body axes — and because the residual is what `_freeze_target_estimate` back-projects, the body→NED rotation carrying the mount vector into the ground frame happens there exactly once, and the open-loop descent inherits it.

The mount vector is now **measured at the servo instant** (`MOUNT_VECTOR_MEASURED`) instead of reasoned about, because reasoning about it failed twice:

| flight | applied right | measured right | red offset | blue offset |
|---|---|---|---|---|
| baseline (no aim) | — | — | 35.0, 35.7 cm | 37.3, 33.7 cm |
| first A2 | −0.28 (SDF sign) | — | **88.9 cm** | **73.9 cm** |
| corrected | +0.28 / −0.28 | — | 37.2 cm | 30.9 cm |
| corrected + measured | +0.28 / −0.28 | **+0.2799 / −0.2801** | 40.1 cm | 32.5 cm |

The SDF's y is in Gazebo's **FLU** frame (Y left) while the correction is consumed in PX4's **FRD** frame (Y right), so taking it across verbatim inverted the sign and doubled the miss — 0.243 m became 0.850 m, almost exactly the predicted 0.243 + 2 × 0.28.

With the sign right and the vector confirmed to 0.2 mm, the offset **did not improve**: 40.1 / 32.5 cm against a 33.7–37.3 cm baseline. The reason is measurable, not mysterious. On the flight above the open-loop hold converged to 1.7 cm of its frozen estimate, and that estimate sat at east **−0.640 m** when the aim point is east −0.28 m. Since `estimate = perceived_target − mount`, the detector was perceiving the hexagon centre 0.36 m off-centre. That is the low-altitude clipping bias ADR-010 P1 already documented — and **A2 aggravates the very measurement it depends on**, because deliberately holding the target 0.28 m off-centre pushes a shape that already fills the frame further into the edge.

**Proposed, not applied:** apply the mount offset as a pure translation of the hold point *after* vision-guided centring completes, rather than as a bias on the vision error. That keeps the target centred while it is being measured — the regime with the least clipping bias — and adds the offset open-loop at the end, decoupling the correction from the measurement.

### Open, measured, not fixed here

- **Target commit rate is down 10–13% at 15 m.** Consistent across both runs and outside the three-run baseline spread, so it is a real effect, not noise. Most likely mechanism: at 0.28 m outboard the nearest payload corner sits just inside the camera frustum (39.7° vs a 41.6° vertical half-FOV), and a second same-colour blob at the frame edge can break `HSV_STREAK_FRAMES` tracking. Neither mission was affected — both targets were found, engaged and dropped on. Reported, not tuned.
- **Detach latency is 1.14 / 1.17 s against a 200 ms goal**, essentially all of it the `gz topic -p` process+discovery cost measured at 0.91 s. An in-process publisher would fix it; see the binding problem below.
- **The release offset is still ~35 cm, and A2 did not move it.** See the A2 section above: the geometric correction is now exact, but the low-altitude perception bias it induces cancels it. The proposed decoupling is the next step.
- **Search-phase flakiness.** One flight (2026-08-17 20:58) failed outright: the hexagon kept dropping out of frame during 15 m centering, both targets were abandoned, and the mission returned without dropping. That step uses neither A1 nor A2. It is consistent with the 8-13% lower target commit rate the true-coloured payloads brought, but one failure in six runs is not enough to attribute it. Worth watching.

### Known limits

- `GzPoseMonitor` reads `dynamic_pose/info`, which only carries entities that **moved this step**. A settled body stops being published, so `get()` returns its last observed pose. That is the correct answer for "where did it come to rest", and `age_s` is exposed rather than hidden.
- Detach latency is bounded by the `gz topic` CLI's ~0.9 s process+discovery cost. The burst and the confirmation poll make it observable and bounded, not instant. Getting under the 200 ms goal needs an in-process gz-transport publisher; the Python bindings exist only for the Homebrew 3.14 interpreter (no protobuf) and not for the 3.12 mission venv, so that is deferred.
- `PAYLOAD_DETACH_HOLD_MAX_S` bounds the hold at 60 s. The operator spec said "until confirmed or operator abort"; an unbounded hover trades one silent failure for another, so the bound exists and expiring it is recorded at CRITICAL rather than passed over.


---

## PHASE 13 (2026-08-17) — misclassification at 15 m, and the A2 redesign

### D1 — the payload hypothesis is refuted

No frames were retained from the 20:59 observation, so 1741 fresh ones were
captured by an independent subscriber running the mission's own detector
alongside a flight, perturbing nothing.

| question | measurement |
|---|---|
| payload pixels in frame | **0** — no frame ever held more than one red or one blue blob |
| misclassified frames | 122 (`KIRMIZI_DIKDORTGEN`) |
| of those, frames also emitting `KIRMIZI_UCGEN` | **122 / 122** |
| red blob area, UCGEN vs DIKDORTGEN sets | median **512 px²** in both |
| vertices at eps 0.03 | **3** in all 122 |
| contours touching the frame edge | **0 / 122** |

So it is not the payload, not a 3→4 flip, and not clipping. **One contour was
committing as two classes.** The triangle approximates to 3 vertices over the
triangle gate's 0.03–0.09 eps sweep and to 4 over the rectangle gate's tighter
0.02–0.06, where a rounded corner splits in two; both gates then pass their
colour and streak checks honestly. Görev 3's pickup target *is* a rectangle,
so the feed was offering it the arena triangle.

This is pre-existing, not a consequence of the colour change: the amber-era
runs show `DIKDORTGEN` commit rates in the same 0.032–0.077 band.

### D2 — fixed at (c) only

(a) and (b) were unnecessary and were **not** applied: with 0 payload pixels
there is nothing to move out of frame and nothing to mask. (c) is implemented
as one-contour-one-class — a rectangle whose centre falls inside an already
committed triangle or hexagon is that same object and is dropped.

| | dual-class frames (red) | dual-class frames (blue) |
|---|---|---|
| before | 122 | 4 |
| after (two flights) | **0** | **0** |

### D3 — mechanism works, sequencing does not (NOT accepted)

The mount offset is now applied after vision-guided centring, as a pure
translation of the hold point, and the vision error is no longer biased.

On the mount axis it works: payload 1 came to rest at world **x = −0.001 m**
against a target at x = 0, where every previous flight sat at ±0.24–0.36 m.

But `descend_to_release` reuses `_open_loop_descend`, which descends *while*
settling laterally and is bounded by `LOW_ALT_OPEN_LOOP_TIMEOUT_S` = 20 s.
The 0.28 m translation is asymptotic with no lateral command floor, so it
eats the budget while the vehicle keeps sinking, and the loop exits on
timeout at whatever altitude it has reached: **0.385 m and 0.159 m**, both
out of band, breaking A1's guarantee. At 0.159 m the payload could not fall
far enough to trip the separation threshold either, so the release went
`PAYLOAD_DETACH_UNCONFIRMED` and held its full 60 s before giving up (the
body did land on target, at z = 0.031).

**Required before this can be accepted:** run the translation as its own
altitude-holding phase with its own budget and a lateral command floor,
*then* descend on the translated hold under the release band.

A residual north-axis error of ~0.30 m is now visible on payload 1 (y =
14.695 against 15.0) that the much larger east error previously masked. It is
not the mount vector, whose forward component measures 0.001 m.


---

## PHASE 14 (2026-08-17) — MOUNT_TRANSLATE as its own phase, camera lever arm, route-finished-early

### T1 — accepted

The mount translation is now a distinct altitude-holding phase with its own
8 s budget and the `CENTERING_MIN_CMD_SPEED_M_S` floor, run before the
descent; the descent then exits on the release band alone and can no longer
be starved. Detach confirmation also accepts a payload already at its rest
height, so a low release cannot hang unconfirmed.

| | payload 1 | payload 2 |
|---|---|---|
| MOUNT_TRANSLATE residual / time | **4.6 cm / 2.65 s** | **4.3 cm / 2.69 s** |
| release altitude | **0.446 m ✓** | **0.444 m ✓** |
| detach latency | 1.829 s | 1.784 s |
| final offset | 22.0 cm | 32.2 cm |

A1 is restored. The 10 cm offset goal is **not** met.

### T2 — applied, and it over-corrects

The camera sits 0.35 m forward of base_link (`mono_cam` include pose), but
the pixel back-projection was added to the vehicle's GPS, putting every
frozen estimate 0.35 m aft of the target. Predicted landing error
0.35 − 0.10 = 0.25 m south; measured 0.252 / 0.264 / 0.290 / 0.305 m south.
Cause (i), confirmed quantitatively, and applied.

North error after the fix: **+0.189 / +0.310 m**, i.e. the sign flipped and
it now over-corrects by ~0.15 m. `PAYLOAD_FINAL_FORWARD_M` = 0.10 m accounts
for two thirds of that by design — the final nudge was harmless while the
aim point was 0.35 m short, and is now a pure error term. The east axis,
where the mount translation acts, is down to **−0.111 / +0.089 m** (4.3–4.6 cm
at translate time, so ~6 cm accrues as descent drift).

**Proposed, not applied:** drop or fold in the final forward nudge, and
re-measure the residual ~0.1–0.2 m north before adding any further trim.

### T3 — route finished early: diagnosis, no fix applied

Read-only, from our own resume log across both failures:

| run | resume indices issued | outcome |
|---|---|---|
| 20:58 | 1, 1, 1, 2 | finished at 2 of 0–3 |
| 22:18 | **3, 3, 3** | finished at 3 of 0–3 |

`_issue_resume` reads `get_current_mission_index()` and sets that same index
back, so there is **no off-by-one**: it never points past the last waypoint.
The failure is that the route is a finite resource. Every pause/resume for a
centring attempt consumes route, and once the vehicle is on the final leg —
index 3 of a 4-item route — resuming re-flies only that leg and PX4 reports
the mission finished. The search then ends because `is_mission_finished()`
is its only terminating condition, whether or not both targets were found.

Not an index/bookkeeping bug, so per instruction nothing was applied.
**Proposed:** when the route finishes with targets outstanding, do not end
the search — either restart the route from index 0 or continue under
Offboard. That is a mission-logic change and needs approval.
