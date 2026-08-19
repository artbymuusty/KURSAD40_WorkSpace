"""ADR-010 P4 (Phase 12 final): smoothing applied to the FINAL setpoint,
after the control law has already decided what it wants.

This is not a control change. The P-law, kp_horizontal / kp_vertical /
kp_altitude, both tolerances and the ADR-009 S1/S2 command floors all run
exactly as before and produce exactly the same commanded velocity; this
stage only limits how fast that command is allowed to CHANGE.

What the V1''' measurement showed (n=1647 setpoints @10 Hz):

    |dv| per tick   mean 0.035   p95 0.150 m/s      <- steady state: smooth
    but 7 ACCELERATING ticks > 0.5 m/s, worst 2.50 m/s IN ONE 0.1s TICK

clustered at 9.99 / 5.24 / 5.00 m -- the first tick of each new descent
step, where a fresh go_to_and_center() starts with a large error and jumps
from the previous call's terminal zero to max speed.

WHAT PHASE 11 GOT WRONG, and Phase 12 removes: this stage also carried a
distance-scaled cap, v_max(d) = clamp(1.0*d, 0.20, 3.0). It compounded with
proportional control in exactly the regime where the convergence tail is
already longest -- at d = 0.3 m it clamped the command to 0.3 m/s on top of
a P-law that was itself shrinking it. Measured cost: tau on the four
pursuits common to V1''' and V1'''' went 9.1 -> 13.2 s (+44.9%), against a
+20% budget, while the rate limit ALONE already accounted for the entire
smoothness gain (accelerating ticks > 0.5 m/s: 7 -> 0, max |dv| 2.50 ->
0.367, actual horizontal accel p95 5.08 -> 4.14 m/s2). The cap bought
nothing the rate limit had not already bought, and cost 25% of the
convergence budget. It is gone; only the rate limit remains.
"""
from dataclasses import dataclass
from typing import Tuple

from core.config.parameters import (
    OFFBOARD_SETPOINT_INTERVAL_S,
    SETPOINT_MAX_DELTA_V_M_S,
)

Vec3 = Tuple[float, float, float]


@dataclass
class SetpointLimiter:
    """Per-axis rate limit, carrying the previous command.

    One instance per controller. `reset()` at the start of each centering
    session so the ramp starts from the vehicle's actual (zero) commanded
    state rather than from whatever the last session left behind."""

    prev: Vec3 = (0.0, 0.0, 0.0)
    max_delta_v: float = SETPOINT_MAX_DELTA_V_M_S
    dt_s: float = OFFBOARD_SETPOINT_INTERVAL_S

    def reset(self) -> None:
        self.prev = (0.0, 0.0, 0.0)

    def limit(self, forward: float, right: float, down: float,
              immediate_stop: bool = True) -> Vec3:
        """Rate-limit one setpoint.

        `immediate_stop` decides what an all-zero request means:

          True  -- a CONVERGENCE stop (or an end-of-session stop). It takes
                   effect on this tick. Ramping it down would mean coasting
                   past the target right after convergence, which is exactly
                   the post-lock drift the 3-second measurement watches.

          False -- a TRANSIENT-LOSS hold. The target dropped out of frame,
                   which is not the same event at all. V1'''' hard-braked
                   from ~1.3 m/s to zero in a single tick four separate
                   times, each one a single-frame dropout; the vehicle then
                   had to re-accelerate from standstill when the target came
                   back one frame later. Here the stop is rate-limited like
                   any other command, so a one-frame dropout costs one tick
                   of deceleration instead of a full stop-start cycle.
        """
        out = []
        for cmd, prev in zip((forward, right, down), self.prev):
            limited = cmd

            # Rate limit, applied per axis so a large change in one axis
            # does not slow the others.
            delta = limited - prev
            if delta > self.max_delta_v:
                limited = prev + self.max_delta_v
            elif delta < -self.max_delta_v:
                limited = prev - self.max_delta_v

            if cmd == 0.0 and immediate_stop:
                limited = 0.0

            out.append(limited)

        result = (out[0], out[1], out[2])
        self.prev = result
        return result
