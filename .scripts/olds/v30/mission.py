import time
import math
from typing import Optional, List, Tuple
from mission_types import FlightIntent, Event, MissionStateData, event_bus

KP_XY = 2.0
KP_Z = 0.8
MAX_XY_SPEED = 4.8
MAX_Z_SPEED = 3.9
ASCENT_SPEED = 1.0
DESCENT_SPEED = 0.5
SEARCH_SPEED = 3.0
RETURN_SPEED = 5.0

ARM_OFFBOARD_TIMEOUT = 15.0
PAYLOAD_DROP_CONFIRM_TIMEOUT = 5.0
# TIMING RACE FIX (found live while validating the post-attach stability fix):
# payload.py's own attach flow (config.py: HOOK_ATTACH_MAX_RETRIES=3 attempts x
# HOOK_ATTACH_CONFIRM_TIMEOUT_S=3.0s each) can legitimately take up to ~9s to
# resolve. The previous 5.0s value here was shorter than that worst case, so
# _state_pickup would give up and fire a PICKUP_PREPARATION retry transition
# before payload.py's own in-flight attach attempt had a chance to finish --
# confirmed live: "Attach confirmed by Gazebo" arrived in the log immediately
# AFTER a "Pickup retry" transition had already fired for the same attempt.
# Set comfortably above the 9s worst case so mission.py's own retry policy
# never races payload.py's.
PICKUP_CONFIRM_TIMEOUT = 12.0
# How many full re-approach attempts (back to PICKUP_PREPARATION for a fresh
# descent + low pass) to make after a pickup is not confirmed, before
# abandoning the bonus task and continuing per mission policy (RETURN_HOME).
# Required so a missed low pass or a transient attach failure gets a real
# retry instead of either stalling forever or silently being treated as a
# success.
PICKUP_MAX_RETRIES = 2
DELIVERY_CONFIRM_TIMEOUT = 5.0
# Same reasoning as PICKUP_MAX_RETRIES, mirrored for the release pass at
# Drop 2: a missed low pass or a transient detach failure gets a real retry
# (re-fly the reverse pass) instead of stalling forever or being silently
# treated as delivered.
DELIVERY_MAX_RETRIES = 2
# KURSAD40 start-position root-cause fix: TAKEOFF's target altitude IS the
# altitude at which the mission start position gets recorded (see
# _state_takeoff) -- one number, not two. Reaching this altitude is
# necessary but not sufficient to record the start position: the vehicle
# must also hold within TAKEOFF_ALT_BAND / TAKEOFF_XY_DRIFT_MAX for a
# continuous TAKEOFF_SETTLE_TIME window first (a single-instant altitude
# crossing is not "stabilized").
TAKEOFF_ALTITUDE = 3.0
TAKEOFF_ALT_BAND = 0.15          # m, altitude band considered "at target"
TAKEOFF_XY_DRIFT_MAX = 0.15      # m, horizontal drift allowed within a settle window before it restarts
TAKEOFF_SETTLE_TIME = 0.8        # s, continuous time required within band+drift before capture
TAKEOFF_SETTLE_TIMEOUT = 4.0     # s, safety valve: capture best-effort rather than stall forever
# Return-to-start arrival gate ("bire bir aynı nokta"): all three must hold
# before RETURN_HOME hands off to LANDING.
RETURN_XY_TOLERANCE = 0.30       # m
RETURN_HEADING_TOLERANCE = 5.0   # deg
RETURN_ALT_TOLERANCE = 0.20      # m
# Ground-contact threshold for the LANDING -> MISSION_COMPLETE transition.
# Deliberately tighter than the final-report vertical-error acceptance bound
# (0.20m, see _state_mission_complete) -- if this were looser than that bound,
# the mission could "complete" while still failing its own vertical
# acceptance check.
LANDING_TOUCHDOWN_ALTITUDE = 0.15
SEARCH_ALTITUDE = 15.0
DROP_ALTITUDE = 1.0
PICKUP_ALTITUDE = 1.0

ALIGN_THRESHOLD_PX = 30
VISION_LOSS_TOLERANCE = 1.5

# ==========================================================
# KURSAD40 BONUS MISSION (hook-pass pickup & reverse release)
# ==========================================================
# Exact geometry from spec: hover 0.30m above Drop1's payload during the
# pickup pass, 0.30m behind it as the pass start point, one continuous
# 0.60m forward pass so the hook sweeps directly over the payload; heading
# locked toward Drop2 the whole time (no rotation during pickup, same
# direction carried through transport, per "the transport flight will
# continue in the same direction").
PICKUP_APPROACH_ALT = 0.30
PICKUP_PASS_SPEED = 0.3       # m/s, constant-speed forward pass (not proportional)
TRANSPORT_ALTITUDE = 3.0
DELIVERY_ALT = 0.30
BACKWARD_EXTRACTION_DIST = 0.30
BACKWARD_SPEED = 0.3          # m/s, constant-speed backward extraction
# Visual servo re-alignment at Drop2 before descending -- reuses the exact
# same detector/servo pipeline the original approach used (HSVContourDetectorBackend
# + VisualServoController), just re-armed for the second target's shape.
DELIVERY_ALIGN_REQUIRED_FRAMES = 15  # ~0.5s at 30fps, matches _state_target_tracking
DELIVERY_VISION_TIMEOUT_S = 10.0

# ==========================================================
# POST-ATTACH FLIGHT STABILITY FIX
# ==========================================================
# Root cause (confirmed live + by code trace, see mission.py's _state_pickup):
# payload.py's contact-wait/attach flow runs concurrently with the forward
# pass and typically confirms ATTACH_STATE:true mid-pass (contact happens as
# the hook sweeps over Drop1, not at the pass's very end). The previous
# _state_pickup only checked self._pickup_result AFTER the geometric
# dist<0.1 condition, so it kept commanding constant-speed forward flight
# for the remaining pass distance while the payload was ALREADY rigidly
# attached and still resting at/near the ground -- yanking it through the
# now-taut fixed joint every tick. Live telemetry showed this produced
# violent oscillation (relative altitude swinging from -3.4m to +1.3m)
# immediately after a confirmed attach. Fix: check _pickup_result on every
# tick regardless of pass-completion, freeze all motion the instant it's
# True, then hold until REAL attitude telemetry (not a timer) shows the
# vehicle has actually settled.
POST_ATTACH_SETTLE_ANGLE_DEG = 5.0      # roll/pitch must be within this of level
POST_ATTACH_SETTLE_RATE_RAD_S = 0.15    # roll/pitch rates must be below this
POST_ATTACH_SETTLE_FRAMES = 15          # consecutive stable ticks required (debounce)
# Safety-net bound only -- the settle check above is the real gate. Without
# some bound, a telemetry glitch (e.g. a dropped attitude stream) could hold
# the hover forever; PICKUP_CONFIRM_TIMEOUT and every other wait-loop in this
# file already uses the same "real signal first, bounded timeout as a net"
# pattern.
POST_ATTACH_SETTLE_TIMEOUT_S = 8.0
POST_ATTACH_ASCENT_SPEED = 0.4          # reduced vs. the mission's normal ASCENT_SPEED=1.0
POST_ATTACH_TRANSPORT_SPEED = 1.5       # moderate vs. NAVIGATE_NED's 4.8 m/s ceiling

class MissionManager:
    """
    Central intelligence engine driving the 22-state state machine.
    """
    def __init__(self, memory, search_planner, servo_controller):
        self.memory = memory
        self.search = search_planner
        self.servo = servo_controller
        self.state_data = MissionStateData()
        
        self.target_data = None
        self.telemetry = None
        self.frame_wh = (1280, 960)
        
        self.locked_target_key = None
        self.target_locked_frames = 0
        self.last_seen_time = 0.0
        
        self.descent_gate = 15.0

        # KURSAD40 start-position capture (_state_takeoff): tracks the
        # continuous settle window used to confirm the vehicle has actually
        # stabilized at TAKEOFF_ALTITUDE before memory.start_position is
        # recorded (see TAKEOFF_SETTLE_TIME).
        self._takeoff_settle_start = None
        self._takeoff_settle_ref_ne = None

        # KURSAD40 RETURN_HOME / LANDING runtime validation logging.
        self._return_home_logged = False
        self._landing_logged = False
        self._return_log_last = 0.0
        self._final_landing_snapshot = None
        self._final_report_printed = False
        self._mission_start_time = time.time()

        self._disarm_requested = False
        self._drop_result = None  # None=pending, True=confirmed, False=confirmed failure

        self._delivery_result = None  # None=pending, True=confirmed, False=confirmed failure
        self._delivery_wait_start = None
        self._delivery_retry_count = 0

        # KURSAD40 bonus mission sub-phase trackers (see _state_pickup_prep,
        # _state_pickup, _state_payload_transfer, _state_payload_release).
        self._pickup_subphase = "to_drop1"       # to_drop1 -> descend -> to_start
        # POST-ATTACH STABILITY FIX state: entered the instant _pickup_result
        # becomes True, regardless of pass-completion (see _state_pickup).
        self._settle_frames = 0
        self._settle_wait_start = None
        self._delivery_subphase = "ascend"       # ascend -> fly_to_drop2 -> vision_search -> vision_align
        self._release_subphase = "descend"       # descend -> detach -> extract -> ascend
        self._detach_requested = False
        self._extract_target = None
        self._extract_overshoot_guard = 0
        self._delivery_align_frames = 0
        self._delivery_vision_wait_start = None

        # Mapping of mission states. NOTE: SECOND_TARGET / SECOND_PAYLOAD_DROP from
        # the original design were never reachable (the second target reuses the
        # SEARCH -> ... -> PAYLOAD_DROP path via SEARCH_RESUME) and have been removed.
        self.states = {
            "BOOT": self._state_boot,
            "INITIALIZATION": self._state_init,
            "ARM_OFFBOARD": self._state_arm_offboard,
            "TAKEOFF": self._state_takeoff,
            "ASCEND_TO_SEARCH_ALTITUDE": self._state_ascend_search,
            "SEARCH": self._state_search,
            "TARGET_DETECTED": self._state_target_detected,
            "TARGET_TRACKING": self._state_target_tracking,
            "PRECISION_ALIGNMENT": self._state_precision_alignment,
            "STEP_DESCENT": self._state_step_descent,
            "PAYLOAD_DROP": self._state_payload_drop,
            "ASCEND_AFTER_DROP": self._state_ascend_after_drop,
            "SEARCH_RESUME": self._state_search_resume,
            "RETURN_HOME": self._state_return_home,
            "LANDING": self._state_landing,
            "MISSION_COMPLETE": self._state_mission_complete,
            "EMERGENCY_ABORT": self._state_emergency_abort,
        }

    def _transition(self, new_state: str, decision: str = ""):
        # RUNTIME DEBUG (KURSAD40 second-payload investigation), STEP 1: log
        # every state transition so a live run shows exactly how far the
        # mission actually got instead of having to infer it from side effects.
        suffix = f" ({decision})" if decision else ""
        print(f"[MISSION] STATE TRANSITION: {self.state_data.current_state} -> {new_state}{suffix}")
        self.state_data.previous_state = self.state_data.current_state
        self.state_data.current_state = new_state
        self.state_data.state_entry_time = time.time()
        if decision:
            self.state_data.current_decision = decision

        event = Event(
            name=f"TRANSITION_{new_state}",
            source="MissionManager",
            payload={"from": self.state_data.previous_state, "reason": decision}
        )
        event_bus.publish(event)

    def update_telemetry(self, tel):
        self.telemetry = tel

    def update_vision(self, target_data, frame_wh):
        self.target_data = target_data
        self.frame_wh = frame_wh
        if target_data is not None:
            self.last_seen_time = time.time()

    def process_event(self, event: Event):
        # Event handler could directly trigger transitions,
        # but here we queue them or just handle immediately.
        # For simplicity in this step method, we'll evaluate conditions directly in states,
        # but allow events to override.
        if event.name == "EmergencyAbort":
            self._transition("EMERGENCY_ABORT", "Emergency trigger received.")
        elif event.name == "PAYLOAD_DROP_RESULT":
            self._drop_result = bool(event.payload.get("success"))
        elif event.name == "PAYLOAD_RELEASE_RESULT":
            self._delivery_result = bool(event.payload.get("success"))

    def step(self) -> FlightIntent:
        """Evaluates current state and returns the desired FlightIntent."""
        state_func = self.states.get(self.state_data.current_state, self._state_emergency_abort)
        return state_func()

    # --- 21 State Implementations ---

    def _state_boot(self) -> FlightIntent:
        self.state_data.active_filter = ["blue_hexagon", "red_triangle"]
        if self.state_data.state_time() > 1.0:
            self._transition("INITIALIZATION", "System Booted.")
        return FlightIntent(mode="HOVER")

    def _state_init(self) -> FlightIntent:
        # NOTE: the mission start position is deliberately NOT captured here.
        # On the ground, pre-arm, NED/GPS can still be settling (EKF origin
        # init, ground effect on the estimator, etc.) and nothing has
        # confirmed the vehicle is actually stable yet. It's captured once,
        # later, in _state_takeoff, only after the vehicle has held steady
        # at TAKEOFF_ALTITUDE for TAKEOFF_SETTLE_TIME -- see there.
        if self.telemetry and self.telemetry.get("alt") is not None:
            self._transition("ARM_OFFBOARD", "Telemetry valid. Arming and entering OFFBOARD.")
        return FlightIntent(mode="HOVER")

    def _state_arm_offboard(self) -> FlightIntent:
        """
        Drives MavController.arm_and_start_offboard() every tick until the vehicle
        reports ARMED + OFFBOARD, then hands off to TAKEOFF. Without this state
        nothing in the mission stack ever armed the vehicle or entered OFFBOARD
        (previously required a manual/external arm+mode-switch before the mission
        loop could do anything useful).
        """
        if self.telemetry and self.telemetry.get("armed") and self.telemetry.get("offboard"):
            self._transition("TAKEOFF", "Armed and in OFFBOARD.")
            return FlightIntent(mode="HOVER")

        if self.state_data.state_time() > ARM_OFFBOARD_TIMEOUT:
            self._transition("EMERGENCY_ABORT", "Failed to arm / enter OFFBOARD within timeout.")
            return FlightIntent(mode="HOVER")

        return FlightIntent(mode="ARM_OFFBOARD")

    def _state_takeoff(self) -> FlightIntent:
        alt = self.telemetry.get("alt", 0) if self.telemetry else 0
        ned = self.telemetry.get("ned") if self.telemetry else None

        # Capture the mission start position exactly once, only after the
        # climb has genuinely SETTLED at TAKEOFF_ALTITUDE -- not the instant
        # altitude first crosses it. "Settled" means: altitude within
        # TAKEOFF_ALT_BAND AND horizontal position hasn't drifted more than
        # TAKEOFF_XY_DRIFT_MAX, continuously, for TAKEOFF_SETTLE_TIME. Any
        # drift beyond that restarts the settle window. TAKEOFF_SETTLE_TIMEOUT
        # is a safety valve so a noisy estimator that never fully settles
        # can't stall the mission in a hover forever -- it captures
        # best-effort and logs a warning instead.
        if self.memory.start_position is None and ned is not None:
            in_band = abs(alt - TAKEOFF_ALTITUDE) <= TAKEOFF_ALT_BAND
            if in_band:
                if self._takeoff_settle_start is None:
                    self._takeoff_settle_start = time.time()
                    self._takeoff_settle_ref_ne = (ned[0], ned[1])
                else:
                    drift = math.hypot(ned[0] - self._takeoff_settle_ref_ne[0],
                                        ned[1] - self._takeoff_settle_ref_ne[1])
                    if drift > TAKEOFF_XY_DRIFT_MAX:
                        self._takeoff_settle_start = time.time()
                        self._takeoff_settle_ref_ne = (ned[0], ned[1])

                settled = self._takeoff_settle_start is not None and \
                    (time.time() - self._takeoff_settle_start) >= TAKEOFF_SETTLE_TIME
                timed_out = self.state_data.state_time() >= TAKEOFF_SETTLE_TIMEOUT
                if settled or timed_out:
                    if timed_out and not settled:
                        print(f"[MISSION] WARNING: start position captured after "
                              f"{TAKEOFF_SETTLE_TIMEOUT}s settle timeout without full stabilization.")
                    self.memory.save_start_position(self.telemetry.get("gps"), ned, alt,
                                                     self.telemetry.get("yaw", 0.0))
            else:
                self._takeoff_settle_start = None
                self._takeoff_settle_ref_ne = None

        if alt >= TAKEOFF_ALTITUDE - 0.5 and self.memory.start_position is not None:
            self._transition("ASCEND_TO_SEARCH_ALTITUDE", "Takeoff altitude reached, start position saved.")
            return FlightIntent(mode="HOVER")

        alt_err = alt - TAKEOFF_ALTITUDE
        v_down = max(-ASCENT_SPEED, min(0.0, KP_Z * alt_err))
        return FlightIntent(mode="VELOCITY_BODY", velocity=(0, 0, v_down))

    def _state_ascend_search(self) -> FlightIntent:
        alt = self.telemetry.get("alt", 0) if self.telemetry else 0
        alt_err = alt - SEARCH_ALTITUDE
        v_down = max(-ASCENT_SPEED, min(0.0, KP_Z * alt_err))
        if alt >= SEARCH_ALTITUDE - 0.5:
            # Lock in the forward search heading exactly once. This is only
            # ever reached on the FIRST ascent (ASCEND_AFTER_DROP takes a
            # separate path straight to SEARCH_RESUME/PICKUP_PREPARATION and
            # never calls this again), so the heading survives untouched for
            # the rest of the mission -- the straight-line search resumes in
            # the same direction after every later drop, as required.
            if not self.search.is_active():
                heading = self.telemetry.get("yaw", 0.0) if self.telemetry else 0.0
                self.search.start_straight_search(heading_deg=heading, altitude_down=-SEARCH_ALTITUDE)
            self._transition("SEARCH", "Search altitude reached.")
            return FlightIntent(mode="HOVER")
        return FlightIntent(mode="VELOCITY_BODY", velocity=(0, 0, v_down))

    def _state_search(self) -> FlightIntent:
        if self.target_data and self.target_data["target_key"] in self.state_data.active_filter:
            self._transition("TARGET_DETECTED", "Target spotted.")
            return FlightIntent(mode="HOVER")
            
        if self.telemetry and self.telemetry.get("ned"):
            self.state_data.search_progress = self.search.get_progress()
            return self.search.get_search_intent(self.telemetry["ned"])
        return FlightIntent(mode="HOVER")

    def _state_target_detected(self) -> FlightIntent:
        if self.target_data:
            self.target_locked_frames += 1
            if self.target_locked_frames >= 3:
                self.locked_target_key = self.target_data["target_key"]
                self.state_data.active_filter = [self.locked_target_key]
                self._transition("TARGET_TRACKING", "Target Locked.")
        else:
            self.target_locked_frames = 0
            if time.time() - self.last_seen_time > VISION_LOSS_TOLERANCE:
                self._transition("SEARCH", "Target Lost.")
        return FlightIntent(mode="HOVER")

    def _state_target_tracking(self) -> FlightIntent:
        if time.time() - self.last_seen_time > VISION_LOSS_TOLERANCE:
            self._transition("SEARCH", "Target Lost.")
            self.state_data.active_filter = ["blue_hexagon", "red_triangle"]
            return FlightIntent(mode="HOVER")
            
        if self.target_data:
            intent = self.servo.get_servo_intent(self.target_data, self.frame_wh)
            
            if self.servo.is_aligned(required_frames=15): # ~0.5s at 30fps
                self.descent_gate = 13.0 # Set first gate
                self.servo.reset_alignment()
                self._transition("PRECISION_ALIGNMENT", "Aligned.")
                
            return intent
            
        return FlightIntent(mode="HOVER")

    def _state_precision_alignment(self) -> FlightIntent:
        # Same as tracking, but tight threshold before transitioning to descent
        if time.time() - self.last_seen_time > VISION_LOSS_TOLERANCE:
            self._transition("SEARCH_RESUME", "Target Lost during precision alignment.")
            return FlightIntent(mode="HOVER")
            
        if self.target_data:
            # Re-use servo intent, but with tighter alignment check for transition
            intent = self.servo.get_servo_intent(self.target_data, self.frame_wh)
            # Override threshold temporarily if needed, or assume servo handles it
            self.servo.align_threshold_px = ALIGN_THRESHOLD_PX * 0.5
            
            if self.servo.is_aligned(required_frames=5): # quick strict check
                self.servo.reset_alignment()
                self.servo.align_threshold_px = ALIGN_THRESHOLD_PX # restore
                if self.descent_gate <= DROP_ALTITUDE + 0.2:
                    self._transition("PAYLOAD_DROP", "Final alignment complete.")
                else:
                    self._transition("STEP_DESCENT", f"Descending to {self.descent_gate}m.")
            
            return intent
            
        return FlightIntent(mode="HOVER")

    def _state_step_descent(self) -> FlightIntent:
        if time.time() - self.last_seen_time > VISION_LOSS_TOLERANCE:
            self._transition("ASCEND_AFTER_DROP", "Target Lost. Aborting descent.")
            return FlightIntent(mode="HOVER")
            
        alt = self.telemetry.get("alt", 0) if self.telemetry else 0
        alt_err = alt - self.descent_gate
        v_down_cmd = max(0.0, min(DESCENT_SPEED, KP_Z * alt_err))
        
        if self.target_data:
            intent = self.servo.get_servo_intent(self.target_data, self.frame_wh)
            v_fwd, v_right, _ = intent.velocity
            
            # Phase 5: Suspend descent if target drifts outside alignment threshold
            if not self.servo.is_aligned(required_frames=1):
                v_down = 0.0
            else:
                v_down = v_down_cmd
        else:
            v_right, v_fwd, v_down = 0.0, 0.0, 0.0

        if alt <= self.descent_gate + 0.2:
            # Reached gate
            self.descent_gate -= 2.0 # Next gate
            if self.descent_gate < DROP_ALTITUDE:
                self.descent_gate = DROP_ALTITUDE
            self._transition("PRECISION_ALIGNMENT", "Gate reached. Re-aligning.")
            
        return FlightIntent(mode="VELOCITY_BODY", velocity=(v_fwd, v_right, v_down), yaw_rate=0.0)

    def _state_payload_drop(self) -> FlightIntent:
        """
        Waits for PayloadManager's async drop request to report a real result
        (via the PAYLOAD_DROP_RESULT event, see main.py's event handler) instead
        of blindly assuming success after a fixed delay. On confirmed failure or
        on timeout (no confirmation arrived) we still disable the target and move
        on -- there is no retry path in this architecture, and getting stuck
        re-attempting forever would be worse than a logged, flagged best-effort
        continuation.
        """
        if self._drop_result is None:
            if self.state_data.state_time() > PAYLOAD_DROP_CONFIRM_TIMEOUT:
                print(f"[MISSION] WARNING: no payload drop confirmation after "
                      f"{PAYLOAD_DROP_CONFIRM_TIMEOUT}s for '{self.locked_target_key}'. "
                      f"Proceeding without confirmation.")
            else:
                return FlightIntent(mode="HOVER")
        elif self._drop_result is False:
            print(f"[MISSION] WARNING: payload drop reported FAILURE for "
                  f"'{self.locked_target_key}'. Proceeding anyway (no retry available).")

        payload_color = "red" if self.locked_target_key == "blue_hexagon" else "blue"
        # RUNTIME DEBUG FIX (KURSAD40 second-payload investigation), STEP 8:
        # drop_confirmed reflects whether PayloadManager's DROP_STATE
        # confirmation actually verified a spawn (see payload.py's
        # _gazebo_boolean_drop) -- captured BEFORE _drop_result is reset
        # below, since completed_targets alone (marked unconditionally, a
        # few lines down, so a genuinely undroppable target doesn't strand
        # the mission in search forever) is no longer sufficient proof that
        # a payload actually reached the ground.
        drop_confirmed = (self._drop_result is True)

        # Save drop
        self.memory.save_drop(
            payload_color,
            self.locked_target_key,
            self.telemetry.get("gps"),
            self.telemetry.get("ned"),
            self.telemetry.get("alt")
        )
        self.memory.mark_target_completed(self.locked_target_key)
        if drop_confirmed:
            self.memory.mark_drop_confirmed(payload_color)
        else:
            print(f"[MISSION] WARNING: '{payload_color}' drop for target "
                  f"'{self.locked_target_key}' was NOT confirmed by Gazebo. "
                  f"confirmed_drops={self.memory.confirmed_drops} -- bonus pickup will not "
                  f"start until both colors are genuinely confirmed.")

        # Dynamic filter: remove target permanently
        active = [t for t in self.state_data.active_filter if t != self.locked_target_key]
        self.state_data.active_filter = active

        self._drop_result = None  # reset for the next drop
        self._transition("ASCEND_AFTER_DROP", "Payload drop resolved. Target disabled.")

        return FlightIntent(mode="HOVER")

    def _state_ascend_after_drop(self) -> FlightIntent:
        alt = self.telemetry.get("alt", 0) if self.telemetry else 0
        alt_err = alt - SEARCH_ALTITUDE
        v_down = max(-ASCENT_SPEED, min(0.0, KP_Z * alt_err))
        if alt >= SEARCH_ALTITUDE - 0.5:
            if len(self.memory.completed_targets) >= 2:
                # RUNTIME DEBUG FIX (KURSAD40 second-payload investigation),
                # STEP 8: "Pickup must NEVER start if Blue payload was not
                # released" -- gate bonus pickup on confirmed_drops (real
                # DROP_STATE confirmations), not just completed_targets
                # (which only means both targets were attempted/disabled,
                # regardless of whether either payload actually spawned).
                both_confirmed = len(self.memory.confirmed_drops) >= 2
                if both_confirmed:
                    self.descent_gate = 15.0 # Reset gate for pickup descent
                    self._transition("RETURN_HOME", "Both drops completed. Heading home.")
                else:
                    print(f"[MISSION] WARNING: skipping bonus pickup -- only "
                          f"{len(self.memory.confirmed_drops)}/2 drops confirmed "
                          f"({self.memory.confirmed_drops}). Returning home instead.")
                    self._transition("RETURN_HOME", "Both drop attempts complete -- returning home.")
            else:
                self._transition("SEARCH_RESUME", "First drop complete. Resume search.")
            return FlightIntent(mode="HOVER")
        return FlightIntent(mode="VELOCITY_BODY", velocity=(0, 0, v_down))

    def _state_search_resume(self) -> FlightIntent:
        # Re-eval filter against completed targets
        active = ["blue_hexagon", "red_triangle", "blue_rectangle", "red_rectangle"]
        active = [x for x in active if x not in self.memory.completed_targets]
        # Only care about hexagons and triangles for drops
        active = [x for x in active if x in ["blue_hexagon", "red_triangle"]]
        self.state_data.active_filter = active
        self._transition("SEARCH", "Resuming search pattern.")
        return FlightIntent(mode="HOVER")





    def _state_return_home(self) -> FlightIntent:
        """
        Own-logic return to the recorded mission start position -- NOT PX4
        RTL, NOT autopilot home, NOT MAVSDK ReturnToLaunch (flight.py never
        calls any of those; NAVIGATE_NED is our own proportional controller,
        see MavController.execute_intent). memory.start_position is the sole
        source of the return target.
        """
        if not self._return_home_logged:
            self._return_home_logged = True
            print("[MISSION] RETURN TO START POSITION")

        start = self.memory.start_position
        if not start:
            print("[MISSION] WARNING: no start position recorded -- landing in place.")
            self._transition("LANDING", "No start position recorded.")
            return FlightIntent(mode="HOVER")

        target_n, target_e = start["north"], start["east"]
        target_heading = start["heading"]
        target_wp = (target_n, target_e, -TAKEOFF_ALTITUDE)

        cur_n, cur_e, _ = self.telemetry["ned"] if self.telemetry and self.telemetry.get("ned") else (0.0, 0.0, 0.0)
        cur_alt = self.telemetry.get("alt", 0.0) if self.telemetry else 0.0
        cur_yaw = self.telemetry.get("yaw", 0.0) if self.telemetry else 0.0

        dist = math.hypot(target_n - cur_n, target_e - cur_e)
        heading_err = abs((target_heading - cur_yaw + 180) % 360 - 180)
        alt_err = abs(TAKEOFF_ALTITUDE - cur_alt)

        now = time.time()
        if now - self._return_log_last >= 1.0:
            self._return_log_last = now
            print(f"[MISSION] RETURN_HOME  current=({cur_n:.2f},{cur_e:.2f},{cur_alt:.2f})  "
                  f"target=({target_n:.2f},{target_e:.2f},{TAKEOFF_ALTITUDE:.2f})  "
                  f"distance_remaining={dist:.2f}m")

        if dist <= RETURN_XY_TOLERANCE and heading_err <= RETURN_HEADING_TOLERANCE and alt_err <= RETURN_ALT_TOLERANCE:
            print("[MISSION] START POSITION REACHED")
            print(f"[MISSION]   Distance Error: {dist:.3f} m")
            print(f"[MISSION]   Heading Error:  {heading_err:.2f} deg")
            print(f"[MISSION]   Altitude Error: {alt_err:.3f} m")
            self._transition("LANDING", "Start position reached within tolerance.")
            return FlightIntent(mode="HOVER")

        return FlightIntent(mode="NAVIGATE_NED", target_ned=target_wp, target_heading=target_heading)

    def _state_landing(self) -> FlightIntent:
        """
        Precise landing: keeps actively correcting horizontal position onto
        the exact start-position NED coordinate for the whole descent (no
        additional horizontal movement once RETURN_HOME's arrival gate has
        already confirmed the vehicle is within tolerance), instead of
        handing off to PX4's native LAND mode. Guarantees touchdown at the
        exact same point the drone crossed on its way up.
        """
        if not self._landing_logged:
            self._landing_logged = True
            print("[MISSION] LANDING OVER START POSITION")

        alt = self.telemetry.get("alt", 10) if self.telemetry else 10
        start = self.memory.start_position

        if alt < LANDING_TOUCHDOWN_ALTITUDE:
            if self._final_landing_snapshot is None:
                ned = self.telemetry.get("ned") if self.telemetry and self.telemetry.get("ned") else (0.0, 0.0, 0.0)
                self._final_landing_snapshot = {
                    "north": ned[0],
                    "east": ned[1],
                    "alt": alt,
                    "heading": self.telemetry.get("yaw", 0.0) if self.telemetry else 0.0,
                }
            self._transition("MISSION_COMPLETE")
            return FlightIntent(mode="HOVER")

        if start:
            # Hold the start heading actively during descent too (not just
            # up to RETURN_HOME's arrival gate) so heading doesn't drift
            # while only altitude is still converging.
            return FlightIntent(mode="NAVIGATE_NED", target_ned=(start["north"], start["east"], 0.0),
                                 target_heading=start["heading"])

        return FlightIntent(mode="LAND")

    def _state_mission_complete(self) -> FlightIntent:
        if not self._final_report_printed and self._final_landing_snapshot is not None:
            self._final_report_printed = True
            self._print_final_validation_report()

        if not self._disarm_requested:
            self._disarm_requested = True
            return FlightIntent(mode="DISARM")
        return FlightIntent(mode="HOVER")

    def _print_final_validation_report(self):
        start = self.memory.start_position
        final = self._final_landing_snapshot
        duration = time.time() - self._mission_start_time

        print("=================================")
        print("MISSION START POSITION")
        if start:
            print(f"N:   {start['north']:.3f}")
            print(f"E:   {start['east']:.3f}")
            print(f"ALT: {start['alt']:.3f}")
            print(f"YAW: {start['heading']:.1f}")
        else:
            print("N/A -- start position was never captured")
        print("----------------------")
        print("FINAL LANDING POSITION")
        print(f"N:   {final['north']:.3f}")
        print(f"E:   {final['east']:.3f}")
        print(f"ALT: {final['alt']:.3f}")
        print(f"YAW: {final['heading']:.1f}")
        print("----------------------")

        if start:
            horizontal_error = math.hypot(start["north"] - final["north"], start["east"] - final["east"])
            # Vertical error is measured against the ground (0m), i.e. how
            # flush the touchdown was -- NOT against start's recorded
            # altitude (TAKEOFF_ALTITUDE, ~3m), which would always differ by
            # ~3m since start was recorded airborne and landing is on the
            # ground; that would make this check meaningless by construction.
            vertical_error = abs(final["alt"] - 0.0)
            heading_error = abs((start["heading"] - final["heading"] + 180) % 360 - 180)

            print(f"Horizontal Error: {horizontal_error:.3f} m")
            print(f"Vertical Error:   {vertical_error:.3f} m")
            print(f"Heading Error:    {heading_error:.2f} deg")
            print(f"Mission Duration: {duration:.1f} s")
            print("=================================")

            accepted = (horizontal_error <= RETURN_XY_TOLERANCE and
                        vertical_error <= RETURN_ALT_TOLERANCE and
                        heading_error <= RETURN_HEADING_TOLERANCE)
            print(f"[MISSION] {'ACCEPTED' if accepted else 'REJECTED'} "
                  f"(H<={RETURN_XY_TOLERANCE}m, V<={RETURN_ALT_TOLERANCE}m, "
                  f"Heading<={RETURN_HEADING_TOLERANCE}deg)")
        else:
            print("Horizontal Error: N/A")
            print("Vertical Error:   N/A")
            print("Heading Error:    N/A")
            print(f"Mission Duration: {duration:.1f} s")
            print("=================================")
            print("[MISSION] REJECTED (no start position recorded)")

    def _state_emergency_abort(self) -> FlightIntent:
        return FlightIntent(mode="HOVER")
