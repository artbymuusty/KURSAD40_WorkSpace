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
# How many full re-approach attempts to make after a virtual pickup is not
# confirmed, before abandoning the bonus task and continuing per mission
# policy (RETURN_HOME). Required so a transient event-bus hiccup gets a
# real retry instead of either stalling forever or silently being treated
# as a success.
PICKUP_MAX_RETRIES = 2
DELIVERY_CONFIRM_TIMEOUT = 5.0
# Same reasoning as PICKUP_MAX_RETRIES, mirrored for the virtual release at
# Drop 2.
DELIVERY_MAX_RETRIES = 2
# GROUND start-position capture: the mission start position is the drone's
# real LOCAL_POSITION_NED reading on the ground, pre-arm -- captured exactly
# once, the very first time valid NED telemetry is observed (see
# MissionManager.update_telemetry). NOT the post-takeoff hover point, NOT a
# settled-in-the-air sample: literally the physical point the vehicle was
# sitting at before its motors ever ran. Never recomputed, never
# overwritten -- memory.save_start_position() is a first-call-wins no-op
# after that.
TAKEOFF_ALTITUDE = 3.0

# ==========================================================
# RETURN-TO-START-COORDINATE + CONTROLLED LANDING
# ==========================================================
# memory.start_position is the ONLY source of truth for where the vehicle
# returns to -- the ground coordinate captured before arming. RETURN_HOME
# first goes to (start_north, start_east, RETURN_ALTITUDE) -- the same X/Y
# as the ground start point, but at a safe transit altitude, not a direct
# 3D dive to the ground coordinate from wherever the mission ends. Only
# once there does CONTROLLED_DESCENT step down to the exact recorded
# start_down. No RTL, no PX4 home, no spawn estimation, no relative/offset/
# heading-compensation math anywhere in this path.
RETURN_ALTITUDE = 3.0            # m AGL, transit altitude for the X/Y approach leg

# HOME_HOVER arrival gate: position error AND speed must both hold within
# tolerance, CONTINUOUSLY, for HOME_HOLD_TIME before CONTROLLED_DESCENT is
# allowed to start. There is no timeout bypass -- if it never stabilizes,
# the vehicle just keeps hovering at the start coordinate.
HOME_XY_TOLERANCE = 0.05         # m, position error
HOME_ALT_TOLERANCE = 0.05        # m
HOME_VELOCITY_TOLERANCE = 0.05   # m/s, 3D speed magnitude
HOME_HOLD_TIME = 2.0             # s continuous hold required

# CONTROLLED_DESCENT: fixed AGL waypoints descended one at a time, holding
# start_north/start_east fixed throughout -- never a single jump to the
# ground. Whichever suffix of this list is at/below the vehicle's actual
# altitude on arrival is walked in order; each step must itself settle
# (same shape of tolerance as HOME_HOVER, shorter hold) before the next,
# smaller step is commanded.
DESCENT_STEPS_AGL = [3.0, 2.7, 2.4, 2.1, 1.8, 1.5, 1.2, 0.9, 0.7, 0.5,
                      0.4, 0.3, 0.2, 0.15, 0.10, 0.05, 0.02]
DESCENT_STEP_XY_TOLERANCE = 0.05
DESCENT_STEP_ALT_TOLERANCE = 0.05
DESCENT_STEP_VELOCITY_TOLERANCE = 0.05
DESCENT_STEP_HOLD_TIME = 0.5     # s per-step settle before advancing

# GROUND_CONFIRMATION: authoritative touchdown check. PX4's own landed-state
# detector (fuses accel/thrust/etc -- not a bare altitude threshold) must
# report ON_GROUND, together with near-zero speed and a sane local-position
# estimate, continuously, for GROUND_CONFIRM_HOLD_TIME. No timeout bypass.
GROUND_CONFIRM_VELOCITY_TOLERANCE = 0.05
GROUND_CONFIRM_HOLD_TIME = 1.0

# Final validation report acceptance bounds -- centimeter-level, matching
# what the landing sequence itself now targets.
FINAL_REPORT_XY_TOLERANCE = HOME_XY_TOLERANCE
FINAL_REPORT_ALT_TOLERANCE = HOME_ALT_TOLERANCE
FINAL_REPORT_HEADING_TOLERANCE = 5.0   # deg

SEARCH_ALTITUDE = 15.0
DROP_ALTITUDE = 1.0
PICKUP_ALTITUDE = 1.0

ALIGN_THRESHOLD_PX = 30
VISION_LOSS_TOLERANCE = 1.5

# ==========================================================
# KURSAD40 BONUS MISSION (fully virtual pickup & release)
# ==========================================================
# ALT_GZ REVISION: the original geometry flew a low forward pass at 0.30m
# AGL to physically sweep a hanging hook rope over the dropped payload
# (pickup confirmed by a real Gazebo contact sensor), then a symmetric low
# backward pass to "extract" the hook after release. Live testing showed
# the hook_rope_link's own collision geometry striking the payload/ground
# at that altitude produced violent, non-settling attitude divergence --
# even AFTER the fixed-joint attach itself was made virtual (no
# DetachableJoint, see payload.py's pickup_payload_simulated), the vehicle
# still would not stabilize within a bounded settle window. The collision
# from flying that low, not the joint, was destabilizing the airframe.
#
# This revision removes both low passes entirely: the drone descends
# straight down to PICKUP_ALTITUDE (comfortably clear of any collision)
# directly above Drop1/Drop2 and simply hovers there for
# SIMULATED_PICKUP_WAIT_SECONDS to visually read as "grabbing" the payload.
# Pickup/release are then purely scripted mission-state changes
# (PayloadManager.pickup_payload_virtual / drop_picked_payload in
# payload.py), never gated on physical contact. Heading stays locked
# toward Drop2 throughout pickup + transport (no rotation -- "the transport
# flight will continue in the same direction").
SIMULATED_PICKUP_WAIT_SECONDS = 3.0   # hover dwell time to simulate the grab/release
TRANSPORT_ALTITUDE = 3.0
VIRTUAL_PICKUP_CONFIRM_TIMEOUT = 5.0  # generous bound for the near-instant virtual confirmation round-trip
# Visual servo re-alignment at Drop2 before descending -- reuses the exact
# same detector/servo pipeline the original approach used (HSVContourDetectorBackend
# + VisualServoController), just re-armed for the second target's shape.
DELIVERY_ALIGN_REQUIRED_FRAMES = 15  # ~0.5s at 30fps, matches _state_target_tracking
DELIVERY_VISION_TIMEOUT_S = 10.0
# Gentler than the mission's normal ASCENT_SPEED/NAVIGATE_NED ceiling for the
# post-pickup transport leg -- no real load is being carried (fully virtual),
# but a smooth, deliberate climb/cruise still reads better on camera than a
# snap to full speed right after a "pickup".
POST_ATTACH_ASCENT_SPEED = 0.4
POST_ATTACH_TRANSPORT_SPEED = 1.5

# ==========================================================
# ALT_GZ MISSION 3 (post-delivery re-verification pass)
# ==========================================================
# Known Gazebo spawn positions (Tools/simulation/gz/worlds/default.sdf:105-118),
# assuming local-NED origin coincides with the drone's world spawn point (E=0
# centerline for both). No detector produces a real-world target position
# (target_ned on FlightIntent is a generic nav field, never populated from
# vision -- see mission_types.py), so these hardcoded spawn positions are the
# only source of truth for "fly back to where the shape is."
TARGET_WORLD_NED = {
    "blue_hexagon": (15.0, 0.0),
    "red_triangle": (40.0, 0.0),
}
VERIFY_LOCK_FRAMES = 3   # same debounce as _state_target_detected

class MissionManager:
    """
    Central intelligence engine driving the 26-state state machine.
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

        self._drop_result = None  # None=pending, True=confirmed, False=confirmed failure

        self._delivery_result = None  # None=pending, True=confirmed, False=confirmed failure
        self._delivery_retry_count = 0

        # KURSAD40 bonus mission sub-phase trackers (see _state_pickup_prep,
        # _state_pickup, _state_payload_transfer, _state_payload_release).
        # ALT_GZ: fully virtual pickup/release, no forward/backward passes --
        # see the "KURSAD40 BONUS MISSION" constants block for why.
        self._pickup_subphase = "to_drop1"       # to_drop1 -> descend
        self._delivery_subphase = "ascend"       # ascend -> fly_to_drop2 -> vision_search -> vision_align
        self._release_subphase = "descend"       # descend -> release -> ascend
        self._release_wait_start = None
        self._detach_requested = False
        self._delivery_align_frames = 0
        self._delivery_vision_wait_start = None

        # ALT_GZ: Mission 3 verification pass + one-time milestone banners
        self._verify_locked_frames = 0
        self._mission1_started_logged = False
        self._return_home_logged = False
        self._mission_complete_logged = False

        # RETURN_HOME / HOME_HOVER / CONTROLLED_DESCENT / GROUND_CONFIRMATION
        # runtime state -- full redesign, see _state_return_home onward.
        self._home_hover_hold_start = None
        self._descent_steps = None
        self._descent_step_idx = 0
        self._descent_step_hold_start = None
        self._last_descent_target = None
        self._ground_confirm_hold_start = None
        self._offboard_exit_requested = False
        self._disarm_pending = False
        self._disarm_result = None       # None=pending, True/False=known result
        self._disarm_fail_reason = ""
        self._kill_pending = False
        self._kill_result = None
        self._kill_fail_reason = ""
        self._log_last_nav = 0.0
        self._log_last_descent = 0.0
        self._log_last_ground = 0.0

        self._final_landing_snapshot = None
        self._final_report_printed = False
        self._mission_start_time = time.time()

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
            "VERIFY_APPROACH_1": self._state_verify_approach_1,
            "VERIFY_1": self._state_verify_1,
            "VERIFY_APPROACH_2": self._state_verify_approach_2,
            "VERIFY_2": self._state_verify_2,
            "RETURN_HOME": self._state_return_home,
            "HOME_HOVER": self._state_home_hover,
            "CONTROLLED_DESCENT": self._state_controlled_descent,
            "GROUND_CONFIRMATION": self._state_ground_confirmation,
            "OFFBOARD_EXIT": self._state_offboard_exit,
            "DISARM": self._state_disarm,
            "KILL": self._state_kill,
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
        # GROUND START CAPTURE: the very first time valid LOCAL_POSITION_NED
        # telemetry is observed -- on the ground, before ARM_OFFBOARD has
        # even run -- record it as the mission's one and only return
        # coordinate. save_start_position() is idempotent (first call wins),
        # so this check+call is also naturally safe to run on every tick
        # without a separate "already captured" guard here.
        if (tel is not None and self.memory.start_position is None
                and tel.get("ned") is not None):
            self.memory.save_start_position(tel.get("gps"), tel.get("ned"),
                                             tel.get("alt"), tel.get("yaw", 0.0))

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
        elif event.name == "DISARM_RESULT":
            self._disarm_result = bool(event.payload.get("success"))
            self._disarm_fail_reason = event.payload.get("reason", "")
        elif event.name == "KILL_RESULT":
            self._kill_result = bool(event.payload.get("success"))
            self._kill_fail_reason = event.payload.get("reason", "")

    def step(self) -> FlightIntent:
        """Evaluates current state and returns the desired FlightIntent."""
        state_func = self.states.get(self.state_data.current_state, self._state_emergency_abort)
        return state_func()

    # --- 21 State Implementations ---

    def _state_boot(self) -> FlightIntent:
        if not self._mission1_started_logged:
            self._mission1_started_logged = True
            print("[MISSION] MISSION 1 STARTED")
        self.state_data.active_filter = ["blue_hexagon", "red_triangle"]
        if self.state_data.state_time() > 1.0:
            self._transition("INITIALIZATION", "System Booted.")
        return FlightIntent(mode="HOVER")

    def _state_init(self) -> FlightIntent:
        # NOTE: the mission start position is NOT captured in this state
        # function -- it's captured in update_telemetry(), unconditionally,
        # the very first tick real NED telemetry is observed (which happens
        # well before this state even runs, typically during BOOT). See
        # update_telemetry() and memory.save_start_position().
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
        # The mission start position is no longer captured here -- it is
        # captured exactly once, on the ground before arming, the first
        # time valid NED telemetry is observed (see update_telemetry()).
        # By the time TAKEOFF runs the vehicle is already armed, so
        # memory.start_position is guaranteed to already be set.
        alt = self.telemetry.get("alt", 0) if self.telemetry else 0

        if alt >= TAKEOFF_ALTITUDE - 0.5:
            self._transition("ASCEND_TO_SEARCH_ALTITUDE", "Takeoff altitude reached.")
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
                    print("[MISSION] MISSION 1 COMPLETE")
                    print("[MISSION] SECOND MISSION COMPLETE")
                    print("[MISSION] MISSION 2 STARTED")
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





    def _state_verify_approach_1(self) -> FlightIntent:
        return self._verify_approach("blue_hexagon", "VERIFY_1")

    def _state_verify_1(self) -> FlightIntent:
        return self._verify_target("blue_hexagon", "HEXAGON VERIFIED", "VERIFY_APPROACH_2")

    def _state_verify_approach_2(self) -> FlightIntent:
        return self._verify_approach("red_triangle", "VERIFY_2")

    def _state_verify_2(self) -> FlightIntent:
        return self._verify_target("red_triangle", "TRIANGLE VERIFIED", None)

    def _agl_to_down(self, agl: float) -> float:
        """
        Converts a desired AGL height into this mission's NED 'down'
        coordinate, using ONLY the single recorded start_position sample as
        the ground reference (ground_down = start.down + start.alt, since
        start.alt was the AGL reading at the exact instant start.down was
        captured). No separate home/spawn estimation -- pure algebra on the
        one recorded point.
        """
        start = self.memory.start_position
        ground_down = start["down"] + start["alt"]
        return ground_down - agl

    def _nav_error(self, target_ned: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
        """Returns (xy_distance, alt_error, 3D speed, vertical_speed) against target_ned."""
        cur = self.telemetry.get("ned") if self.telemetry else None
        cur_n, cur_e, cur_d = cur if cur else (0.0, 0.0, 0.0)
        vel = self.telemetry.get("velocity_ned") if self.telemetry else None
        v_n, v_e, v_d = vel if vel else (0.0, 0.0, 0.0)
        dist = math.hypot(target_ned[0] - cur_n, target_ned[1] - cur_e)
        alt_err = abs(target_ned[2] - cur_d)
        speed = math.sqrt(v_n * v_n + v_e * v_e + v_d * v_d)
        return dist, alt_err, speed, v_d

    def _log_nav(self, tag: str, target_ned: Tuple[float, float, float],
                 dist: float, alt_err: float, speed: float, v_d: float):
        now = time.time()
        if now - self._log_last_nav < 0.5:
            return
        self._log_last_nav = now
        cur = self.telemetry.get("ned") if self.telemetry and self.telemetry.get("ned") else (0.0, 0.0, 0.0)
        cur_alt = self.telemetry.get("alt", 0.0) if self.telemetry else 0.0
        target_alt = -target_ned[2]
        print(f"[MISSION] [{tag}] POSITION ERROR: {dist:.3f}m  ALT ERROR: {alt_err:.3f}m  "
              f"CURRENT POSITION: ({cur[0]:.3f},{cur[1]:.3f},{cur[2]:.3f})  "
              f"TARGET POSITION: ({target_ned[0]:.3f},{target_ned[1]:.3f},{target_ned[2]:.3f})  "
              f"CURRENT ALTITUDE: {cur_alt:.3f}m  TARGET ALTITUDE: {target_alt:.3f}m  "
              f"VERTICAL SPEED: {v_d:.3f}m/s  VELOCITY: {speed:.3f}m/s")

    def _state_return_home(self) -> FlightIntent:
        """
        Return-to-start-coordinate, transit leg: a single literal
        goto_position(start_north, start_east, RETURN_ALTITUDE) toward the
        X/Y of the ONE ground coordinate recorded once before arming (see
        update_telemetry). Deliberately approaches at RETURN_ALTITUDE, not a
        direct 3D dive to the recorded ground_down -- no RTL, no PX4 home,
        no MAVSDK ReturnToLaunch, no relative offset, no forward-vector/
        body-frame math, no spawn/home estimation. Hands off to HOME_HOVER
        the first moment position and speed are within tolerance;
        CONTROLLED_DESCENT (after HOME_HOVER confirms a stable hold) is what
        actually steps down to the recorded ground_down.
        """
        start = self.memory.start_position
        if not start:
            print("[MISSION] CRITICAL: no start position recorded -- cannot return.")
            self._transition("EMERGENCY_ABORT", "No start position recorded.")
            return FlightIntent(mode="HOVER")

        if not self._return_home_logged:
            self._return_home_logged = True
            print("[MISSION] RETURN TARGET")
            print(f"[MISSION] North: {start['north']:.3f}")
            print(f"[MISSION] East:  {start['east']:.3f}")
            print(f"[MISSION] Down:  {start['down']:.3f}")

        target = (start["north"], start["east"], self._agl_to_down(RETURN_ALTITUDE))
        dist, alt_err, speed, v_d = self._nav_error(target)
        self._log_nav("RETURN_HOME", target, dist, alt_err, speed, v_d)

        if dist <= HOME_XY_TOLERANCE and alt_err <= HOME_ALT_TOLERANCE and speed <= HOME_VELOCITY_TOLERANCE:
            print("[MISSION] HOME POSITION REACHED")
            self._transition("HOME_HOVER", "Within arrival tolerance, verifying stable hold.")

        return FlightIntent(mode="GOTO_NED", target_ned=target, target_heading=start["heading"])

    def _state_home_hover(self) -> FlightIntent:
        """
        Confirms the vehicle is actually stable above the start coordinate
        (at RETURN_ALTITUDE, same X/Y as ground start): position error <
        HOME_XY_TOLERANCE/HOME_ALT_TOLERANCE AND speed <
        HOME_VELOCITY_TOLERANCE, CONTINUOUSLY, for HOME_HOLD_TIME -- before
        CONTROLLED_DESCENT is allowed to begin. Any excursion outside
        tolerance resets the hold timer. No timeout bypass: if it never
        stabilizes, the vehicle just keeps hovering here.
        """
        start = self.memory.start_position
        target = (start["north"], start["east"], self._agl_to_down(RETURN_ALTITUDE))
        dist, alt_err, speed, v_d = self._nav_error(target)
        self._log_nav("HOME_HOVER", target, dist, alt_err, speed, v_d)

        ok = dist <= HOME_XY_TOLERANCE and alt_err <= HOME_ALT_TOLERANCE and speed <= HOME_VELOCITY_TOLERANCE
        now = time.time()
        if ok:
            if self._home_hover_hold_start is None:
                self._home_hover_hold_start = now
            held = now - self._home_hover_hold_start
            if held >= HOME_HOLD_TIME:
                print(f"[MISSION] HOME HOVER STABLE for {held:.1f}s -- beginning controlled descent.")
                self._transition("CONTROLLED_DESCENT", f"Stable hold at start coordinate for {held:.1f}s.")
        else:
            self._home_hover_hold_start = None

        return FlightIntent(mode="GOTO_NED", target_ned=target, target_heading=start["heading"])

    def _state_controlled_descent(self) -> FlightIntent:
        """
        Steps down through DESCENT_STEPS_AGL one waypoint at a time, holding
        start_north/start_east fixed throughout -- never a single jump to
        the ground. Each step must settle (position + speed within
        tolerance) for DESCENT_STEP_HOLD_TIME before the next, smaller step
        is commanded.
        """
        start = self.memory.start_position

        if self._descent_steps is None:
            cur_agl = self.telemetry.get("alt", 0.0) if self.telemetry else 0.0
            self._descent_steps = [s for s in DESCENT_STEPS_AGL if s <= cur_agl + 1e-6] or [DESCENT_STEPS_AGL[-1]]
            self._descent_step_idx = 0
            self._descent_step_hold_start = None
            print(f"[MISSION] DESCENT STEP SCHEDULE ({len(self._descent_steps)} steps): "
                  f"{[round(s, 2) for s in self._descent_steps]}")

        target_agl = self._descent_steps[self._descent_step_idx]
        target = (start["north"], start["east"], self._agl_to_down(target_agl))
        self._last_descent_target = target

        dist, alt_err, speed, v_d = self._nav_error(target)
        now = time.time()
        if now - self._log_last_descent >= 0.5:
            self._log_last_descent = now
            print(f"[MISSION] DESCENT STEP {self._descent_step_idx + 1}/{len(self._descent_steps)}: "
                  f"target={target_agl:.2f}m AGL  POSITION ERROR: {dist:.3f}m  ALT ERROR: {alt_err:.3f}m  "
                  f"VERTICAL SPEED: {v_d:.3f}m/s  VELOCITY: {speed:.3f}m/s")

        ok = (dist <= DESCENT_STEP_XY_TOLERANCE and alt_err <= DESCENT_STEP_ALT_TOLERANCE
              and speed <= DESCENT_STEP_VELOCITY_TOLERANCE)
        if ok:
            if self._descent_step_hold_start is None:
                self._descent_step_hold_start = now
            held = now - self._descent_step_hold_start
            if held >= DESCENT_STEP_HOLD_TIME:
                if self._descent_step_idx < len(self._descent_steps) - 1:
                    self._descent_step_idx += 1
                    self._descent_step_hold_start = None
                else:
                    print("[MISSION] FINAL DESCENT STEP STABILIZED -- verifying ground contact.")
                    self._transition("GROUND_CONFIRMATION", "Reached final descent waypoint, checking landed state.")
        else:
            self._descent_step_hold_start = None

        return FlightIntent(mode="GOTO_NED", target_ned=target, target_heading=start["heading"])

    def _state_ground_confirmation(self) -> FlightIntent:
        """
        Authoritative touchdown check: PX4's own landed-state detector
        (fuses accel/thrust/etc, not a bare altitude threshold) must report
        ON_GROUND, together with near-zero speed and a sane local-position
        estimate, continuously, for GROUND_CONFIRM_HOLD_TIME. No timeout
        bypass -- if the vehicle never reports landed, this state simply
        keeps holding position instead of proceeding to
        OFFBOARD_EXIT/DISARM/KILL while still airborne.
        """
        start = self.memory.start_position
        target = self._last_descent_target or (start["north"], start["east"], self._agl_to_down(DESCENT_STEPS_AGL[-1]))

        tel = self.telemetry or {}
        landed_state = tel.get("landed_state")
        local_pos_ok = bool(tel.get("local_position_ok"))
        _, _, speed, v_d = self._nav_error(target)
        alt = tel.get("alt", 999.0)

        now = time.time()
        if now - self._log_last_ground >= 0.5:
            self._log_last_ground = now
            print(f"[MISSION] GROUND CHECK: landed_state={landed_state}  alt={alt:.3f}m  "
                  f"VERTICAL SPEED: {v_d:.3f}m/s  VELOCITY: {speed:.3f}m/s  local_position_ok={local_pos_ok}")

        ok = (landed_state == "ON_GROUND" and speed <= GROUND_CONFIRM_VELOCITY_TOLERANCE and local_pos_ok)
        if ok:
            if self._ground_confirm_hold_start is None:
                self._ground_confirm_hold_start = now
            held = now - self._ground_confirm_hold_start
            if held >= GROUND_CONFIRM_HOLD_TIME:
                print(f"[MISSION] GROUND CONTACT CONFIRMED (stable for {held:.1f}s)")
                ned = tel.get("ned") or (0.0, 0.0, 0.0)
                self._final_landing_snapshot = {
                    "north": ned[0],
                    "east": ned[1],
                    "alt": tel.get("alt", 0.0),
                    "heading": tel.get("yaw", 0.0),
                }
                self._transition("OFFBOARD_EXIT", f"Landed state ON_GROUND, stable for {held:.1f}s.")
        else:
            self._ground_confirm_hold_start = None

        return FlightIntent(mode="GOTO_NED", target_ned=target, target_heading=start["heading"])

    def _state_offboard_exit(self) -> FlightIntent:
        """
        Only ever entered after GROUND_CONFIRMATION has verified touchdown.
        Stops OFFBOARD (PX4 drops to Hold mode) and waits for telemetry to
        confirm the mode actually changed before handing off to DISARM.
        """
        if not self._offboard_exit_requested:
            self._offboard_exit_requested = True
            print("[MISSION] OFFBOARD EXIT")
            return FlightIntent(mode="OFFBOARD_STOP")

        if self.telemetry and not self.telemetry.get("offboard", True):
            self._transition("DISARM", "Offboard mode exited.")

        return FlightIntent(mode="IDLE")

    def _state_disarm(self) -> FlightIntent:
        """
        First choice for stopping the motors once landed. GROUND_CONFIRMATION
        has already verified touchdown before this state is ever reached, so
        PX4 rejecting a plain disarm here means something PX4-side disagrees
        -- not "still flying by design". KILL is only used if this fails.
        """
        if not self._disarm_pending:
            self._disarm_pending = True
            print("[MISSION] DISARM ATTEMPT")
            return FlightIntent(mode="DISARM")

        if self._disarm_result is None:
            return FlightIntent(mode="IDLE")

        if self._disarm_result:
            print("[MISSION] DISARM RESULT: SUCCESS")
            self._transition("MISSION_COMPLETE", "Disarmed on ground.")
        else:
            print(f"[MISSION] DISARM RESULT: FAILED ({self._disarm_fail_reason}) -- falling back to KILL.")
            self._transition("KILL", "Disarm rejected after ground confirmation; using kill as last resort.")

        return FlightIntent(mode="IDLE")

    def _state_kill(self) -> FlightIntent:
        """
        Last resort ONLY: reached solely because DISARM was attempted and
        rejected after GROUND_CONFIRMATION already verified touchdown.
        Re-checks landed_state itself before sending KILL regardless of how
        this state was entered -- refuses to kill motors while telemetry
        says the vehicle is airborne.
        """
        landed_state = self.telemetry.get("landed_state") if self.telemetry else None
        if landed_state != "ON_GROUND":
            print(f"[MISSION] CRITICAL: KILL requested but landed_state={landed_state} -- "
                  f"refusing, re-verifying ground contact.")
            self._ground_confirm_hold_start = None
            self._transition("GROUND_CONFIRMATION", "Landed state no longer confirmed before kill.")
            return FlightIntent(mode="IDLE")

        if not self._kill_pending:
            self._kill_pending = True
            print("[MISSION] KILL ATTEMPT (last resort, ground-confirmed)")
            return FlightIntent(mode="KILL")

        if self._kill_result is None:
            return FlightIntent(mode="IDLE")

        if self._kill_result:
            print("[MISSION] KILL RESULT: SUCCESS")
        else:
            print(f"[MISSION] KILL RESULT: FAILED ({self._kill_fail_reason}) -- manual intervention required.")
        self._transition("MISSION_COMPLETE", "Kill executed after disarm rejection.")
        return FlightIntent(mode="IDLE")

    def _state_mission_complete(self) -> FlightIntent:
        if not self._mission_complete_logged:
            self._mission_complete_logged = True
            print("[MISSION] MISSION COMPLETE")

        if not self._final_report_printed and self._final_landing_snapshot is not None:
            self._final_report_printed = True
            self._print_final_validation_report()

        return FlightIntent(mode="IDLE")

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
            # altitude (~0.30m AGL), which would always differ by that much
            # since start was recorded airborne and landing is on the
            # ground; that would make this check meaningless by construction.
            vertical_error = abs(final["alt"] - 0.0)
            heading_error = abs((start["heading"] - final["heading"] + 180) % 360 - 180)

            print(f"Horizontal Error: {horizontal_error:.3f} m")
            print(f"Vertical Error:   {vertical_error:.3f} m")
            print(f"Heading Error:    {heading_error:.2f} deg")
            print(f"Mission Duration: {duration:.1f} s")
            print("=================================")

            accepted = (horizontal_error <= FINAL_REPORT_XY_TOLERANCE and
                        vertical_error <= FINAL_REPORT_ALT_TOLERANCE and
                        heading_error <= FINAL_REPORT_HEADING_TOLERANCE)
            print(f"[MISSION] {'ACCEPTED' if accepted else 'REJECTED'} "
                  f"(H<={FINAL_REPORT_XY_TOLERANCE}m, V<={FINAL_REPORT_ALT_TOLERANCE}m, "
                  f"Heading<={FINAL_REPORT_HEADING_TOLERANCE}deg)")
        else:
            print("Horizontal Error: N/A")
            print("Vertical Error:   N/A")
            print("Heading Error:    N/A")
            print(f"Mission Duration: {duration:.1f} s")
            print("=================================")
            print("[MISSION] REJECTED (no start position recorded)")

    def _state_emergency_abort(self) -> FlightIntent:
        return FlightIntent(mode="HOVER")
