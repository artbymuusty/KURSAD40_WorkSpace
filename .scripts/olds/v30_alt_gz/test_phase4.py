# test_phase4.py
import unittest
import time
from config import *
from mission import MissionManager
from memory import MissionMemory
from search import SearchPlanner
from servo import VisualServoController
from target_selector import TargetSelector
from mission_types import TargetData

class TestPhase4Validation(unittest.TestCase):
    def setUp(self):
        self.memory = MissionMemory()
        self.search = SearchPlanner()
        self.servo = VisualServoController()
        self.manager = MissionManager(self.memory, self.search, self.servo)
        
    def test_scenario_5_mission_filter_transition(self):
        """Verify the Dynamic Detection Filter changes correctly."""
        # 1. Mission Start -> Active: blue_hexagon, red_triangle
        # active_filter is seeded by _state_boot() (the manager starts in BOOT),
        # so run one tick there before forcing the state to SEARCH.
        self.manager.step()
        self.manager._transition("SEARCH", "Starting search")
        active = self.manager.state_data.active_filter
        self.assertIn("blue_hexagon", active)
        self.assertIn("red_triangle", active)
        
        # 2. After blue_hexagon drop -> completely ignored
        # completed_targets alone means "attempted/disabled"; confirmed_drops
        # is the real DROP_STATE-verified signal PayloadManager reports (see
        # mission.py's _state_payload_drop) -- bonus pickup now requires the
        # latter, so this simulated drop must set both to match a real one.
        self.memory.completed_targets.append("blue_hexagon")
        self.memory.confirmed_drops.append("red")  # blue_hexagon target -> red payload
        self.manager._transition("SEARCH_RESUME")
        self.manager.step() # Trigger _state_search_resume which updates active_filter
        active = self.manager.state_data.active_filter
        self.assertNotIn("blue_hexagon", active)

        # 3. After red_triangle drop -> Search ends
        self.memory.completed_targets.append("red_triangle")
        self.memory.confirmed_drops.append("blue")  # red_triangle target -> blue payload
        self.manager._transition("ASCEND_AFTER_DROP")
        # ASCEND_AFTER_DROP logic checks length of completed targets
        self.manager.update_telemetry({"alt": 20.0, "ned": (0,0,0), "gps": (0,0)})
        self.manager.step() # Triggers transition to PICKUP_PREPARATION
        self.assertEqual(self.manager.state_data.current_state, "PICKUP_PREPARATION")
        
        # 4. Pickup phase -> Only required pickup target enabled
        # If blue payload was dropped first, we need to pick up blue_rectangle
        self.memory.first_dropped_payload_color = "blue"
        pickup_target = self.memory.get_target_for_pickup()
        self.assertEqual(pickup_target, "blue_payload_target")

    def test_scenario_6_search_resume(self):
        """Verify temporary target loss does not permanently terminate search."""
        # Seed active_filter via _state_boot() before forcing SEARCH, otherwise
        # _state_search can never match target_data against an empty filter.
        self.manager.step()
        self.manager._transition("SEARCH", "Init")
        
        # Simulate target detected
        det: TargetData = {"target_key": "blue_hexagon", "name": "hexagon", "color": "blue", "confidence": 0.8, "center": (320, 240), "bounding_box": (0,0,0,0), "tracking_id": 1, "is_locked": True, "distance_from_center": 0, "pixel_error": (0,0), "is_stale": False, "timestamp": time.time()}
        self.manager.update_vision(det, (640, 480))
        self.manager.step() # SEARCH -> TARGET_DETECTED
        # _state_target_detected requires target_locked_frames >= 3 (a 3-frame
        # debounce against flicker) before locking on to TARGET_TRACKING -- that
        # takes 3 more step() calls from here, not 2.
        self.manager.step() # target_locked_frames = 1
        self.manager.step() # target_locked_frames = 2
        self.manager.step() # target_locked_frames = 3 -> Transitions to TARGET_TRACKING
        self.assertEqual(self.manager.state_data.current_state, "TARGET_TRACKING")
        
        # Target Lost
        self.manager.update_vision(None, (640, 480))
        # simulate time passing
        self.manager.last_seen_time = time.time() - 6.0
        self.manager.step()
        # Should transition back to search if no stale tracking
        self.assertEqual(self.manager.state_data.current_state, "SEARCH")

    def test_scenario_8_step_descent(self):
        """Verify every descent gate stops and aligns."""
        self.manager._transition("STEP_DESCENT", "Init")
        self.manager.descent_gate = 13.0
        self.manager.last_seen_time = time.time() # Prevent timeout
        
        det: TargetData = {"target_key": "blue_hexagon", "name": "hexagon", "color": "blue", "confidence": 0.8, "center": (320, 240), "bounding_box": (0,0,0,0), "tracking_id": 1, "is_locked": True, "distance_from_center": 0, "pixel_error": (0,0), "is_stale": False, "timestamp": time.time()}
        self.manager.update_vision(det, (640, 480))
        
        # Altimeter > Gate -> Descend
        self.manager.update_telemetry({"alt": 14.0, "ned": (0,0,0), "gps": (0,0)})
        intent = self.manager.step()
        self.assertAlmostEqual(intent.velocity[2], DESCENT_SPEED, places=1)
        
        # Altimeter <= Gate -> Stop descent, transition to PRECISION_ALIGNMENT
        self.manager.update_telemetry({"alt": 12.9, "ned": (0,0,0), "gps": (0,0)})
        intent = self.manager.step()
        self.assertEqual(intent.velocity[2], 0.0) # Descent halts
        self.assertEqual(self.manager.state_data.current_state, "PRECISION_ALIGNMENT")
        
if __name__ == '__main__':
    unittest.main()
