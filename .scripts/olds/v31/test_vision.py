# test_vision.py
import unittest
import time
from target_selector import TargetSelector
from vision_interfaces import DetectorBackend, TrackerBackend
from config import CONFIDENCE_THRESHOLD_NEW, CONFIDENCE_THRESHOLD_TRACK, GRACE_PERIOD_SEC
from mission_types import TargetData

class TestTargetSelector(unittest.TestCase):
    def setUp(self):
        self.selector = TargetSelector()
        self.frame_shape = (640, 480) # w, h
        
    def test_selection_policy_closest_to_center(self):
        cx, cy = 320, 240
        
        det1 = {
            "class_name": "blue_hexagon", "confidence": 0.8,
            "center": (100, 100), "bbox": (0,0,0,0), "tracking_id": 1
        }
        det2 = {
            "class_name": "red_triangle", "confidence": 0.7,
            "center": (300, 250), "bbox": (0,0,0,0), "tracking_id": 2
        }
        
        best = self.selector.select([det1, det2], self.frame_shape)
        self.assertIsNotNone(best)
        self.assertEqual(best["target_key"], "red_triangle")

    def test_confidence_hysteresis(self):
        # Establish lock on tracking_id = 1 (blue_hexagon)
        self.selector.locked_tracking_id = 1
        
        det_weak_lock = {
            "class_name": "blue_hexagon", "confidence": 0.50, # Below 0.60 new, above 0.45 track
            "center": (100, 100), "bbox": (0,0,0,0), "tracking_id": 1
        }
        det_strong_new = {
            "class_name": "red_triangle", "confidence": 0.55, # Below 0.60 new
            "center": (300, 250), "bbox": (0,0,0,0), "tracking_id": 2
        }
        
        best = self.selector.select([det_weak_lock, det_strong_new], self.frame_shape)
        self.assertIsNotNone(best)
        self.assertEqual(best["target_key"], "blue_hexagon")

    def test_grace_period_stale_tracking(self):
        # Setup existing lock
        valid_det = {
            "class_name": "blue_hexagon", "confidence": 0.8,
            "center": (300, 200), "bbox": (0,0,0,0), "tracking_id": 1
        }
        best1 = self.selector.select([valid_det], self.frame_shape)
        self.assertIsNotNone(best1)
        self.assertFalse(best1["is_stale"])
        
        # Next frame: empty detection list
        best2 = self.selector.select([], self.frame_shape)
        self.assertIsNotNone(best2)
        self.assertTrue(best2["is_stale"])
        self.assertEqual(best2["tracking_id"], 1)
        
        # Fast forward time beyond grace period
        self.selector.last_seen_time -= (GRACE_PERIOD_SEC + 0.5)
        best3 = self.selector.select([], self.frame_shape)
        self.assertIsNone(best3)

    def test_reset_lock(self):
        self.selector.locked_tracking_id = 1
        self.selector.last_locked_det = {"tracking_id": 1}
        self.selector.reset_lock()
        self.assertIsNone(self.selector.locked_tracking_id)
        self.assertIsNone(self.selector.last_locked_det)

if __name__ == '__main__':
    unittest.main()
