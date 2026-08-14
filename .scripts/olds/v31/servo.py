# servo.py
import math
import time
from typing import Optional, Tuple
from mission_types import FlightIntent

class VisualServoController:
    """
    Dedicated controller for calculating visual servoing velocities.
    MissionManager requests an alignment, and this outputs the FlightIntent.
    """
    def __init__(self):
        self.kp_xy = 2.0
        self.max_xy_speed = 4.8
        self.align_threshold_px = 30
        
        self.aligned_frames = 0
        self.last_seen_time = 0.0

    def get_servo_intent(self, target_data: dict, frame_wh: Tuple[int, int], target_locked: bool = True) -> FlightIntent:
        """
        Calculates the velocity required to center the target in the frame.
        """
        if not target_data:
            return FlightIntent(mode="HOVER")
            
        self.last_seen_time = time.time()
        
        # Use pre-computed pixel error if available (TargetData guarantees this natively now via Phase 3)
        if "pixel_error" in target_data:
            dx, dy = target_data["pixel_error"]
            dist_px = target_data.get("distance_from_center", 0.0)
        else:
            tx, ty = target_data["center"]
            cx, cy = frame_wh[0] // 2, frame_wh[1] // 2
            
            dx = (tx - cx) / float(cx)
            dy = (cy - ty) / float(cy)
            
            dist_px = math.hypot(tx - cx, ty - cy)
        
        if dist_px <= self.align_threshold_px:
            self.aligned_frames += 1
        else:
            self.aligned_frames = 0
            
        v_right = max(-self.max_xy_speed, min(self.max_xy_speed, self.kp_xy * dx))
        v_fwd = max(-self.max_xy_speed, min(self.max_xy_speed, self.kp_xy * dy))
        
        # Phase 5 Heading Hold logic: Target orientation is not given, yaw_rate is strictly 0.0
        return FlightIntent(mode="VELOCITY_BODY", velocity=(v_fwd, v_right, 0.0), yaw_rate=0.0)

    def is_aligned(self, required_frames: int = 15) -> bool:
        """Returns True if the system has been continuously aligned for the required duration."""
        return self.aligned_frames >= required_frames

    def reset_alignment(self):
        self.aligned_frames = 0
