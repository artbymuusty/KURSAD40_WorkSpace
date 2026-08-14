# target_selector.py
import time
import math
from typing import List, Dict, Optional, Tuple
from config import CONFIDENCE_THRESHOLD_NEW, CONFIDENCE_THRESHOLD_TRACK, GRACE_PERIOD_SEC
from mission_types import TargetData, TrackedDetection

class TargetSelector:
    """
    Evaluates tracked detections and selects the single active target for the mission.
    Maintains target lock and grace period logic.
    """
    def __init__(self):
        self.locked_tracking_id: Optional[int] = None
        self.last_locked_det: Optional[TrackedDetection] = None
        self.last_seen_time: float = 0.0

    def select(self, tracked_detections: List[TrackedDetection], frame_shape: Tuple[int, int]) -> Optional[TargetData]:
        now = time.time()
        cx, cy = frame_shape[0] // 2, frame_shape[1] // 2
        
        valid_candidates = []
        
        for det in tracked_detections:
            # Hysteresis Check
            if self.locked_tracking_id is not None and det.get("tracking_id") == self.locked_tracking_id:
                if det["confidence"] >= CONFIDENCE_THRESHOLD_TRACK:
                    valid_candidates.append(det)
            else:
                if det["confidence"] >= CONFIDENCE_THRESHOLD_NEW:
                    valid_candidates.append(det)
                    
        if valid_candidates:
            # Sort by distance to center
            valid_candidates.sort(key=lambda d: math.hypot(d["center"][0] - cx, d["center"][1] - cy))
            best_raw = valid_candidates[0]
            
            self.locked_tracking_id = best_raw.get("tracking_id", 0)
            self.last_locked_det = best_raw
            self.last_seen_time = now
            
            return self._build_target_data(best_raw, frame_shape, is_stale=False, is_locked=True, ts=now)
            
        else:
            # Grace period logic
            if self.last_locked_det is not None:
                if now - self.last_seen_time <= GRACE_PERIOD_SEC:
                    return self._build_target_data(self.last_locked_det, frame_shape, is_stale=True, is_locked=True, ts=now)
                else:
                    self.locked_tracking_id = None
                    self.last_locked_det = None
                    
        return None
        
    def _build_target_data(self, raw_det: TrackedDetection, frame_shape: Tuple[int, int], is_stale: bool, is_locked: bool, ts: float) -> TargetData:
        cx, cy = frame_shape[0] // 2, frame_shape[1] // 2
        tx, ty = raw_det["center"]
        
        dx = (tx - cx) / float(cx)
        dy = (cy - ty) / float(cy)
        
        dist = math.hypot(tx - cx, ty - cy)
        
        # Parse shape and color
        parts = raw_det["class_name"].split("_")
        color = parts[0] if len(parts) > 1 else "unknown"
        shape = parts[1] if len(parts) > 1 else raw_det["class_name"]
        
        return {
            "target_key": raw_det["class_name"],
            "name": shape,
            "color": color,
            "confidence": raw_det["confidence"],
            "center": (tx, ty),
            "bounding_box": raw_det["bbox"],
            "polygon": raw_det.get("polygon"),
            "tracking_id": raw_det.get("tracking_id", 0),
            "is_locked": is_locked,
            "distance_from_center": dist,
            "pixel_error": (dx, dy),
            "is_stale": is_stale,
            "timestamp": ts
        }

    def reset_lock(self):
        self.locked_tracking_id = None
        self.last_locked_det = None
