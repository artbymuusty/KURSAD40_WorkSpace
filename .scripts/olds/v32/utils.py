# utils.py
import math

def clamp(value, min_val, max_val):
    """Clamps a value between min_val and max_val."""
    return max(min_val, min(value, max_val))

def get_adaptive_threshold(altitude):
    """
    Returns adaptive pixel threshold for centering based on altitude.
    - altitude > 8m: threshold = 40 px
    - altitude between 5m and 8m: threshold = 25 px
    - altitude below 5m: threshold = 12 px
    """
    if altitude > 8.0:
        return 40.0
    elif 5.0 <= altitude <= 8.0:
        return 25.0
    else:
        return 12.0

def distance_ned(ned1, ned2):
    """Calculates 2D horizontal distance between two NED coordinates."""
    if not ned1 or not ned2:
        return float('inf')
    return math.hypot(ned1[0] - ned2[0], ned1[1] - ned2[1])

def distance_px(p1, p2):
    """Calculates Euclidean distance between two pixel points."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def make_target_key(det: dict) -> str:
    """Builds canonical underscore target key from a raw detection dict."""
    return f"{det['color']}_{det['name']}"

def selected_payload_for_target_key(target_key: str):
    """Maps target key to the payload color that should be dropped. Returns None for unknown keys."""
    if target_key == "blue_hexagon":
        return "red"
    if target_key == "red_triangle":
        return "blue"
    return None

def normalize_detection(det: dict):
    """
    Validates a raw vision detection dict and attaches target_key + selected_payload.
    Returns None if the detection is invalid or not a known target type.
    Does not mutate the original dict.
    """
    if det is None:
        return None
    if not det.get("center") or not det.get("color") or not det.get("name"):
        return None
    key = make_target_key(det)
    payload = selected_payload_for_target_key(key)
    if payload is None:
        return None  # unknown target type (e.g. circles in TARGET_MODE)
    out = dict(det)
    out["target_key"] = key
    out["selected_payload"] = payload
    return out

def get_current_or_last_target(current_target, last_valid_target_info, last_valid_target_time,
                                grace_seconds: float, now: float):
    """
    Returns (target_to_use, using_last_valid) tuple.
    - If current_target is valid: returns (current_target, False).
    - If current_target is None but last_valid exists within grace_seconds: returns (last_valid, True).
    - Otherwise: returns (None, False).
    """
    if current_target is not None:
        return current_target, False
    age = (now - last_valid_target_time) if last_valid_target_time > 0 else float('inf')
    if last_valid_target_info is not None and age <= grace_seconds:
        return last_valid_target_info, True
    return None, False

def payload_for_target(target_key: str) -> str:
    """Legacy alias. Raises ValueError for unknown keys."""
    result = selected_payload_for_target_key(target_key)
    if result is None:
        raise ValueError(f"Unknown target_key: {target_key!r}")
    return result
