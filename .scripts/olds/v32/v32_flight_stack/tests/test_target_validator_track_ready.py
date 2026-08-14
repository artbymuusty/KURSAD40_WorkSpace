"""
Regression guard for the operator-reported Mission -> Offboard handover
bug: is_track_ready() used to require the target to already be centered
(within 20px) on a raw, uncontrolled frame before pursuit could even begin
-- an almost-never-true condition during real Mission-mode flight, since
nothing is actively centering anything yet at that point. Centering is
step 9 of the state machine, which comes AFTER the Offboard switch (step 7)
and Go To Target (step 8) -- so it cannot also be a precondition for step 6.
"""
from core.detection.target_validator import TargetValidator
from core.detection.types import Detection


def _detection_far_from_center(shape_type="MAVI_ALTIGEN"):
    # 300px away from an assumed 320,240 frame center -- would have failed
    # the old is_centered(<=20px) precondition.
    return Detection(shape_type=shape_type, confidence=0.9, center_px=(620.0, 240.0), bbox_px=(600, 220, 640, 260))


def test_track_ready_does_not_require_pre_centering():
    validator = TargetValidator(min_consecutive_frames=3, center_tolerance_px=20.0)
    frame_center = (320.0, 240.0)
    det = _detection_far_from_center()

    for _ in range(3):
        validator.update(det, current_altitude_m=15.0, frame_center=frame_center, target_altitude_m=15.0)

    assert validator.get_track_state("MAVI_ALTIGEN")["is_centered"] is False  # genuinely not centered
    assert validator.is_track_ready("MAVI_ALTIGEN") is True  # but track-ready anyway


def test_track_ready_still_requires_consecutive_frames():
    validator = TargetValidator(min_consecutive_frames=5, center_tolerance_px=20.0)
    det = _detection_far_from_center()
    validator.update(det, current_altitude_m=15.0, frame_center=(320.0, 240.0))
    assert validator.is_track_ready("MAVI_ALTIGEN") is False


def test_track_ready_still_requires_correct_altitude():
    validator = TargetValidator(min_consecutive_frames=1, center_tolerance_px=20.0)
    det = _detection_far_from_center()
    validator.update(det, current_altitude_m=3.0, frame_center=(320.0, 240.0), target_altitude_m=15.0)
    assert validator.is_track_ready("MAVI_ALTIGEN") is False


def test_is_validated_still_requires_all_four_conditions_including_centered():
    # is_validated() (the FINAL post-hover check) intentionally keeps the
    # centered requirement -- by that point go_to_and_center() should have
    # actually achieved it. Only the PRE-pursuit gate (is_track_ready) changed.
    validator = TargetValidator(min_consecutive_frames=1, center_tolerance_px=20.0)
    det = _detection_far_from_center()
    validator.update(det, current_altitude_m=15.0, frame_center=(320.0, 240.0))
    validator.set_navigating_to("MAVI_ALTIGEN", True)
    assert validator.is_validated("MAVI_ALTIGEN") is False  # still not centered -> still not validated
