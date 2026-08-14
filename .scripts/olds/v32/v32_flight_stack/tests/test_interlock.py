import pytest
from core.mission.interlock import PayloadInterlock

def test_payload_2_cannot_release_before_payload_1():
    interlock = PayloadInterlock()
    with pytest.raises(RuntimeError) as exc_info:
        interlock.mark_payload_2_released()
    assert "INTERLOCK IHLALI" in str(exc_info.value)

def test_payload_2_can_release_after_payload_1():
    interlock = PayloadInterlock()
    interlock.mark_payload_1_released()
    interlock.mark_payload_2_released() # Shouldn't raise
    assert interlock.both_released() is True

def test_can_release_payload_2_reflects_state():
    interlock = PayloadInterlock()
    assert interlock.can_release_payload_2() is False
    interlock.mark_payload_1_released()
    assert interlock.can_release_payload_2() is True
