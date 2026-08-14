import pytest
from core.navigation.checkpoint import MissionCheckpoint

def test_get_before_save_raises():
    checkpoint = MissionCheckpoint()
    with pytest.raises(RuntimeError) as exc_info:
        checkpoint.get()
    assert "KRİTİK IHLAL" in str(exc_info.value)

def test_get_after_save_returns_value():
    checkpoint = MissionCheckpoint()
    checkpoint.save(41.0, 29.0, 15.0)
    assert checkpoint.get() == (41.0, 29.0, 15.0)
    assert checkpoint.is_saved() is True
