import pytest
import os
import json
from core.position_log.position_store import PositionStore
from core.config.parameters import YOLO_CONFIDENCE_THRESHOLD

def test_rejects_low_confidence(tmp_path):
    store = PositionStore(str(tmp_path / "test.json"))
    res = store.try_save("MAVI_ALTIGEN", YOLO_CONFIDENCE_THRESHOLD - 0.1, True, True, (0,0,0), "ilk")
    assert res is None
    assert len(store.all_points()) == 0

def test_rejects_not_centered(tmp_path):
    store = PositionStore(str(tmp_path / "test.json"))
    res = store.try_save("MAVI_ALTIGEN", 0.9, False, True, (0,0,0), "ilk")
    assert res is None

def test_rejects_hover_incomplete(tmp_path):
    store = PositionStore(str(tmp_path / "test.json"))
    res = store.try_save("MAVI_ALTIGEN", 0.9, True, False, (0,0,0), "ilk")
    assert res is None

def test_accepts_all_conditions_met(tmp_path):
    file_path = str(tmp_path / "test.json")
    store = PositionStore(file_path)
    res = store.try_save("MAVI_ALTIGEN", 0.9, True, True, (41.0, 29.0, 15.0), "ilk")
    
    assert res is not None
    assert res.shape_type == "MAVI_ALTIGEN"
    
    assert os.path.exists(file_path)
    with open(file_path, "r") as f:
        data = json.load(f)
        assert len(data) == 1
        assert data[0]["shape_type"] == "MAVI_ALTIGEN"
