import pytest

from mocks.mock_flight_backend import MockFlightBackend
from mocks.mock_camera_source import MockCameraSource
from mocks.mock_payload_actuator import MockPayloadActuator

from core.detection.types import Detection
from core.mission.gorev3_pickup import Gorev3PickupPhase
from core.mission.rectangle_alignment_strategy import RectangleAlignmentStrategy
from core.position_log.position_store import PositionStore
from core.config.parameters import GOREV3_TRANSIT_ALTITUDE_M


class _RectangleUntilPickedUpDetector:
    """Reports a fixed KIRMIZI_DIKDORTGEN (with rotation_deg) until
    `picked_up` flips True, then reports nothing -- simulates a successful
    physical pickup (the shape stops being visible on the ground)."""
    def __init__(self):
        self.picked_up = False

    async def detect(self, frame):
        if self.picked_up:
            return []
        return [Detection(shape_type="KIRMIZI_DIKDORTGEN", confidence=0.9,
                          center_px=(320, 240), bbox_px=(300, 220, 340, 260), rotation_deg=15.0)]


class _PickupTriggeringActuator(MockPayloadActuator):
    def __init__(self, detector: _RectangleUntilPickedUpDetector):
        super().__init__()
        self._detector = detector

    async def activate_pickup_mechanism(self) -> bool:
        result = await super().activate_pickup_mechanism()
        self._detector.picked_up = True  # THIRD MISSION SERVO succeeded -- shape leaves the ground
        return result


class _RecordingCentering:
    def __init__(self, converges: bool = True):
        self.calls = []
        self._converges = converges

    async def goto_global_position_and_wait(self, lat, lon, alt) -> bool:
        self.calls.append((lat, lon, alt))
        return self._converges


def _build_phase(flight, camera, detector, actuator, store, centering=None):
    return Gorev3PickupPhase(flight, camera, detector, actuator, store, RectangleAlignmentStrategy(),
                             centering or _RecordingCentering())


@pytest.mark.asyncio
async def test_pickup_raises_without_recorded_mavi_altigen(tmp_path):
    flight = MockFlightBackend()
    camera = MockCameraSource()
    detector = _RectangleUntilPickedUpDetector()
    actuator = _PickupTriggeringActuator(detector)
    store = PositionStore(str(tmp_path / "positions.json"))
    phase = _build_phase(flight, camera, detector, actuator, store)

    with pytest.raises(RuntimeError):
        await phase.run()


@pytest.mark.asyncio
async def test_pickup_full_sequence_succeeds_and_confirms_shape_gone(tmp_path):
    flight = MockFlightBackend()
    camera = MockCameraSource()
    detector = _RectangleUntilPickedUpDetector()
    actuator = _PickupTriggeringActuator(detector)
    store = PositionStore(str(tmp_path / "positions.json"))
    store.try_save("MAVI_ALTIGEN", 0.9, True, True, (41.0, 29.0, 15.0), "ilk")
    centering = _RecordingCentering()

    phase = _build_phase(flight, camera, detector, actuator, store, centering)
    result = await phase.run()

    assert result is True
    assert ('activate_pickup_mechanism', {}) in actuator.calls
    # Real navigation to the recorded Mavi Altıgen GPS position (BUG FIX,
    # continuous audit 2026-08-13) -- previously this never happened at all.
    assert centering.calls == [(41.0, 29.0, GOREV3_TRANSIT_ALTITUDE_M)]
    hold_calls = [c for c in flight.calls if c[0] == 'goto_position_ned_and_hold']
    assert len(hold_calls) >= 3  # align, retreat, advance (+ climb steps)


@pytest.mark.asyncio
async def test_pickup_fails_when_rectangle_never_found(tmp_path):
    flight = MockFlightBackend()
    camera = MockCameraSource()

    class _NeverFindsDetector:
        async def detect(self, frame):
            return []

    detector = _NeverFindsDetector()
    actuator = MockPayloadActuator()
    store = PositionStore(str(tmp_path / "positions.json"))
    store.try_save("MAVI_ALTIGEN", 0.9, True, True, (41.0, 29.0, 15.0), "ilk")

    phase = _build_phase(flight, camera, detector, actuator, store)
    result = await phase.run()

    assert result is False
    assert actuator.calls == []  # never reached the servo trigger
