import pytest

from mocks.mock_payload_actuator import MockPayloadActuator

from core.mission.gorev3_redrop import Gorev3RedropPhase
from core.position_log.position_store import PositionStore
from core.config.parameters import GOREV3_DESCENT_ALTITUDE_M


class _RecordingCentering:
    def __init__(self, converges: bool = True):
        self.calls = []
        self._converges = converges

    async def go_to_and_center(self, shape_type: str, altitude_m: float,
                               alt_tolerance_m: float = None, aim_offset_body_m=None) -> bool:
        self.calls.append(('go_to_and_center', shape_type, altitude_m))
        return self._converges


@pytest.mark.asyncio
async def test_redrop_raises_without_recorded_kirmizi_ucgen(tmp_path):
    actuator = MockPayloadActuator()
    store = PositionStore(str(tmp_path / "positions.json"))
    centering = _RecordingCentering()
    phase = Gorev3RedropPhase(flight=None, actuator=actuator, position_store=store, centering=centering)

    with pytest.raises(RuntimeError):
        await phase.run()


@pytest.mark.asyncio
async def test_redrop_centers_at_descent_altitude_then_triggers_grab_servo(tmp_path):
    actuator = MockPayloadActuator()
    store = PositionStore(str(tmp_path / "positions.json"))
    store.try_save("KIRMIZI_UCGEN", 0.9, True, True, (41.0, 29.0, 15.0), "ilk")
    centering = _RecordingCentering()

    phase = Gorev3RedropPhase(flight=None, actuator=actuator, position_store=store, centering=centering)
    result = await phase.run()

    assert result is True
    assert centering.calls == [('go_to_and_center', 'KIRMIZI_UCGEN', GOREV3_DESCENT_ALTITUDE_M)]
    assert ('activate_drop_mechanism', {}) in actuator.calls


@pytest.mark.asyncio
async def test_redrop_still_drops_when_final_centering_does_not_converge(tmp_path):
    """Best-effort: a failed final re-centering must not abort the drop."""
    actuator = MockPayloadActuator()
    store = PositionStore(str(tmp_path / "positions.json"))
    store.try_save("KIRMIZI_UCGEN", 0.9, True, True, (41.0, 29.0, 15.0), "ilk")
    centering = _RecordingCentering(converges=False)

    phase = Gorev3RedropPhase(flight=None, actuator=actuator, position_store=store, centering=centering)
    result = await phase.run()

    assert result is True
    assert ('activate_drop_mechanism', {}) in actuator.calls
