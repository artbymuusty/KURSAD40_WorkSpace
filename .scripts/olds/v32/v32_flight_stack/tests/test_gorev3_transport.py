import pytest

from mocks.mock_flight_backend import MockFlightBackend

from core.mission.gorev3_transport import Gorev3TransportPhase
from core.position_log.position_store import PositionStore
from core.config.parameters import GOREV3_TRANSIT_ALTITUDE_M, GOREV3_TRANSIT_SPEED_M_S


class _RecordingCentering:
    def __init__(self, converges: bool = True):
        self.calls = []
        self._converges = converges

    async def goto_global_position_and_wait(self, lat, lon, alt) -> bool:
        self.calls.append((lat, lon, alt))
        return self._converges


@pytest.mark.asyncio
async def test_transport_raises_without_recorded_kirmizi_ucgen(tmp_path):
    flight = MockFlightBackend()
    store = PositionStore(str(tmp_path / "positions.json"))
    centering = _RecordingCentering()
    phase = Gorev3TransportPhase(flight, store, centering)

    with pytest.raises(RuntimeError):
        await phase.run()


@pytest.mark.asyncio
async def test_transport_navigates_to_recorded_kirmizi_ucgen_position(tmp_path):
    """BUG FIX (continuous audit, 2026-08-13): this phase used to hold
    north=0/east=0 -- never actually navigating to the recorded position at
    all. Now it must actually go there."""
    flight = MockFlightBackend()
    store = PositionStore(str(tmp_path / "positions.json"))
    store.try_save("KIRMIZI_UCGEN", 0.9, True, True, (41.5, 29.5, 15.0), "ilk")
    centering = _RecordingCentering()

    phase = Gorev3TransportPhase(flight, store, centering)
    await phase.run()

    assert centering.calls == [(41.5, 29.5, GOREV3_TRANSIT_ALTITUDE_M)]


@pytest.mark.asyncio
async def test_transport_still_completes_when_navigation_times_out(tmp_path):
    flight = MockFlightBackend()
    store = PositionStore(str(tmp_path / "positions.json"))
    store.try_save("KIRMIZI_UCGEN", 0.9, True, True, (41.5, 29.5, 15.0), "ilk")
    centering = _RecordingCentering(converges=False)

    phase = Gorev3TransportPhase(flight, store, centering)
    await phase.run()  # must not raise

    assert len(centering.calls) == 1


def test_transit_speed_is_explicitly_configured_not_none():
    # Operator revision (2026-08-13): "2 m/s seyir hızı" -- explicitly
    # authorized, must not silently regress back to an unfilled TODO.
    assert GOREV3_TRANSIT_SPEED_M_S == 2.0
