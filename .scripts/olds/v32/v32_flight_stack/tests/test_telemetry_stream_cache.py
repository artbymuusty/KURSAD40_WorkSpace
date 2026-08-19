"""
ADR-008 B0 -- root cause 3 of the 2026-08-16 investigation.

MavsdkBackendBase used to open a fresh `async for ... in
telemetry.position()` subscription on EVERY get_global_position() call and
return its first pushed value. PX4's default position rate is 1 Hz, so each
call blocked ~1s. CenteringController.go_to_and_center() calls it once per
iteration, so the whole centering loop ran at ~1 Hz instead of its designed
10 Hz (OFFBOARD_SETPOINT_INTERVAL_S) -- proven by 81 VEHICLE_TELEMETRY
events at exactly 1.000s spacing across an 82.0s centering window. At that
rate the setpoint stream is 2x past PX4's ~500ms Offboard timeout.

These tests drive the real methods against a fake MAVSDK `drone` whose
streams are deliberately SLOW, so a regression to per-call subscription
shows up as a latency assertion failure, not as a subtle rate change nobody
notices until the next flight.
"""
import asyncio
import time

import pytest

from mavsdk_common.mavsdk_backend_base import MavsdkBackendBase
from core.config.parameters import OFFBOARD_SETPOINT_INTERVAL_S


class _Pos:
    def __init__(self, lat, lon, alt):
        self.latitude_deg, self.longitude_deg, self.relative_altitude_m = lat, lon, alt


class _Euler:
    def __init__(self, yaw):
        self.yaw_deg = yaw


class _NedVec:
    def __init__(self, n, e, d):
        self.north_m, self.east_m, self.down_m = n, e, d
        self.north_m_s, self.east_m_s, self.down_m_s = n, e, d


class _PosVelNed:
    def __init__(self, n, e, d):
        self.position = _NedVec(n, e, d)
        self.velocity = _NedVec(n, e, d)


class _SlowTelemetry:
    """Every stream pushes one sample per `interval_s` -- the behaviour that
    made per-call subscription so expensive. Counts subscriptions so a test
    can assert the backend opens each stream exactly once."""

    def __init__(self, interval_s=0.4):
        self.interval_s = interval_s
        self.subscriptions = {"position": 0, "position_velocity_ned": 0,
                              "flight_mode": 0, "attitude_euler": 0}
        self.rate_calls = {}

    async def _stream(self, name, make_value):
        self.subscriptions[name] += 1
        i = 0
        while True:
            yield make_value(i)
            i += 1
            await asyncio.sleep(self.interval_s)

    def position(self):
        return self._stream("position", lambda i: _Pos(47.0 + i * 1e-6, 8.0, 15.0))

    def position_velocity_ned(self):
        return self._stream("position_velocity_ned", lambda i: _PosVelNed(float(i), 2.0, -15.0))

    def flight_mode(self):
        return self._stream("flight_mode", lambda i: "OFFBOARD")

    def attitude_euler(self):
        return self._stream("attitude_euler", lambda i: _Euler(90.0))

    async def set_rate_position(self, hz):
        self.rate_calls["position"] = hz

    async def set_rate_position_velocity_ned(self, hz):
        self.rate_calls["position_velocity_ned"] = hz

    async def set_rate_attitude_euler(self, hz):
        self.rate_calls["attitude_euler"] = hz


class _FakeMission:
    async def mission_progress(self):
        while True:
            await asyncio.sleep(3600)
            yield None


class _FakeCore:
    async def connection_state(self):
        class _S:
            is_connected = True
        yield _S()


class _FakeDrone:
    def __init__(self, telemetry):
        self.telemetry = telemetry
        self.mission = _FakeMission()
        self.core = _FakeCore()

    async def connect(self, system_address=None):
        return None


async def _connected_backend(interval_s=0.4):
    backend = MavsdkBackendBase("udp://:14540")
    telemetry = _SlowTelemetry(interval_s=interval_s)
    backend.drone = _FakeDrone(telemetry)
    await backend.connect()
    return backend, telemetry


def _cancel_watchers(backend):
    for task in backend._watchers:
        task.cancel()


@pytest.mark.asyncio
async def test_getters_serve_from_cache_instead_of_paying_the_stream_interval():
    """THE regression guard. With a 0.4s stream interval, the old per-call
    subscription made each of these cost ~0.4s; from cache they are
    effectively free. Asserted against OFFBOARD_SETPOINT_INTERVAL_S, the
    budget the centering loop actually has for a whole iteration."""
    backend, telemetry = await _connected_backend(interval_s=0.4)
    try:
        started = time.monotonic()
        for _ in range(10):
            await backend.get_global_position()
            await backend.get_flight_mode()
            await backend.get_position_ned()
            await backend.get_velocity_ned()
            await backend.get_yaw_deg()
        elapsed = time.monotonic() - started

        # 50 getter calls. Under the old implementation this would have cost
        # ~50 x 0.4s = 20s; one centering iteration's worth of them must now
        # fit inside a single setpoint interval.
        assert elapsed < OFFBOARD_SETPOINT_INTERVAL_S
    finally:
        _cancel_watchers(backend)


@pytest.mark.asyncio
async def test_each_stream_is_subscribed_exactly_once():
    backend, telemetry = await _connected_backend(interval_s=0.05)
    try:
        for _ in range(20):
            await backend.get_global_position()
            await backend.get_yaw_deg()
            await backend.get_velocity_ned()
            await backend.get_flight_mode()

        assert telemetry.subscriptions == {
            "position": 1, "position_velocity_ned": 1, "flight_mode": 1, "attitude_euler": 1,
        }
    finally:
        _cancel_watchers(backend)


@pytest.mark.asyncio
async def test_connect_requests_the_fast_stream_rates():
    from core.config.parameters import TELEMETRY_STREAM_RATE_HZ

    backend, telemetry = await _connected_backend(interval_s=0.05)
    try:
        assert telemetry.rate_calls == {
            "position": TELEMETRY_STREAM_RATE_HZ,
            "position_velocity_ned": TELEMETRY_STREAM_RATE_HZ,
            "attitude_euler": TELEMETRY_STREAM_RATE_HZ,
        }
    finally:
        _cancel_watchers(backend)


@pytest.mark.asyncio
async def test_a_declined_set_rate_degrades_instead_of_failing_connect():
    """Some PX4/MAVSDK combinations reject an individual set_rate_*. That
    must leave one stream slow, not abort the connection."""
    backend = MavsdkBackendBase("udp://:14540")
    telemetry = _SlowTelemetry(interval_s=0.05)

    async def _reject(hz):
        raise RuntimeError("COMMAND_DENIED")
    telemetry.set_rate_attitude_euler = _reject

    backend.drone = _FakeDrone(telemetry)
    try:
        await backend.connect()  # must not raise
        assert await backend.get_yaw_deg() == 90.0
    finally:
        _cancel_watchers(backend)


@pytest.mark.asyncio
async def test_heartbeat_is_published_by_the_watcher_not_by_the_getter():
    """ADR-008 B0: the flight-backend heartbeat used to be emitted inside
    get_global_position(), so it stopped whenever the current mission loop
    stopped calling it -- go_to_and_center()'s "target lost" branch never
    did, which starved MavsdkBackendBase into a DEGRADED<->STALE flap for
    the whole 77s of the 2026-08-16 failed centering."""
    published = []

    class _Recorder:
        def publish(self, event):
            published.append(event)

    backend = MavsdkBackendBase("udp://:14540", publisher=_Recorder())
    backend.drone = _FakeDrone(_SlowTelemetry(interval_s=0.05))
    try:
        await backend.connect()
        published.clear()
        # Nobody calls get_global_position() at all here -- exactly the
        # "target lost" branch's behaviour.
        await asyncio.sleep(1.2)

        heartbeats = [e for e in published if e.code == "VEHICLE_TELEMETRY"]
        assert len(heartbeats) >= 2
        assert heartbeats[-1].data["flight_mode"] == "OFFBOARD"
    finally:
        _cancel_watchers(backend)
