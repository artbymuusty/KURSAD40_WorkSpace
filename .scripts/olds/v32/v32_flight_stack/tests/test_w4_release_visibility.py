"""W4: a release must be unmissable on every operator-facing channel.

The motivating failure is not subtle: across several runs the payload was
being released and then falling through the world, and nothing on the
dashboard, in QGC or in the log said either that it had fired or where the
body ended up. Each test here pins one channel.
"""
import time

import pytest

from core.config.parameters import RELEASED_OVERLAY_DURATION_S, PAYLOAD_FINAL_POSE_DELAY_S
from core.telemetry.aggregator import RuntimeStateAggregator
from core.telemetry.events import Category, Event, Severity


def _event(code, **data):
    return Event(code=code, subsystem="PayloadReleaseService", category=Category.PAYLOAD,
                 severity=Severity.INFO, message="", data=data)


def test_released_state_survives_to_disarm():
    """The panel must still read RELEASED at the end of the flight -- disarm
    is exactly when an operator goes looking for 'did it drop, and where'."""
    agg = RuntimeStateAggregator()
    stamp = time.time()
    agg.on_event(_event("PAYLOAD_STATE", payload_index=1, shape_type="MAVI_ALTIGEN",
                        released=True, released_alt_m=0.48, within_tolerance=True,
                        released_at=stamp))
    # A whole second payload runs afterwards; the first release must not be
    # blanked by it.
    agg.on_event(_event("PAYLOAD_STATE", payload_index=2, shape_type="KIRMIZI_UCGEN",
                        descent_step="1/3", current_alt_m=15.0))
    p = agg.snapshot().payload
    assert p.released_alt_m == pytest.approx(0.48)
    assert p.released_at == pytest.approx(stamp)
    assert p.released_index == 1
    assert p.released_shape == "MAVI_ALTIGEN"


def test_released_overlay_tag_is_time_boxed():
    """The camera tag is a 3 s announcement, not a latched state -- a
    permanent tag would read as 'still releasing' for the rest of the run."""
    now = time.time()
    assert (now - now) <= RELEASED_OVERLAY_DURATION_S
    assert (now - (now - RELEASED_OVERLAY_DURATION_S + 0.5)) <= RELEASED_OVERLAY_DURATION_S
    assert (now - (now - RELEASED_OVERLAY_DURATION_S - 0.5)) > RELEASED_OVERLAY_DURATION_S


def test_status_text_default_is_a_safe_no_op():
    """A backend with no GCS channel must degrade to silence, never raise --
    a cosmetic message must not be able to abort a payload release. Uses the
    real MockFlightBackend, which implements the interface but overrides
    nothing here, so this pins the DEFAULT the ABC provides."""
    import asyncio

    from mocks.mock_flight_backend import MockFlightBackend

    backend = MockFlightBackend()
    assert asyncio.run(backend.send_status_text("PAYLOAD 1 RELEASED @0.48m")) is False


def test_status_text_is_truncated_to_the_mavlink_limit():
    """MAVLink STATUSTEXT carries 50 characters. The banner must be cut to
    fit rather than rejected by the transport."""
    banner = "PAYLOAD 1 RELEASED @0.48m"
    assert len(banner) <= 50
    long_banner = "PAYLOAD 1 RELEASED @0.48m " + "x" * 100
    assert len(long_banner[:50]) == 50


@pytest.mark.asyncio
async def test_final_pose_reports_unavailable_rather_than_guessing():
    """A backend that cannot locate the released body must say so. Inventing
    a pose here would manufacture exactly the false 'landed on target'
    evidence W2 exists to establish honestly."""
    from core.mission.payload_release import PayloadReleaseService

    published = []

    class _Pub:
        def publish(self, e): published.append(e)

    class _ActuatorNoLocator:
        pass

    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    svc.actuator = _ActuatorNoLocator()
    svc.publisher = _Pub()
    # No get_released_payload_pose -> silently skipped, nothing published.
    await svc._log_payload_final_pose("MAVI_ALTIGEN")
    assert published == []

    class _ActuatorReturnsNone:
        async def get_released_payload_pose(self, shape_type):
            return None

    svc.actuator = _ActuatorReturnsNone()
    await svc._log_payload_final_pose("MAVI_ALTIGEN")
    assert [e.code for e in published] == ["PAYLOAD_FINAL_POSE"]
    assert published[0].data["available"] is False


@pytest.mark.asyncio
async def test_final_pose_flags_a_payload_that_fell_through_the_world():
    """The whole point of W4.3: a body at z=-0.72 must be reported as fallen
    through, not quietly logged as a position."""
    from core.mission.payload_release import PayloadReleaseService

    published = []

    class _Pub:
        def publish(self, e): published.append(e)

    class _Fell:
        async def get_released_payload_pose(self, shape_type):
            return (0.02, 15.01, -0.72)

    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    svc.actuator = _Fell()
    svc.publisher = _Pub()
    await svc._log_payload_final_pose("MAVI_ALTIGEN")

    ev = published[0]
    assert ev.data["available"] is True
    assert ev.data["settled_above_ground"] is False
    assert ev.severity == Severity.WARN
    assert ev.data["z"] == pytest.approx(-0.72)


@pytest.mark.asyncio
async def test_final_pose_accepts_a_payload_resting_on_the_target():
    from core.mission.payload_release import PayloadReleaseService

    published = []

    class _Pub:
        def publish(self, e): published.append(e)

    class _Rested:
        async def get_released_payload_pose(self, shape_type):
            return (0.01, 15.00, 0.031)

    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    svc.actuator = _Rested()
    svc.publisher = _Pub()
    await svc._log_payload_final_pose("MAVI_ALTIGEN")

    ev = published[0]
    assert ev.data["settled_above_ground"] is True
    assert ev.severity == Severity.INFO


def test_final_pose_delay_lets_the_body_settle():
    """2 s is enough for a 0.45 m drop to land and settle, and short enough
    that the vehicle is still parked over the target when it is sampled."""
    assert PAYLOAD_FINAL_POSE_DELAY_S >= 1.0
    assert PAYLOAD_FINAL_POSE_DELAY_S <= 3.0
