"""F2/F3: a release is confirmed, and a landing is scored against the shape.

Both come from one flight (ADR-011, first nominal). The servo fired, every
operator channel said RELEASED, and payload 2 was still attached -- it let
go during the climb-out and came to rest 4.9 m past the triangle, while the
post-drop check reported success because the only thing it asked was "is z
above ground". These tests pin the two halves of that: don't climb away
from an unconfirmed release (F2), and don't call a miss a landing (F3).
"""
import pytest

from core.config.parameters import (
    PAYLOAD_EXPECTED_REST_Z_M, PAYLOAD_ON_TARGET_RADIUS_M,
)
from core.mission.payload_release import PayloadReleaseService
from core.telemetry.events import Severity


class _Pub:
    def __init__(self):
        self.events = []

    def publish(self, e):
        self.events.append(e)

    def codes(self):
        return [e.code for e in self.events]


class _Actuator:
    """Stands in for the gz actuator: knows where the target is, where the
    body ended up, and how flat it is lying."""

    def __init__(self, pose, tilt_deg=0.0, centre=(0.0, 40.0)):
        self.pose = pose
        self.tilt_deg = tilt_deg
        self.centre = centre

    async def get_released_payload_pose(self, shape_type):
        return self.pose

    def get_released_payload_tilt_deg(self, shape_type):
        return self.tilt_deg

    def landing_reference(self, shape_type):
        return (self.centre[0], self.centre[1], PAYLOAD_EXPECTED_REST_Z_M)


def _service(actuator, publisher):
    svc = PayloadReleaseService.__new__(PayloadReleaseService)
    svc.actuator = actuator
    svc.publisher = publisher
    return svc


async def _final_pose(actuator):
    pub = _Pub()
    await _service(actuator, pub)._log_payload_final_pose("KIRMIZI_UCGEN")
    return pub.events[0]


@pytest.mark.asyncio
async def test_the_49m_miss_is_no_longer_scored_as_a_landing():
    """THE regression. Measured pose from the first ADR-011 flight: on bare
    ground, above z=0, and nowhere near the triangle it was aimed at."""
    ev = await _final_pose(_Actuator((-2.882, 43.969, 0.025)))
    assert ev.data["settled_above_ground"] is True   # the old check passed
    assert ev.data["settled_on_target"] is False     # the real one does not
    assert ev.data["offset_from_center_cm"] == pytest.approx(490, abs=5)
    assert ev.severity == Severity.WARN


@pytest.mark.asyncio
async def test_a_payload_standing_on_its_edge_is_not_on_target():
    """The other half of that flight: payload 1 rested 0.41 m from the
    hexagon centre at z=0.156 -- which is not a landing, it is a 0.30 m slab
    balanced on its long edge (0.006 surface + 0.150 half-side)."""
    ev = await _final_pose(_Actuator((0.368, 14.805, 0.156), tilt_deg=89.4,
                                     centre=(0.0, 15.0)))
    assert ev.data["settled_on_target"] is False
    assert ev.data["z_error_m"] == pytest.approx(0.125, abs=0.002)
    assert ev.data["tilt_deg"] == 89.4
    assert ev.data["offset_from_center_cm"] < PAYLOAD_ON_TARGET_RADIUS_M * 100


@pytest.mark.asyncio
async def test_a_flat_landing_on_the_shape_passes():
    ev = await _final_pose(_Actuator((0.04, 39.97, 0.031)))
    assert ev.data["settled_on_target"] is True
    assert ev.severity == Severity.INFO
    assert ev.data["offset_from_center_cm"] == pytest.approx(5.0, abs=1.0)


@pytest.mark.asyncio
async def test_an_actuator_with_no_ground_truth_degrades_gracefully():
    """The real-flight backend cannot say where a payload landed. It must
    fall back to the weaker answer rather than reporting a false miss."""
    class _NoReference:
        async def get_released_payload_pose(self, shape_type):
            return (0.0, 40.0, 0.03)

    ev = await _final_pose(_NoReference())
    assert ev.data["settled_on_target"] is True
    assert "offset_from_center_cm" not in ev.data


class _StubFlight:
    def __init__(self):
        self.holds = 0
        self.status_texts = []

    async def hold_position(self, duration_s):
        self.holds += 1

    async def send_status_text(self, text, severity="INFO"):
        self.status_texts.append(text)
        return True


class _StuckActuator:
    """Never lets go, then does after `confirm_after` checks."""

    def __init__(self, confirm_after=2):
        self.confirm_after = confirm_after
        self.checks = 0
        self.retries = 0

    async def is_release_confirmed(self, shape_type):
        self.checks += 1
        return self.checks >= self.confirm_after

    async def retry_release(self, shape_type):
        self.retries += 1
        return True


def _hold_service(actuator, flight, publisher):
    svc = _service(actuator, publisher)
    svc.flight = flight
    svc._payload_index = 1
    return svc


@pytest.mark.asyncio
async def test_unconfirmed_release_holds_position_and_announces_critical():
    """No climb-out on an unconfirmed release: holding at 0.45 m over the
    target is recoverable, carrying an attached payload up to 15 m is not."""
    flight, pub = _StubFlight(), _Pub()
    actuator = _StuckActuator(confirm_after=3)
    svc = _hold_service(actuator, flight, pub)

    assert await svc._hold_until_detached("KIRMIZI_UCGEN") is True
    assert "PAYLOAD_DETACH_UNCONFIRMED" in pub.codes()
    assert pub.events[0].severity == Severity.CRITICAL
    assert "PAYLOAD_DETACH_RECOVERED" in pub.codes()
    # It kept the offboard setpoint stream alive rather than sleeping, and
    # it kept re-publishing the detach while it waited.
    assert flight.holds >= 2
    assert actuator.retries >= 1
    assert any("UNCONFIRMED" in t for t in flight.status_texts)


@pytest.mark.asyncio
async def test_hold_is_skipped_for_backends_that_cannot_confirm():
    """A backend with no way to observe separation (the real aircraft) must
    not be parked in a hold it can never leave."""
    class _Blind:
        pass

    flight, pub = _StubFlight(), _Pub()
    svc = _hold_service(_Blind(), flight, pub)
    assert await svc._hold_until_detached("KIRMIZI_UCGEN") is False
    assert flight.holds == 0
    assert pub.codes() == []
