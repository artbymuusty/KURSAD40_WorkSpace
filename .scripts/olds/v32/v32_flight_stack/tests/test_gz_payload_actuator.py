"""Gazebo payload actuator: detach topics, and CONFIRMED release (F2).

Rewritten for ADR-011 (release detaches a world-loaded body instead of
spawning one) and F2 (a release is not believed until the body is seen to
leave the vehicle). The failure these pin down is concrete: on the first
ADR-011 flight the servo fired, the log said RELEASED, and the payload was
still bolted on -- it let go seconds later during the climb-out and landed
4.9 m past the target.
"""
import pytest
from unittest.mock import AsyncMock, patch

from gz_system.gz_payload_actuator import (
    GzPayloadActuator,
    PAYLOAD_DETACH_TOPIC,
    VEHICLE_MODEL_NAME,
)


def _mock_proc(returncode: int, stderr: bytes = b""):
    proc = AsyncMock()
    proc.returncode = returncode
    proc.communicate.return_value = (b"", stderr)
    return proc


class _FakeMonitor:
    """Scripted pose source. `drop_after` is how many payload reads stay
    attached before the body starts falling; read 1 is the pre-publish
    baseline, so 1 means "separates on the first poll after the servo" and
    None means it never separates at all."""

    def __init__(self, drop_after=1, known=True):
        self.drop_after = drop_after
        self.known = known
        self.reads = 0

    def get(self, name):
        if not self.known:
            return None
        if name == VEHICLE_MODEL_NAME:
            return (0.0, 0.0, 0.65)
        self.reads += 1
        attached_z = 0.47  # 0.18 m below the vehicle
        if self.drop_after is not None and self.reads > self.drop_after:
            return (0.0, 0.0, 0.03)
        return (0.0, 0.0, attached_z)

    def get_quat(self, name):
        return (0.0, 0.0, 0.0, 1.0)


def _actuator(monitor):
    return GzPayloadActuator("dummy_service", pose_monitor=monitor)


@pytest.mark.asyncio
async def test_release_at_mavi_altigen_detaches_the_red_payload():
    """The servo->colour mapping is a deliberate team assignment (RED
    payload on the MAVI hexagon) and must not drift."""
    actuator = _actuator(_FakeMonitor())
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)) as exec_mock:
        assert await actuator.release_payload_at_mavi_altigen() is True
    topics = [c.args for c in exec_mock.call_args_list]
    assert all(PAYLOAD_DETACH_TOPIC % "red" in args for args in topics)
    assert all("gz.msgs.Empty" in args for args in topics)


@pytest.mark.asyncio
async def test_release_at_kirmizi_ucgen_detaches_the_blue_payload():
    actuator = _actuator(_FakeMonitor())
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)) as exec_mock:
        assert await actuator.release_payload_at_kirmizi_ucgen() is True
    assert all(PAYLOAD_DETACH_TOPIC % "blue" in c.args for c in exec_mock.call_args_list)


@pytest.mark.asyncio
async def test_detach_is_published_more_than_once():
    """gz-transport is a slow joiner: a one-shot publisher can advertise and
    send before the plugin has finished subscribing, and the message is
    simply lost. A single publish is what made the first flight's detach
    arrive seconds late."""
    actuator = _actuator(_FakeMonitor())
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)) as exec_mock:
        await actuator.release_payload_at_mavi_altigen()
    assert exec_mock.call_count > 1


@pytest.mark.asyncio
async def test_release_reports_failure_when_the_payload_never_separates():
    """THE regression. The payload is visible and demonstrably still hanging
    off the vehicle, so the release must come back False -- the caller uses
    that to hold position instead of climbing away."""
    actuator = _actuator(_FakeMonitor(drop_after=None))
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)):
        assert await actuator.release_payload_at_mavi_altigen() is False
    assert actuator.detach_latency("MAVI_ALTIGEN") is None


@pytest.mark.asyncio
async def test_confirmed_release_records_its_latency():
    actuator = _actuator(_FakeMonitor(drop_after=1))
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)):
        assert await actuator.release_payload_at_kirmizi_ucgen() is True
    latency = actuator.detach_latency("KIRMIZI_UCGEN")
    assert latency is not None and latency >= 0.0


@pytest.mark.asyncio
async def test_missing_pose_data_is_unknown_not_failure():
    """A dead observer must not ground a flight. With no pose at all we
    cannot distinguish attached from separated, so we claim neither and let
    the mission proceed -- loudly unconfirmed, not falsely failed."""
    actuator = _actuator(_FakeMonitor(known=False))
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)):
        assert await actuator.release_payload_at_mavi_altigen() is True
    assert actuator.detach_latency("MAVI_ALTIGEN") is None


@pytest.mark.asyncio
async def test_release_returns_false_when_gz_cli_missing():
    actuator = _actuator(_FakeMonitor())
    with patch("asyncio.create_subprocess_exec", side_effect=FileNotFoundError()):
        assert await actuator.release_payload_at_kirmizi_ucgen() is False


@pytest.mark.asyncio
async def test_release_returns_false_when_every_publish_fails():
    actuator = _actuator(_FakeMonitor())
    with patch("asyncio.create_subprocess_exec",
               return_value=_mock_proc(1, b"gz: command not found")):
        assert await actuator.release_payload_at_mavi_altigen() is False


def test_landing_reference_carries_the_target_centre_and_rest_height():
    """F3: without a reference, "settled" could only ever mean "not below
    ground" -- which is how a 4.9 m miss passed."""
    actuator = _actuator(_FakeMonitor())
    assert actuator.landing_reference("MAVI_ALTIGEN")[:2] == (0.0, 15.0)
    assert actuator.landing_reference("KIRMIZI_UCGEN")[:2] == (0.0, 40.0)
    assert actuator.landing_reference("KIRMIZI_DIKDORTGEN") is None


def test_tilt_is_reported_so_edge_landings_are_visible():
    actuator = _actuator(_FakeMonitor())
    assert actuator.get_released_payload_tilt_deg("MAVI_ALTIGEN") == 0.0


@pytest.mark.asyncio
async def test_gorev3_hooks_still_report_success_as_simulated_placeholder():
    actuator = _actuator(_FakeMonitor())
    assert await actuator.activate_pickup_mechanism() is True
    assert await actuator.activate_drop_mechanism() is True
