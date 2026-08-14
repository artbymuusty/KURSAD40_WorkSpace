import pytest
from unittest.mock import AsyncMock, patch

from gz_system.gz_payload_actuator import (
    GzPayloadActuator,
    PAYLOAD_DROP_COLOR_TOPIC,
)


def _mock_proc(returncode: int, stderr: bytes = b""):
    proc = AsyncMock()
    proc.returncode = returncode
    proc.communicate.return_value = (b"", stderr)
    return proc


@pytest.mark.asyncio
async def test_release_at_mavi_altigen_publishes_red():
    actuator = GzPayloadActuator("dummy_service")
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)) as exec_mock:
        result = await actuator.release_payload_at_mavi_altigen()

    assert result is True
    args = exec_mock.call_args.args
    assert PAYLOAD_DROP_COLOR_TOPIC in args
    assert 'data: "red"' in args


@pytest.mark.asyncio
async def test_release_at_kirmizi_ucgen_publishes_blue():
    actuator = GzPayloadActuator("dummy_service")
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(0)) as exec_mock:
        result = await actuator.release_payload_at_kirmizi_ucgen()

    assert result is True
    args = exec_mock.call_args.args
    assert 'data: "blue"' in args


@pytest.mark.asyncio
async def test_release_returns_false_on_nonzero_exit():
    actuator = GzPayloadActuator("dummy_service")
    with patch("asyncio.create_subprocess_exec", return_value=_mock_proc(1, b"gz: command not found")):
        result = await actuator.release_payload_at_mavi_altigen()

    assert result is False


@pytest.mark.asyncio
async def test_release_returns_false_when_gz_cli_missing():
    actuator = GzPayloadActuator("dummy_service")
    with patch("asyncio.create_subprocess_exec", side_effect=FileNotFoundError()):
        result = await actuator.release_payload_at_kirmizi_ucgen()

    assert result is False


@pytest.mark.asyncio
async def test_gorev3_hooks_still_report_success_as_simulated_placeholder():
    actuator = GzPayloadActuator("dummy_service")
    assert await actuator.activate_pickup_mechanism() is True
    assert await actuator.activate_drop_mechanism() is True
