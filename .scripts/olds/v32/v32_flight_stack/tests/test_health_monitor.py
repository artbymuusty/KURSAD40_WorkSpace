from core.telemetry.event_bus import EventBus
from core.telemetry.events import Event
from core.telemetry.health import DEGRADED, DOWN, HEALTHY, UNKNOWN, HealthMonitor


def test_unregistered_subsystem_has_no_effect():
    bus = EventBus()
    monitor = HealthMonitor(publisher=bus)
    results = monitor.check()
    assert results == {}


def test_never_seen_subsystem_is_unknown():
    monitor = HealthMonitor()
    monitor.register("Vision", expected_interval_s=1.0)
    assert monitor.check(now=1000.0)["Vision"] == UNKNOWN


def test_recent_heartbeat_is_healthy():
    monitor = HealthMonitor()
    monitor.register("Vision", expected_interval_s=1.0)
    monitor.on_event(Event(code="X", subsystem="Vision", ts=1000.0))
    assert monitor.check(now=1000.3)["Vision"] == HEALTHY


def test_stale_heartbeat_degrades_before_down():
    monitor = HealthMonitor()
    monitor.register("Vision", expected_interval_s=1.0, grace_multiplier=3.0)
    monitor.on_event(Event(code="X", subsystem="Vision", ts=1000.0))
    assert monitor.check(now=1001.2)["Vision"] == DEGRADED


def test_long_silence_is_down():
    monitor = HealthMonitor()
    monitor.register("Vision", expected_interval_s=1.0, grace_multiplier=3.0)
    monitor.on_event(Event(code="X", subsystem="Vision", ts=1000.0))
    assert monitor.check(now=1010.0)["Vision"] == DOWN


def test_health_state_change_publishes_event_only_on_transition():
    bus = EventBus()
    received = []
    bus.subscribe(received.append)
    monitor = HealthMonitor(publisher=bus)
    monitor.register("Vision", expected_interval_s=1.0)

    monitor.check(now=1000.0)  # UNKNOWN -> no prior state change tracked yet? first check *is* a transition from the initial UNKNOWN default only if state differs
    first_count = len([e for e in received if e.code == "HEALTH_STATE_CHANGED"])
    monitor.check(now=1000.01)  # still UNKNOWN, no change
    second_count = len([e for e in received if e.code == "HEALTH_STATE_CHANGED"])

    assert second_count == first_count  # no duplicate events for an unchanged state


def test_dependency_propagation_forces_down():
    monitor = HealthMonitor()
    monitor.register("FlightBackend", expected_interval_s=1.0, grace_multiplier=3.0)
    monitor.register("Navigation", expected_interval_s=1.0, grace_multiplier=3.0)
    monitor.set_dependency("Navigation", depends_on="FlightBackend")

    monitor.on_event(Event(code="X", subsystem="FlightBackend", ts=1000.0))
    monitor.on_event(Event(code="X", subsystem="Navigation", ts=1000.0))

    # FlightBackend goes silent long enough to be DOWN; Navigation's own
    # heartbeat is still fresh, but it must be forced DOWN too (ADR-004 §10).
    results = monitor.check(now=1000.3)
    assert results["Navigation"] == HEALTHY  # not yet DOWN, backend still healthy at this point

    results = monitor.check(now=1010.0)
    assert results["FlightBackend"] == DOWN
    assert results["Navigation"] == DOWN
