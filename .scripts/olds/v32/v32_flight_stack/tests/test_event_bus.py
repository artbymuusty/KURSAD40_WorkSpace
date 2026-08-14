from core.telemetry.event_bus import EventBus
from core.telemetry.events import Category, Event, Severity


def test_publish_delivers_to_all_subscribers():
    bus = EventBus(mission_id="m1")
    received_a, received_b = [], []
    bus.subscribe(received_a.append)
    bus.subscribe(received_b.append)

    bus.publish(Event(code="X", subsystem="Test"))

    assert len(received_a) == 1
    assert len(received_b) == 1
    assert received_a[0].code == "X"


def test_publish_stamps_mission_id_when_missing():
    bus = EventBus(mission_id="mission-42")
    received = []
    bus.subscribe(received.append)

    bus.publish(Event(code="X", subsystem="Test"))

    assert received[0].mission_id == "mission-42"


def test_publish_does_not_overwrite_explicit_mission_id():
    bus = EventBus(mission_id="mission-42")
    received = []
    bus.subscribe(received.append)

    bus.publish(Event(code="X", subsystem="Test", mission_id="explicit"))

    assert received[0].mission_id == "explicit"


def test_a_failing_subscriber_does_not_break_others_or_the_publisher():
    bus = EventBus()
    received = []

    def bad_subscriber(event):
        raise RuntimeError("boom")

    bus.subscribe(bad_subscriber)
    bus.subscribe(received.append)

    # Must not raise -- a subscriber failure is isolated (ADR-004 §12).
    bus.publish(Event(code="X", subsystem="Test"))

    assert len(received) == 1


def test_unsubscribe_stops_delivery():
    bus = EventBus()
    received = []
    cb = received.append
    bus.subscribe(cb)
    bus.unsubscribe(cb)

    bus.publish(Event(code="X", subsystem="Test"))

    assert received == []
