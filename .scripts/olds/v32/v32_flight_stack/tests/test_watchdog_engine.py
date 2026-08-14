import time
from core.telemetry.event_bus import EventBus
from core.telemetry.watchdog import WatchdogEngine


def test_armed_watchdog_does_not_fire_before_threshold():
    fired = []
    wd = WatchdogEngine()
    wd.arm("W1", "TestSubsystem", threshold_s=10.0, on_fire=lambda n: fired.append(n))
    wd.check(now=time.time())
    assert fired == []


def test_watchdog_fires_exactly_once_past_threshold():
    fired = []
    wd = WatchdogEngine()
    start = time.time()
    wd.arm("W1", "TestSubsystem", threshold_s=1.0, on_fire=lambda n: fired.append(n))
    wd.check(now=start + 2.0)
    wd.check(now=start + 3.0)  # already fired -- must not fire again
    assert fired == ["W1"]


def test_feed_resets_the_deadline():
    fired = []
    wd = WatchdogEngine()
    start = time.time()
    wd.arm("W1", "TestSubsystem", threshold_s=1.0, on_fire=lambda n: fired.append(n))
    wd.feed("W1", now=start + 0.9)  # simulate a kick right before it would have fired
    wd.check(now=start + 1.5)  # 0.6s after the kick -- well within the renewed 1.0s budget
    assert fired == []  # deadline was pushed forward by feed()


def test_disarm_removes_the_timer():
    wd = WatchdogEngine()
    wd.arm("W1", "TestSubsystem", threshold_s=1.0)
    assert wd.is_armed("W1") is True
    wd.disarm("W1")
    assert wd.is_armed("W1") is False


def test_on_fire_exception_does_not_crash_check():
    wd = WatchdogEngine()
    start = time.time()
    wd.arm("W1", "TestSubsystem", threshold_s=0.0, on_fire=lambda n: (_ for _ in ()).throw(RuntimeError("boom")))
    # Must not raise -- a bad on_fire callback is isolated (mirrors EventBus's
    # subscriber isolation guarantee).
    wd.check(now=start + 1.0)


def test_publishes_watchdog_fired_event():
    bus = EventBus()
    received = []
    bus.subscribe(received.append)
    wd = WatchdogEngine(publisher=bus)
    start = time.time()
    wd.arm("W1", "TestSubsystem", threshold_s=0.0)
    wd.check(now=start + 1.0)

    codes = [e.code for e in received]
    assert "WATCHDOG_ARMED" in codes
    assert "WATCHDOG_FIRED" in codes
