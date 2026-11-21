import cquick_servo.watchdog as watchdog_module


def test_timeout_latches_until_fresh_heartbeat_and_external_reset(
    monkeypatch,
):
    clock = [0.0]
    monkeypatch.setattr(
        watchdog_module.time, "monotonic", lambda: clock[0]
    )
    timeouts = []
    watchdog = watchdog_module.SafetyWatchdog(
        lambda: timeouts.append("PULK"), timeout=1.0
    )

    clock[0] = 1.1
    watchdog.check()
    assert watchdog.is_tripped()
    assert timeouts == ["PULK"]

    watchdog.check()
    assert timeouts == ["PULK"]
    assert not watchdog.reset()

    watchdog.feed()
    assert watchdog.is_tripped()
    assert watchdog.reset()
    assert not watchdog.is_tripped()
