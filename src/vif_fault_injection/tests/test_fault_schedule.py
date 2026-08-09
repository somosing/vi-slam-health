from vif_fault_injection.fault_schedule import FaultSchedule


def test_active_only_inside_half_open_window():
    schedule = FaultSchedule(enabled=True, start_after_sec=8.0, duration_sec=3.0)
    assert not schedule.active(7.999)
    assert schedule.active(8.0)
    assert schedule.active(10.999)
    assert not schedule.active(11.0)


def test_disabled_schedule_is_never_active():
    schedule = FaultSchedule(enabled=False, start_after_sec=0.0, duration_sec=10.0)
    assert not schedule.active(5.0)
