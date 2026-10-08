import pytest

pytest.importorskip('exd_interfaces')  # needs the built workspace

from exd_flight_control.backends import SimBackend  # noqa: E402
from exd_interfaces.msg import FlightState  # noqa: E402


def run(backend, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        backend.update(dt)


def test_full_flight():
    b = SimBackend(max_speed=1.0, max_climb=1.0)
    assert b.takeoff(1.0)[0] is False          # must arm first
    assert b.arm(True)[0]
    assert b.takeoff(1.0)[0]
    run(b, 2.0)
    s = b.state()
    assert s.in_air and abs(s.z - 1.0) < 0.05
    assert s.mode == FlightState.MODE_HOLD

    b.set_setpoint(2.0, 0.0, 1.0, 0.0)
    assert b.state().mode == FlightState.MODE_OFFBOARD
    run(b, 3.0)
    assert abs(b.state().x - 2.0) < 0.05

    assert b.arm(False)[0] is False            # can't disarm in the air
    assert b.land()[0]
    run(b, 2.0)
    s = b.state()
    assert not s.in_air and not s.armed


def test_setpoint_ignored_on_ground():
    b = SimBackend()
    b.arm(True)
    b.set_setpoint(5.0, 5.0, 2.0, 0.0)
    run(b, 1.0)
    assert b.state().x == 0.0 and not b.state().in_air
