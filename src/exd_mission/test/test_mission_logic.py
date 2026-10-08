import math

import pytest

from exd_mission import mission_logic as ml


def make():
    return ml.Mission([(1.0, 0.0, 1.0), (1.0, 1.0, 1.0)], takeoff_altitude=1.0,
                      tolerance=0.2, stop_distance=0.5)


def test_happy_path():
    m = make()
    assert m.start().commands == [ml.CMD_ARM]
    assert m.on_command_result(ml.CMD_ARM, True).commands == [ml.CMD_TAKEOFF]
    assert m.state == ml.TAKING_OFF

    m.update(ml.Telemetry(armed=True, in_air=True, position=(0, 0, 1.0)))
    assert m.state == ml.EXECUTING

    step = m.update(ml.Telemetry(armed=True, in_air=True, position=(0, 0, 1.0)))
    assert step.setpoint == (1.0, 0.0, 1.0)

    step = m.update(ml.Telemetry(armed=True, in_air=True, position=(1.0, 0.0, 1.0)))
    assert step.setpoint == (1.0, 1.0, 1.0) and m.index == 1

    step = m.update(ml.Telemetry(armed=True, in_air=True, position=(1.0, 1.0, 1.0)))
    assert step.commands == [ml.CMD_LAND] and m.state == ml.LANDING

    m.update(ml.Telemetry(armed=False, in_air=False, position=(1.0, 1.0, 0.0)))
    assert m.state == ml.DONE


def test_obstacle_hold_and_resume():
    m = make()
    m.start()
    m.on_command_result(ml.CMD_ARM, True)
    m.update(ml.Telemetry(armed=True, in_air=True, position=(0, 0, 1.0)))

    step = m.update(ml.Telemetry(True, True, (0.3, 0, 1.0), min_obstacle_distance=0.3))
    assert m.state == ml.HOLDING and step.setpoint == (0.3, 0, 1.0)

    m.update(ml.Telemetry(True, True, (0.3, 0, 1.0), min_obstacle_distance=math.inf))
    assert m.state == ml.EXECUTING


def test_arm_failure_aborts():
    m = make()
    m.start()
    m.on_command_result(ml.CMD_ARM, False, 'no GPS')
    assert m.state == ml.ABORTED and 'no GPS' in m.detail


def test_abort_in_air_lands():
    m = make()
    m.start()
    m.on_command_result(ml.CMD_ARM, True)
    assert m.abort().commands == [ml.CMD_LAND]
    assert m.state == ml.LANDING


def test_parse_waypoints():
    assert ml.parse_waypoints([1, 2, 3, 4, 5, 6]) == [(1, 2, 3), (4, 5, 6)]
    with pytest.raises(ValueError):
        ml.parse_waypoints([1, 2])
