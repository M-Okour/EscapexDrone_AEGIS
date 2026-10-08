"""Flight controller backends.

The flight_interface_node exposes ONE stable ROS API (topics/services in
exd_common.topics) and forwards every call to a backend:

  SimBackend     - pure-Python point-mass drone. No ROS, no hardware. Default.
  MavrosBackend  - TODO(flight): talks to PX4 / ArduPilot through MAVROS.

Everyone else on the team only ever sees the ROS API, so swapping the backend
never breaks their code.
"""
import math
from dataclasses import dataclass

from exd_interfaces.msg import FlightState


@dataclass
class VehicleState:
    connected: bool = False
    armed: bool = False
    in_air: bool = False
    mode: int = FlightState.MODE_UNKNOWN
    battery_percent: float = -1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


class FlightBackend:
    """Interface every backend must implement. Return (success: bool, message: str)."""

    def update(self, dt: float) -> None:
        pass

    def state(self) -> VehicleState:
        raise NotImplementedError

    def arm(self, value: bool):
        raise NotImplementedError

    def takeoff(self, altitude: float):
        raise NotImplementedError

    def land(self):
        raise NotImplementedError

    def hold(self) -> None:
        raise NotImplementedError

    def set_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        raise NotImplementedError


class SimBackend(FlightBackend):
    """Very simple kinematic drone: flies straight to the target at max_speed."""

    GROUND_Z = 0.05

    def __init__(self, max_speed: float = 1.0, max_climb: float = 0.8,
                 battery_drain_per_s: float = 0.05):
        self.max_speed = max_speed
        self.max_climb = max_climb
        self.drain = battery_drain_per_s
        self.s = VehicleState(connected=True, mode=FlightState.MODE_MANUAL, battery_percent=100.0)
        self.target = (0.0, 0.0, 0.0, 0.0)

    def state(self) -> VehicleState:
        return self.s

    def arm(self, value: bool):
        if value == self.s.armed:
            return True, 'already ' + ('armed' if value else 'disarmed')
        if self.s.in_air:
            return False, 'refusing to arm/disarm while in the air'
        self.s.armed = value
        self.s.mode = FlightState.MODE_HOLD if value else FlightState.MODE_MANUAL
        self.target = (self.s.x, self.s.y, self.s.z, self.s.yaw)
        return True, 'armed' if value else 'disarmed'

    def takeoff(self, altitude: float):
        if not self.s.armed:
            return False, 'not armed'
        if self.s.in_air:
            return False, 'already in the air'
        if altitude <= 0.0:
            return False, 'altitude must be > 0'
        self.target = (self.s.x, self.s.y, altitude, self.s.yaw)
        self.s.mode = FlightState.MODE_TAKEOFF
        return True, f'taking off to {altitude:.1f} m'

    def land(self):
        if not self.s.armed:
            return False, 'not armed'
        self.target = (self.s.x, self.s.y, 0.0, self.s.yaw)
        self.s.mode = FlightState.MODE_LANDING
        return True, 'landing'

    def hold(self) -> None:
        if self.s.mode in (FlightState.MODE_OFFBOARD,):
            self.target = (self.s.x, self.s.y, self.s.z, self.s.yaw)
            self.s.mode = FlightState.MODE_HOLD

    def set_setpoint(self, x, y, z, yaw) -> None:
        # Like PX4: setpoints are only followed in the air and not during takeoff/landing.
        if self.s.in_air and self.s.mode in (FlightState.MODE_HOLD, FlightState.MODE_OFFBOARD):
            self.target = (x, y, z, yaw)
            self.s.mode = FlightState.MODE_OFFBOARD

    def update(self, dt: float) -> None:
        s = self.s
        if not s.armed or dt <= 0.0:
            s.vx = s.vy = s.vz = 0.0
            return
        s.battery_percent = max(0.0, s.battery_percent - self.drain * dt)

        tx, ty, tz, tyaw = self.target
        dx, dy, dz = tx - s.x, ty - s.y, tz - s.z
        horiz = math.hypot(dx, dy)
        h_step = min(horiz, self.max_speed * dt)
        v_step = max(-self.max_climb * dt, min(self.max_climb * dt, dz))
        if horiz > 1e-9:
            s.vx, s.vy = dx / horiz * h_step / dt, dy / horiz * h_step / dt
        else:
            s.vx = s.vy = 0.0
        s.vz = v_step / dt
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z = max(0.0, s.z + v_step)
        s.yaw = tyaw
        s.in_air = s.z > self.GROUND_Z

        if s.mode == FlightState.MODE_TAKEOFF and abs(tz - s.z) < 0.05:
            s.mode = FlightState.MODE_HOLD
        if s.mode == FlightState.MODE_LANDING and not s.in_air:
            s.z = 0.0
            s.armed = False
            s.mode = FlightState.MODE_MANUAL
            s.vx = s.vy = s.vz = 0.0


class MavrosBackend(FlightBackend):
    """TODO(flight): connect to the real autopilot through MAVROS.

    Suggested mapping (PX4 or ArduPilot via `ros2 launch mavros px4.launch`):
      state()        <- /mavros/state, /mavros/local_position/pose,
                        /mavros/local_position/velocity_local, /mavros/battery
      arm()          -> /mavros/cmd/arming       (mavros_msgs/srv/CommandBool)
      takeoff()      -> /mavros/cmd/takeoff or set mode AUTO.TAKEOFF / GUIDED
      land()         -> /mavros/set_mode "AUTO.LAND" (PX4) / "LAND" (ArduPilot)
      set_setpoint() -> /mavros/setpoint_position/local (stream >= 2 Hz, then
                        switch to OFFBOARD / GUIDED)
    `node` is the ROS node, so you can create subscriptions/clients on it.
    """

    def __init__(self, node):
        self.node = node
        self.s = VehicleState(connected=False)

    def state(self) -> VehicleState:
        return self.s

    def arm(self, value: bool):
        return False, 'MavrosBackend not implemented yet'

    def takeoff(self, altitude: float):
        return False, 'MavrosBackend not implemented yet'

    def land(self):
        return False, 'MavrosBackend not implemented yet'

    def hold(self) -> None:
        pass

    def set_setpoint(self, x, y, z, yaw) -> None:
        pass
