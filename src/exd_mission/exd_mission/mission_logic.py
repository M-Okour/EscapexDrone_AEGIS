"""Mission state machine with no ROS inside, so it can be unit-tested on any laptop.

The ROS node (mission_node.py) feeds it telemetry and executes the commands it
returns. Flow:

  IDLE --start--> ARMING --ok--> TAKING_OFF --at altitude--> EXECUTING
  EXECUTING --obstacle too close--> HOLDING --clear--> EXECUTING
  EXECUTING --last waypoint reached--> LANDING --on ground--> DONE
  any flying state --abort--> LANDING ;  failures --> ABORTED

TODO(mission): this is a "fly fixed waypoints" demo. The real escape mission
(search, use detections, plan around obstacles, return home, battery failsafe
...) gets built here.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

IDLE = 'IDLE'
ARMING = 'ARMING'
TAKING_OFF = 'TAKING_OFF'
EXECUTING = 'EXECUTING'
HOLDING = 'HOLDING'
LANDING = 'LANDING'
DONE = 'DONE'
ABORTED = 'ABORTED'

# commands the node must execute
CMD_ARM = 'arm'
CMD_TAKEOFF = 'takeoff'
CMD_LAND = 'land'

Point = Tuple[float, float, float]


@dataclass
class Telemetry:
    armed: bool = False
    in_air: bool = False
    position: Point = (0.0, 0.0, 0.0)
    min_obstacle_distance: float = math.inf


@dataclass
class Step:
    """What the node should do after an update."""
    commands: List[str] = field(default_factory=list)
    setpoint: Optional[Point] = None


class Mission:
    def __init__(self, waypoints: List[Point], takeoff_altitude: float = 1.5,
                 tolerance: float = 0.3, stop_distance: float = 0.6):
        self.waypoints = list(waypoints)
        self.takeoff_altitude = takeoff_altitude
        self.tolerance = tolerance
        self.stop_distance = stop_distance
        self.state = IDLE
        self.detail = 'waiting for start'
        self.index = 0
        self.hold_point: Optional[Point] = None

    # ---------------------------------------------------------------- events
    def start(self) -> Step:
        if self.state not in (IDLE, DONE, ABORTED):
            return Step()
        if not self.waypoints:
            self._go(ABORTED, 'no waypoints configured')
            return Step()
        self.index = 0
        self._go(ARMING, 'arming')
        return Step(commands=[CMD_ARM])

    def abort(self, reason: str = 'abort requested') -> Step:
        if self.state in (TAKING_OFF, EXECUTING, HOLDING):
            self._go(LANDING, reason)
            return Step(commands=[CMD_LAND])
        if self.state in (IDLE, ARMING):
            self._go(ABORTED, reason)
        return Step()

    def on_command_result(self, command: str, success: bool, message: str = '') -> Step:
        if command == CMD_ARM and self.state == ARMING:
            if success:
                self._go(TAKING_OFF, f'taking off to {self.takeoff_altitude:.1f} m')
                return Step(commands=[CMD_TAKEOFF])
            self._go(ABORTED, f'arm failed: {message}')
        elif command == CMD_TAKEOFF and self.state == TAKING_OFF and not success:
            self._go(ABORTED, f'takeoff failed: {message}')
        elif command == CMD_LAND and not success:
            self.detail = f'land failed: {message}'
        return Step()

    # ---------------------------------------------------------------- periodic
    def update(self, t: Telemetry) -> Step:
        if self.state == TAKING_OFF:
            if t.in_air and t.position[2] >= self.takeoff_altitude - self.tolerance:
                self._go(EXECUTING, 'flying waypoints')
            return Step()

        if self.state == EXECUTING:
            if t.min_obstacle_distance < self.stop_distance:
                self.hold_point = t.position
                self._go(HOLDING, f'obstacle at {t.min_obstacle_distance:.2f} m')
                return Step(setpoint=self.hold_point)
            target = self.waypoints[self.index]
            if _dist(t.position, target) < self.tolerance:
                self.index += 1
                if self.index >= len(self.waypoints):
                    self._go(LANDING, 'mission complete, landing')
                    return Step(commands=[CMD_LAND])
                target = self.waypoints[self.index]
            self.detail = f'to waypoint {self.index + 1}/{len(self.waypoints)}'
            return Step(setpoint=target)

        if self.state == HOLDING:
            if t.min_obstacle_distance >= self.stop_distance * 1.2:   # hysteresis
                self._go(EXECUTING, 'path clear, resuming')
                return Step(setpoint=self.waypoints[self.index])
            return Step(setpoint=self.hold_point)

        if self.state == LANDING and not t.armed and not t.in_air:
            self._go(DONE, 'landed')
        return Step()

    def _go(self, state: str, detail: str) -> None:
        self.state = state
        self.detail = detail


def _dist(a: Point, b: Point) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def parse_waypoints(flat: List[float]) -> List[Point]:
    """[x1, y1, z1, x2, y2, z2, ...] -> [(x1, y1, z1), ...] (ROS params can't nest lists)."""
    if len(flat) % 3:
        raise ValueError('waypoints must be a flat list of x, y, z triples')
    return [(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
            for i in range(0, len(flat), 3)]
