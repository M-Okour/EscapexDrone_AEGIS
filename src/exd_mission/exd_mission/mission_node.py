"""Mission node: decides what the drone does.

Subscribes: /exd/flight/state     (exd_interfaces/FlightState)
            /exd/lidar/obstacles  (exd_interfaces/ObstacleArray)
            /exd/camera/detections (exd_interfaces/DetectionArray)  -- logged only, for now
Publishes:  /exd/flight/setpoint  (geometry_msgs/PoseStamped)
            /exd/mission/status   (exd_interfaces/MissionStatus)
Services:   /exd/mission/start, /exd/mission/abort (std_srvs/Trigger)
Calls:      /exd/flight/arm, /exd/flight/takeoff, /exd/flight/land

Start a mission from a terminal:
    ros2 service call /exd/mission/start std_srvs/srv/Trigger
"""
import math

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_interfaces.msg import DetectionArray, FlightState, MissionStatus, ObstacleArray
from exd_interfaces.srv import Takeoff
from exd_mission import mission_logic as ml


class MissionNode(ExdNode):
    def __init__(self):
        super().__init__('mission_node', subsystem='mission')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('takeoff_altitude', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('obstacle_stop_distance', 0.6)
        self.declare_parameter('obstacle_timeout', 1.0)   # s; stale LiDAR data = unsafe
        self.declare_parameter('waypoints', [2.0, 0.0, 1.5])
        self.declare_parameter('rate_hz', 5.0)

        self.mission = ml.Mission(
            ml.parse_waypoints(list(self.get_parameter('waypoints').value)),
            takeoff_altitude=float(self.get_parameter('takeoff_altitude').value),
            tolerance=float(self.get_parameter('waypoint_tolerance').value),
            stop_distance=float(self.get_parameter('obstacle_stop_distance').value))
        self.obstacle_timeout = float(self.get_parameter('obstacle_timeout').value)

        self.flight_state = None
        self.min_obstacle = math.inf
        self.last_obstacle_time = None
        self.pending = {}   # command -> future

        self.create_subscription(FlightState, topics.FLIGHT_STATE, self.on_flight_state, 10)
        self.create_subscription(ObstacleArray, topics.LIDAR_OBSTACLES, self.on_obstacles, 10)
        self.create_subscription(DetectionArray, topics.CAMERA_DETECTIONS, self.on_detections, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, topics.FLIGHT_SETPOINT, 10)
        self.status_pub = self.create_publisher(MissionStatus, topics.MISSION_STATUS, 10)

        self.arm_cli = self.create_client(SetBool, topics.FLIGHT_ARM)
        self.takeoff_cli = self.create_client(Takeoff, topics.FLIGHT_TAKEOFF)
        self.land_cli = self.create_client(Trigger, topics.FLIGHT_LAND)

        self.create_service(Trigger, topics.MISSION_START, self.on_start)
        self.create_service(Trigger, topics.MISSION_ABORT, self.on_abort)

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)

    # ------------------------------------------------------------------ inputs
    def on_flight_state(self, msg):
        self.flight_state = msg

    def on_obstacles(self, msg):
        self.min_obstacle = msg.min_distance
        self.last_obstacle_time = self.get_clock().now()

    def on_detections(self, msg):
        # TODO(mission): use detections (e.g. fly towards the exit sign)
        for d in msg.detections:
            self.get_logger().debug(f'saw {d.label} ({d.confidence:.2f})')

    def on_start(self, req, res):
        if self.flight_state is None:
            res.success, res.message = False, 'no flight state yet'
            return res
        self.execute(self.mission.start())
        res.success = self.mission.state == ml.ARMING
        res.message = self.mission.detail
        return res

    def on_abort(self, req, res):
        self.execute(self.mission.abort())
        res.success, res.message = True, self.mission.detail
        return res

    # ------------------------------------------------------------------ loop
    def tick(self):
        if self.flight_state is None:
            self.set_status(self.WARN, 'waiting for /exd/flight/state')
            return
        if self.auto_start and self.mission.state == ml.IDLE and self.arm_cli.service_is_ready():
            self.auto_start = False
            self.get_logger().info('auto_start: starting mission')
            self.execute(self.mission.start())

        self.collect_results()

        p = self.flight_state.pose.position
        telemetry = ml.Telemetry(
            armed=self.flight_state.armed,
            in_air=self.flight_state.in_air,
            position=(p.x, p.y, p.z),
            min_obstacle_distance=self.current_obstacle_distance())
        self.execute(self.mission.update(telemetry))
        self.publish_status()

    def current_obstacle_distance(self) -> float:
        if self.last_obstacle_time is None:
            return math.inf   # TODO(mission): decide if flying without LiDAR is allowed
        age = (self.get_clock().now() - self.last_obstacle_time).nanoseconds * 1e-9
        if age > self.obstacle_timeout:
            return 0.0        # stale data -> treat as blocked -> HOLD
        return self.min_obstacle

    # ------------------------------------------------------------------ outputs
    def execute(self, step: ml.Step):
        for cmd in step.commands:
            self.call(cmd)
        if step.setpoint is not None:
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = topics.FRAME_MAP
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = step.setpoint
            sp.pose.orientation.w = 1.0
            self.setpoint_pub.publish(sp)

    def call(self, cmd: str):
        if cmd == ml.CMD_ARM:
            client, req = self.arm_cli, SetBool.Request(data=True)
        elif cmd == ml.CMD_TAKEOFF:
            client, req = self.takeoff_cli, Takeoff.Request(altitude=self.mission.takeoff_altitude)
        else:
            client, req = self.land_cli, Trigger.Request()
        if not client.service_is_ready():
            self.get_logger().error(f'{client.srv_name} not available')
            self.execute(self.mission.on_command_result(cmd, False, 'service not available'))
            return
        self.pending[cmd] = client.call_async(req)

    def collect_results(self):
        for cmd, fut in list(self.pending.items()):
            if fut.done():
                del self.pending[cmd]
                res = fut.result()
                ok = bool(res and res.success)
                msg = res.message if res else 'no response'
                self.execute(self.mission.on_command_result(cmd, ok, msg))

    def publish_status(self):
        m = self.mission
        st = MissionStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.state = m.state
        st.detail = m.detail
        st.waypoint_index = m.index
        st.waypoint_count = len(m.waypoints)
        self.status_pub.publish(st)
        level = self.ERROR if m.state == ml.ABORTED else self.OK
        self.set_status(level, f'{m.state}: {m.detail}')


def main(args=None):
    run_node(MissionNode, args)


if __name__ == '__main__':
    main()
