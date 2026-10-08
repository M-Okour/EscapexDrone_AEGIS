"""Flight interface node: the ONLY node allowed to talk to the autopilot.

Publishes:  /exd/flight/state     (exd_interfaces/FlightState, 20 Hz)
            TF map -> base_link
Subscribes: /exd/flight/setpoint  (geometry_msgs/PoseStamped, frame map)
Services:   /exd/flight/arm       (std_srvs/SetBool)
            /exd/flight/takeoff   (exd_interfaces/Takeoff)
            /exd/flight/land      (std_srvs/Trigger)

Safety rule (same as PX4 offboard): if no setpoint arrives for
`setpoint_timeout` seconds while following setpoints, the drone holds position.
"""
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_flight_control.backends import MavrosBackend, SimBackend
from exd_interfaces.msg import FlightState
from exd_interfaces.srv import Takeoff


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FlightInterfaceNode(ExdNode):
    def __init__(self):
        super().__init__('flight_interface_node', subsystem='flight')
        self.declare_parameter('backend', 'sim')          # sim | mavros
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('setpoint_timeout', 1.0)
        self.declare_parameter('sim_max_speed', 1.0)
        self.declare_parameter('sim_max_climb', 0.8)

        backend = 'sim' if self.sim_mode else self.get_parameter('backend').value
        if backend == 'sim':
            self.backend = SimBackend(
                max_speed=float(self.get_parameter('sim_max_speed').value),
                max_climb=float(self.get_parameter('sim_max_climb').value))
        elif backend == 'mavros':
            self.backend = MavrosBackend(self)
        else:
            raise ValueError(f'unknown backend "{backend}"')
        self.backend_name = backend

        self.setpoint_timeout = float(self.get_parameter('setpoint_timeout').value)
        self.last_setpoint_time = None

        self.state_pub = self.create_publisher(FlightState, topics.FLIGHT_STATE, 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, topics.FLIGHT_SETPOINT, self.on_setpoint, 10)
        self.create_service(SetBool, topics.FLIGHT_ARM, self.on_arm)
        self.create_service(Takeoff, topics.FLIGHT_TAKEOFF, self.on_takeoff)
        self.create_service(Trigger, topics.FLIGHT_LAND, self.on_land)

        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(self.dt, self.tick)

    # ------------------------------------------------------------------ ROS callbacks
    def on_setpoint(self, msg: PoseStamped):
        if msg.header.frame_id not in ('', topics.FRAME_MAP):
            self.get_logger().warn(f'setpoint in frame "{msg.header.frame_id}" ignored (need map)',
                                   throttle_duration_sec=2.0)
            return
        p = msg.pose.position
        self.backend.set_setpoint(p.x, p.y, p.z, quaternion_to_yaw(msg.pose.orientation))
        self.last_setpoint_time = self.get_clock().now()

    def on_arm(self, req, res):
        res.success, res.message = self.backend.arm(req.data)
        self.get_logger().info(f'arm({req.data}) -> {res.success}: {res.message}')
        return res

    def on_takeoff(self, req, res):
        res.success, res.message = self.backend.takeoff(req.altitude)
        self.get_logger().info(f'takeoff({req.altitude:.1f}) -> {res.success}: {res.message}')
        return res

    def on_land(self, req, res):
        res.success, res.message = self.backend.land()
        self.get_logger().info(f'land -> {res.success}: {res.message}')
        return res

    # ------------------------------------------------------------------ main loop
    def tick(self):
        now = self.get_clock().now()
        s = self.backend.state()

        if (s.mode == FlightState.MODE_OFFBOARD and self.last_setpoint_time is not None
                and (now - self.last_setpoint_time).nanoseconds * 1e-9 > self.setpoint_timeout):
            self.get_logger().warn('setpoint stream lost -> HOLD')
            self.backend.hold()

        self.backend.update(self.dt)
        s = self.backend.state()

        msg = FlightState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = topics.FRAME_MAP
        msg.connected, msg.armed, msg.in_air = s.connected, s.armed, s.in_air
        msg.mode = s.mode
        msg.battery_percent = float(s.battery_percent)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = s.x, s.y, s.z
        q = yaw_to_quaternion(s.yaw)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = q
        msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z = s.vx, s.vy, s.vz
        self.state_pub.publish(msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = topics.FRAME_BASE
        t.transform.translation.x, t.transform.translation.y = s.x, s.y
        t.transform.translation.z = s.z
        t.transform.rotation = msg.pose.orientation
        self.tf.sendTransform(t)

        if not s.connected:
            self.set_status(self.ERROR, f'[{self.backend_name}] autopilot not connected')
        elif 0.0 <= s.battery_percent < 20.0:
            self.set_status(self.WARN, f'battery low {s.battery_percent:.0f}%')
        else:
            self.set_status(self.OK, f'[{self.backend_name}] armed={s.armed} z={s.z:.2f}')


def main(args=None):
    run_node(FlightInterfaceNode, args)


if __name__ == '__main__':
    main()
