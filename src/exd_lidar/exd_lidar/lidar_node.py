"""LiDAR driver node.

Publishes: /exd/lidar/points (sensor_msgs/PointCloud2, frame lidar_link)

sim_mode:=true  -> fake scan of a rectangular room, using the drone pose from
                   /exd/flight/state so the walls get closer as the drone moves
sim_mode:=false -> converts the 2D LaserScan of the vendor driver (Slamtec
                   RPLIDAR A2M12 via rplidar_ros, started by exd_bringup and
                   remapped to `scan_topic`) into the PointCloud2 the rest of
                   the stack expects.

TODO(lidar):
  * measure the sensor mount position/heading and update the static TF in exd_bringup
  * filter out the drone's own arms/legs (min_range in config/lidar.yaml, or angle masks)
"""
import math
import time

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_interfaces.msg import FlightState
from exd_lidar.geometry import scan_to_points, simulate_room_scan


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LidarNode(ExdNode):
    def __init__(self):
        super().__init__('lidar_node', subsystem='lidar')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('frame_id', topics.FRAME_LIDAR)
        self.declare_parameter('sim_room_half_x', 5.0)
        self.declare_parameter('sim_room_half_y', 3.0)
        self.declare_parameter('sim_noise', 0.01)
        self.declare_parameter('scan_topic', '/exd/lidar/scan')   # hardware: vendor LaserScan
        self.declare_parameter('stale_after', 1.0)                # s without scans -> ERROR

        self.frame_id = self.get_parameter('frame_id').value
        self.pose = (0.0, 0.0, 0.0)  # x, y, yaw of the drone (sim only)

        self.pub = self.create_publisher(PointCloud2, topics.LIDAR_POINTS, qos_profile_sensor_data)

        if self.sim_mode:
            self.create_subscription(FlightState, topics.FLIGHT_STATE, self.on_flight_state, 10)
            self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.sim_tick)
            self.set_status(self.OK, 'publishing simulated room scan')
        else:
            self.scan_topic = str(self.get_parameter('scan_topic').value)
            self.last_scan = None
            self.last_count = 0
            self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                     qos_profile_sensor_data)
            self.create_timer(1.0, self.check_scans)
            self.set_status(self.WARN, f'waiting for scans on {self.scan_topic}')

    # ------------------------------------------------------------ hardware
    def on_scan(self, msg: LaserScan):
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or self.frame_id
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, pts.tolist()))
        self.last_scan = time.monotonic()
        self.last_count = len(pts)

    def check_scans(self):
        stale = float(self.get_parameter('stale_after').value)
        if self.last_scan is None:
            self.set_status(self.WARN, f'waiting for scans on {self.scan_topic}')
        elif time.monotonic() - self.last_scan > stale:
            self.set_status(self.ERROR, f'no scans on {self.scan_topic} for > {stale:.0f} s')
        else:
            self.set_status(self.OK, f'{self.last_count} points per scan')

    # ------------------------------------------------------------ sim
    def on_flight_state(self, msg: FlightState):
        p = msg.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def sim_tick(self):
        x, y, yaw = self.pose
        pts = simulate_room_scan(
            x, y, yaw,
            half_x=float(self.get_parameter('sim_room_half_x').value),
            half_y=float(self.get_parameter('sim_room_half_y').value),
            noise=float(self.get_parameter('sim_noise').value))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, pts.tolist()))


def main(args=None):
    run_node(LidarNode, args)


if __name__ == '__main__':
    main()
