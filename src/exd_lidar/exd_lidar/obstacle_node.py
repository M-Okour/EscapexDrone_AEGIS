"""Obstacle extraction node.

Subscribes: /exd/lidar/points    (sensor_msgs/PointCloud2)
Publishes:  /exd/lidar/obstacles (exd_interfaces/ObstacleArray)

The mission node uses ObstacleArray.min_distance to stop before hitting things.

TODO(lidar): replace nearest_per_sector() with real processing (filtering,
clustering, tracking) and tune the parameters in config/lidar.yaml.
"""
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_interfaces.msg import Obstacle, ObstacleArray
from exd_lidar.geometry import nearest_per_sector


class ObstacleNode(ExdNode):
    def __init__(self):
        super().__init__('obstacle_node', subsystem='lidar')
        self.declare_parameter('sectors', 8)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 10.0)

        self.pub = self.create_publisher(ObstacleArray, topics.LIDAR_OBSTACLES, 10)
        self.create_subscription(PointCloud2, topics.LIDAR_POINTS, self.on_points,
                                 qos_profile_sensor_data)
        self.set_status(self.WARN, 'waiting for point clouds')

    def on_points(self, msg: PointCloud2):
        raw = point_cloud2.read_points_list(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array([[p.x, p.y, p.z] for p in raw], dtype=np.float32).reshape(-1, 3)

        nearest = nearest_per_sector(
            pts,
            sectors=int(self.get_parameter('sectors').value),
            min_range=float(self.get_parameter('min_range').value),
            max_range=float(self.get_parameter('max_range').value))

        out = ObstacleArray()
        out.header = msg.header
        for x, y, z, d in nearest:
            o = Obstacle()
            o.position.x, o.position.y, o.position.z = x, y, z
            o.distance = d
            o.radius = 0.0
            out.obstacles.append(o)
        out.min_distance = nearest[0][3] if nearest else float('inf')
        self.pub.publish(out)
        self.set_status(self.OK, f'closest obstacle {out.min_distance:.2f} m')


def main(args=None):
    run_node(ObstacleNode, args)


if __name__ == '__main__':
    main()
