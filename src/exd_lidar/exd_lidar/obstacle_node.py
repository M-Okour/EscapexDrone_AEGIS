
"""LiDAR obstacle detection using filtering and clustering."""

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_interfaces.msg import Obstacle, ObstacleArray
from exd_lidar.geometry import (
    filter_lidar_points,
    segment_lidar_with_corners,
    calculate_min_distance,
)


class ObstacleNode(ExdNode):
    def __init__(self):
        super().__init__('obstacle_node', subsystem='lidar')

        self.declare_parameter('sectors', 8)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('cluster_tolerance', 0.3)
        self.declare_parameter('cluster_min_points', 3)

        self.pub = self.create_publisher(
            ObstacleArray, topics.LIDAR_OBSTACLES, 10
        )

        self.create_subscription(
            PointCloud2,
            topics.LIDAR_POINTS,
            self.on_points,
            qos_profile_sensor_data
        )

        self.set_status(self.WARN, 'waiting for point clouds')

    def on_points(self, msg: PointCloud2):
        raw = point_cloud2.read_points_list(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False
        )

        pts = np.array(
            [[p.x, p.y, p.z] for p in raw],
            dtype=np.float32
        ).reshape(-1, 3)

        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)

        filtered = filter_lidar_points(
            pts,
            min_range=min_range,
            max_range=max_range
        )

        # Preserve scan order and NaN gaps for segmentation.
        ordered_points = np.array(
            [[p.x, p.y, p.z] for p in raw],
            dtype=np.float32
        ).reshape(-1, 3)

        clusters = segment_lidar_with_corners(
            ordered_points,
            gap_threshold=float(
                self.get_parameter('cluster_tolerance').value
            ),
            min_points=int(
                self.get_parameter('cluster_min_points').value
            )
        )

        out = ObstacleArray()
        out.header = msg.header

        for cluster in clusters:
            center = np.mean(cluster, axis=0)

            distances_from_center = np.linalg.norm(
                cluster[:, :2] - center[:2],
                axis=1
            )

            radius = float(np.max(distances_from_center))

            o = Obstacle()
            o.position.x = float(center[0])
            o.position.y = float(center[1])
            o.position.z = float(center[2])
            o.distance = float(np.min(np.hypot(cluster[:, 0], cluster[:, 1])))
            o.radius = radius

            out.obstacles.append(o)

        # No valid returns means the path must be treated as blocked.
        out.min_distance = calculate_min_distance(filtered)

        self.pub.publish(out)

        self.set_status(
            self.OK,
            f'{len(clusters)} clusters, '
            f'closest return {out.min_distance:.2f} m'
        )


def main(args=None):
    run_node(ObstacleNode, args)


if __name__ == '__main__':
    main()
