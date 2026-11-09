import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control_node')
        self.get_logger().info('Flight Control Node initialized (Hardware-Ready).')

        # Placeholder Subscribers (Update topic names when teammates provide them)
        self.create_subscription(String, '/camera/detections', self.camera_callback, 10)
        self.create_subscription(Float32, '/lidar/closest_obstacle', self.lidar_callback, 10)

        # Internal State Management
        self.latest_obstacle_distance = 999.0
        self.target_detected = False

    def camera_callback(self, msg):
        self.get_logger().info(f'Camera Data Received: {msg.data}')
        self.target_detected = True

    def lidar_callback(self, msg):
        self.latest_obstacle_distance = msg.data
        if self.latest_obstacle_distance < 1.0:
            self.get_logger().warn('Obstacle too close! Initiating hold position...')

def main(args=None):
    rclpy.init(args=args)
    node = FlightControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
