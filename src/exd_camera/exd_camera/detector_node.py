"""Object detection node.

Subscribes: /exd/camera/image_raw  (sensor_msgs/Image)
Publishes:  /exd/camera/detections (exd_interfaces/DetectionArray)

Right now it runs a toy detector (finds the white square in the sim image).

TODO(camera):
  * decide what the drone must see (exit sign? ArUco markers? people? doors?)
  * replace `detect()` with a real model; on the Jetson use TensorRT / DeepStream
    or Isaac ROS for speed
  * fill in Detection.position (3D) using depth or the LiDAR, if needed
"""
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_camera.image_utils import find_bright_square
from exd_interfaces.msg import Detection, DetectionArray


class DetectorNode(ExdNode):
    def __init__(self):
        super().__init__('detector_node', subsystem='camera')
        self.declare_parameter('process_every_n', 3)   # skip frames to save CPU/GPU
        self.every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.count = 0

        self.pub = self.create_publisher(DetectionArray, topics.CAMERA_DETECTIONS, 10)
        self.create_subscription(Image, topics.CAMERA_IMAGE, self.on_image, qos_profile_sensor_data)
        self.set_status(self.WARN, 'waiting for images')

    def on_image(self, msg: Image):
        self.count += 1
        if self.count % self.every_n:
            return
        if msg.encoding != 'bgr8':
            self.set_status(self.ERROR, f'unsupported encoding {msg.encoding}')
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        out = DetectionArray()
        out.header = msg.header
        out.detections = self.detect(frame)
        self.pub.publish(out)
        self.set_status(self.OK, f'{len(out.detections)} detection(s)')

    def detect(self, frame: np.ndarray):
        """Return a list of exd_interfaces/Detection. Replace me!"""
        box = find_bright_square(frame)
        if box is None:
            return []
        d = Detection()
        d.label = 'target'
        d.confidence = 1.0
        d.x, d.y, d.width, d.height = box
        d.has_position = False
        return [d]


def main(args=None):
    run_node(DetectorNode, args)


if __name__ == '__main__':
    main()
