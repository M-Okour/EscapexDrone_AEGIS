"""Camera driver node.

Publishes:  /exd/camera/image_raw   (sensor_msgs/Image, bgr8)
            /exd/camera/camera_info (sensor_msgs/CameraInfo)

sim_mode:=true  -> synthetic test pattern (no hardware needed)
sim_mode:=false -> OpenCV VideoCapture on `device` (USB / CSI via GStreamer string)

TODO(camera):
  * pick the real camera (USB, CSI, RealSense, ZED ...) and its driver
  * calibrate it and load real intrinsics into CameraInfo
  * consider replacing this node with the vendor's ROS 2 driver and remapping
    its topics onto topics.CAMERA_IMAGE / CAMERA_INFO
"""
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_camera.image_utils import make_test_pattern


class CameraNode(ExdNode):
    def __init__(self):
        super().__init__('camera_node', subsystem='camera')

        self.declare_parameter('device', '0')       # index "0" or a GStreamer pipeline string
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', topics.FRAME_CAMERA)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = self.get_parameter('frame_id').value
        fps = float(self.get_parameter('fps').value)

        self.image_pub = self.create_publisher(Image, topics.CAMERA_IMAGE, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, topics.CAMERA_INFO, qos_profile_sensor_data)

        self.cap = None
        if not self.sim_mode:
            self._open_hardware(str(self.get_parameter('device').value))

        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / fps, self.tick)
        if self.sim_mode:
            self.set_status(self.OK, 'publishing synthetic images')

    def _open_hardware(self, device: str) -> None:
        try:
            import cv2  # OpenCV ships with JetPack; on a PC: sudo apt install python3-opencv
        except ImportError:
            self.set_status(self.ERROR, 'OpenCV (cv2) not installed')
            return
        source = int(device) if device.isdigit() else device
        backend = cv2.CAP_GSTREAMER if not device.isdigit() else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(source, backend)
        if not self.cap.isOpened():
            self.set_status(self.ERROR, f'cannot open camera "{device}"')
            self.cap = None
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.set_status(self.OK, f'camera "{device}" open')

    def grab_frame(self):
        """Return a BGR uint8 numpy image, or None."""
        if self.sim_mode:
            return make_test_pattern(self.width, self.height, time.monotonic() - self.t0)
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        if not ok:
            self.set_status(self.WARN, 'frame grab failed')
            return None
        self.set_status(self.OK, 'streaming')
        return frame

    def tick(self):
        frame = self.grab_frame()
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height, img.width = frame.shape[:2]
        img.encoding = 'bgr8'
        img.step = img.width * 3
        img.data = np.ascontiguousarray(frame).tobytes()
        self.image_pub.publish(img)

        info = CameraInfo()
        info.header = img.header
        info.width, info.height = img.width, img.height
        # TODO(camera): replace with calibrated intrinsics (camera_calibration package)
        f = float(img.width)
        cx, cy = img.width / 2.0, img.height / 2.0
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        self.info_pub.publish(info)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None):
    run_node(CameraNode, args)


if __name__ == '__main__':
    main()
