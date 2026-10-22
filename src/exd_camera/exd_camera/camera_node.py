"""Camera driver node.

Publishes:  /exd/camera/image_raw   (sensor_msgs/Image, bgr8)
            /exd/camera/camera_info (sensor_msgs/CameraInfo)

sim_mode:=true  -> synthetic test pattern (no hardware needed)
sim_mode:=false, input_topic set (default, see config/camera.yaml)
                -> relay a vendor ROS 2 driver: the Intel RealSense D455 driver
                   (realsense2_camera, started by exd_bringup) publishes the
                   colour stream; this node converts it to bgr8 and republishes
                   it on the /exd/camera/* topics, keeping the heartbeat alive.
sim_mode:=false, input_topic ''
                -> OpenCV VideoCapture on `device` (USB / CSI via GStreamer string)

TODO(camera):
  * calibrate the camera (or trust the RealSense factory intrinsics, which
    are already in the relayed CameraInfo)
  * use the depth stream (/camera/camera/depth/image_rect_raw) to give
    detections a 3D position
"""
import array
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_camera.image_utils import make_test_pattern, to_bgr8


class CameraNode(ExdNode):
    def __init__(self):
        super().__init__('camera_node', subsystem='camera')

        self.declare_parameter('device', '0')       # index "0" or a GStreamer pipeline string
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', topics.FRAME_CAMERA)
        # hardware relay mode: topics of a vendor driver ('' = use OpenCV `device`)
        self.declare_parameter('input_topic', '')
        self.declare_parameter('input_info_topic', '')
        self.declare_parameter('stale_after', 1.0)  # s without input images -> WARN

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = self.get_parameter('frame_id').value
        fps = float(self.get_parameter('fps').value)

        self.image_pub = self.create_publisher(Image, topics.CAMERA_IMAGE, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, topics.CAMERA_INFO, qos_profile_sensor_data)

        self.cap = None
        self.t0 = time.monotonic()
        input_topic = str(self.get_parameter('input_topic').value)

        if not self.sim_mode and input_topic:
            self._start_relay(input_topic, str(self.get_parameter('input_info_topic').value))
            return

        if not self.sim_mode:
            self._open_hardware(str(self.get_parameter('device').value))
        self.timer = self.create_timer(1.0 / fps, self.tick)
        if self.sim_mode:
            self.set_status(self.OK, 'publishing synthetic images')

    # ------------------------------------------------------------ relay mode (RealSense etc.)
    def _start_relay(self, image_topic: str, info_topic: str) -> None:
        self.input_topic = image_topic
        self.last_frame = None          # monotonic time of the last relayed frame
        self.relayed = 0
        self.create_subscription(Image, image_topic, self.on_input_image, qos_profile_sensor_data)
        if info_topic:
            self.create_subscription(CameraInfo, info_topic, self.info_pub.publish,
                                     qos_profile_sensor_data)
        self.create_timer(1.0, self.check_relay)
        self.set_status(self.WARN, f'waiting for images on {image_topic}')

    def on_input_image(self, msg: Image) -> None:
        if msg.encoding == 'bgr8':
            out = msg                   # already in the format the stack expects
        else:
            try:
                frame = to_bgr8(msg.data, msg.width, msg.height, msg.step, msg.encoding)
            except ValueError as e:
                self.set_status(self.ERROR, f'{self.input_topic}: {e}')
                return
            out = Image()
            out.header = msg.header     # keep the driver's stamp and optical frame
            out.height, out.width = frame.shape[:2]
            out.encoding = 'bgr8'
            out.step = out.width * 3
            out.data = array.array('B', frame.tobytes())
        self.image_pub.publish(out)
        self.last_frame = time.monotonic()
        self.relayed += 1

    def check_relay(self) -> None:
        stale = float(self.get_parameter('stale_after').value)
        if self.last_frame is None:
            self.set_status(self.WARN, f'waiting for images on {self.input_topic}')
        elif time.monotonic() - self.last_frame > stale:
            self.set_status(self.ERROR, f'no images on {self.input_topic} for > {stale:.0f} s')
        else:
            self.set_status(self.OK, f'relaying {self.input_topic} ({self.relayed} frames)')

    # ------------------------------------------------------------ OpenCV / sim mode
    def _open_hardware(self, device: str) -> None:
        try:
            import cv2  # on Ubuntu: sudo apt install python3-opencv
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
        # array.array skips rclpy's slow per-byte validation of uint8[] fields
        img.data = array.array('B', np.ascontiguousarray(frame).tobytes())
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
