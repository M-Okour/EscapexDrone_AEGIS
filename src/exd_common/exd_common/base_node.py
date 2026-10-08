"""ExdNode: the base class every Escape-x-Drone node should inherit from.

It gives you, for free:
  * a ``sim_mode`` parameter (True = no hardware, generate fake data)
  * a 1 Hz heartbeat on /exd/heartbeat so the system monitor can see you
  * ``self.set_status(level, text)`` to report OK / WARN / ERROR

Minimal example::

    from exd_common.base_node import ExdNode, run_node

    class MyNode(ExdNode):
        def __init__(self):
            super().__init__('my_node', subsystem='camera')
            self.set_status(ExdNode.OK, 'ready')

    def main(args=None):
        run_node(MyNode, args)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from exd_interfaces.msg import Heartbeat
from exd_common import topics


class ExdNode(Node):
    OK = Heartbeat.OK
    WARN = Heartbeat.WARN
    ERROR = Heartbeat.ERROR

    def __init__(self, node_name: str, subsystem: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.subsystem = subsystem

        self.declare_parameter('sim_mode', True)
        self.sim_mode = bool(self.get_parameter('sim_mode').value)

        self._hb_level = Heartbeat.OK
        self._hb_text = 'starting'
        self._hb_pub = self.create_publisher(Heartbeat, topics.HEARTBEAT, 10)
        self._hb_timer = self.create_timer(1.0, self._publish_heartbeat)

        self.get_logger().info(
            f'[{subsystem}] {node_name} starting ({"SIM" if self.sim_mode else "HARDWARE"} mode)')

    def set_status(self, level: int, text: str) -> None:
        """Update what this node reports in its heartbeat. Logs when the level changes."""
        if level != self._hb_level:
            log = {Heartbeat.OK: self.get_logger().info,
                   Heartbeat.WARN: self.get_logger().warn}.get(level, self.get_logger().error)
            log(f'status: {text}')
        self._hb_level = level
        self._hb_text = text

    def _publish_heartbeat(self) -> None:
        msg = Heartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = self.get_name()
        msg.subsystem = self.subsystem
        msg.level = self._hb_level
        msg.message = self._hb_text
        self._hb_pub.publish(msg)


def run_node(node_cls, args=None) -> None:
    """Standard main(): init, spin until Ctrl-C, clean shutdown."""
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
