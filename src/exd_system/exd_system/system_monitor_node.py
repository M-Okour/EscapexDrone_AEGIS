"""System monitor: is every node alive, and is the Jetson healthy?

Subscribes: /exd/heartbeat      (exd_interfaces/Heartbeat, from every ExdNode)
Publishes:  /exd/system/health  (diagnostic_msgs/DiagnosticArray, 1 Hz)

View it with:  ros2 run rqt_robot_monitor rqt_robot_monitor
          or:  ros2 topic echo /exd/system/health

TODO(jetson):
  * decide what happens when a critical node dies (tell mission to land?)
  * add GPU / power stats in host_stats.py
  * systemd service so the stack starts on boot (see scripts/)
"""
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from exd_common import topics
from exd_common.base_node import ExdNode, run_node
from exd_interfaces.msg import Heartbeat
from exd_system import host_stats

LEVELS = {Heartbeat.OK: DiagnosticStatus.OK, Heartbeat.WARN: DiagnosticStatus.WARN,
          Heartbeat.ERROR: DiagnosticStatus.ERROR}


def kv(key, value):
    return KeyValue(key=str(key), value=f'{value:.1f}' if isinstance(value, float) else str(value))


class SystemMonitorNode(ExdNode):
    def __init__(self):
        super().__init__('system_monitor_node', subsystem='system')
        self.declare_parameter('expected_nodes', [''])
        self.declare_parameter('stale_after', 3.0)
        self.declare_parameter('temp_warn_c', 80.0)
        self.declare_parameter('mem_warn_percent', 90.0)

        self.expected = [n for n in self.get_parameter('expected_nodes').value if n]
        self.stale_after = float(self.get_parameter('stale_after').value)
        self.nodes = {}  # name -> (last Heartbeat, receive time)

        self.create_subscription(Heartbeat, topics.HEARTBEAT, self.on_heartbeat, 50)
        self.pub = self.create_publisher(DiagnosticArray, topics.SYSTEM_HEALTH, 10)
        self.create_timer(1.0, self.tick)

    def on_heartbeat(self, msg: Heartbeat):
        self.nodes[msg.node_name] = (msg, self.get_clock().now())

    def tick(self):
        now = self.get_clock().now()
        arr = DiagnosticArray()
        arr.header.stamp = now.to_msg()
        problems = []

        for name in sorted(set(self.nodes) | set(self.expected)):
            st = DiagnosticStatus(name=f'exd/nodes/{name}', hardware_id=name)
            if name not in self.nodes:
                st.level, st.message = DiagnosticStatus.ERROR, 'never seen'
            else:
                hb, t = self.nodes[name]
                age = (now - t).nanoseconds * 1e-9
                st.values = [kv('subsystem', hb.subsystem), kv('age_s', age)]
                if age > self.stale_after:
                    st.level, st.message = DiagnosticStatus.STALE, f'no heartbeat for {age:.0f}s'
                else:
                    st.level, st.message = LEVELS.get(hb.level, DiagnosticStatus.ERROR), hb.message
            if st.level != DiagnosticStatus.OK:
                problems.append(name)
            arr.status.append(st)

        arr.status.append(self.host_status())
        self.pub.publish(arr)

        if problems:
            self.set_status(self.WARN, 'problems: ' + ', '.join(problems))
        else:
            self.set_status(self.OK, f'{len(self.nodes)} nodes healthy')

    def host_status(self) -> DiagnosticStatus:
        st = DiagnosticStatus(name='exd/host', hardware_id='onboard_computer')
        st.level, st.message = DiagnosticStatus.OK, 'ok'
        warnings = []

        temps = host_stats.read_temperatures()
        for zone, c in temps.items():
            st.values.append(kv(f'temp_{zone}_c', c))
        if temps and max(temps.values()) > float(self.get_parameter('temp_warn_c').value):
            warnings.append(f'hot {max(temps.values()):.0f}C')

        mem = host_stats.read_memory()
        for k, v in mem.items():
            st.values.append(kv(k, v))
        if mem and mem['used_percent'] > float(self.get_parameter('mem_warn_percent').value):
            warnings.append(f'memory {mem["used_percent"]:.0f}%')

        for k, v in {**host_stats.read_load(), **host_stats.read_disk()}.items():
            st.values.append(kv(k, v))

        if warnings:
            st.level, st.message = DiagnosticStatus.WARN, ', '.join(warnings)
        return st


def main(args=None):
    run_node(SystemMonitorNode, args)


if __name__ == '__main__':
    main()
