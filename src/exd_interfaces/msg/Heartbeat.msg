# Published once per second by every Escape-x-Drone node on /exd/heartbeat.
# The system monitor uses it to know which nodes are alive and healthy.

uint8 OK=0
uint8 WARN=1
uint8 ERROR=2

std_msgs/Header header
string node_name      # e.g. "camera_node"
string subsystem      # e.g. "camera", "lidar", "flight", "mission", "system"
uint8 level           # OK / WARN / ERROR
string message        # short human-readable status
