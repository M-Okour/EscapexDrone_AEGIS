# Snapshot of the flight controller, published by exd_flight_control at ~20 Hz.

uint8 MODE_UNKNOWN=0
uint8 MODE_MANUAL=1
uint8 MODE_HOLD=2
uint8 MODE_OFFBOARD=3     # following /exd/flight/setpoint
uint8 MODE_TAKEOFF=4
uint8 MODE_LANDING=5

std_msgs/Header header         # frame_id = "map"
bool connected                 # link to flight controller (always true in sim)
bool armed
bool in_air
uint8 mode
float32 battery_percent        # 0 - 100, negative if unknown
geometry_msgs/Pose pose        # in "map" frame, ENU, metres
geometry_msgs/Twist velocity   # in "map" frame, m/s
