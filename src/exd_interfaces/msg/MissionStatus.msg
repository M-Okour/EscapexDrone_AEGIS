# Published by exd_mission on /exd/mission/status.
std_msgs/Header header
string state            # IDLE, ARMING, TAKING_OFF, EXECUTING, HOLDING, LANDING, DONE, ABORTED
string detail           # human-readable reason / progress
uint32 waypoint_index
uint32 waypoint_count
