# Obstacles around the drone. header.frame_id is normally "lidar_link".
std_msgs/Header header
Obstacle[] obstacles
float32 min_distance   # distance to the closest obstacle, +inf if none
