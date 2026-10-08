# One object found by the camera pipeline.

string label          # class name, e.g. "person", "door", "aruco_7"
float32 confidence    # 0.0 - 1.0

# Bounding box in image pixels (top-left corner + size)
int32 x
int32 y
int32 width
int32 height

# Optional 3D position in the camera frame (metres). Only valid if has_position.
bool has_position
geometry_msgs/Point position
