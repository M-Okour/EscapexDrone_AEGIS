cmake_minimum_required(VERSION 3.8)
project(exd_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Every message/service shared between subsystems lives here.
# Changing a file in this list changes the contract for the WHOLE team:
# open a PR and get a review from every affected subsystem owner.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heartbeat.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/FlightState.msg"
  "msg/MissionStatus.msg"
  "srv/Takeoff.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
