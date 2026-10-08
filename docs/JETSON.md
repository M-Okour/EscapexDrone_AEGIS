# Jetson setup (owned by Student 5)

Starter notes — Student 5 should turn this into a tested, step-by-step guide.

## Base system

- JetPack 6.x (Ubuntu 22.04) → ROS 2 **Humble**, same as the laptops and CI.
- Install ROS 2 Humble from the official apt repository (`ros-humble-ros-base`),
  then `python3-colcon-common-extensions` and `python3-rosdep`.
- Alternative: `dustynv/ros:humble-*` containers from jetson-containers.

## Build on the Jetson

```bash
cd ~/Escape-x-Drone
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch exd_bringup exd.launch.py              # sim first, always
ros2 launch exd_bringup exd.launch.py sim:=false   # then hardware
```

## Networking

- Same `ROS_DOMAIN_ID` on Jetson and ground laptop (pick one per team, e.g. 42).
- Keep them on the same Wi-Fi / Ethernet; test with `ros2 topic list` from the laptop.

## TODO

- [ ] udev rules for flight controller, LiDAR, camera (stable `/dev/` names)
- [ ] `nvpmodel` / `jetson_clocks` power mode for flight
- [ ] systemd service to start `exd.launch.py sim:=false` on boot
- [ ] rosbag recording of every flight (`ros2 bag record -a` with exclusions)
- [ ] GPU / power stats in `exd_system/host_stats.py` (jetson-stats `jtop`)
