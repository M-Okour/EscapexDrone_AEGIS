# Jetson setup and hardware bring-up (owned by Student 5)

How to set up the drone's Jetson and run/test the real hardware: flight
controller, camera and LiDAR. Everything here was tested on the drone on
21 Oct 2026.

## Hardware

| Device | Model | Connection | Shows up as |
|---|---|---|---|
| Companion computer | Jetson Orin Nano, boots from SD card | — | hostname `escapex` |
| Flight controller | Holybro Pixhawk 6X, ArduPilot | Ethernet (Pixhawk **ETH** port ↔ Jetson `enP8p1s0`) | `192.168.144.14` |
| Depth camera | Intel RealSense D455 | USB 3 (blue port / USB 3 hub) | `/dev/video*` |
| 2D LiDAR | Slamtec RPLIDAR A2M12 | USB (CP210x adapter) | `/dev/ttyUSB0` |

The Pixhawk is also visible over USB (`/dev/ttyACM0/1`); that link is not used.

## 1. Base system

- **JetPack 7.2** (L4T R39.2, Ubuntu 24.04) → **ROS 2 Jazzy**.
  The rest of this repo's docs, the Dockerfile and CI still say Humble/22.04;
  the code itself builds unchanged on Jazzy.
- One script installs ROS 2 Jazzy, the repo's dependencies, builds the
  workspace and runs the unit tests and the simulation smoke test:

```bash
cd ~/Documents/EscapexDrone_Vortex
bash scripts/setup_jetson_jazzy.sh              # safe to re-run
WITH_GUI=1 bash scripts/setup_jetson_jazzy.sh   # also rviz2 / rqt (~1 GB)
```

  It adds `source /opt/ros/jazzy/setup.bash`, the workspace and
  `ROS_DOMAIN_ID=42` to `~/.bashrc`. Use the same `ROS_DOMAIN_ID` on any laptop
  that should see the drone's topics.
- **SD card:** keep the install minimal (`--no-install-recommends`,
  `sudo apt clean`), build with `--parallel-workers 2`, and don't record
  long rosbags to the card.

### Vendor drivers

```bash
sudo apt install --no-install-recommends -y \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-realsense2-camera ros-jazzy-rplidar-ros
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
sudo apt clean
```

Viewers (optional, ~1 GB; or run them on a laptop):

```bash
sudo apt install -y ros-jazzy-rviz2 ros-jazzy-rqt-image-view && sudo apt clean
```

## 2. Pixhawk over Ethernet (MAVLink → MAVROS)

### Jetson side (once)

```bash
sudo nmcli con mod "Wired connection 1" connection.autoconnect no
sudo nmcli con add type ethernet ifname enP8p1s0 con-name pixhawk \
  ipv4.method manual ipv4.addresses 192.168.144.10/24 ipv6.method disabled \
  connection.autoconnect yes
sudo nmcli con up pixhawk
```

No gateway on this link, so Wi-Fi keeps the internet connection.

### Pixhawk side (Mission Planner / QGroundControl over USB, ArduPilot ≥ 4.5)

| Parameter | Value | Notes |
|---|---|---|
| `NET_ENABLE` | 1 | write, **reboot**, then the other `NET_` params appear |
| `NET_DHCP` | **0** | if 1, the Pixhawk gives itself a `169.254.x.x` address and replies can't reach the Jetson |
| `NET_IPADDR0..3` | 192, 168, 144, 14 | one number per parameter |
| `NET_NETMASK` | 24 | |
| `NET_P1_TYPE` | 1 | UDP Client (Pixhawk sends to the Jetson) |
| `NET_P1_PROTOCOL` | 2 | MAVLink 2 |
| `NET_P1_IP0..3` | 192, 168, 144, 10 | the Jetson; one number per parameter |
| `NET_P1_PORT` | 14550 | |

Write, then power-cycle the Pixhawk (USB **and** battery unplugged).

### Run and test

```bash
ping -c 3 192.168.144.14                                     # Pixhawk answers
ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@
# second terminal
ros2 topic echo --once /mavros/state                         # connected: true
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: false}"
                                                              # success: True, ~1086 params
```

If something is wrong, look at the raw traffic (`sudo apt install tcpdump`):

```bash
sudo tcpdump -i enP8p1s0 -n -c 20
```

- nothing at all → Pixhawk networking is off (`NET_ENABLE`, reboot, cable in **ETH**)
- `BOOTP/DHCP Request` / `169.254.x.x` → set `NET_DHCP = 0`
- `192.168.144.14 > 192.168.144.10.14550: UDP` → link is fine

> `exd_flight_control`'s `MavrosBackend` is still a stub, so the stack can't
> command the Pixhawk yet. Run MAVROS on its own for now.

## 3. Camera and LiDAR

### Each driver on its own (first test after plugging in)

```bash
# RealSense D455
ros2 launch realsense2_camera rs_launch.py
ros2 topic hz /camera/camera/color/image_raw          # second terminal

# RPLIDAR A2M12 (executable is rplidar_composition; baud 256000)
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000 \
  -p frame_id:=lidar_link -p angle_compensate:=false
ros2 topic hz /scan                                   # ~13 Hz
```

Stop these before starting the stack: each sensor can only be opened once.

### With the stack

`sim:=false` starts the vendor drivers together with our nodes:

- `realsense2_camera` (640×480 @ 15 fps colour + depth) →
  `camera_node` relays `/camera/camera/color/image_raw` (rgb8) to
  `/exd/camera/image_raw` (bgr8) and `camera_info`
- `rplidar_ros` → `/exd/lidar/scan` → `lidar_node` converts it to
  `/exd/lidar/points` → `obstacle_node`

```bash
cd ~/Documents/EscapexDrone_Vortex
colcon build --symlink-install --parallel-workers 2 && source install/setup.bash
colcon test --packages-select exd_camera exd_lidar && colcon test-result

# sensors only (flight control and mission off)
ros2 launch exd_bringup exd.launch.py sim:=false flight:=false mission:=false
# other serial port / LiDAR model:
#   ... lidar_port:=/dev/ttyUSB1 lidar_baud:=256000
```

In a second terminal:

```bash
ros2 topic hz /exd/camera/image_raw                   # ~15 Hz
ros2 topic hz /camera/camera/depth/image_rect_raw     # depth, 15-25 Hz
ros2 topic hz /exd/lidar/points                       # ~13 Hz
ros2 topic echo --once /exd/lidar/obstacles | grep min_distance   # changes when you move a hand near the LiDAR
ros2 topic echo --once /exd/system/health
```

Expected launch log: `camera_node … status OK: relaying …`,
`lidar_node … status OK: ~700 points per scan`,
`obstacle_node … status OK: closest obstacle …`.
With `flight:=false mission:=false` the system monitor still lists
`flight_interface_node` and `mission_node` as problems; that's expected.

### Look at the data

```bash
ros2 run rqt_image_view rqt_image_view /exd/camera/image_raw
rviz2
```

In RViz:

1. **Global Options → Fixed Frame**: type `base_link` (there is no `map` frame
   while flight control is off).
2. **Add → By topic → `/exd/lidar/scan` → LaserScan**; set
   **Topic → Reliability Policy = Best Effort** (otherwise nothing shows) and
   **Size (m) = 0.05**.
3. Optional: **Add → By topic → `/exd/camera/image_raw` → Image** (Best Effort).
4. Optional 3D from the depth camera (extra CPU):
   `ros2 param set /camera/camera pointcloud.enable true`, then add
   `/camera/camera/depth/color/points` → PointCloud2 (Best Effort).

The RPLIDAR is a **2D** LiDAR: one horizontal slice around the drone at the
sensor's height. 3D comes from the D455 depth camera (≈ 87° × 58°, up to ~6 m).

### Known issues

| Symptom | Cause / fix |
|---|---|
| `rplidar_node`: "No executable found" | the apt package (SDK 1.12) names it `rplidar_composition` |
| RPLIDAR: `malloc(): invalid size (unsorted)`, driver aborts, LiDAR keeps spinning | bug in the apt driver with `angle_compensate:=true` in Sensitivity mode; keep it `false` (the launch file does) |
| RealSense: `Frames didn't arrived within 5 seconds` repeating | the driver (4.58.4) opens the unused infrared streams at 848×480 @ 30 next to depth @ 15 and ignores `enable_infra1/2:=false` at start-up. Colour and depth keep working; harmless for now. Try `ros2 param set /camera/camera enable_infra1 false` (and `infra2`) at runtime |
| RealSense starts but sends nothing | another driver still holds the camera (`ps aux \| grep realsense`), or it is stuck: unplug/replug. The launch file sets `initial_reset: true` |
| RViz shows nothing | Reliability Policy must be Best Effort; Fixed Frame `base_link` |

## TODO

- [x] JetPack 7.2 + ROS 2 Jazzy, repo builds and passes tests on the Jetson
- [x] Pixhawk over Ethernet, MAVROS connected
- [x] RealSense D455 and RPLIDAR A2M12 drivers integrated in `exd.launch.py`
- [ ] `MavrosBackend` in `exd_flight_control` (arm / takeoff / land / setpoints, GUIDED mode)
- [ ] measure camera and LiDAR mounts; update the static TFs in `exd.launch.py`
- [ ] udev rules for stable `/dev/` names (RPLIDAR → `/dev/rplidar`)
- [ ] 2D map with `slam_toolbox` from the RPLIDAR
- [ ] `nvpmodel` / `jetson_clocks` power mode for flight
- [ ] systemd service to start `exd.launch.py sim:=false` on boot
- [ ] rosbag recording of every flight (not on the SD card)
- [ ] GPU / power stats in `exd_system/host_stats.py` (jetson-stats `jtop`)
- [ ] update README, Dockerfile and CI from Humble to Jazzy
