# Architecture and interface contract

This file and `src/exd_common/exd_common/topics.py` must always agree. If you
change one, change the other in the same PR.

## Design rules

1. **Talk only through the contract.** Nodes never import another subsystem's
   Python code; they only use topics/services below and messages from
   `exd_interfaces` or standard ROS packages.
2. **One node owns the autopilot.** Only `flight_interface_node` talks to
   PX4/ArduPilot. Everyone else commands the drone through `/exd/flight/*`.
3. **Every node inherits `ExdNode`** — it gets `sim_mode` and a heartbeat for free.
4. **Every node runs in sim.** A node that needs hardware must still start in
   `sim_mode` and publish plausible fake data.
5. **Logic in plain Python, ROS in a thin node.** Put algorithms in a module with
   no `rclpy` import (e.g. `mission_logic.py`, `geometry.py`) so they can be unit-tested.
6. **Frames:** `map` (world, ENU, metres) → `base_link` (drone body, published by
   flight control) → `camera_link`, `lidar_link` (static, published by bringup).

## Topics

| Topic | Type | Publisher | Subscribers | Rate |
|---|---|---|---|---|
| `/exd/camera/image_raw` | `sensor_msgs/Image` (bgr8) | camera_node | detector_node | 15 Hz |
| `/exd/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_node | — | 15 Hz |
| `/exd/camera/detections` | `exd_interfaces/DetectionArray` | detector_node | mission_node | ~5 Hz |
| `/exd/lidar/points` | `sensor_msgs/PointCloud2` (`lidar_link`) | lidar_node | obstacle_node | 10 Hz |
| `/exd/lidar/obstacles` | `exd_interfaces/ObstacleArray` | obstacle_node | mission_node | 10 Hz |
| `/exd/flight/state` | `exd_interfaces/FlightState` | flight_interface_node | mission_node, lidar_node (sim) | 20 Hz |
| `/exd/flight/setpoint` | `geometry_msgs/PoseStamped` (`map`) | mission_node | flight_interface_node | ≥ 2 Hz while flying |
| `/exd/mission/status` | `exd_interfaces/MissionStatus` | mission_node | — | 5 Hz |
| `/exd/heartbeat` | `exd_interfaces/Heartbeat` | **every node** | system_monitor_node | 1 Hz each |
| `/exd/system/health` | `diagnostic_msgs/DiagnosticArray` | system_monitor_node | — | 1 Hz |
| `/tf` | `map → base_link` | flight_interface_node | all | 20 Hz |
| `/tf_static` | `base_link → camera_link, lidar_link` | bringup | all | — |

Sensor topics (image, points) use the `SensorData` QoS (best effort). Subscribe
with `qos_profile_sensor_data` or you will receive nothing.

## Services

| Service | Type | Server | Notes |
|---|---|---|---|
| `/exd/flight/arm` | `std_srvs/SetBool` | flight_interface_node | refused in the air |
| `/exd/flight/takeoff` | `exd_interfaces/Takeoff` | flight_interface_node | must be armed |
| `/exd/flight/land` | `std_srvs/Trigger` | flight_interface_node | auto-disarms on ground |
| `/exd/mission/start` | `std_srvs/Trigger` | mission_node | |
| `/exd/mission/abort` | `std_srvs/Trigger` | mission_node | lands if flying |

## Safety behaviour already in the skeleton

- Setpoint stream lost for > 1 s while following setpoints → flight control HOLDs.
- LiDAR data older than 1 s → mission treats the path as blocked and HOLDs.
- Obstacle closer than `obstacle_stop_distance` → mission HOLDs until clear.
- Node stops sending heartbeats for 3 s → system monitor marks it STALE.

## Changing the contract

1. Open an issue describing the change and who is affected.
2. Edit `exd_interfaces` / `topics.py` / this file in **one** PR.
3. Every affected subsystem owner approves before merge.
4. Prefer **adding** fields/topics over renaming or deleting them.
