# Roles: suggested scope per student

The project scope is deliberately open. This is a **starting point** — each
student should turn their section into concrete GitHub issues in week 1 and
agree them with the supervisor.

Common to everyone:
- Keep your node running in `sim_mode` at all times (CI checks it).
- Write unit tests for your logic modules.
- Keep your section of `docs/ARCHITECTURE.md` correct.
- Demo your subsystem at each milestone.

---

## Student 1 — Camera & Perception (`exd_camera`)

**Owns:** `camera_node`, `detector_node`, `config/camera.yaml`

| Milestone | Deliverable |
|---|---|
| M1 | Real camera streaming on `/exd/camera/image_raw` on laptop and Jetson; view in `rqt_image_view` |
| M2 | Camera calibrated; real intrinsics in `CameraInfo`; camera TF measured on the airframe |
| M3 | Detector for the agreed targets (e.g. exit sign / ArUco / person) publishing `DetectionArray` |
| M4 | Runs in real time on the Jetson (TensorRT / Isaac ROS); 3D position of targets if needed |

Questions to answer early: which camera (USB, CSI, depth)? what must the drone recognise?

---

## Student 2 — LiDAR & Obstacle Detection (`exd_lidar`)

**Owns:** `lidar_node`, `obstacle_node`, `config/lidar.yaml`

| Milestone | Deliverable |
|---|---|
| M1 | Vendor driver running; point cloud on `/exd/lidar/points`, visible in RViz |
| M2 | Filtering: remove drone body, ground/ceiling, noise |
| M3 | Real obstacle extraction (clustering) publishing `ObstacleArray` with sizes |
| M4 | Optional: local map (occupancy grid / OctoMap) or SLAM for localisation without GPS |

Questions to answer early: 2D or 3D LiDAR? indoor (no GPS) → is LiDAR also used for localisation?

---

## Student 3 — Flight Control (`exd_flight_control`)

**Owns:** `flight_interface_node`, `backends.py`, `config/flight.yaml`

| Milestone | Deliverable |
|---|---|
| M1 | Choose PX4 or ArduPilot; run SITL (Gazebo) + MAVROS on a laptop |
| M2 | `MavrosBackend` implemented: state, arm, takeoff, land, setpoints — works against SITL with the **unchanged** mission node |
| M3 | Bench test with the real flight controller (props off!) over serial from the Jetson |
| M4 | Failsafes: RC override, battery, geofence, link loss; first tethered flight |

Questions to answer early: which flight controller hardware? how will position be estimated indoors?

---

## Student 4 — Mission & Navigation (`exd_mission`)

**Owns:** `mission_node`, `mission_logic.py`, `config/mission.yaml`

| Milestone | Deliverable |
|---|---|
| M1 | Define the escape mission on paper (states, success criteria, failure cases) |
| M2 | Extend the state machine: return-to-home, battery/health failsafes, uses detections |
| M3 | Local avoidance: go around obstacles instead of only stopping |
| M4 | Full mission in Gazebo SITL with Student 3, then on the real drone |

Questions to answer early: what does "escape" mean exactly — find an exit? explore an unknown room?

---

## Student 5 — Jetson & System Integration (`exd_system`, `exd_bringup`)

**Owns:** `system_monitor_node`, launch files, `docker/`, CI, Jetson setup, `docs/JETSON.md`

| Milestone | Deliverable |
|---|---|
| M1 | Jetson flashed (JetPack 6), ROS 2 Humble installed, this repo builds and runs in sim on it |
| M2 | Hardware wiring + power plan; all sensors and the flight controller visible from the Jetson; udev rules |
| M3 | Stack auto-starts on boot (systemd); health monitor reports GPU/power; rosbag recording |
| M4 | Integration lead: weekly "merge day", Gazebo SITL environment for the whole team, flight test checklist |

Questions to answer early: which Jetson? how do we connect to it in the field (Wi-Fi, ROS_DOMAIN_ID)?
