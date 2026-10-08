"""Single source of truth for topic, service and frame names.

Always import names from here instead of typing strings in your node:

    from exd_common import topics
    self.create_publisher(Image, topics.CAMERA_IMAGE, qos)

If you need a new topic that other subsystems will use, add it here (and to
docs/ARCHITECTURE.md) in a PR so the whole team sees it. Topics that are
private to your subsystem can live in your own package.
"""

# ---------------------------------------------------------------- camera (Student: Camera)
CAMERA_IMAGE = '/exd/camera/image_raw'          # sensor_msgs/Image
CAMERA_INFO = '/exd/camera/camera_info'         # sensor_msgs/CameraInfo
CAMERA_DETECTIONS = '/exd/camera/detections'    # exd_interfaces/DetectionArray

# ---------------------------------------------------------------- lidar (Student: LiDAR)
LIDAR_POINTS = '/exd/lidar/points'              # sensor_msgs/PointCloud2
LIDAR_OBSTACLES = '/exd/lidar/obstacles'        # exd_interfaces/ObstacleArray

# ---------------------------------------------------------------- flight (Student: Flight control)
FLIGHT_STATE = '/exd/flight/state'              # exd_interfaces/FlightState
FLIGHT_SETPOINT = '/exd/flight/setpoint'        # geometry_msgs/PoseStamped (frame: map)
FLIGHT_ARM = '/exd/flight/arm'                  # std_srvs/SetBool  (srv)
FLIGHT_TAKEOFF = '/exd/flight/takeoff'          # exd_interfaces/Takeoff (srv)
FLIGHT_LAND = '/exd/flight/land'                # std_srvs/Trigger (srv)

# ---------------------------------------------------------------- mission (Student: Mission)
MISSION_STATUS = '/exd/mission/status'          # exd_interfaces/MissionStatus
MISSION_START = '/exd/mission/start'            # std_srvs/Trigger (srv)
MISSION_ABORT = '/exd/mission/abort'            # std_srvs/Trigger (srv)

# ---------------------------------------------------------------- system (Student: Jetson/System)
HEARTBEAT = '/exd/heartbeat'                    # exd_interfaces/Heartbeat (every node)
SYSTEM_HEALTH = '/exd/system/health'            # diagnostic_msgs/DiagnosticArray

# ---------------------------------------------------------------- TF frames
FRAME_MAP = 'map'
FRAME_BASE = 'base_link'
FRAME_CAMERA = 'camera_link'
FRAME_LIDAR = 'lidar_link'
