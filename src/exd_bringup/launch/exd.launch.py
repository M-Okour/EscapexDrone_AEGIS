"""Start the Escape-x-Drone stack.

Examples
  Whole stack in simulation (no hardware), mission starts by itself:
      ros2 launch exd_bringup exd.launch.py auto_start:=true

  Only my subsystem (e.g. camera) while I develop it:
      ros2 launch exd_bringup exd.launch.py lidar:=false flight:=false mission:=false

  On the real drone:
      ros2 launch exd_bringup exd.launch.py sim:=false

  Hardware mode also starts the vendor sensor drivers (Jetson only, install with
  `sudo apt install ros-jazzy-realsense2-camera ros-jazzy-rplidar-ros`):
      camera -> realsense2_camera (Intel RealSense D455)
      lidar  -> rplidar_ros       (Slamtec RPLIDAR A2M12 on lidar_port)
  Run only the sensors:
      ros2 launch exd_bringup exd.launch.py sim:=false flight:=false mission:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SUBSYSTEMS = ['camera', 'lidar', 'flight', 'mission', 'system']


def config(pkg, name):
    return PathJoinSubstitution([FindPackageShare(pkg), 'config', name])


def generate_launch_description():
    sim = ParameterValue(LaunchConfiguration('sim'), value_type=bool)
    common = {'sim_mode': sim}

    args = [
        DeclareLaunchArgument('sim', default_value='true',
                              description='true = no hardware, every node fakes its data'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='start the mission automatically'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='serial port of the RPLIDAR (hardware only)'),
        DeclareLaunchArgument('lidar_baud', default_value='256000',
                              description='RPLIDAR baud rate (A2M12 = 256000)'),
    ] + [
        DeclareLaunchArgument(s, default_value='true', description=f'launch the {s} subsystem')
        for s in SUBSYSTEMS
    ]

    def on(name):
        return IfCondition(LaunchConfiguration(name))

    def on_hw(name):
        """Subsystem enabled AND hardware mode (sim:=false)."""
        return IfCondition(PythonExpression([
            "'", LaunchConfiguration(name), "'.lower() == 'true' and '",
            LaunchConfiguration('sim'), "'.lower() == 'false'"]))

    drivers = [
        # ---------------- vendor drivers, hardware mode only
        # RealSense D455 -> /camera/camera/color/*, relayed by camera_node.
        # Its TF tree hangs off camera_link, which is mounted on base_link below.
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='camera', namespace='camera',
             parameters=[{
                 'camera_name': 'camera',
                 # reset the D455 on start: recovers it if a previous driver left it
                 # stuck ("Frames didn't arrive within 5 seconds")
                 'initial_reset': True,
                 'enable_color': True,
                 'enable_depth': True,
                 'rgb_camera.color_profile': '640,480,15',
                 'depth_module.depth_profile': '640,480,15',
                 'pointcloud.enable': False,
                 'align_depth.enable': False,
             }],
             condition=on_hw('camera'), output='screen'),
        # RPLIDAR A2M12 -> /exd/lidar/scan, converted by lidar_node.
        # angle_compensate must stay false: the apt driver (SDK 1.12) crashes
        # with it on in the A2M12's Sensitivity mode.
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar_node',
             parameters=[{
                 'serial_port': LaunchConfiguration('lidar_port'),
                 'serial_baudrate': ParameterValue(LaunchConfiguration('lidar_baud'),
                                                   value_type=int),
                 'frame_id': 'lidar_link',
                 'angle_compensate': False,
             }],
             remappings=[('scan', '/exd/lidar/scan')],
             condition=on_hw('lidar'), output='screen'),
    ]

    nodes = drivers + [
        # ---------------- camera
        Node(package='exd_camera', executable='camera_node', name='camera_node',
             parameters=[config('exd_camera', 'camera.yaml'), common],
             condition=on('camera'), output='screen'),
        Node(package='exd_camera', executable='detector_node', name='detector_node',
             parameters=[config('exd_camera', 'camera.yaml'), common],
             condition=on('camera'), output='screen'),
        # ---------------- lidar
        Node(package='exd_lidar', executable='lidar_node', name='lidar_node',
             parameters=[config('exd_lidar', 'lidar.yaml'), common],
             condition=on('lidar'), output='screen'),
        Node(package='exd_lidar', executable='obstacle_node', name='obstacle_node',
             parameters=[config('exd_lidar', 'lidar.yaml'), common],
             condition=on('lidar'), output='screen'),
        # ---------------- flight control
        Node(package='exd_flight_control', executable='flight_interface_node',
             name='flight_interface_node',
             parameters=[config('exd_flight_control', 'flight.yaml'), common],
             condition=on('flight'), output='screen'),
        # ---------------- mission
        Node(package='exd_mission', executable='mission_node', name='mission_node',
             parameters=[config('exd_mission', 'mission.yaml'), common,
                         {'auto_start': ParameterValue(LaunchConfiguration('auto_start'),
                                                       value_type=bool)}],
             condition=on('mission'), output='screen'),
        # ---------------- system / jetson
        Node(package='exd_system', executable='system_monitor_node', name='system_monitor_node',
             parameters=[config('exd_system', 'system.yaml'), common],
             condition=on('system'), output='screen'),

        # ---------------- static sensor mounts (base_link -> sensor)
        # TODO(system/camera/lidar): measure the real mounts on the airframe (x y z yaw pitch roll)
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_camera',
             arguments=['--x', '0.10', '--y', '0', '--z', '-0.05',
                        '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_lidar',
             arguments=['--x', '0', '--y', '0', '--z', '0.08',
                        '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'lidar_link']),
    ]

    return LaunchDescription(args + nodes)
