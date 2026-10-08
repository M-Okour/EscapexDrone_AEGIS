"""Start the Escape-x-Drone stack.

Examples
  Whole stack in simulation (no hardware), mission starts by itself:
      ros2 launch exd_bringup exd.launch.py auto_start:=true

  Only my subsystem (e.g. camera) while I develop it:
      ros2 launch exd_bringup exd.launch.py lidar:=false flight:=false mission:=false

  On the real drone:
      ros2 launch exd_bringup exd.launch.py sim:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    ] + [
        DeclareLaunchArgument(s, default_value='true', description=f'launch the {s} subsystem')
        for s in SUBSYSTEMS
    ]

    def on(name):
        return IfCondition(LaunchConfiguration(name))

    nodes = [
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
