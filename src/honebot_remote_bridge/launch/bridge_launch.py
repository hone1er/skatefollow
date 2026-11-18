"""Drive turtlesim with the handheld ESP32 remote.

    # Bench: only the remote board, plugged into this machine's USB
    ros2 launch honebot_remote_bridge bridge_launch.py
    ros2 launch honebot_remote_bridge bridge_launch.py port:=/dev/ttyUSB1

    # Real chain: remote --ESP-NOW--> robot ESP32 (plugged into the Pi) --> esp32_bridge
    ros2 launch honebot_remote_bridge bridge_launch.py source:=esp32 port:=/dev/ttyUSB0

    # The remote drives turtle1 (stand-in for the skater); the follow-bot code chases it with turtle2
    ros2 launch honebot_remote_bridge bridge_launch.py follow:=true

Arguments:
  source           serial (remote board's USB printout) | esp32 (robot ESP32's real wheel outputs)
  port             serial device: the remote board (source:=serial) or the robot ESP32 (source:=esp32)
  scale            shrink speeds for the small turtlesim window (path shape is unchanged); default 0.5
  follow           also start the follower (turtle2) from honebot_follow, via the simulated-UWB pipeline
  follow_distance  metres turtle2 stays behind turtle1
Parameter defaults are in config/remote.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _true(context, name):
    return context.launch_configurations.get(name, 'false').lower() in ('true', '1', 'yes')


def _setup(context):
    cfg = context.launch_configurations
    source = cfg['source']
    port = cfg['port']
    follow = _true(context, 'follow')
    params_file = os.path.join(get_package_share_directory('honebot_remote_bridge'), 'config', 'remote.yaml')

    actions = []
    if follow:
        # turtle_follow starts turtlesim itself. No safety gate here: that would start a second esp32_bridge.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('honebot_follow'), 'launch', 'turtle_follow.launch.py')),
            launch_arguments={'use_safety': 'false', 'relative': 'true',
                              'follow_distance': cfg['follow_distance']}.items()))
    else:
        actions.append(Node(package='turtlesim', executable='turtlesim_node', name='sim'))

    if source == 'esp32':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('honebot_hardware'), 'launch', 'esp32.launch.py')),
            launch_arguments={'port': port}.items()))

    actions.append(Node(
        package='honebot_remote_bridge', executable='remote_bridge', name='remote_bridge',
        output='screen',
        parameters=[params_file, {'source': source, 'port': port, 'scale': float(cfg['scale'])}],
        remappings=[('cmd_vel_out', '/turtle1/cmd_vel')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='serial',
                              description="'serial' = remote board on USB, 'esp32' = robot ESP32 on USB"),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial device'),
        DeclareLaunchArgument('scale', default_value='0.5', description='Speed scale for turtlesim'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='Also run the follower (turtle2) chasing turtle1'),
        DeclareLaunchArgument('follow_distance', default_value='1.5',
                              description='Follower distance (m), with follow:=true'),
        OpaqueFunction(function=_setup),
    ])
