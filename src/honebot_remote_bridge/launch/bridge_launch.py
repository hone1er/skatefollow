"""turtlesim with the handheld ESP32 remote, alone or together with the follow-bot code.

Remote only (it drives turtle1):
    ros2 launch honebot_remote_bridge bridge_launch.py                         # remote board on /dev/ttyUSB0
    ros2 launch honebot_remote_bridge bridge_launch.py source:=esp32           # robot ESP32 on /dev/ttyUSB0

Follow bot + remote override (turtle1 = the skater, turtle2 = the robot):
    ros2 launch honebot_remote_bridge bridge_launch.py follow:=true                  # remote board on USB
    ros2 launch honebot_remote_bridge bridge_launch.py follow:=true source:=esp32    # robot ESP32 on USB
  then drive the skater from a second terminal:   ros2 run turtlesim turtle_teleop_key

  turtle2 is driven by the follower (simulated UWB: pose_to_target -> /target -> follower -> /cmd_vel_raw),
  and the remote decides who really drives it, as on the robot:
    remote STOP / off / out of range  -> turtle2 stops (the follower too)
    RUN, stick centred                -> the follower drives turtle2
    RUN, stick pushed                 -> you drive turtle2; 0.8 s after you let go the follower takes over
  source:=serial  makes that decision in Python (same rules as the firmware); no safety gate.
  source:=esp32   the robot ESP32's firmware makes it, and the follower goes through the real safety gate
                  (ultrasonic) and esp32_bridge first:
                  follower -> /cmd_vel_raw -> safety_gate -> /cmd_vel -> esp32_bridge -> ESP32 -> /esp32/outputs
                  -> remote_bridge -> /turtle2/cmd_vel. Point the HC-SR04 at open space (> 1 m) or the gate
                  slows/stops the follower (cover it with your hand to test that: the stick still drives).

Arguments:
  source           serial (remote board on USB) | esp32 (robot ESP32 on USB, remote by radio)
  port             serial device of that board (default /dev/ttyUSB0)
  follow           false: the remote drives turtle1. true: follow bot on turtle2 + remote override
  scale            stick speed scale for the small window (path unchanged). Default 0.5 without follow,
                   1.0 with follow (so the stick and the follower use real robot speeds)
  follow_distance  metres turtle2 stays behind turtle1 (1.5)
  max_linear       follower top speed, m/s (2.0)
Node parameter defaults: config/remote.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _true(context, name):
    return context.launch_configurations.get(name, 'false').lower() in ('true', '1', 'yes')


def _setup(context):
    cfg = context.launch_configurations
    source = cfg['source']
    port = cfg['port']
    follow = _true(context, 'follow')
    scale = float(cfg['scale']) if cfg['scale'] not in ('', 'auto') else (1.0 if follow else 0.5)
    params_file = os.path.join(get_package_share_directory('honebot_remote_bridge'), 'config', 'remote.yaml')

    actions = [Node(package='turtlesim', executable='turtlesim_node', name='sim')]

    if source == 'esp32':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('honebot_hardware'), 'launch', 'esp32.launch.py')),
            launch_arguments={'port': port}.items()))

    remaps = [('cmd_vel_out', '/turtle2/cmd_vel' if follow else '/turtle1/cmd_vel')]
    if follow:
        robot = 'turtle2'
        actions.append(Node(
            package='honebot_follow', executable='pose_to_target', name='pose_to_target', output='screen',
            parameters=[{'follower': robot}], remappings=[('target', '/target')]))
        actions.append(Node(
            package='honebot_follow', executable='follower', name='follower', output='screen',
            parameters=[{'follower': robot, 'spawn_follower': True, 'target_topic': '/target',
                         'cmd_vel_topic': '/cmd_vel_raw',
                         'follow_distance': float(cfg['follow_distance']),
                         'max_linear': float(cfg['max_linear'])}]))
        if source == 'esp32':
            # the real robot's path: gate (ultrasonic, link, stale command) -> /cmd_vel -> esp32_bridge
            actions.append(Node(package='honebot_safety', executable='safety_gate', name='safety_gate',
                                output='screen'))
        else:
            remaps.append(('cmd_vel_in', '/cmd_vel_raw'))
        actions.append(LogInfo(msg='Drive the skater (turtle1) from another terminal: '
                                   'ros2 run turtlesim turtle_teleop_key'))

    actions.append(Node(
        package='honebot_remote_bridge', executable='remote_bridge', name='remote_bridge', output='screen',
        parameters=[params_file, {'source': source, 'port': port, 'scale': scale}],
        remappings=remaps))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='serial',
                              description="'serial' = remote board on USB, 'esp32' = robot ESP32 on USB"),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial device'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='true: follow bot on turtle2, remote overrides it'),
        DeclareLaunchArgument('scale', default_value='auto',
                              description='Stick speed scale (auto: 0.5, or 1.0 with follow)'),
        DeclareLaunchArgument('follow_distance', default_value='1.5', description='Follower distance (m)'),
        DeclareLaunchArgument('max_linear', default_value='2.0', description='Follower top speed (m/s)'),
        OpaqueFunction(function=_setup),
    ])
