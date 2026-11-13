"""Start turtlesim plus the follower, optionally through the safety gate and/or
via the relative-target path that the real robot will use.

    ros2 launch honebot_follow turtle_follow.launch.py
    ros2 launch honebot_follow turtle_follow.launch.py follow_distance:=2.5
    ros2 launch honebot_follow turtle_follow.launch.py use_safety:=false      # no ESP32 needed
    ros2 launch honebot_follow turtle_follow.launch.py relative:=true         # UWB-style pipeline
    ros2 launch honebot_follow turtle_follow.launch.py relative:=true ackermann:=true

use_safety (default true): follower -> /cmd_vel_raw -> safety_gate -> /turtle2/cmd_vel,
    and the ESP32 bridge is started so the gate has a range sensor and link status.
    Cover the ultrasonic sensor with your hand: the chasing turtle freezes.
relative (default false): pose_to_target converts the two turtle poses into a
    relative target on /target (simulated UWB) and the follower consumes that
    instead of the poses. Same behaviour on screen, different plumbing.
ackermann: the follower may not turn in place (creeps forward instead).

Drive the leader from a second terminal:
    ros2 run turtlesim turtle_teleop_key
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _true(context, name):
    return context.launch_configurations.get(name, 'false').lower() in ('true', '1', 'yes')


def _setup(context):
    use_safety = _true(context, 'use_safety')
    relative = _true(context, 'relative')
    ackermann = _true(context, 'ackermann')
    follower_turtle = 'turtle2'
    turtle_cmd = f'/{follower_turtle}/cmd_vel'

    follower_params = {
        'follow_distance': float(context.launch_configurations['follow_distance']),
        'max_linear': float(context.launch_configurations['max_linear']),
        'ackermann': ackermann,
        'follower': follower_turtle,
        'spawn_follower': True,
    }
    # Where the follower's output goes: into the gate, or straight to the turtle.
    follower_params['cmd_vel_topic'] = '/cmd_vel_raw' if use_safety else turtle_cmd
    if relative:
        follower_params['target_topic'] = '/target'

    actions = [Node(package='turtlesim', executable='turtlesim_node', name='sim')]

    if relative:
        actions.append(Node(
            package='honebot_follow', executable='pose_to_target', name='pose_to_target',
            output='screen', parameters=[{'follower': follower_turtle}],
            remappings=[('target', '/target')]))

    actions.append(Node(
        package='honebot_follow', executable='follower', name='follower',
        output='screen', parameters=[follower_params]))

    if use_safety:
        actions.append(Node(
            package='honebot_safety', executable='safety_gate', name='safety_gate',
            output='screen',
            parameters=[{'stop_distance': float(context.launch_configurations['stop_distance'])}],
            remappings=[('cmd_vel', turtle_cmd)]))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('honebot_hardware'), 'launch', 'esp32.launch.py'])),
            launch_arguments={'port': LaunchConfiguration('port')}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='1.5',
                              description='Metres to stay behind the leader'),
        DeclareLaunchArgument('max_linear', default_value='2.0',
                              description='Follower top speed (m/s)'),
        DeclareLaunchArgument('use_safety', default_value='true',
                              description='Route commands through the safety gate (needs the ESP32)'),
        DeclareLaunchArgument('relative', default_value='false',
                              description='Use the relative-target (simulated UWB) pipeline'),
        DeclareLaunchArgument('ackermann', default_value='false',
                              description='Car-like follower: no turning in place'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='ESP32 serial device (only with use_safety)'),
        DeclareLaunchArgument('stop_distance', default_value='0.4',
                              description='Gate: no forward motion closer than this (m)'),
        OpaqueFunction(function=_setup),
    ])
