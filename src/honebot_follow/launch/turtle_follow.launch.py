"""Start turtlesim plus the follower, optionally through the safety gate.

    ros2 launch honebot_follow turtle_follow.launch.py
    ros2 launch honebot_follow turtle_follow.launch.py follow_distance:=2.5
    ros2 launch honebot_follow turtle_follow.launch.py use_safety:=false   # no ESP32 needed

With use_safety:=true (default) the follower's output goes to /cmd_vel_raw,
the safety gate decides what reaches /turtle2/cmd_vel, and the ESP32 bridge is
started so the gate has a range sensor and link status. Cover the ultrasonic
sensor with your hand: the chasing turtle freezes.

Drive the leader from a second terminal:
    ros2 run turtlesim turtle_teleop_key
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    follow_distance = LaunchConfiguration('follow_distance')
    max_linear = LaunchConfiguration('max_linear')
    use_safety = LaunchConfiguration('use_safety')

    follower_params = [{
        # value_type=float so "follow_distance:=2" isn't rejected as an int
        'follow_distance': ParameterValue(follow_distance, value_type=float),
        'max_linear': ParameterValue(max_linear, value_type=float),
    }]

    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='1.5',
                              description='Metres to stay behind the leader'),
        DeclareLaunchArgument('max_linear', default_value='2.0',
                              description='Follower top speed (m/s)'),
        DeclareLaunchArgument('use_safety', default_value='true',
                              description='Route commands through the safety gate (needs the ESP32)'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='ESP32 serial device (only with use_safety)'),
        DeclareLaunchArgument('stop_distance', default_value='0.4',
                              description='Gate: no forward motion closer than this (m)'),

        Node(package='turtlesim', executable='turtlesim_node', name='sim'),

        # Direct: follower -> /turtle2/cmd_vel
        Node(
            package='honebot_follow', executable='follower', name='follower', output='screen',
            parameters=follower_params,
            condition=UnlessCondition(use_safety),
        ),

        # Gated: follower -> /cmd_vel_raw -> safety_gate -> /turtle2/cmd_vel
        Node(
            package='honebot_follow', executable='follower', name='follower', output='screen',
            parameters=follower_params,
            remappings=[('/turtle2/cmd_vel', '/cmd_vel_raw')],
            condition=IfCondition(use_safety),
        ),
        Node(
            package='honebot_safety', executable='safety_gate', name='safety_gate', output='screen',
            parameters=[{
                'stop_distance': ParameterValue(LaunchConfiguration('stop_distance'), value_type=float),
            }],
            remappings=[('cmd_vel', '/turtle2/cmd_vel')],
            condition=IfCondition(use_safety),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('honebot_hardware'), 'launch', 'esp32.launch.py'])),
            launch_arguments={'port': LaunchConfiguration('port')}.items(),
            condition=IfCondition(use_safety),
        ),
    ])
