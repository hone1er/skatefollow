"""Gamepad teleop into the safety gate.

    ros2 launch honebot_bringup teleop_joy.launch.py                 # xbox layout
    ros2 launch honebot_bringup teleop_joy.launch.py joy_config:=ps5

Run bringup.launch.py first. Hold the enable button (xbox: LB, button 2 in the
default config) and use the left stick. Releasing the button stops publishing,
so the gate reports CMD_STALE and the robot stops: that is the intended
dead-man behaviour. Output goes to /cmd_vel_raw, never straight to the motors.

Check the controller is seen:  ros2 topic echo /joy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='xbox',
                              description='teleop_twist_joy config: xbox, ps3, ps5, ...'),
        DeclareLaunchArgument('joy_dev', default_value='0', description='joystick device id'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])),
            launch_arguments={
                'joy_config': LaunchConfiguration('joy_config'),
                'joy_dev': LaunchConfiguration('joy_dev'),
                'joy_vel': '/cmd_vel_raw',
            }.items(),
        ),
    ])
