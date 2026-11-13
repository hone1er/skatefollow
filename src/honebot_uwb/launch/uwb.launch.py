"""Start the UWB node.

    ros2 launch honebot_uwb uwb.launch.py port:=/dev/ttyUSB1 baseline:=0.30
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB1', description='UWB board serial device'),
        DeclareLaunchArgument('baseline', default_value='0.30', description='Anchor spacing (m)'),
        Node(
            package='honebot_uwb', executable='uwb_node', name='uwb_node', output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baseline': ParameterValue(LaunchConfiguration('baseline'), value_type=float),
            }],
        ),
    ])
