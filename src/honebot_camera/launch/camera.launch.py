"""Camera + aiming: camera_node (video in, person detection) and camera_aim (pan servo command).

    ros2 launch honebot_camera camera.launch.py                        # GoPro over USB, HOG detector
    ros2 launch honebot_camera camera.launch.py record:=true           # also save the 1080p stream to ~/videos
    ros2 launch honebot_camera camera.launch.py source:=/dev/video0    # a USB webcam instead
    ros2 launch honebot_camera camera.launch.py source:=~/clip.mp4 aim:=false   # bench-test the detector
    ros2 launch honebot_camera camera.launch.py model_path:=''                 # force the HOG detector

The detector is YOLOv8n from ~/models/yolov8n.onnx when that file exists (export: docs/camera_plan.md §5),
otherwise OpenCV's HOG. 'auto' uses cv2.dnn if it can run the model and onnxruntime otherwise.

The pan command reaches the servo through esp32_bridge (bringup.launch.py), as `P,<pan>` lines.
Watch camera/image/compressed and camera/target in Foxglove.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='gopro',
                              description="'gopro', '/dev/videoN', a URL or a video file"),
        DeclareLaunchArgument('record', default_value='false',
                              description='GoPro only: tee the raw H.264 stream to record_dir'),
        DeclareLaunchArgument('record_dir', default_value='~/videos'),
        DeclareLaunchArgument('gopro_res', default_value='1080', description='1080 | 720 | 480'),
        DeclareLaunchArgument('gopro_fov', default_value='4', description='0 wide, 2 narrow, 3 superview, 4 linear'),
        DeclareLaunchArgument('hfov_deg', default_value='87.0',
                              description='horizontal field of view of the chosen lens (Linear ~87, Wide ~118)'),
        DeclareLaunchArgument('model_path', default_value='~/models/yolov8n.onnx',
                              description='YOLOv8n/YOLO11n .onnx (falls back to HOG if the file is missing); '
                                          "'' = OpenCV HOG people detector"),
        DeclareLaunchArgument('input_size', default_value='320', description='model input size used at export'),
        DeclareLaunchArgument('detector', default_value='auto',
                              description='auto (onnx if model_path set, else hog) | hog | onnx | ort'),
        DeclareLaunchArgument('preview_hz', default_value='5.0', description='JPEG preview rate, 0 = off'),
        DeclareLaunchArgument('aim', default_value='true', description='start camera_aim (pan servo control)'),
        DeclareLaunchArgument('pan_sign', default_value='1.0', description='-1.0 if the servo pans the wrong way'),
        DeclareLaunchArgument('half_range_deg', default_value='45.0', description='servo travel either side'),

        Node(
            package='honebot_camera', executable='camera_node', name='camera_node', output='screen',
            parameters=[{
                'source': LaunchConfiguration('source'),
                'record': LaunchConfiguration('record'),
                'record_dir': LaunchConfiguration('record_dir'),
                'gopro_res': LaunchConfiguration('gopro_res'),
                'gopro_fov': LaunchConfiguration('gopro_fov'),
                'hfov_deg': LaunchConfiguration('hfov_deg'),
                'model_path': LaunchConfiguration('model_path'),
                'input_size': LaunchConfiguration('input_size'),
                'detector': LaunchConfiguration('detector'),
                'preview_hz': LaunchConfiguration('preview_hz'),
            }],
        ),
        Node(
            package='honebot_camera', executable='camera_aim', name='camera_aim', output='screen',
            parameters=[{
                'pan_sign': LaunchConfiguration('pan_sign'),
                'half_range_deg': LaunchConfiguration('half_range_deg'),
            }],
            condition=IfCondition(LaunchConfiguration('aim')),
        ),
    ])
