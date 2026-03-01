from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    capture_image_params = {
        "video_port": "/dev/video0",
        "output_rate": 10,
        "pub_tf": True,
        "image_raw_topic": "/camera/image_raw",
        "camera_info_topic": "/camera/color/camera_info",
        "image_rect_topic": "/camera/image_rect",
        "calibration_file": "ost.yaml"
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "capture_image_rviz",
            default_value="0",
            description="Whether to launch rviz"
        ),
        Node(
            package="capture_image",
            executable="capture_image_node",
            name="capture_image",
            output="screen",
            parameters=[capture_image_params],
        ),
    ])


# ros2 run camera_calibration cameracalibrator --size 10x7 --square 0.021 image:=/camera/image_raw camera:=/camera --no-service-check
