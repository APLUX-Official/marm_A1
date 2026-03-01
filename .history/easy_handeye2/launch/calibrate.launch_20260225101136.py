"""
Hand-eye calibration server launch file.

All parameters are configured via YAML config file.
Services provided:
  - ~/take_sample
  - ~/compute_calibration
  - ~/save_calibration

Usage:
    ros2 launch easy_handeye2 calibrate.launch.py
    ros2 launch easy_handeye2 calibrate.launch.py config_file:=/path/to/config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('easy_handeye2')
    default_config = os.path.join(pkg_share, 'config', 'calibration_params.yaml')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the YAML configuration file'
    )
    
    calibration_server_node = Node(
        package='easy_handeye2',
        executable='handeye_calibration_server',
        name='handeye_calibration_server',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )
    
    return LaunchDescription([
        config_file_arg,
        calibration_server_node
    ])
