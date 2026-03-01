"""
Hand-eye calibration launch file.

Starts the calibration server with all parameters loaded from YAML config file.

Usage:
    ros2 launch easy_handeye2 handeye.launch.py
    ros2 launch easy_handeye2 handeye.launch.py config_file:=/path/to/config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('easy_handeye2')
    default_config = os.path.join(pkg_share, 'config', 'calibration_params.yaml')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the YAML configuration file'
    )
    
    calibration_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'calibrate.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file')
        }.items()
    )
    
    return LaunchDescription([
        config_file_arg,
        calibration_server_launch
    ])
