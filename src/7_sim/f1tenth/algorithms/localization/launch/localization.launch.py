from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'params.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to localization params.yaml'
        ),

        Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[params_file]
        )
    ])
