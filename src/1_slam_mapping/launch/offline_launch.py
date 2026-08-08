import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

# slam_toolbox 기본 offline_launch.py는 파라미터 경로가
# /opt/ros/humble/share/slam_toolbox/config 로 하드코딩되어 있어 수정이 불가능하다.
# 여기서는 저장소 안의 yaml을 기본값으로 쓰고, 필요하면 인자로 덮어쓴다.
DEFAULT_PARAMS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'config', 'mapper_params_offline.yaml'
)


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=DEFAULT_PARAMS,
            description='SLAM 파라미터 yaml 경로'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='offline 매핑은 bag 재생 기반이므로 기본 true'
        ),
        launch_ros.actions.Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])
