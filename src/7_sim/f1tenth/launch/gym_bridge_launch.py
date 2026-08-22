# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, config, config_dict):
    parameters = config_dict['bridge']['ros__parameters']
    map_path = LaunchConfiguration('map_path').perform(context)
    map_ext = LaunchConfiguration('map_ext').perform(context)
    centerline = LaunchConfiguration('centerline').perform(context)
    bridge_overrides = {
        'map_path': map_path,
        'map_img_ext': map_ext,
        'sx': float(LaunchConfiguration('start_x').perform(context)),
        'sy': float(LaunchConfiguration('start_y').perform(context)),
        'stheta': float(LaunchConfiguration('start_yaw').perform(context)),
        'friction_mu': float(LaunchConfiguration('friction').perform(context)),
        'obstacle_path_csv': centerline,
        'random_obstacles_enabled': _as_bool(
            LaunchConfiguration('obstacles').perform(context)),
        # 회피를 두 번 이상 비교하려면 장애물 배치가 재현돼야 한다.
        # -1 은 매번 무작위. 실차 bringup 은 이미 obstacle_seed 를 넘긴다.
        'random_obstacle_seed': int(
            LaunchConfiguration('obstacle_seed').perform(context)),
    }
    has_opp = parameters['num_agent'] > 1

    actions = []
    if _as_bool(LaunchConfiguration('rviz').perform(context)):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(
                get_package_share_directory('f1tenth_gym_ros'),
                'launch', 'gym_bridge.rviz')]
        ))

    actions.extend([
        Node(
            package='f1tenth_gym_ros',
            executable='gym_bridge',
            name='bridge',
            output='screen',
            parameters=[config, bridge_overrides]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_path + '.yaml',
                'topic': 'map',
                'frame_id': 'map',
                'use_sim_time': False,
            }]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('f1tenth_gym_ros'),
                    'config', 'amcl.yaml'),
                # 초기 포즈는 차를 놓는 위치와 같아야 한다.  amcl.yaml 에
                # 하드코딩해 두면 tracks.yaml 의 start 를 바꿀 때마다 두 곳을
                # 손으로 맞춰야 하고, 어긋난 만큼이 그대로 위치추정 오차로
                # 잡힌다.  gym_bridge 가 쓰는 값을 그대로 넘긴다.
                {
                    'initial_pose.x': float(
                        LaunchConfiguration('start_x').perform(context)),
                    'initial_pose.y': float(
                        LaunchConfiguration('start_y').perform(context)),
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': float(
                        LaunchConfiguration('start_yaw').perform(context)),
                },
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='ego_robot_state_publisher',
            parameters=[{'robot_description': Command([
                'xacro ', os.path.join(
                    get_package_share_directory('f1tenth_gym_ros'),
                    'launch', 'ego_racecar.xacro')
            ])}],
            remappings=[('/robot_description', 'ego_robot_description')]
        ),
    ])

    if has_opp:
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[{'robot_description': Command([
                'xacro ', os.path.join(
                    get_package_share_directory('f1tenth_gym_ros'),
                    'launch', 'opp_racecar.xacro')
            ])}],
            remappings=[('/robot_description', 'opp_robot_description')]
        ))

    return actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )
    with open(config, 'r') as stream:
        config_dict = yaml.safe_load(stream)
    parameters = config_dict['bridge']['ros__parameters']

    arguments = [
        DeclareLaunchArgument('map_path', default_value=parameters['map_path']),
        DeclareLaunchArgument('map_ext', default_value=parameters['map_img_ext']),
        DeclareLaunchArgument('start_x', default_value=str(parameters['sx'])),
        DeclareLaunchArgument('start_y', default_value=str(parameters['sy'])),
        DeclareLaunchArgument('start_yaw', default_value=str(parameters['stheta'])),
        DeclareLaunchArgument(
            'centerline', default_value=parameters['obstacle_path_csv']),
        DeclareLaunchArgument(
            'friction', default_value=str(parameters['friction_mu'])),
        DeclareLaunchArgument('obstacles', default_value='false'),
        DeclareLaunchArgument('obstacle_seed', default_value='-1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(
            function=_launch_setup,
            kwargs={'config': config, 'config_dict': config_dict},
        ),
    ]
    return LaunchDescription(arguments)
