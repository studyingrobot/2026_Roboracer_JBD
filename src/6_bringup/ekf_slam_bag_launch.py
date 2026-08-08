"""bag 재생 기반 EKF + slam_toolbox + RViz 브링업.

  ros2 launch src/6_bringup/ekf_slam_bag_launch.py
  ros2 launch src/6_bringup/ekf_slam_bag_launch.py bag:=0808_test_3 play:=true

src/0_EKF/local_ekf.yaml 은 수정하지 않는다. VESC 원본 IMU 와의 차이(deg/s, g,
빈 frame_id, 0 covariance)는 imu_relay 가 흡수한다.

TF 소유권:
  map  -> odom       slam_toolbox
  odom -> base_link  ekf_filter_node   (bag 의 /tf 는 재생하지 않는다)
  base_link -> laser bag 의 /tf_static
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
EKF_DIR = os.path.join(ROOT, 'src', '0_EKF')
EKF_YAML = os.path.join(EKF_DIR, 'local_ekf.yaml')
IMU_RELAY = os.path.join(EKF_DIR, 'imu_relay.py')
SLAM_YAML = os.path.join(ROOT, 'src', '1_slam_mapping', 'slam_toolbox',
                         'mapper_params_offline.yaml')
BAGS_DIR = os.path.join(ROOT, 'src', '1_slam_mapping', 'maps', 'test_bags')
RVIZ_CFG = os.path.join(ROOT, 'src', '6_bringup', 'ekf_slam.rviz')

# bag 의 /tf 는 vesc 가 만든 odom->base_link 라서 EKF 와 충돌한다. 재생하지 않는다.
PLAY_TOPICS = ['/scan', '/odom', '/sensors/imu/raw', '/tf_static']


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    play = LaunchConfiguration('play')
    use_rviz = LaunchConfiguration('rviz')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='0808_test_2',
                              description='재생할 bag 이름 (maps/test_bags/ 안)'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='bag 재생 속도'),
        DeclareLaunchArgument('play', default_value='false',
                              description='true 면 launch 가 bag 재생까지 한다'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # 1) VESC raw IMU -> EKF 가 기대하는 단위/frame_id/covariance
        ExecuteProcess(
            cmd=['python3', IMU_RELAY, '--ros-args',
                 '-p', 'use_sim_time:=true',
                 '-p', 'in_topic:=/sensors/imu/raw',
                 '-p', 'out_topic:=/camera/camera/imu'],
            output='screen',
        ),

        # 2) EKF. yaml 은 원본 그대로, 토픽만 remap. /tf 는 여기서 나가야 한다.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[EKF_YAML, sim_time],
            remappings=[('/wheel/odom', '/odom')],
        ),

        # 3) SLAM. odom->base_link 를 EKF 로부터 받는다.
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[SLAM_YAML, sim_time],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', RVIZ_CFG],
            parameters=[sim_time],
            condition=IfCondition(use_rviz),
        ),

        # 4) bag 재생 (play:=true 일 때만. 기본은 다른 터미널에서 직접 실행)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 PathJoinSubstitution([BAGS_DIR, bag]),
                 '--clock', '100', '--rate', rate,
                 '--topics'] + PLAY_TOPICS,
            output='screen',
            condition=IfCondition(play),
        ),
    ])
