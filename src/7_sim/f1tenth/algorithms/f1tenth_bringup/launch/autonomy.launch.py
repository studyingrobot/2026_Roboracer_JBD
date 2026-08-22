import math
import re
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _speed_from_profile(profile_name, fallback='5.5'):
    """mpc_profile=speed_<m/s> 가 담은 속도.  규약을 벗어나면 fallback."""
    if not profile_name.startswith('speed_'):
        return fallback
    encoded = profile_name[len('speed_'):]
    if re.fullmatch(r'\d+(?:\.\d+)?', encoded):
        return encoded
    if re.fullmatch(r'\d+_\d+', encoded):
        return encoded.replace('_', '.', 1)
    return fallback


def _include(package, launch_file, arguments):
    source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', launch_file))
    return IncludeLaunchDescription(
        source,
        launch_arguments={key: str(value) for key, value in arguments.items()}.items(),
    )


def _launch_setup(context, catalog_path):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'real':
        controller = LaunchConfiguration('controller').perform(context)
        requested_speed = float(LaunchConfiguration('speed').perform(context))
        if (not math.isfinite(requested_speed)
                or not 0.0 < requested_speed <= 5.5):
            raise RuntimeError('speed must be greater than 0 and at most 5.5 m/s')
        speed_profile = 'speed_%g' % requested_speed
        waypoint_csv = LaunchConfiguration('waypoint_csv').perform(context)
        return [
            LogInfo(msg=(
                f'mode=real controller={controller} '
                f'speed={requested_speed:.2f}m/s '
                'obstacle_policy=automatic '
                'output=/auto enabled=false')),
            _include('planning', 'planning.launch.py', {
                'waypoint_csv': waypoint_csv,
                # The planner stays active for Q1/Q2/Q3. With no detected
                # obstacle it republishes the global path; otherwise it
                # selects a collision-free local path automatically.
                'local_planner': 'true',
                'odom_topic': '/odom',
                'base_frame_id': 'base_link',
                'maximum_planning_speed': requested_speed,
                'max_lateral_acceleration': LaunchConfiguration(
                    'max_lateral_acceleration').perform(context),
                'planning_deceleration': LaunchConfiguration(
                    'max_longitudinal_deceleration').perform(context),
            }),
            _include('control', 'control.launch.py', {
                'controller': controller,
                'mpc_profile': speed_profile,
                'speed': requested_speed,  # minjae changes
                'drive_mode': 'real',
                'enabled': 'false',
                'global_frame_id': 'map',
                'base_frame_id': 'base_link',
                'odom_topic': '/odom',
                'drive_topic': '/auto',
                'min_command_speed': LaunchConfiguration(
                    'min_command_speed').perform(context),
                'max_lateral_acceleration': LaunchConfiguration(
                    'max_lateral_acceleration').perform(context),
                'max_longitudinal_acceleration': LaunchConfiguration(
                    'max_longitudinal_acceleration').perform(context),
                'max_longitudinal_deceleration': LaunchConfiguration(
                    'max_longitudinal_deceleration').perform(context),
                'avoidance_speed_limit': LaunchConfiguration(
                    'avoidance_speed_limit').perform(context),
                'collision_topic': '/control/collision',
                'emergency_stop_topic': '/safety/emergency_stop',
            }),
        ]
    if mode != 'sim':
        raise RuntimeError("mode must be 'sim' or 'real'")

    with open(catalog_path, 'r') as stream:
        catalog = yaml.safe_load(stream)

    track_name = LaunchConfiguration('track').perform(context)
    tracks = catalog.get('tracks', {})
    if track_name not in tracks:
        available = ', '.join(sorted(tracks))
        raise RuntimeError(
            f'Unknown track {track_name!r}; available tracks: {available}')

    track = tracks[track_name]
    controller = LaunchConfiguration('controller').perform(context)
    mpc_profile = LaunchConfiguration('mpc_profile').perform(context)
    # 플래너가 발행하는 /planning/speed_limit 의 상한이다.  제어기가 실제로
    # 노리는 속도와 같아야 한다.  5.5 로 고정해 두던 동안 시뮬에서는 상한이
    # 사실상 없어서, 요청 속도를 그대로 쓰는 실차(real 모드)와 회피 중 감속
    # 거동이 달랐다.  minjae_pp 는 speed:= 하나만 쓰고 나머지는
    # mpc_profile=speed_<m/s> 규약을 따르므로 제어기별로 갈라 뽑는다.
    if controller == 'minjae_pp':
        planning_speed = LaunchConfiguration('speed').perform(context)
    else:
        planning_speed = _speed_from_profile(mpc_profile)
    friction_arg = LaunchConfiguration('friction').perform(context)
    friction = track['friction_mu'] if friction_arg == 'auto' else friction_arg
    obstacle_mode = LaunchConfiguration('obstacles').perform(context)

    start_x, start_y, start_yaw = track['start']
    common = {
        'map_path': track['map_path'],
        'map_ext': track['map_ext'],
        'start_x': start_x,
        'start_y': start_y,
        'start_yaw': start_yaw,
        'centerline': track['centerline'],
        'friction': friction,
        'obstacles': obstacle_mode,
        'obstacle_seed': LaunchConfiguration(
            'obstacle_seed').perform(context),
        'rviz': LaunchConfiguration('rviz').perform(context),
    }

    return [
        LogInfo(msg=(
            f'track={track_name} controller={controller} '
            f'mpc_profile={mpc_profile} friction={friction}')),
        _include('f1tenth_gym_ros', 'gym_bridge_launch.py', common),
        _include('planning', 'planning.launch.py', {
            'waypoint_csv': track['raceline'],
            'local_planner': obstacle_mode,
            'maximum_planning_speed': planning_speed,
            'max_lateral_acceleration': LaunchConfiguration(
                'max_lateral_acceleration').perform(context),
            'planning_deceleration': LaunchConfiguration(
                'max_longitudinal_deceleration').perform(context),
        }),
        _include('control', 'control.launch.py', {
            'controller': controller,
            'mpc_profile': mpc_profile,
            'speed': LaunchConfiguration('speed').perform(context),  # minjae changes
            'drive_mode': 'sim',
            'max_lateral_acceleration': LaunchConfiguration(
                'max_lateral_acceleration').perform(context),
            'max_longitudinal_acceleration': LaunchConfiguration(
                'max_longitudinal_acceleration').perform(context),
            'max_longitudinal_deceleration': LaunchConfiguration(
                'max_longitudinal_deceleration').perform(context),
            'avoidance_speed_limit': LaunchConfiguration(
                'avoidance_speed_limit').perform(context),
        }),
    ]


def generate_launch_description():
    catalog_path = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'tracks.yaml',
    )
    default_waypoint = os.path.join(
        get_package_share_directory('planning'),
        'waypoints',
        'track04_raceline.csv',
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('track', default_value='track04'),
        DeclareLaunchArgument('waypoint_csv', default_value=default_waypoint),
        DeclareLaunchArgument(
            'controller',
            default_value='unicorn_l1',
            description=(
                'none, minjae_pp, unicorn_l1, unicorn_l1_dynamic, or mpc'),
        ),
        DeclareLaunchArgument(
            'speed',
            default_value='1.0',
            # minjae changes: sim/real 공통 속도 인자
            description='Controller target speed in m/s (sim and real)',
        ),
        DeclareLaunchArgument(
            'mpc_profile',
            default_value='speed_0.55',
            description='Dynamic speed_<m/s> (e.g. speed_0.85 or speed_2)',
        ),
        DeclareLaunchArgument(
            'min_command_speed',
            default_value='0.30',
            description='Real vehicle non-zero command floor in m/s',
        ),
        DeclareLaunchArgument(
            'max_lateral_acceleration',
            default_value='0.80',
            description='Tyre and surface lateral limit in m/s^2',
        ),
        DeclareLaunchArgument(
            'max_longitudinal_acceleration',
            default_value='0.80',
            description='UNICORN L1 acceleration limit in m/s^2',
        ),
        DeclareLaunchArgument(
            'max_longitudinal_deceleration',
            default_value='2.0',
            description='UNICORN L1 deceleration limit in m/s^2',
        ),
        DeclareLaunchArgument(
            'avoidance_speed_limit',
            default_value='auto',
            description=(
                'Hard obstacle speed cap; auto uses the local planner '
                'curvature-derived speed limit'),
        ),
        DeclareLaunchArgument(
            'friction',
            default_value='auto',
            description='auto uses the selected track friction_mu',
        ),
        DeclareLaunchArgument('obstacles', default_value='false'),
        # 장애물 배치 재현용.  -1 은 매번 무작위.
        DeclareLaunchArgument('obstacle_seed', default_value='-1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(
            function=_launch_setup,
            kwargs={'catalog_path': catalog_path},
        ),
    ])
