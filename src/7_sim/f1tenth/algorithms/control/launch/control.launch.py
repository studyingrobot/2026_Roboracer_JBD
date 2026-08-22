import os
import math
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_dynamic_speed(profile_name):
    """Return m/s encoded by speed_<value>, or None for a named profile."""
    if not profile_name.startswith('speed_'):
        return None

    encoded = profile_name[len('speed_'):]
    if re.fullmatch(r'\d+(?:\.\d+)?', encoded):
        numeric = encoded
    elif re.fullmatch(r'\d+_\d+', encoded):
        # Keep compatibility with shell-friendly names such as speed_0_85.
        numeric = encoded.replace('_', '.', 1)
    else:
        raise RuntimeError(
            f'Invalid dynamic MPC speed {profile_name!r}; '
            'use speed_0.85, speed_1.2, or speed_2.')

    speed = float(numeric)
    if not math.isfinite(speed) or speed <= 0.0 or speed > 5.5:
        raise RuntimeError(
            f'Controller speed must be greater than 0 and at most 5.5 m/s; '
            f'got {speed!r}.')
    return speed


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    controller = LaunchConfiguration('controller').perform(context)
    if controller == 'none':
        return [LogInfo(msg='Controller disabled (controller:=none)')]

    # minjae changes
    # 이전에는 speed:= 를 무시하고 yaml 의 target_speed 로만 달렸다.
    # 실차에서 speed:=1.0 을 줘도 4.0 으로 나가는 문제라 반드시 덮어쓴다.
    # min_speed / max_lateral_acceleration 등 튜닝값은 yaml 을 그대로 쓴다.
    if controller == 'minjae_pp':
        requested_speed = float(LaunchConfiguration('speed').perform(context))
        if not math.isfinite(requested_speed) or requested_speed <= 0.0:
            raise RuntimeError(
                'minjae_pp requires speed:=<m/s> greater than 0; '
                f'got {requested_speed!r}.')
        return [
            LogInfo(msg=f'Controller=minjae_pp speed={requested_speed:.2f}m/s'),
            Node(
                package='control',
                executable='minjae_pp_node',
                name='minjae_pp_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('control_params_file').perform(context),
                    {
                        'drive_mode': LaunchConfiguration(
                            'drive_mode').perform(context),
                        'global_frame_id': LaunchConfiguration(
                            'global_frame_id').perform(context),
                        'base_frame_id': LaunchConfiguration(
                            'base_frame_id').perform(context),
                        'odom_topic': LaunchConfiguration(
                            'odom_topic').perform(context),
                        'drive_topic': LaunchConfiguration(
                            'drive_topic').perform(context),
                        'emergency_stop_topic': LaunchConfiguration(
                            'emergency_stop_topic').perform(context),
                        'target_speed': requested_speed,
                        'max_speed': requested_speed,
                        'min_command_speed': float(LaunchConfiguration(
                            'min_command_speed').perform(context)),
                        # 타이어/노면 한계는 플래너와 제어기가 같은 값을
                        # 써야 한다.  이걸 넘기지 않던 동안 플래너는 1.50,
                        # 제어기는 yaml 의 4.00 으로 돌아서 회피가 시작되는
                        # 순간 속도 상한이 계단처럼 떨어졌다.
                        'max_lateral_acceleration': float(
                            LaunchConfiguration(
                                'max_lateral_acceleration').perform(context)),
                        # 감속도도 같은 이유로 넘긴다.  플래너는 런치의
                        # max_longitudinal_deceleration 을 받는데 제어기만
                        # yaml 의 2.00 에 묶여 있어서, 현장에서 측정값을
                        # 런치 인자로 주면 앞의 코너까지 감속 가능한 속도를
                        # 양쪽이 서로 다르게 계산했다.
                        'speed_profile_deceleration': float(
                            LaunchConfiguration(
                                'max_longitudinal_deceleration').perform(
                                    context)),
                    },
                ],
            ),
        ]
    # minjae changes

    if controller in ('unicorn_l1', 'unicorn_l1_dynamic'):
        use_dynamic_speed_limit = controller == 'unicorn_l1_dynamic'
        profile_name = LaunchConfiguration('mpc_profile').perform(context)
        requested_speed = _parse_dynamic_speed(profile_name)
        if requested_speed is None:
            raise RuntimeError(
                'UNICORN L1 requires mpc_profile:=speed_<m/s> '
                '(for example speed_1.0).')
        min_reference_speed = min(
            requested_speed,
            max(0.20, min(0.45, requested_speed * 0.40)),
        )
        avoidance_value = LaunchConfiguration(
            'avoidance_speed_limit').perform(context)
        avoidance_speed_limit = (
            requested_speed if avoidance_value == 'auto'
            else float(avoidance_value))
        if avoidance_speed_limit <= 0.0:
            raise RuntimeError(
                'avoidance_speed_limit must be auto or a positive m/s value')
        parameters = {
            'enabled': _as_bool(
                LaunchConfiguration('enabled').perform(context)),
            'global_frame_id': LaunchConfiguration(
                'global_frame_id').perform(context),
            'base_frame_id': LaunchConfiguration(
                'base_frame_id').perform(context),
            'odom_topic': LaunchConfiguration('odom_topic').perform(context),
            'drive_topic': LaunchConfiguration('drive_topic').perform(context),
            'collision_topic': LaunchConfiguration(
                'collision_topic').perform(context),
            'emergency_stop_topic': LaunchConfiguration(
                'emergency_stop_topic').perform(context),
            'target_speed': requested_speed,
            'max_speed': requested_speed,
            'max_lateral_acceleration': float(LaunchConfiguration(
                'max_lateral_acceleration').perform(context)),
            'max_longitudinal_acceleration': float(LaunchConfiguration(
                'max_longitudinal_acceleration').perform(context)),
            'max_longitudinal_deceleration': float(LaunchConfiguration(
                'max_longitudinal_deceleration').perform(context)),
            'avoidance_speed_limit': avoidance_speed_limit,
            'use_dynamic_speed_limit': use_dynamic_speed_limit,
            'min_reference_speed': min_reference_speed,
            'min_command_speed': float(LaunchConfiguration(
                'min_command_speed').perform(context)),
        }
        return [
            LogInfo(msg=(
                f'Controller={controller} speed={requested_speed:.2f}m/s '
                f'corner_min={min_reference_speed:.2f}m/s')),
            Node(
                package='control',
                executable='unicorn_l1_node',
                name='unicorn_l1_node',
                output='screen',
                parameters=[parameters],
            ),
        ]

    if controller != 'mpc':
        raise RuntimeError(
            f'Unknown controller {controller!r}; use none, minjae_pp, '
            'unicorn_l1, unicorn_l1_dynamic, or mpc.')

    config_path = LaunchConfiguration('mpc_params_file').perform(context)
    profile_name = LaunchConfiguration('mpc_profile').perform(context)
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)

    profiles = config.get('profiles', {})
    requested_speed = _parse_dynamic_speed(profile_name)
    if requested_speed is not None:
        parameters = dict(config.get('common', {}))
        parameters.update(config.get('speed_template', {}))
        parameters['target_speed'] = requested_speed
        parameters['max_speed'] = requested_speed
        parameters['min_reference_speed'] = min(
            requested_speed,
            max(0.20, min(0.45, requested_speed * 0.40)),
        )
        selection_log = (
            f'Controller=mpc dynamic_speed={requested_speed:.2f}m/s '
            f'corner_min={parameters["min_reference_speed"]:.2f}m/s')
    elif profile_name in profiles:
        parameters = dict(config.get('common', {}))
        parameters.update(profiles[profile_name])
        selection_log = f'Controller=mpc profile={profile_name}'
    else:
        available = ', '.join(sorted(profiles))
        raise RuntimeError(
            f'Unknown MPC profile {profile_name!r}; use speed_<m/s> '
            f'(for example speed_0.85 or speed_2), or: {available}')

    parameters.update({
        'enabled': _as_bool(LaunchConfiguration('enabled').perform(context)),
        'global_frame_id': LaunchConfiguration(
            'global_frame_id').perform(context),
        'base_frame_id': LaunchConfiguration(
            'base_frame_id').perform(context),
        'odom_topic': LaunchConfiguration('odom_topic').perform(context),
        'drive_topic': LaunchConfiguration('drive_topic').perform(context),
        'min_command_speed': float(LaunchConfiguration(
            'min_command_speed').perform(context)),
        'collision_topic': LaunchConfiguration(
            'collision_topic').perform(context),
        'emergency_stop_topic': LaunchConfiguration(
            'emergency_stop_topic').perform(context),
    })

    return [
        LogInfo(msg=selection_log),
        Node(
            package='control',
            executable='linear_mpc_node',
            name='linear_mpc_node',
            output='screen',
            parameters=[parameters],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('control')
    return LaunchDescription([
        DeclareLaunchArgument(
            'control_params_file',
            default_value=os.path.join(package_share, 'config', 'minjae_pp_params.yaml'),
        ),
        DeclareLaunchArgument('drive_mode', default_value='sim'),
        # minjae changes
        DeclareLaunchArgument(
            'speed',
            default_value='1.0',
            description='minjae_pp target/max speed in m/s'),
        # minjae changes
        DeclareLaunchArgument(
            'controller',
            default_value='minjae_pp',
            description=(
                'none, minjae_pp, unicorn_l1, unicorn_l1_dynamic, or mpc'),
        ),
        DeclareLaunchArgument(
            'mpc_profile',
            default_value='speed_0.55',
            description='speed_<m/s> or a named profile in mpc_params.yaml',
        ),
        DeclareLaunchArgument(
            'mpc_params_file',
            default_value=os.path.join(
                package_share, 'config', 'mpc_params.yaml'),
        ),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument(
            'base_frame_id', default_value='ego_racecar/base_link'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/ego_racecar/odom'),
        DeclareLaunchArgument('drive_topic', default_value='/drive'),
        DeclareLaunchArgument(
            'min_command_speed',
            default_value='0.0',
            description='Minimum non-zero speed command for actuator deadband'),
        DeclareLaunchArgument(
            'max_lateral_acceleration',
            default_value='1.50',
            description='Tyre and surface lateral limit in m/s^2'),
        DeclareLaunchArgument(
            'max_longitudinal_acceleration',
            default_value='2.0',
            description='UNICORN L1 acceleration command limit in m/s^2'),
        DeclareLaunchArgument(
            'max_longitudinal_deceleration',
            default_value='4.0',
            description='UNICORN L1 deceleration command limit in m/s^2'),
        DeclareLaunchArgument(
            'avoidance_speed_limit',
            default_value='auto',
            description=(
                'Hard UNICORN L1 obstacle speed cap; auto lets the planner '
                'publish a curvature-derived limit')),
        DeclareLaunchArgument(
            'collision_topic', default_value='/ego_racecar/collision'),
        DeclareLaunchArgument(
            'emergency_stop_topic',
            default_value='/safety/emergency_stop'),
        OpaqueFunction(function=_launch_setup),
    ])
