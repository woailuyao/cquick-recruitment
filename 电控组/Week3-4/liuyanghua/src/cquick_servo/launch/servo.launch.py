import os

import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PARAM_NAMES = [
    'port', 'baudrate', 'pwm_min', 'pwm_max',
    'default_duration_ms', 'servo_ids', 'startup_reset',
]

INT_PARAMS = {'baudrate', 'pwm_min', 'pwm_max', 'default_duration_ms'}
BOOL_PARAMS = {'startup_reset'}
LIST_PARAMS = {'servo_ids'}

TRUE_WORDS = ('true', '1', 'yes', 'on')


def _convert(name, val):
    if name in INT_PARAMS:
        return int(val)
    if name in BOOL_PARAMS:
        return val.lower() in TRUE_WORDS
    if name in LIST_PARAMS:
        s = val.strip().strip('[]')
        return [int(x) for x in s.split(',') if x.strip() != '']
    return val


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('cquick_servo')
    param_yaml_path = os.path.join(pkg_share, 'config', 'servo_params.yaml')
    actions_yaml_path = os.path.join(pkg_share, 'config', 'actions', 'execute.yaml')

    params = [param_yaml_path]
    overrides = {}
    for name in PARAM_NAMES:
        raw = LaunchConfiguration(name).perform(context)
        if raw != '':
            overrides[name] = _convert(name, raw)
    if overrides:
        params.append(overrides)

    actions = [Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',          # 必须等于 yaml 最外层 key
        parameters=params,
        output='screen',
    )]

    actions.append(Node(
        package='cquick_servo',
        executable='sequence_player',
        name='sequence_player',
        parameters=[actions_yaml_path],
        output='screen',
    ))

    # 条件启动客户端：默认不起，client:=true 才起
    client_raw = LaunchConfiguration('client').perform(context)
    if client_raw.lower() in TRUE_WORDS:
        actions.append(Node(
            package='cquick_servo',
            executable='move_client',
            name='move_client',
            parameters=[param_yaml_path],
            output='screen',
        ))
    else:
        print("================默认: client不启动================")

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(n, default_value='',
                              description=f'override {n} from yaml')
        for n in PARAM_NAMES
    ]
    args.append(
        DeclareLaunchArgument('client', default_value='false',
                              description='start move_client if true')
    )

    return launch.LaunchDescription(
        args + [OpaqueFunction(function=launch_setup)]
    )