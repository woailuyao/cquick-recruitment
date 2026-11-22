import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """在 context 里读 launch 参数，决定参数覆盖和条件启动。"""
    pkg_share = get_package_share_directory('cquick_servo')
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    port_value = LaunchConfiguration('port').perform(context)

    # port 参数：给了就覆盖 yaml，没给（空字符串）就只用 yaml 的值
    parameters = [params_file]
    if port_value.strip():
        parameters.append({'port': port_value})

    driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=parameters,
    )

    # demo 客户端默认不起；demo:=true 才启动
    demo_client = GroupAction(
        condition=IfCondition(LaunchConfiguration('demo')),
        actions=[
            Node(
                package='cquick_servo',
                executable='move_client',
                name='move_client',
                output='screen',
            )
        ],
    )

    # 动作组播放器：action:=<名字> 才启动（留空表示只起驱动节点）
    nodes = [driver, demo_client]
    action_value = LaunchConfiguration('action').perform(context).strip()
    if action_value:
        repeat_value = LaunchConfiguration('repeat').perform(context).strip() or '1'
        nodes.append(Node(
            package='cquick_servo',
            executable='sequence_player',
            name='sequence_player',
            output='screen',
            arguments=['--action', action_value, '--repeat', repeat_value],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='',
            description='串口设备名；留空用 yaml 的值，给值则覆盖（例如设备枚举成另一个编号时）'),
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='是否启动演示客户端，true/false'),
        DeclareLaunchArgument(
            'action', default_value='',
            description='动作组名字，给值则启动 sequence_player 播放 config/actions/<名字>.yaml；'
                        '留空只起驱动节点'),
        DeclareLaunchArgument(
            'repeat', default_value='1',
            description='动作组连跑几遍（配合 action 使用，三遍一致性演示用 3）'),
        OpaqueFunction(function=launch_setup),
    ])
