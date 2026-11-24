"""一条命令启动舵机驱动；可覆盖端口；demo:=true 时同时起客户端"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')
    # ★ 用包里查出来的绝对路径，不写相对路径（相对路径在别人机器上必挂）
    params_file = pkg_share + '/config/servo_params.yaml'

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='串口设备名，例如 /dev/ttyUSB1')
    pwm_max_arg = DeclareLaunchArgument(
        'pwm_max', default_value='2100', description='安全上限')
    demo_arg = DeclareLaunchArgument(
        'demo', default_value='false',
        description='true 时额外启动演示客户端')

    servo_driver = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[
            params_file,                       # 先读 yaml
            {
                # ★ value_type=str 不能省，否则 /dev/ttyUSB1 会被当数字解析
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'pwm_max': ParameterValue(LaunchConfiguration('pwm_max'), value_type=int),
            },
        ],
    )

    demo_client = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo')),
    )

    return LaunchDescription([
        port_arg, pwm_max_arg, demo_arg,
        servo_driver,
        demo_client,
    ])
