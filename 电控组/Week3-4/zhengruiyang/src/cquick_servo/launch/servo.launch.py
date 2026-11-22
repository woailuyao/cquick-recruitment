from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument       # 声明 launch 参数
from launch.conditions import IfCondition              # 条件启动
from launch.substitutions import LaunchConfiguration   # 读 launch 参数的值
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cquick_servo')   # 查包路径
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')   # 参数 yaml 绝对路径

    port_arg = DeclareLaunchArgument('port', default_value='/dev/cq_servo')
    demo_arg = DeclareLaunchArgument('demo', default_value='false')          # 声明 demo 开关

    driver = Node(
        package='cquick_servo', executable='servo_driver', name='servo_driver',
        parameters=[params_file, {'port': LaunchConfiguration('port')}],   # 先读 yaml 再用 port 覆盖
        output='screen',
    )
    client = Node(
        package='cquick_servo', executable='move_client', name='move_client',
        condition=IfCondition(LaunchConfiguration('demo')),   # demo:=true 才起客户端
        output='screen',
    )
    watchdog = Node(
    package='cquick_servo',
    executable='watchdog',
    name='watchdog',
    parameters=[params_file],   # ← 加这行：读同一个 yaml
    output='screen',
)
    return LaunchDescription([port_arg, demo_arg, driver, client, watchdog])
