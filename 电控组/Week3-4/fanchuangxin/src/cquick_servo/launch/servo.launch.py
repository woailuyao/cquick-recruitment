from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
# 查找包的共享安装目录
from ament_index_python.packages import get_package_share_directory
# 拼接路径
import os


def generate_launch_description():
    # ========== 1. 声明launch可配置参数==========
    # 声明port参数：默认值从yaml取，命令行可以直接覆盖
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/cq_servo',
        description='串口设备路径'
    )

    # 声明demo开关：默认false，客户端不启动；demo:=true时启动客户端
    demo_arg = DeclareLaunchArgument(
        'demo',
        default_value='false',
        description='是否启动演示客户端'
    )

    # 用 LaunchConfiguration 接住参数，后面节点里可以直接用
    port = LaunchConfiguration('port')
    demo = LaunchConfiguration('demo')

    # ========== 2. 找到yaml配置文件的绝对路径 ==========
    # 先找到接口包的共享安装目录
    pkg_share = get_package_share_directory('cquick_servo')
    # 拼接出yaml参数文件的绝对路径，永远不会找不到
    param_file_path = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    # ========== 3. 定义驱动节点 ==========
    servo_driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        # 加载参数文件，从yaml读取所有配置
         parameters=[param_file_path, {'port': port}]

    )

    # ========== 4. 定义客户端节点，有条件启动 ==========
    move_client_node = Node(
        package='cquick_servo',
        executable='move_client',
        name='move_client',
        output='screen',
        # 条件：只有 demo 参数为 true 时才启动这个节点
        condition=IfCondition(demo)
    )

    # ========== 5. 把所有配置和节点全部返回，launch系统按顺序启动 ==========
    return LaunchDescription([
        port_arg,
        demo_arg,
        servo_driver_node,
        move_client_node
    ])
