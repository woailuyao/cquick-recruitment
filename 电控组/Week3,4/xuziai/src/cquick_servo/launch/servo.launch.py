import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('cquick_servo')
    param_file = os.path.join(pkg_dir, 'config', 'servo_params.yaml')
    
    # 拼接出默认的 yaml 动作文件路径
    default_action_yaml = os.path.join(pkg_dir, 'config', 'actions', 'stand_up.yaml')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='串口号'
    )
    demo_arg = DeclareLaunchArgument(
        'demo',
        default_value='false',
        description='是否启动测试客户端'
    )
    
    # 服务端
    driver_node = Node(
        package='cquick_servo',
        executable='servo_driver',
        name='servo_driver', 
        output='screen',     
        parameters=[
            param_file, 
            {'port': ParameterValue(LaunchConfiguration('port'), value_type=str)}
        ]
    )

    # 客户端节点
    client_node = Node(
        package='cquick_servo',
        executable='sequence_player',
        name='sequence_player',
        output='screen',
        # 👇 加上这一段，通过 ROS 2 参数传路径
        parameters=[{
            'yaml_file': default_action_yaml
        }],
        condition=IfCondition(LaunchConfiguration('demo'))
    )

    return LaunchDescription([
        port_arg,
        demo_arg,
        driver_node,
        client_node
    ])