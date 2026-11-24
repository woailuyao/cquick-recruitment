from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="soccer_vision",
            executable="ball_detector",
            name="ball_detector",
            output="screen",
            parameters=[{
                "source": "http://192.168.56.1:8080/video",
                "conf": 0.4,
            }],
        ),
        Node(
            package="soccer_vision",
            executable="field_locator",
            name="field_locator",
            output="screen",
        ),
    ])
