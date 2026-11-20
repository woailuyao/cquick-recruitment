from glob import glob
import os

from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament 靠这个文件索引包，少一行 `ros2 run` 就找不到
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch / config 默认【不会】进 install，必须显式列出来。
        # 改完这个文件一定要重新 colcon build，否则 install 里还是旧的，
        # 你会对着空气改半天 launch。
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'),
            glob('config/actions/*.yaml')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tangjiayi',
    maintainer_email='373863921@qq.com',
    description='CQuick 电控组总线舵机驱动（Action Server + 客户端 + 动作组播放器）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'sequence_player = cquick_servo.sequence_player:main',
            'servo_cli = cquick_servo.servo_cli:main',
        ],
    },
)
