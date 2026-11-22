import os
from glob import glob

from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # ament 靠这个索引文件找到包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/ 和 config/ 默认不会进 install，必须显式列出来
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'),
         glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cquick',
    maintainer_email='cquick@example.com',
    description='CQuick 总线舵机驱动、Action 客户端与 yaml 动作组播放器',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'sequence_player = cquick_servo.sequence_player:main',
        ],
    },
)
