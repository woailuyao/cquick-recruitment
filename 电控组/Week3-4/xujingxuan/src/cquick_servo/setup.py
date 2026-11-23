import os
from glob import glob

from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/ 和 config/ 默认不会进 install，必须显式声明
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 动作组也是数据，同样要装进去，否则 install 后找不到
        (os.path.join('share', package_name, 'config', 'actions'), glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xujingxuan',
    maintainer_email='xujingxuan@cquick.local',
    description='CQuick 舵机驱动：Action 服务端 + 客户端 + 动作组播放器',
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
