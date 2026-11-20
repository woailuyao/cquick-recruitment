import os
from glob import glob

from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/ 和 config/ 默认不会进 install，必须显式加上
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'),
         glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xuyuhan',
    maintainer_email='xuyuhan@cquick.local',
    description='CQuick 总线舵机驱动',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'cancel_demo = cquick_servo.cancel_demo:main',
            'sequence_player = cquick_servo.sequence_player:main',
            'joint_selfcheck = cquick_servo.joint_selfcheck:main',
            'joint_range_check = cquick_servo.joint_range_check:main',
        ],
    },
)
