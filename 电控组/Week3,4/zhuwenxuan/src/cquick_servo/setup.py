import os
from glob import glob

from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/ 和 config/ 必须显式安装，否则 install 里没有，launch 会找不到
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'), glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhuwenxuan',
    maintainer_email='85414@example.com',
    description='第3/4周：串口驱动节点（Action Server）+ 客户端 + launch + 参数',
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
