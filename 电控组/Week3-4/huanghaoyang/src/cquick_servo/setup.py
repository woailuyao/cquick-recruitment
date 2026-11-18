from glob import glob
from setuptools import find_packages, setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 这三行是任务的硬性要求：launch/ 和 config/ 默认不会进 install，
        # 不加进 data_files，你改完 launch 重新 build，install 里还是旧文件，
        # 然后你会对着空气改半天，以为 launch 写错了。
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/actions',
            glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='CQuick 舵机驱动',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 左边是 ros2 run 用的可执行名，右边是 包名.模块名:入口函数
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'action_player = cquick_servo.action_player:main',
        ],
    },
)
