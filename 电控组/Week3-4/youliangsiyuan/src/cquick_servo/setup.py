from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # ⭐ data_files 里面这几行是 launch/config 能不能被"装"进 install 的关键。
    #    漏了 launch 那行 -> ros2 launch 报 file not found；
    #    漏了 config 那行 -> yaml 加载不到、ros2 param get 全是默认值；
    #    漏了 actions 那行 -> sequence_player 找不到动作文件。
    #    改完这里**必须重新 colcon build**。
    data_files=[
        # ament 索引：告诉系统"这个包存在"
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 主参数 yaml（config/servo_params.yaml）
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # 动作组 yaml（config/actions/*.yaml）—— 任务 3 的动作是数据，必须一起装进去
        (os.path.join('share', package_name, 'config', 'actions'),
            glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youliangsiyuan',
    maintainer_email='yolo1811227775@outlook.com',
    description='总线舵机 Action 驱动、客户端与动作组播放器（CQuick 电控组 第 3、4 周考核）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'sequence_player = cquick_servo.sequence_player:main',
        ],
    },
)
