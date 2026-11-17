import os
from glob import glob

from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ① launch 文件装到 share/<包名>/launch/ —— 只装这一处
        #    同一个文件装两个位置会报 file 'xxx' was found more than once
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # ② 参数 yaml
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ③ 动作组 yaml（任务 3 用；目录不存在时 glob 返回空列表，不影响 build）
        (os.path.join('share', package_name, 'config', 'actions'),
         glob('config/actions/*.yaml')),
        # ④ 下面两行是 python 包模板自带的，删了会导致包索引缺失，别动
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haozhihua',
    maintainer_email='haozhihua@example.com',
    description='CQuick servo driver: action server + client',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 名字 = 模块路径:函数名；左边这个就是 ros2 run 用的可执行名
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            # 任务 3 写到 sequence_player.py 之后，把下面这行前面的 # 去掉再重编
            # 'sequence_player = cquick_servo.sequence_player:main',
        ],
    },
)
