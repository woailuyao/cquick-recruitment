import glob
from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 和 config 默认不会被装进 install，必须显式加
        ('share/' + package_name + '/launch', ['launch/servo.launch.py']),
        ('share/' + package_name + '/config', ['config/servo_params.yaml']),
        # 动作组 yaml：用 glob 收，新增动作文件不用改 setup.py
        ('share/' + package_name + '/config/actions',
            glob.glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenqishu',
    maintainer_email='chenqishu@todo.todo',
    description='舵机驱动：Action Server + 串口 + 参数化 + 动作组播放器',
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
