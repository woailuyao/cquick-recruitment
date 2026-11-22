import os
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
        # 安装 launch 文件夹
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装 config 文件夹
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'), glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cquer',
    maintainer_email='860727656@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'sequence_player = cquick_servo.sequence_player:main',
        ],
        
    },
)
