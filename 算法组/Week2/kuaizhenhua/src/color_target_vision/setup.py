"""安装 Python 节点和 launch 文件；ROS 依赖由 package.xml / rosdep 管理。"""

from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'color_target_vision'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='蒯振华',
    maintainer_email='245372864+whao34011-alt@users.noreply.github.com',
    description='Week 2 image color detection and ROS 2 position messages.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_publisher = color_target_vision.publisher:main',
            'target_subscriber = color_target_vision.subscriber:main',
        ],
    },
)
