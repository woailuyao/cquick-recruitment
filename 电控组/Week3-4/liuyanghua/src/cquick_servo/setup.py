from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'cquick_servo', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'cquick_servo', 'config'), glob('config/*.yaml')),
        (os.path.join('share', 'cquick_servo', 'config', 'actions'), glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'], #TODO
    zip_safe=True,
    maintainer='hakashin',
    maintainer_email='2687289881@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_client = cquick_servo.move_client:main',
            'servo_driver = cquick_servo.servo_driver:main',
            'sequence_player = cquick_servo.sequence_player:main',
        ],
    },
)
