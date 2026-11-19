from setuptools import setup
from glob import glob
import os

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'), glob('config/actions/*.yaml'))
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='evan',
    maintainer_email='evan@xxx.com',
    description='Servo driver node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'sequence_player = cquick_servo.sequence_player:main',
            'scan_ids = cquick_servo.scan_ids:main'
        ],
    },
)
