import os
from setuptools import find_packages, setup
import glob

setup(
    name='cquick_servo',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cquick_servo']),
        ('share/cquick_servo', ['package.xml']),
        (os.path.join('share', 'cquick_servo', 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', 'cquick_servo', 'config'), glob.glob('config/*.yaml')),
        (os.path.join('share', 'cquick_servo', 'config', 'actions'), glob.glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    description='Servo driver',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
            'sequence_player = cquick_servo.sequence_player:main',
        ],
    },
)
