import glob
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
        
        ('share/cquick_servo/config', glob.glob('config/*.yaml')),
        ('share/cquick_servo/launch', glob.glob('launch/*.launch.py')),
        ('share/cquick_servo/config/actions', glob.glob('config/actions/*.yaml')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fan',
    maintainer_email='fan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'servo_driver = cquick_servo.servo_driver:main',
           'move_client = cquick_servo.move_client:main',
           'sequence_player = cquick_servo.sequence_player:main',
        ],
    },
)
