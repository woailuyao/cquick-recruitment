from glob import glob
from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ launch / config 默认不会被装进去，必须显式列出
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/actions', glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='chenbaixi',
    maintainer_email='chenbaixi@users.noreply.github.com',
    description='Servo driver with Action interface',
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
