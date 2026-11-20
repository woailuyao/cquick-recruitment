from setuptools import find_packages, setup
import os

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     data_files=[
        (os.path.join('share', package_name, 'config'), ['config/servo_params.yaml']),
        (os.path.join('share', package_name, 'launch'), ['launch/servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ray',
    maintainer_email='ray@todo.todo',
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
        ],
    },

)
