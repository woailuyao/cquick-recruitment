from setuptools import setup

package_name = 'cquick_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/servo.launch.py']),
        ('share/' + package_name + '/config', ['config/servo_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loupengyv',
    maintainer_email='loupengyv@todo.todo',
    description='servo driver',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver = cquick_servo.servo_driver:main',
            'move_client = cquick_servo.move_client:main',
        ],
    },
)
