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
        ('share/' + package_name + '/config', ['config/servo_params.yaml']),
         ('share/' + package_name + '/launch', ['launch/servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangxilu',
    maintainer_email='1634530274@qq.com',
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
