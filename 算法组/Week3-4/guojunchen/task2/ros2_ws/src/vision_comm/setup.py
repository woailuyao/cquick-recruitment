from setuptools import find_packages, setup

package_name = 'vision_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localize.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guojunchen',
    maintainer_email='guojunchen@todo.todo',
    description='足球检测与场地定位的 ROS2 节点',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pub = vision_comm.pub:main',
            'sub = vision_comm.sub:main',
            'localize = vision_comm.localize:main',
        ],
    },
)
