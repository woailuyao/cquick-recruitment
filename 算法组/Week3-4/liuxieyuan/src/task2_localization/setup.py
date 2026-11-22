from setuptools import setup
from glob import glob

package_name = 'task2_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Liu Xieyuan',
    maintainer_email='20263312@users.noreply.github.com',
    description='Monocular field localization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task2_node = task2_localization.task2_node:main',
            'task1_node = task2_localization.task1_node:main',
            'sim_publisher = task2_localization.sim_publisher:main',
        ],
    },
)


