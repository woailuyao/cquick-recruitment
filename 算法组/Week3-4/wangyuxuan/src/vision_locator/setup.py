from setuptools import setup

package_name = 'vision_locator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='Vision Locator',
    license='TODO',
    entry_points={
        'console_scripts': [
            'locator = vision_locator.ros2_locator:main',
        ],
    },
)
