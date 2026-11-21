from setuptools import find_packages, setup

package_name = 'my_vision_pkg'

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
    maintainer='chensiyou29',
    maintainer_email='chensiyou29@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_publisher = my_vision_pkg.vision_publisher:main',
            'vision_subscriber = my_vision_pkg.vision_subscriber:main',
        ],
    },
)