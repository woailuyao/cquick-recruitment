from setuptools import find_packages, setup

package_name = 'vision_target_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuxiaoxiao',
    maintainer_email='yuxiaoxiao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts':[
          'publisher=vision_target_node.publisher_node:main',
          'subscriber=vision_target_node.subscriber_node:main',],
    },
)
