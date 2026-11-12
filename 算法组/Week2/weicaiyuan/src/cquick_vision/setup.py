from setuptools import setup
package_name = 'cquick_vision'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weicaiyuan',
    maintainer_email='you@example.com',
    description='week2 pub sub',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_publisher = cquick_vision.target_publisher:main',
            'target_subscriber = cquick_vision.target_subscriber:main',
        ],
    },
)
