from setuptools import setup
package_name = 'cquick_vision'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vision nodes',
    license='TODO',
    entry_points={'console_scripts': ['detect_target = cquick_vision.detect_target:main', 'target_subscriber = cquick_vision.target_subscriber:main']},
)
