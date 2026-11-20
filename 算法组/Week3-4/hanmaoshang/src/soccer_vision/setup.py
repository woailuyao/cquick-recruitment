from setuptools import setup
package_name = 'soccer_vision'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@todo.todo',
    description='Vision Localization',
    license='TODO',
    entry_points={
        'console_scripts': [
            'localization_node = soccer_vision.localization_node:main',
        ],
    },
)