from setuptools import find_packages, setup

package_name = "soccer_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="you@example.com",
    description="CQuick Week3-4 monocular ball detection and field self-localization",
    license="MIT",
    # 注意：可执行入口由 CMakeLists.txt 的 install(PROGRAMS) 负责，
    # 这里不再声明 console_scripts，避免与 CMake 重复注册 target。
)
