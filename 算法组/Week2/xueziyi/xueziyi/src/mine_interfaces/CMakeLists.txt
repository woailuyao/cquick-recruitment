# CMakeLists.txt = 这个包的「构建脚本」
# =====================================================================
# colcon build 会按这里写的步骤，把 msg/TargetPosition.msg 编译成
# C++ 头文件和 Python 类，安装到 install/ 下。

cmake_minimum_required(VERSION 3.8)

# 项目名：必须和 package.xml 的 <name> 一致
project(mine_interfaces)

# 加载 ROS2 的 CMake 扩展（提供 rosidl_generate_interfaces / ament_package 等宏）
find_package(ament_cmake REQUIRED)

# 加载消息代码生成器，少了它会报"未知命令 rosidl_generate_interfaces"
find_package(rosidl_default_generators REQUIRED)

# 核心：把 .msg 翻译成代码
# 生成结果会出现在 install/mine_interfaces/ 下：
#   - Python 类  : local/lib/python3.10/dist-packages/mine_interfaces/msg/_target_position.py
#   - C++ 头文件 : include/mine_interfaces/mine_interfaces/msg/target_position.hpp
#   - IDL 描述   : share/mine_interfaces/msg/TargetPosition.idl
# 节点里 from mine_interfaces.msg import TargetPosition 能成立，就靠这一句。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 把运行时依赖传递给"依赖本包的其他包"
ament_export_dependencies(rosidl_default_runtime)

# 收尾，必须是最后一行；少了它 colcon 报 "ament_package() not called"
ament_package()
