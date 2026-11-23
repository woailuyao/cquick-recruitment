cmake_minimum_required(VERSION 3.8)
project(cquick_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) 找 ament_cmake（所有 ROS2 的 CMake 包都要）
find_package(ament_cmake REQUIRED)
# 2) 找消息生成器：这一行决定能不能把 .msg 变成 Python/C++ 代码
find_package(rosidl_default_generators REQUIRED)

# 3) 声明要生成哪些消息。路径相对于本包的根目录
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 4) 运行期依赖：别人 import 这个消息时也需要它
ament_export_dependencies(rosidl_default_runtime)

ament_package()
