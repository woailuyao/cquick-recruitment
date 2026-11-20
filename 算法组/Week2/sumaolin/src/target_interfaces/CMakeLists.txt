cmake_minimum_required(VERSION 3.8)
project(target_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 这一行才是真正"把 .msg 翻译成 Python/C++ 代码"的动作，
# 它发生在 colcon build 阶段，所以改了 .msg 必须重新 build 并重新 source。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
