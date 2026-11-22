cmake_minimum_required(VERSION 3.8)
project(cquick_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 由 TargetPosition.msg 生成 C++/Python 消息代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
