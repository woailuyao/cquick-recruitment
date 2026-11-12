cmake_minimum_required(VERSION 3.8)
project(color_target_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 本消息只使用基础字段，不需要额外依赖 geometry_msgs 等消息包。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 导出运行时依赖，Python 节点可导入生成的 color_target_interfaces.msg。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
