cmake_minimum_required(VERSION 3.8)
project(vision_interfaces)

# 构建工具与消息生成器
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 由 msg/TargetPosition.msg 生成消息代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 让依赖本包的下游包能拿到消息运行库
ament_export_dependencies(rosidl_default_runtime)

ament_package()
