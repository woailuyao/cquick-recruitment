cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

# 纯接口包：只生成 msg / action，不编译任何节点
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

# 导出运行时依赖：别的包链接你的接口时需要它
ament_export_dependencies(rosidl_default_runtime)

ament_package()
