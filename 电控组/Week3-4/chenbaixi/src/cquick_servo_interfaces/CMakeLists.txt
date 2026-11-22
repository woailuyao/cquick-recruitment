cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 注意：msg 和 action 两个文件都要列进来
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
