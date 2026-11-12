cmake_minimum_required(VERSION 3.8)
project(ball_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS
  scripts/publisher.py
  scripts/subscriber.py
  scripts/nav_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
