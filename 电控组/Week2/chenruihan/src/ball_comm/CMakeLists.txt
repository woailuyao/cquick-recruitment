cmake_minimum_required(VERSION 3.8)
project(ball_comm)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/ball_pub.py
  scripts/ball_sub.py
  scripts/turtle_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
