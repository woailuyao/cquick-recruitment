cmake_minimum_required(VERSION 3.8)
project(ball_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS
  scripts/talker.py
  scripts/listener.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
