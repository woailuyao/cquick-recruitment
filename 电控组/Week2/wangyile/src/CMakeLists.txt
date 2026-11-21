cmake_minimum_required(VERSION 3.8)
project(cquick_week2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/go_to_balls.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()