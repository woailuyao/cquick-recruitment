cmake_minimum_required(VERSION 3.8)
project(ball_talker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS
  scripts/publisher.py
  scripts/subscriber.py
  scripts/ball_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
