cmake_minimum_required(VERSION 3.8)
project(ball_pos_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/nav_to_ball.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
