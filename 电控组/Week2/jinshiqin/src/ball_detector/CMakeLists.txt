cmake_minimum_required(VERSION 3.8)
project(ball_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS
  scripts/publisher
  scripts/subscriber
  scripts/turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
