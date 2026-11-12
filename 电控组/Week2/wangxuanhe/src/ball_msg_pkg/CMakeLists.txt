cmake_minimum_required(VERSION 3.8)
project(ball_msg_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ========== 1. 生成自定义消息 ==========
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ========== 2. 安装scripts下python脚本 ==========
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/turtle_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

