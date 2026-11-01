cmake_minimum_required(VERSION 3.8)
project(ballpos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# 导出消息运行时依赖，给python用
ament_export_dependencies(rosidl_default_runtime)

# 安装python脚本
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/turtle_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

# 把生成的python消息模块安装到dist-packages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()

