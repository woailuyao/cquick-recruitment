cmake_minimum_required(VERSION 3.8)
project(ball_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 【重点】消息生成一定要放在 ament_package() 前面！
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# 这一句必须放最后！
ament_package()
