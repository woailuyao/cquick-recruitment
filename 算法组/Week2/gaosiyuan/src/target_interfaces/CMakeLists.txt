cmake_minimum_required(VERSION 3.8)
project(target_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 找到必需的依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. 告诉编译器生成我们的消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

ament_package()
