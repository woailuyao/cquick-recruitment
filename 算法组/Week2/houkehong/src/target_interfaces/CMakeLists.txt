cmake_minimum_required(VERSION 3.8)
project(target_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 生成 .msg 需要 rosidl 代码生成器
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 运行时要能找到生成的 Python/C++ 类型支持
ament_export_dependencies(rosidl_default_runtime)
ament_package()
