cmake_minimum_required(VERSION 3.8)
project(target_position_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 找 ament_cmake 与 rosidl 生成器
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口：这里只有一个消息 TargetPosition.msg
# 该消息只用内置类型(bool/float32)，无需 DEPENDENCIES
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 导出 rosidl 运行时依赖，供其他包链接
ament_export_dependencies(rosidl_default_runtime)
ament_package()
