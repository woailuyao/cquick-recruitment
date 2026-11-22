cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 必须先找到接口生成器，再调用 rosidl_generate_interfaces
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

# 导出运行期依赖，否则引用本包的节点 import 时报找不到类型
ament_export_dependencies(rosidl_default_runtime)

ament_package()
