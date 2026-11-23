cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 找到接口生成器
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. msg/ 和 action/ 里的文件都要列进来，漏一个那个接口就不存在
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

# 3. 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()
