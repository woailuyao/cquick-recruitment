cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ① 先找到接口生成器，才能生成 msg/action
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ② msg/ 与 action/ 都要列进来
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  "action/MoveServo.action"
  DEPENDENCIES builtin_interfaces
)

# ③ 导出运行时依赖，别的包才能加载
ament_export_dependencies(rosidl_default_runtime)

ament_package()
