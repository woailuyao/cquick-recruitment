cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# ① 找到接口生成器（必须在 rosidl_generate_interfaces 之前）
find_package(rosidl_default_generators REQUIRED)

# ② 把 msg 和 action 都列进来生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ③ 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()
