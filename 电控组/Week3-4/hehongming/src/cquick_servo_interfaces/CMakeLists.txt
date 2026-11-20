cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# 1. 查找接口生成器依赖
find_package(rosidl_default_generators REQUIRED)

# 2. 生成接口定义（必须在 if(BUILD_TESTING) 外面！）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 3. 导出运行时依赖（写给其他包用的）
ament_export_dependencies(rosidl_default_runtime)

ament_package()