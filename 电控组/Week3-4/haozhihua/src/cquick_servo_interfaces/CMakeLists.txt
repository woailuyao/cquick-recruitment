cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 只放接口定义，所以除了 ament 本身只需要接口生成器
find_package(ament_cmake REQUIRED)
# ① 接口生成器：少这一行，rosidl_generate_interfaces 直接找不到命令
find_package(rosidl_default_generators REQUIRED)

# ② 把 msg 和 action 都列进去生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
)

# ③ 导出运行时依赖：别的包运行时才能加载这几个 .so
ament_export_dependencies(rosidl_default_runtime)

ament_package()
