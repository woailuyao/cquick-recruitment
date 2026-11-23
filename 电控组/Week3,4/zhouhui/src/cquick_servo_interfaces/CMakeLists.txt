cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) 找到接口生成器：这一行不写，下面 rosidl_generate_interfaces 会报找不到宏
find_package(rosidl_default_generators REQUIRED)

# 2) msg 和 action 都要列进来，缺一个就只生成一半接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  "action/MoveServo.action"
)

# 3) 导出运行时依赖，否则别的包在运行期 load 不到这些类型
ament_export_dependencies(rosidl_default_runtime)

ament_package()
