cmake_minimum_required(VERSION 3.8)
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ① 先找到接口生成器 —— 必须在 rosidl_generate_interfaces 之前，否则下面那个函数根本没定义
find_package(rosidl_default_generators REQUIRED)

# ServoState.msg 用了 builtin_interfaces/Time
find_package(builtin_interfaces REQUIRED)

# ② msg/ 和 action/ 两个目录的文件都要列出来。
#    漏列不会报错，只是那个接口悄悄不存在 —— 然后你在节点里 import 不到，查到半夜。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  "action/MoveServo.action"
  DEPENDENCIES builtin_interfaces
)

# ③ 导出运行时依赖，别的包 exec 依赖这个包时才加载得到消息类型
ament_export_dependencies(rosidl_default_runtime)

ament_package()
