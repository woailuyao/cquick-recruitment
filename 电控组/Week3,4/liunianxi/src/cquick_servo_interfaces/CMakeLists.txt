cmake_minimum_required(VERSION 3.8)
# 包名必须与 package.xml 的 <name> 完全一致，否则 rosidl 生成路径对不上。
project(cquick_servo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# interfaces 包必须是 ament_cmake：msg/action 要在编译期由 rosidl 生成代码。
find_package(ament_cmake REQUIRED)

# ① 先找到接口生成器，才能生成 msg/action
# 找不到这个包 = 没装 rosidl 生成器，编译会直接失败。
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ② msg/ 与 action/ 都要列进来
# 三个接口一次生成；新增 .msg 必须在这里补一行，否则“生成了但不认识”。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "msg/ServoState.msg"
  "action/MoveServo.action"
  # 用到 builtin_interfaces/Time，就必须在这里声明依赖，否则生成时报找不到类型。
  DEPENDENCIES builtin_interfaces
)

# ③ 导出运行时依赖，别的包才能加载
# 导出运行时依赖：别的包 import 我们的接口时才加载得到序列化库。
ament_export_dependencies(rosidl_default_runtime)

ament_package()
