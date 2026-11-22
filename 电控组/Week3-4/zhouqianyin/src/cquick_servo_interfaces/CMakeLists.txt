cmake_minimum_required(VERSION 3.8)

# 项目名必须与 package.xml 的 <name> 严格一致
project(cquick_servo_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # 代码生成器
find_package(builtin_interfaces REQUIRED)          # 基础类型依赖

# 一次性声明本包所有接口文件；新增 msg/action 必须在这里追加，否则不会被生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCommand.msg"
  "action/MoveServo.action"
  DEPENDENCIES builtin_interfaces
)

# 把运行时依赖导出给下游包，否则 cquick_servo 编译时会报找不到消息类型
ament_export_dependencies(rosidl_default_runtime)

ament_package()
