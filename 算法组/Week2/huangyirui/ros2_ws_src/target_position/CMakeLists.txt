cmake_minimum_required(VERSION 3.8)
project(target_position)  # 包名和package.xml里的<name>一样

find_package(ament_cmake REQUIRED)                      # 构建工具
find_package(rosidl_default_generators REQUIRED)        # msg-C++/Python代码生成器

rosidl_generate_interfaces(${PROJECT_NAME}              # 把.msg编译成消息类型
  "msg/TargetPosition.msg"
)

ament_export_dependencies(rosidl_default_runtime)       
ament_package()                                         #注册成 ament 包
