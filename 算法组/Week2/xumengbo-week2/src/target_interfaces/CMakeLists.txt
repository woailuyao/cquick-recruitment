cmake_minimum_required(VERSION 3.8)
project(target_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) 构建工具
find_package(ament_cmake REQUIRED)
# 2) 消息代码生成器
find_package(rosidl_default_generators REQUIRED)

# 3) 声明要生成的接口文件：<包名> <相对路径>
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 4) 把运行期依赖导出给使用者（发布/订阅节点）
ament_export_dependencies(rosidl_default_runtime)

ament_package()
