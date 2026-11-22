cmake_minimum_required(VERSION 3.8)
project(soccer_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 自定义消息
set(msg_files
  "msg/FieldPose.msg"
  "msg/BallDetection.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 安装 Python 节点脚本（去掉 .py 后缀，重命名为 ros2 run 可直接调用的可执行名）
install(PROGRAMS
  scripts/ros2_ball_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ros2_ball_node
)
install(PROGRAMS
  scripts/ros2_localization_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ros2_localization_node
)

# 安装 Python 库包（soccer_vision 纯 Python 模块）
install(DIRECTORY
  soccer_vision
  DESTINATION lib/python3.12/site-packages
)

ament_package()
