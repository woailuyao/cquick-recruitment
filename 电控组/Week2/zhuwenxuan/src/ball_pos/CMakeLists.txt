cmake_minimum_required(VERSION 3.8)
project(ball_pos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ① 消息生成：把 msg/BallPos.msg 编译成可 import 的消息类型
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ② 把 scripts/ 下的 Python 装成可执行程序，放进 lib/ball_pos
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/turtle_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
