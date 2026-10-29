cmake_minimum_required(VERSION 3.8)
project(ball_tracker)

# 默认使用 C++14（ament_cmake 推荐）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 把 msg/BallPos.msg 编译成消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# 把 scripts/ 下的 Python 脚本安装成可执行程序，放进 lib/<包名>
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/go_to_balls.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
