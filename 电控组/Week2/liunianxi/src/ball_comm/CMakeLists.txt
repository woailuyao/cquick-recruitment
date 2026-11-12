cmake_minimum_required(VERSION 3.8)
project(ball_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ① 调用消息生成宏，把 msg/BallPos.msg 编成消息类型
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ② 把 scripts/ 下的 Python 脚本安装成可执行程序，放进 lib/<包名>
#    （不装的话 ros2 run 会报 No executable found）
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/ball_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
