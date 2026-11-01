cmake_minimum_required(VERSION 3.8)
project(ball_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ① 把 msg/BallPos.msg 编译成可用的消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ② 把 scripts/ 下的 Python 脚本装成可执行程序，放进 lib/ball_comm
#    不装的话 ros2 run 会报 No executable found
#    RENAME 去掉 .py 后缀，这样命令就是干净的一句：
#        ros2 run ball_comm ball_publisher
install(PROGRAMS scripts/ball_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_publisher
)
install(PROGRAMS scripts/ball_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_subscriber
)

ament_package()
