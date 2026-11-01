cmake_minimum_required(VERSION 3.8)
project(ball_pos)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ① 把 msg/BallPos.msg 编译成 Python 里能 import 的消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ② 把 scripts/ 下的 Python 脚本安装到 lib/ball_pos，ros2 run 才找得到
#    RENAME 去掉 .py 后缀，这样命令是 ros2 run ball_pos ball_publisher
install(PROGRAMS scripts/ball_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_publisher
)
install(PROGRAMS scripts/ball_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_subscriber
)
install(PROGRAMS scripts/ball_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_navigator
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
