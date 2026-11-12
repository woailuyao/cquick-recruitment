cmake_minimum_required(VERSION 3.8)
project(ball_pos_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # ← 对应 package.xml 的 ②

# 把 msg/BallPos.msg 生成为消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

ament_export_dependencies(rosidl_default_runtime)  # ← 对应 ④

# 脚本装进 lib/<包名>
install(PROGRAMS
  scripts/ball_pub.py
  scripts/ball_sub.py
  scripts/ball_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
