cmake_minimum_required(VERSION 3.8)
project(cquick_ball_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ① 调用消息生成宏，把 msg/BallPos.msg 编成 BallPos 类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ② 把 scripts/ 下的 Python 脚本装成可执行程序，装到 lib/<包名>
#    不写这一段，ros2 run 就会报 "No executable found"
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
