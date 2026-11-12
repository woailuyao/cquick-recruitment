cmake_minimum_required(VERSION 3.8)
project(ball_pos_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 基础依赖 ---
find_package(ament_cmake REQUIRED)

# --- ① 消息生成：把 msg/BallPos.msg 编成 ball_pos_pkg/msg/BallPos ---
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# --- ② 安装 Python 脚本到 lib/<包名>，安装后才算可执行程序 ---
install(PROGRAMS
  scripts/ball_pos_publisher.py
  scripts/ball_pos_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
