cmake_minimum_required(VERSION 3.8)
project(ball_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
# 2. 定义消息文件
set(msg_files "msg/BallPos.msg")

# 3. 生成自定义消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# 4. 编译发布者节点
add_executable(ball_publisher src/ball_publisher.cpp)
ament_target_dependencies(ball_publisher rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ball_publisher "${cpp_typesupport_target}")

# 5. 编译订阅者节点
add_executable(ball_subscriber src/ball_subscriber.cpp)
ament_target_dependencies(ball_subscriber rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ball_subscriber "${cpp_typesupport_target2}")

# 6. 编译任务2导航节点
add_executable(turtle_navigator src/turtle_navigator.cpp)
ament_target_dependencies(turtle_navigator rclcpp geometry_msgs turtlesim)
rosidl_get_typesupport_target(nav_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(turtle_navigator "${nav_typesupport}")

# 7. 安装节点
install(TARGETS
  ball_publisher
  ball_subscriber
  turtle_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# 8. 必须在最后
ament_package()