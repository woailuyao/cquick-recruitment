cmake_minimum_required(VERSION 3.8)
project(zxy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}  "msg/BallPos.msg")

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp)
add_executable(turtle src/turtle.cpp)

ament_target_dependencies(turtle rclcpp geometry_msgs turtlesim)

rosidl_get_typesupport_target( cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(publisher "${cpp_typesupport_target}")
target_link_libraries(subscriber "${cpp_typesupport_target}")
target_link_libraries(turtle "${cpp_typesupport_target}")

install(TARGETS publisher subscriber turtle DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
