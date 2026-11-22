cmake_minimum_required(VERSION 3.8)
project(cquick_ball)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ① 把 msg/BallPos.msg 生成成消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

# ② 把 scripts/ 下的 Python 脚本安装成可执行程序，放到 lib/<包名>
install(PROGRAMS
  scripts/ball_publisher.py
  scripts/ball_subscriber.py
  scripts/ball_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ③ 额外生成不带 .py 后缀的别名，这样 `ros2 run cquick_ball ball_publisher` 也能用
install(CODE "
  foreach(script ball_publisher ball_subscriber ball_navigator)
    set(src \"$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/\${script}.py\")
    set(dst \"$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/\${script}\")
    message(STATUS \"creating alias \${dst}\")
    file(COPY_FILE \"\${src}\" \"\${dst}\")
    file(CHMOD \"\${dst}\" PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  endforeach()
")

ament_package()
