cmake_minimum_required(VERSION 3.8)
project(vision_interfaces)

# ---------------------------------------------------------------------------
# 为什么这个包必须是 ament_cmake：
#   自定义 .msg 的代码生成（rosidl）依赖 CMake 流程，
#   ament_python 包不能直接定义消息，所以接口包单独拆出来，节点包去依赖它。
#   这也是"接口包 + 节点包"两个包分开建的的原因。
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

# rosidl_default_generators：编译期用来把 .msg 生成 Python/C++ 数据类
find_package(rosidl_default_generators REQUIRED)

# 声明本包要生成的接口：一个消息 TargetPosition.msg
# 以后加新的 msg/srv，在下面再添一行 "msg/Xxx.msg" 即可
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPosition.msg"
)

# 声明本包属于 rosidl 接口包组（框架会据此注册运行时支持），
# 对应 package.xml 里的 <member_of_group>rosidl_interface_packages</member_of_group>

# 把 rosidl_default_runtime 作为导出依赖：下游包 find_package 本包时自动获得运行时支持
ament_export_dependencies(rosidl_default_runtime)

ament_package()
