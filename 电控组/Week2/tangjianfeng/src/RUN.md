# 运行说明

## 1. 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cquick_ball \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
               -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

> **必须带这两个 `-D` 参数。** 本机 PATH 里有个 Python 3.13 排在前面，
> CMake 会挑错解释器：
> - 少 `Python3_EXECUTABLE` → 报 `No module named 'em'`
> - 少 `PYTHON_EXECUTABLE` → 报 `import numpy;print(numpy.get_include())`
>   returned error code 1
>
> 两个变量名不一样（第二个没有 3），是 ROS 的历史遗留，需要同时给。
> 嫌麻烦直接跑 `./build.sh`。

## 2. 任务 1

```bash
# 终端 A
ros2 run cquick_ball ball_publisher

# 终端 B（记得先 source ~/ros2_ws/install/setup.bash）
ros2 run cquick_ball ball_subscriber

# 另开终端检查
ros2 interface show cquick_ball/msg/BallPos
ros2 topic info /ball_pos
```

改频率（不用改代码）：

```bash
ros2 run cquick_ball ball_publisher --ros-args -p publish_rate:=5.0
```

## 3. 任务 2

```bash
# 终端 A
ros2 run turtlesim turtlesim_node

# 终端 B：清屏 + 设笔（红色，粗 3）
ros2 service call /clear std_srvs/srv/Empty
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
  "{r: 220, g: 30, b: 30, width: 3, off: 0}"

# 终端 C：任务 1 的发布者原样跑着
ros2 run cquick_ball ball_publisher

# 终端 D：导航节点
ros2 run cquick_ball ball_navigator
```

或者直接一键：`./run_task2.sh`

### 可调参数

```bash
ros2 run cquick_ball ball_navigator --ros-args \
  -p goal_tolerance:=0.15 \
  -p max_linear:=1.4 \
  -p max_angular:=2.6
```

## 4. 画轨迹图（可选）

```bash
# 记录真实位姿
python3 tools/record_pose.py pose.csv

# 画成图
python3 tools/plot_trajectory.py pose.csv trajectory.png
```
