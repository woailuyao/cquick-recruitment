# 第 2 周提交说明 · 刘年玺（liunianxi）

## 一、基本信息

- 姓名拼音：liunianxi
- 姓名：刘年玺
- 完成时间：2026-10-31
- 运行环境：Ubuntu 22.04.5 LTS（VirtualBox 虚拟机）+ ROS 2 Humble，工作空间 `~/ros2_ws`

## 二、三项任务完成情况

### 任务 1 · `BallPos` + 发布/订阅

- 包名：`ball_comm`（ament_cmake，代码在 `src/ball_comm/`）
- 话题名：`/ball_pos`
- 发布频率：默认 **2 Hz**，频率做成了 ROS 参数 `freq`，运行中可用 `ros2 param set` 改（实测改成 5 Hz 立即生效）
- 消息字段：`int32 id` / `float32 x` / `float32 y`
- 做了什么：`ball_publisher.py` 声明参数 `freq`（默认 2.0），用一个 100 Hz 的定时器自己数节拍，到点就把 3 个球（id=1/2/3，坐标 2.0,3.0 / 5.5,8.0 / 9.0,4.0）依次发一遍并打印；`ball_subscriber.py` 订阅 `/ball_pos`，回调里打印 `id / x / y` 和累计条数。
- 验证结果：`ros2 interface show ball_comm/msg/BallPos` 能看到三个字段；`ros2 topic info /ball_pos` 显示 Publisher count: 1 / Subscription count: 1；`ros2 topic hz /ball_pos` 在 2 Hz 时是 5.95 条/s（3 球 × 2 Hz），`ros2 param set /ball_publisher freq 5.0` 之后变成 14.59 条/s。

### 任务 2 · 三个球的目标点导航

- 实现路线：**闭环**（订阅 `/turtle1/pose` 拿实时位姿，算距离和方位差）
- 到达情况：

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.08, 3.06) | 0.10 |
| 2 | (5.5, 8.0) | (5.44, 7.92) | 0.10 |
| 3 | (9.0, 4.0) | (8.94, 4.07) | 0.10 |

- 关键参数：`MAX_V = 0.8 m/s`、`MIN_V = 0.12 m/s`、`MAX_W = 1.6 rad/s`、到点阈值 `GOAL_TOL = 0.10`、方位差阈值 `ANGLE_TOL = 0.12 rad`
- 为什么这样调：一开始线速度给 1.5，靠近目标时来不及减速，冲过头再绕回来，误差经常 0.6 以上；后来改成「速度 = 1.2 × 距离」并在接近时夹到 `MIN_V`，误差就稳定在 0.10。方位差大于 0.12 rad 时只原地转（`linear.x = 0`），对准了再走，轨迹才是一条直线而不是弧线。开始前先 `ros2 service call /clear std_srvs/srv/Empty` 清屏，所以截图里的三角形轨迹只有这一次的。

### 任务 3 · 串口与总线舵机（学习笔记）

- 详细笔记见：`03_串口与总线舵机笔记.md`
- 回环测试：**未做**（手上没有 CH340，也没有舵机控制器，计划下周拿到舵机后补做）

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人场景 |
|---|---|---|---|
| 话题 Topic | 发布 / 订阅，多对多 | 单向、持续流，发的人不管有没有人在听，也不等回话 | 视觉节点持续广播球的坐标；战术节点持续广播速度指令 |
| 服务 Service | 请求 / 响应，一对一 | 同步，调一次答一次，有返回值，调用方要等 | 「现在比分多少」「切到防守模式」，还有我任务 2 开头的 `/clear` |
| 动作 Action | 目标 / 反馈 / 结果 | 长任务能报进度、能中途取消 | 「走到 (1,1)」这种要跑几秒的活，过程中报进度，随时能取消 |

**我实际用到的服务调用**：任务 2 一开始的 `ros2 service call /clear std_srvs/srv/Empty`。它必须是服务——我发一次请求，`turtlesim` 清完屏给我一个应答，这件事就结束了；如果做成话题，就没有「完成」这个时刻，我也没法确认它到底清没清。

## 四、差速三公式

- `v = (vL + vR) / 2` → 车往前走的整体速度取两个轮子的平均值：两轮同速就是直行，一个快一个慢，平均下来就是中心的推进速度。
- `ω = (vR − vL) / L` → 转弯快慢看两轮的**速度差**，再除以轮距 `L`：差值越大转得越急，轮距越宽同样的差转得越慢，两轮同速时 `ω = 0` 走直线。
- `r = v / ω` → 转弯半径。比如 `v = 0.4`、`ω = 0.2`，半径就是 2 m；`ω` 越小弯越大，`ω = 0` 就是直线（半径无穷大）。代码里「先原地转再走」就是先让 `v = 0` 把方向掉过来，再给 `v` 直着过去。

## 五、踩过的坑 & 怎么解决的

1. **想用 `ament_python` 建包再塞消息** → `ros2 pkg create --build-type ament_python` 建完发现根本没有办法编译 `.msg`，`rosidl` 只认 CMake → 换成 `--build-type ament_cmake`，`CMakeLists.txt` 里加 `rosidl_generate_interfaces`。
2. **`rosidl` 一堆链接错误** → 一开始 `package.xml` 只写了 `rclpy`，忘了 `rosidl_default_generators` / `rosidl_default_runtime` / `member_of_group` 那行 → 按报错里的 `rosidl_interface_packages` 提示补上，重新 `colcon build` 才过。
3. **`ros2 run ball_comm ball_publisher.py` 报 No executable found** → 脚本忘了 `chmod +x`，而且 `CMakeLists.txt` 里没写 `install(PROGRAMS ...)` → 两个都补上。
4. **`ros2 param set freq` 有时不生效**（设了 2 Hz，实测还是 15 条/s） → 一开始在参数回调里 `destroy_timer` 再 `create_timer`，时机不对就会漏掉 → 改成固定 100 Hz 的定时器，自己判断「距上次发送是否够 1/freq 秒」，改参数立刻生效，实测 5.95 → 14.59。
5. **`ros2 topic info` 显示 Publisher count: 0** → 节点刚起来、发现还没完成 → 等几秒再查（或者先 `ros2 topic list` 预热一下）就变成 1 / 1。

## 六、给下周自己的提醒

跑节点前先 `source /opt/ros/humble/setup.bash` 再 `source ~/ros2_ws/install/setup.bash`，两个终端都要 source，不然一会儿 ModuleNotFoundError、一会儿 topic 对不上号。
