# 第 2 周提交说明 · 娄鹏宇

## 一、基本信息

- 姓名拼音：loupengyv
- 姓名：娄鹏宇
- 完成时间：2026-10-27
- 运行环境：Ubuntu 22.04 + ROS2 Humble

## 二、三项任务完成情况

### 任务 1 · `BallPos` + 发布/订阅

- 包名：`ball_pkg`（`ament_cmake` 类型）
- 话题名：`ball_pos`
- 发布频率：**5 Hz**（0.2 s；最初是 2 Hz，做完改频实验后保留 5 Hz）
- 消息字段：`int32 id` / `float32 x` / `float32 y`
- 做了什么：定时器每 0.2 s 触发一次，每个周期把 id=1/2/3 三个球的坐标依次发一遍，发的时候打印自己发了什么；订阅端收到后打印 `id / x / y`。
- 验证结果：`ros2 topic info /ball_pos` 显示 `Publisher count: 1` / `Subscription count: 1`；订阅端持续收到并打印。频率从 2 Hz 改到 5 Hz 后，订阅端代码一行没动，但单位时间内收到的条数明显变多。

### 任务 2 · 三个球的目标点导航

- 实现路线：**闭环**（订阅 `/turtle1/pose` 拿实时位姿，结合 `/ball_pos` 算速度指令）
- 导航节点位置：`src/ball_pkg/scripts/goto_balls.py`。因为它依赖本包的自定义消息 `BallPos`，所以放进同一个包里，`colcon build` 之后可以直接 `ros2 run ball_pkg goto_balls.py`。
- 到达情况：

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.13, 3.02) | 0.13 |
| 2 | (5.5, 8.0) | (5.42, 7.88) | 0.15 |
| 3 | (9.0, 4.0) | (8.90, 4.11) | 0.15 |

- 关键参数：最大线速度 1.2 m/s、最大角速度 2.0 rad/s、到点阈值 0.15、对准阈值 0.25 rad
- 为什么这样调：`Twist` 给的是速度不是位移，如果全程用恒定速度，接近目标时一定会冲过头、来回折返。所以线速度跟着距离走——`max(0.15, min(1.2, dist))`，越近越慢；再配合"角度差大于 0.25 就先原地转、不往前走"，误差才稳定在 0.15 以内。

### 任务 3 · 串口与总线舵机（学习笔记）

- 详细笔记见：`03_串口与总线舵机笔记.md`
- 回环测试：**未做**。手头没有 CH340（USB 转 TTL）模块，也还没拿到实验室的总线舵机，无法把 TX 和 RX 短接成回路。计划下周拿到舵机后用 24 路舵机控制器补做，并把现象补进笔记。

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人场景 |
|---|---|---|---|
| 话题 Topic | 发布/订阅，多对多 | 单向、持续流、不管对方在不在 | 视觉持续广播球位置；战术持续广播速度指令 |
| 服务 Service | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切到防守模式」 |
| 动作 Action | 目标/反馈/结果 | 长任务、报进度、可取消 | 「走到点 (1,1)」→ 过程报进度，可中途取消 |

**我实际用到的服务调用**：任务 2 开始前的

```bash
ros2 service call /clear std_srvs/srv/Empty
```

它和话题的差别正好体现服务的特点：发出请求后必须等到一个响应（`std_srvs.srv.Empty_Response()`）才结束，调一次就完；而话题是发布者一直发、根本不管有没有人听。清屏的作用是把画布擦干净，让乌龟划出的轨迹从空白开始，截图才看得清。

## 四、差速三公式

- `v = (vL + vR) / 2` → 两个轮的转速取平均就是车体的前进速度；两轮同速时走直线。
- `ω = (vR − vL) / L` → 两个轮转速差除以轮距，就是车体绕自己中心转的角速度；两轮同速时 ω=0，不转。
- `r = v / ω` → 转弯半径。ω 越小弯越大，ω 趋近 0 时就是直线（半径无穷大）。

对应到 `Twist`：`linear.x` 就是 `v`，`angular.z` 就是 `ω`。所以要直行就给 `linear.x > 0, angular.z = 0`；要原地转就给 `linear.x = 0, angular.z ≠ 0`；两个都非 0 就是画弧。

## 五、踩过的坑 & 怎么解决的

1. **`Unknown CMake command "rosidl_generate_interfaces"`**
   现象：`colcon build` 直接在 `CMakeLists.txt` 第 3 行报这个错。
   查法：报错指向第 3 行，说明是调用位置的问题。
   解决：`rosidl_generate_interfaces` 这个宏是 `rosidl_default_generators` 提供的，必须先 `find_package(rosidl_default_generators REQUIRED)` 把宏加载进来再调用，CMake 是按行顺序执行的。

2. **`No executable found`**
   现象：`ros2 run ball_pkg ball_publisher.py` 说找不到可执行文件，但 `scripts/` 里明明有文件。
   查法：`ls install/ball_pkg/lib/ball_pkg/` 是空的。
   解决：`CMakeLists.txt` 里漏了 `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})`。补上之后重新 `colcon build`，脚本才会被装进 `install/`，`ros2 run` 才找得到。

3. **`OSError: [Errno 8] Exec format error`**
   现象：文件装进去了、权限也是可执行，但一跑就报这个。
   查法：想到 Linux 靠 shebang 判断用什么解释器。
   解决：脚本第一行加 `#!/usr/bin/env python3`。没有这行，系统会把它当成二进制程序去执行。

4. **反复的 `IndentationError`**
   现象：用图形编辑器改代码，缩进多一个空格或少一个空格，`ros2 run` 就甩一大段 traceback。
   解决：改用 `cat > 文件 << 'PYEOF'` 整份覆盖写入，并且跑之前先执行 `python3 -m py_compile 文件.py` 做语法检查——能在一秒内定位问题，不用等 `ros2 run` 报错。

5. **日志刷屏**
   现象：`收齐 3 个球，开始导航` 每秒打好几遍，把到达日志淹没了。
   原因：`on_ball` 每收到一条消息都判断 `len(balls) == 3`，而发布端 5 Hz 连发三条，所以一直被触发。
   解决：加一个 `started` 标志，只在第一次收齐时打印。

## 六、给下周自己的提醒

回环测试记得补（拿到舵机第一件事）；改完代码先 `python3 -m py_compile` 再 `ros2 run`，能省掉一大半排查时间。
