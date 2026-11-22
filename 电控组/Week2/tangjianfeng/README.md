# 第 2 周考核 · 通信与运动控制

> 姓名拼音：`tangjianfeng`（唐渐锋）　｜　ROS 2 Humble / Ubuntu 22.04

三项都已完成：任务 1（自定义消息 + 发布/订阅）、任务 2（闭环依次到达三个球位）、
任务 3（串口与总线舵机笔记，回环测试下周补做）。

> 所有截图都含「姓名 + 实时时间戳」：先把
> `echo "姓名: tangjianfeng (唐渐锋)"; date "+%Y-%m-%d %H:%M:%S"` 的输出
> 和命令结果截在同一张图里，证明是本人当场做的。

---

## 一、三项任务各做了什么

### 任务 1 · `BallPos` 消息 + 发布/订阅（40 分）

**包名**：`cquick_ball`（`ament_cmake` 类型）

**消息定义** `msg/BallPos.msg`：

```
int32 id      # 球编号 1/2/3
float32 x     # 场地坐标系下的 x（米）
float32 y     # 场地坐标系下的 y（米）
```

**三个球的坐标（写死，未改动）**：

| 球 | x | y |
| --- | --- | --- |
| 1 | 2.0 | 3.0 |
| 2 | 5.5 | 8.0 |
| 3 | 9.0 | 4.0 |

**文件说明**

| 文件 | 作用 |
| --- | --- |
| `msg/BallPos.msg` | 消息定义 |
| `scripts/ball_publisher.py` | 发布者，定时器每周期连发 3 条（id 1→2→3） |
| `scripts/ball_subscriber.py` | 订阅者，回调打印 id / x / y |
| `scripts/ball_navigator.py` | 任务 2 的闭环导航节点 |

**关键配置点**

- `package.xml` 里除了 `ament_cmake`，还必须有：
  `rosidl_default_generators`（buildtool）、`rclpy`、`rosidl_default_runtime`（exec），
  以及 **`<member_of_group>rosidl_interface_packages</member_of_group>`**。
- `CMakeLists.txt` 里两件事：
  1. `rosidl_generate_interfaces(${PROJECT_NAME} "msg/BallPos.msg")`；
  2. `install(PROGRAMS scripts/xxx.py DESTINATION lib/${PROJECT_NAME})`。

**发布频率可调（加分项）**：用 ROS 参数 `publish_rate`，不改代码

```bash
ros2 run cquick_ball ball_publisher --ros-args -p publish_rate:=5.0
```

**实测结果**：两档都留了图（`01_频率变化_2Hz.png` / `01_频率变化_5Hz.png`）。
同一个发布端、同一个订阅端，只改 `publish_rate`：2 Hz 时日志里相邻两组的
时间戳间隔约 0.5 s，5 Hz 时缩到约 0.2 s，订阅端序号涨得更快；速率随参数线性变化，
代码一行没改。

---

### 任务 2 · 依次走到 3 个坐标（35 分）

**实现路线**：路线 A · 闭环跟随（加分项 +5）

**节点**：`ball_navigator`，接口如下

| 方向 | 话题 / 服务 | 类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/ball_pos` | `cquick_ball/msg/BallPos` | 收球坐标，用字典按 id 存 |
| 订阅 | `/turtle1/pose` | `turtlesim/msg/Pose` | 拿当前 (x, y, θ) |
| 发布 | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | 发速度指令 |
| 发布 | `/reached_ball` | `cquick_ball/msg/BallPos` | 到达后广播（可选） |
| 服务 | `/turtle1/set_pen` | `turtlesim/srv/SetPen` | 结束时抬笔 |

**控制逻辑（20 Hz 控制周期）**

```
对当前目标球：
    dx, dy = 目标 - 当前
    dist   = sqrt(dx² + dy²)
    alpha  = normalize(atan2(dy, dx) - theta)      # 方位角差

    if dist < 0.15:            → 到点，停车、记日志、切下一个球
    elif |alpha| > 0.35:       → 原地转：linear.x = 0, angular.z = kp_w * alpha
    else:                      → 边走边调：
                                   v = clamp(kp_v * dist, v_floor, v_max)
                                   v *= max(0.35, 1 - 0.6*|alpha|/0.35)
                                   angular.z = kp_w * alpha
```

**实测结果**（一次跑完，全程无人工按键）：

| 球 | 目标 | 实际 | 误差 |
| --- | --- | --- | --- |
| 1 | (2.0, 3.0) | (2.106, 3.072) | **0.128** |
| 2 | (5.5, 8.0) | (5.423, 7.892) | **0.132** |
| 3 | (9.0, 4.0) | (8.910, 4.103) | **0.137** |

全部远小于 0.5（我设的到点阈值是 0.15），总耗时约 15 秒。
`02_三个球轨迹.png` 用的是同一次运行实录的 `/turtle1/pose` 数据（2131 个点）画的，
不是示意图。

**为什么“先对准再走”**：两轮差速底盘只能沿车头方向前进（`linear.x` 是沿 x 轴的），
方向不对时给线速度只会走偏、绕大圈。所以先用 `angular.z` 把 θ 掰到目标方向
（此时 `linear.x = 0`，纯原地转，不移动位置），对准后再边走边微调。
**为什么越近越慢**：`v` 和 `dist` 成正比，接近目标时 v 趋近于 0；
否则以固定速度冲过去，惯性 + 一个控制周期的延迟一定冲过头。

---

### 任务 3 · 串口与总线舵机笔记（25 分）

正文见 `03_串口与总线舵机笔记.md`，包含细则要求的 5 块内容。
回环测试**未做**，计划下周拿到舵机补做，笔记里已写预期现象。

---

## 二、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人典型场景 |
| --- | --- | --- | --- |
| **话题 Topic** | 发布/订阅，多对多 | 单向、持续流、不管对方在不在 | 视觉持续广播球位置；战术持续广播速度指令 |
| **服务 Service** | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切到防守模式」 |
| **动作 Action** | 目标/反馈/结果 | 长任务、报进度、可取消 | 「走到点 (1,1)」→ 过程报进度，可中途取消 |

**任务 1 用的是话题**：发布者 2 Hz 一直发，不管有没有订阅者。

**任务 2 里实际用到的服务调用**（开头清屏）：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

它和话题的差别一眼就能看出来——**会阻塞等回应，回完就结束**，
不是持续流。返回的 `std_srvs.srv.Empty_Response()` 就是“收到并执行了”。

另外代码里抬笔用的 `/turtle1/set_pen` 也是服务（`turtlesim/srv/SetPen`）。

**两个节点靠什么“对上号”**：话题名 + 消息类型，两者都要一致。
名字不一样连不上；名字一样但类型不同（比如一边 `std_msgs/String`）也连不上。
这就是为什么 `ros2 topic info /ball_pos` 里会同时显示 Type。

**五个词**：
- **节点 Node**：一个独立跑着的小程序（一个进程里可以有多个），干一件具体的事；
- **话题 Topic**：一条有名字的“广播频道”，比如 `ball_pos`；
- **消息 Message**：在话题上传的数据格式，是结构体，比如 `BallPos`；
- **发布者 Publisher**：往话题里写数据的那个对象；
- **订阅者 Subscription**：从话题里读数据的那个对象。

---

## 三、差速三公式

设左右轮速 `vL`、`vR`，轮距 `L`：

| 公式 | 含义 |
| --- | --- |
| `v = (vL + vR) / 2` | 线速度 = 两轮速度的平均。两轮一样快 → 走直线；一快一慢 → 平均下来还是往前走 |
| `ω = (vR − vL) / L` | 角速度 = 两轮速差除以轮距。`vR > vL` 左转（ω > 0），`vL > vR` 右转。轮距越大，同样的速差转得越慢 |
| `r = v / ω` | 转弯半径。ω → 0 时 r → ∞，即走直线；ω = 0 且 v 也 0 就是原地不动 |

`Twist` 里 `linear.x` 对应 `v`、`angular.z` 对应 `ω`，所以这三条式子是
「我给的 Twist」和「电机要转多快」之间的桥。

---

## 四、遇到什么问题、怎么解决的

### 坑 1 · `colcon build` 报 `No module named 'em'`（卡了最久）

现象：`rosidl_adapter` 导入 `em` 失败，`rosidl_generate_interfaces` 直接崩。

排查：`/usr/bin/python3 -c "import em"` 正常，说明模块在系统 Python 里是有的。
但 `build/cquick_ball/CMakeCache.txt` 里 `Python3_EXECUTABLE` 指向的是
`/home/miku/.workbuddy/.../python3.13`——环境里有个 Python 3.13 排在 PATH 前面，
CMake 挑错了解释器，而 ROS Humble 的 `em` 只装在 Python 3.10 下。

**第二次踩（更隐蔽）**：加上 `-DPython3_EXECUTABLE=/usr/bin/python3` 之后，
在旧构建目录里能过；但换一个干净工作空间重新编译就挂了：

```
CMake Error at rosidl_generator_py_generate_interfaces.cmake:204:
execute_process(.../python3.13 -c 'import numpy;print(numpy.get_include())')
returned error code 1
```

查 `CMakeCache.txt` 才发现里面**有两个变量**：

```
Python3_EXECUTABLE:UNINITIALIZED=/usr/bin/python3
PYTHON_EXECUTABLE:FILEPATH=/home/miku/.workbuddy/.../python3.13   ← 元凶
```

`rosidl_adapter` 用 `Python3_EXECUTABLE`，但 `rosidl_generator_py`
用的是**不带 3 的旧变量 `PYTHON_EXECUTABLE`**，我只指定了一个，没管到它。

解决：两个都要指定

```bash
colcon build --packages-select cquick_ball \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
               -DPYTHON_EXECUTABLE=/usr/bin/python3
```

顺便把脚本的 shebang 从 `#!/usr/bin/env python3` 改成 `#!/usr/bin/python3`，
否则 `ros2 run` 会用错解释器，报 `ModuleNotFoundError: No module named 'rclpy'`。

**教训**：看到 `ModuleNotFoundError` 先确认「哪个 Python 在跑」，
`which -a python3` 和 `printenv PATH` 比瞎猜有用。

### 坑 2 · `ros2 run` 报 No executable found，但 `ros2 pkg executables` 能列出来

现象非常迷惑：`ros2 pkg executables cquick_ball` 明明有输出，
`ros2 run cquick_ball ball_publisher` 却说找不到。

原因：**`ros2 run` 按文件名精确匹配，不会去掉 `.py` 后缀**。
安装后 `lib/cquick_ball/` 里只有 `ball_publisher.py`，所以要写全名
`ros2 run cquick_ball ball_publisher.py` 才行。

解决：在 `CMakeLists.txt` 里 `install(PROGRAMS ...)` 之后再生成一份无后缀别名：

```cmake
install(CODE "
  foreach(script ball_publisher ball_subscriber ball_navigator)
    set(src \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/\${script}.py\")
    set(dst \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/\${script}\")
    file(COPY_FILE \"\${src}\" \"\${dst}\")
    file(CHMOD \"\${dst}\" PERMISSIONS ...)
  endforeach()
")
```

这样 `ros2 run cquick_ball ball_publisher` 和 `... ball_publisher.py` 都能用。

### 坑 3 · 发布者一启动就崩（`__init__` 里创建定时器）

第一版把 `1.0 / rate` 写在 `declare_parameter` 之后，但 `rate` 变量从
`self.get_parameter('publish_rate').value` 取——参数还没被上游覆盖，
`1.0 / None` 或者除零直接抛异常，节点还没 spin 就死了。

解决：把频率存成 `self.publish_rate = float(...)`，并且**定时器必须在参数
读取之后创建**。参数声明和读取顺序不能乱。

### 坑 4 · 订阅者一条都收不到

两次踩到：
1. 发布者和订阅者几乎同时启动，DDS 发现还没完成，前几条消息发在空气里；
2. 话题名写成了 `/ball_pos`，发布端是 `ball_pos`（相对名，实际展开是 `/ball_pos`）——
   这个其实能对上，真正的问题是第 1 条。

解决：测试时先起发布端，等 4 秒再起订阅端；顺便用
`ros2 topic info /ball_pos` 确认 `Publisher count: 1 / Subscription count: 1`。

### 坑 5 · 乌龟到点附近打转

初版 `align_threshold` 给得太大（0.6 rad），乌龟总在“转一点就能对准”和
“边走边转”之间反复横跳，于是绕着目标点画圈。

解决：
- 把阈值收到 0.35 rad；
- `v` 下限（`v_floor = 0.6`）避免距离很近时速度掉到 0 而卡住；
- 转弯时按 `|alpha|` 降速（最多降到 35%），防止大弧线。

---

## 五、给下周自己的提醒

**“先确认环境，再怀疑代码。”**
这一周有三次以为是逻辑写错，结果都是环境问题——Python 解释器选错、
shebang 指错、`source` 漏掉。下周接舵机之前，先跑一遍
`printenv ROS_DISTRO`、`ros2 pkg list | grep cquick`、`which python3`，
三条命令 5 秒钟，能省掉半小时的瞎查。
还有一条：**接线之前先看 GND**，别等收不到数据才想起来。

---

## 附录 · 截图对照表

细则要求的每一项证据，对应到本目录下的文件：

### 任务 1

| 细则要求 | 文件 |
| --- | --- |
| `ros2 interface show` 输出（证明消息定义成功） | `01_interface_show.png` |
| `colcon build` 无报错 + 两终端同时运行、订阅端在打印（分屏） | `01_发布订阅分屏.png` |
| `ros2 topic info /ball_pos`（Publisher 1 / Subscription 1） | `01_话题信息.png` |
| 改频率后订阅端输出变化 | `01_频率变化_2Hz.png`、`01_频率变化_5Hz.png` |

### 任务 2

| 细则要求 | 文件 |
| --- | --- |
| 走完三个点、轨迹清楚可见 | `02_三个球轨迹.png` |
| 运行日志（到达球 1/2/3 + 误差） | `02_运行日志.png`、`02_运行日志.txt` |
| `ros2 node list`（能看到导航节点） | `02_nodelist.png` |
| 三个点的最终误差汇总 | `02_运行日志.txt` 最后四行 / `02_轨迹数据/navigator_raw.txt` |

### 任务 3

| 细则要求 | 文件 |
| --- | --- |
| 串口与总线舵机笔记（5 块内容） | `03_串口与总线舵机笔记.md` |
| 回环测试 | **未做**，下周拿到舵机后补做，笔记 §2 已写预期现象 |

> 所有截图均含 `姓名: tangjianfeng (唐渐锋)` + 实时时间戳，均为本人当场拍摄。
> `02_轨迹数据/pose.csv` 是那次运行的 `/turtle1/pose` 实录（2131 个采样点），`plot_trajectory.py` 用它重绘出 `02_三个球轨迹.png`。

