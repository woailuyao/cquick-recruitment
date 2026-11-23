# 第 2 周考核 · zhuwenxuan

- 姓名：zhuwenxuan
- 学号 / 联系方式：20266268（重庆大学）· 19123100156
- 完成日期：2026-11-01
- 运行环境：WSL2 + Ubuntu 22.04.5 + ROS2 Humble

---

## 一、三项任务做了什么

### 任务 1：自定义 `BallPos` 消息 + 发布/订阅（40 分）

建了一个 `ament_cmake` 包 `ball_pos`，里面三样东西：

```
src/ball_pos/
├── package.xml            声明 rosidl 生成器 + member_of_group
├── CMakeLists.txt         rosidl_generate_interfaces + install(PROGRAMS)
├── msg/BallPos.msg        int32 id / float32 x / float32 y
└── scripts/
    ├── ball_publisher.py  2 Hz 循环发 3 个球
    ├── ball_subscriber.py 收到就打印
    └── turtle_navigator.py（任务 2）
```

三个球的坐标严格用细则给的值：`(1: 2,3)`、`(2: 5.5,8)`、`(3: 9,4)`。

**运行方式**（注意：脚本装进 `lib/ball_pos/` 时保留了 `.py` 后缀，所以 `ros2 run` 后面必须带 `.py`，这也是我踩过的一个坑）：

```bash
ros2 run ball_pos ball_publisher.py     # 发布端
ros2 run ball_pos ball_subscriber.py    # 订阅端
ros2 run ball_pos turtle_navigator.py   # 任务 2 的导航节点
```

**发布频率做成了 ROS 参数**，运行中就能改，不用重新编译：

```bash
ros2 param set /ball_publisher freq 5.0
```

改的时候节点内部会 `cancel()` 掉旧 timer、按新频率重建一个，日志会打印 `>>> 频率已切换为 5.0 Hz <<<`。

### 任务 2：订阅球坐标，闭环走到三个点（35 分）

`turtle_navigator.py` 干了这些事：

1. 订阅 `ball_pos`，用字典按 `id` 存坐标，**收齐 3 个才开始动**（长度到 3 才触发，发布端 2 Hz，最多等 1 秒）；
2. 同时订阅 `/turtle1/pose` 拿到乌龟实时位姿 —— **这是闭环的关键**；
3. 20 Hz 控制循环：算距离和方位差 → 方位差大就原地转 → 对准后边走边微调 → 距离小于 0.30 判定到点；
4. 每到一个点打印实际坐标和误差，全部走完后打印误差汇总。

**核心策略是「先对准再走 + 越近越慢」**：
- 方位差 `|diff| > 0.08` 时，`linear.x = 0`，只给 `angular.z`，原地转；
- 对准后 `linear.x = min(1.0, max(0.15, 1.0 × dist))`，**速度跟距离成正比**，快到点时自动减速，不会冲过头。

**实跑结果**（2026-11-01 13:49，全程无人工按键，用时约 22 秒）：

| 目标 | 目标坐标 | 实际到达 | 误差 |
|---|---|---|---|
| 球 1 | (2.00, 3.00) | (2.239, 3.169) | **0.293** |
| 球 2 | (5.50, 8.00) | (5.339, 7.764) | **0.286** |
| 球 3 | (9.00, 4.00) | (8.799, 4.209) | **0.290** |

平均误差 **0.290**，最大误差 **0.293**，全部 ≤ 0.5 的考核要求。

误差全部在 0.28~0.30 之间，而且**都很接近 0.30** —— 因为到点阈值 `ARRIVE_DIST` 就是 0.30，乌龟一进入这个半径就判定到点。如果想把误差压到 0.1 以内，把阈值调到 0.10 就行，代价是多绕几秒。

### 任务 3：串口与总线舵机笔记（25 分）

见 `03_串口与总线舵机笔记.md`。**回环测试未做**（没有 CH340），笔记里写了计划和预期现象，下周补做。

---

## 二、踩的坑（这次真踩了两个）

### 坑 1：`setup.bash: AMENT_TRACE_SETUP_FILES: unbound variable`

我自己写的构建脚本一跑就崩，报错位置在 `/opt/ros/humble/setup.bash` 第 8 行，看起来像是 ROS 装坏了。

**真实原因**：我在脚本开头写了 `set -uo pipefail`。`-u` 的意思是「遇到未定义变量就报错退出」，而 ROS 的 `setup.bash` 内部会引用一堆**可能没定义**的变量（`AMENT_TRACE_SETUP_FILES`、`COLCON_TRACE` 等），靠的是 shell 默认「未定义当空字符串」的行为。开了 `-u`，一 `source` 就把人家的正常代码当成错误。

**解决**：去掉 `u`，只保留 `set -o pipefail`。

**教训**：任何要 `source` ROS 环境的脚本，都**不能**用 `set -euo pipefail` 里那个 `u`。这个坑特别隐蔽，因为你自己写的一个字没错，报错却指向系统目录。

### 坑 2：`colcon: command not found`

装完 `ros-humble-desktop`，以为构建工具也有了，结果 `colcon build` 直接找不到命令。

**真实原因**：`colcon` **不是 ROS2 的一部分**，是个独立的 Python 构建工具。`ros-humble-desktop` 只给你运行时和库，构建工具要另外装。

**解决**：

```bash
sudo apt install -y python3-colcon-common-extensions
```

装完在 `/usr/bin/colcon`。顺带发现 `~/.local/bin` 不在 PATH 里（pip 装的工具都放那儿），已经在 `~/.bashrc` 补了一行。

### 坑 3（提前避开了）：`member_of_group`

查文档时看到一堆人说自定义消息编译报 `rosidl` 错，99% 是 `package.xml` 少声明接口包分组。所以我在写 `package.xml` 时就直接加上了：

```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

这次编译才一次过（`Finished <<< ball_pos [6.29s]`）。

### 坑 4：`No executable found`

编译明明成功了，`ros2 run ball_pos ball_publisher` 却报：

```
No executable found
```

**真实原因**：`install(PROGRAMS scripts/ball_publisher.py DESTINATION lib/ball_pos)` 是把文件**原样拷过去**，文件名仍然带 `.py`。而 `ros2 run` 是**按文件名精确匹配**的，少一个 `.py` 就找不到。

**验证**：

```bash
ls ~/ros2_ws/install/ball_pos/lib/ball_pos/
# ball_publisher.py  ball_subscriber.py  turtle_navigator.py
```

**解决**：命令带上 `.py`：

```bash
ros2 run ball_pos ball_publisher.py
```

**教训**：看到 `No executable found`（注意不是 `Package not found`）时，**先去 `ls lib/<包名>/` 看真实文件名**，别凭记忆敲。包找到了但里面没这个名字，报错才会是这条。

### 坑 5：`ros2` 命令莫名抛一大段 Python 堆栈

```
http.client.RemoteDisconnected: Remote end closed connection without response
```

`ros2 topic info`、`ros2 param set`、`ros2 node list` 都中过招，每次都抛几十行 traceback。

**真实原因**：这些命令默认会先连接 `ros2cli` 的**后台守护进程（daemon）**，问它要节点图。这个 daemon 在我的 WSL 里反复崩 —— 进程还活着、端口还占着，但不应答，客户端一连上就被 reset。

**解决**（两个都管用）：

```bash
pkill -9 -f _ros2_daemon          # 杀掉，下次调用会拉起一个干净的
ros2 node list --no-daemon        # 或者直接绕过 daemon
```

**教训**：**报错位置在 `/opt/ros/` 里，不代表环境坏了**。这类"看着像系统问题"的报错，先想想是不是某个后台服务卡住了。

---

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人典型场景 |
|---|---|---|---|
| **话题 Topic** | 发布/订阅，多对多 | **单向、持续流、不管对方在不在**。发布者不知道有几个订阅者，甚至不知道有没有 | 视觉持续广播球位置；战术持续广播速度指令 |
| **服务 Service** | 请求/响应，一对一 | **同步、有返回值、调一次答一次**。客户端必须等服务器回话 | 「现在比分多少」「切到防守模式」「把场地清空」 |
| **动作 Action** | 目标/反馈/结果 | **长任务、中间报进度、可中途取消**。是「服务 + 持续反馈」的组合 | 「走到点 (1,1)」→ 一路上报「还剩 30%」→ 可以中途取消 |

### 任务 2 里实际用到的服务调用

跑导航之前我执行了这条：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

它是**服务**，不是话题。区别一眼就能看出来：

- 敲下去之后终端会打印 `response: std_srvs.srv.Empty_Response()` —— **有回应**，这就是"请求/响应"；
- 执行完就结束，不会持续输出 —— 而话题是**一直发**的；
- 它必须有一个 `/clear` 服务端在跑（turtlesim 提供的），如果 turtlesim 没启动，这条命令会一直等，等不到就报错。

**一句话对比**：话题像广播电台（一直播，爱听不听），服务像打电话（拨通、问一句、听到回答、挂断）。

---

## 四、差速底盘三个公式

乌龟是两轮差速底盘。设左轮速度 `vL`、右轮速度 `vR`、两轮间距（轮距）`L`：

$$
v = \frac{v_L + v_R}{2} \qquad
\omega = \frac{v_R - v_L}{L} \qquad
r = \frac{v}{\omega}
$$

**怎么理解：**

- **`v = (vL + vR) / 2`** —— 车身的线速度是两个轮子的**平均**。两个轮子都转 1 m/s，车就走 1 m/s；一个 1 一个 0，车走 0.5 m/s。
- **`ω = (vR − vL) / L`** —— 角速度来自两个轮子的**差**，再除以轮距。右轮比左轮快，车就往左转（正的角速度）。两个轮子一样快，差为 0，`ω = 0`，走直线。
- **`r = v / ω`** —— 转弯半径。`ω` 越小弯越大；`ω → 0` 时 `r → ∞`，就是直线。可以验证：`vL = 0`、`vR = v` 时，`v = v/2`、`ω = v/L`，`r = L/2` —— 正好绕左轮转圈，符合直觉。

**反过来用**：ROS 的 `cmd_vel` 给的是 `v` 和 `ω`，要控制真实电机，就得解这个方程组反推两个轮子的目标转速：

```
vL = v − ω·L/2
vR = v + ω·L/2
```

这就是「运动控制节点把速度换算成左右轮转速」这一步在做的事，也就是下周串口要发下去的东西。

---

## 五、给下周自己的一句提醒

> **别一看到报错在 `/opt/` 里就以为环境坏了 —— 先怀疑自己的脚本。**
>
> 这次的 `AMENT_TRACE_SETUP_FILES` 就是典型：报错位置在天上，问题在地上（我开的 `set -u`）。下周接串口，八成的"舵机不动"也会是这样 —— 先查 `PULR` 发了没、GND 接了没、波特率对不对，别急着怀疑舵机烧了。


---

## 六、截图清单（对应考核要求）

| 文件 | 对应哪条要求 | 内容 |
|---|---|---|
| `01_interface_show.png` | 任务 1 · 截图① | `ros2 interface show ball_pos/msg/BallPos` 输出，id / x / y 三字段 |
| `01_发布订阅分屏.png` | 任务 1 · 截图② | 左：`colcon build` 成功 + 2 Hz 发布日志；右：订阅端持续收到；同屏 |
| `01_话题信息.png` | 任务 1 · 截图③ | `ros2 topic info /ball_pos` → Publisher 1 / Subscription 1 |
| `01_改频率后.png` | 任务 1 · 截图④ | `ros2 param set ... freq 5.0` → `Set parameter successful` + `Double value is: 5.0` |
| `01_改频率后_2.png` | 任务 1 · 截图④（补充） | 以 5.0 Hz 启动的发布端日志 |
| `02_三个球轨迹.png` | 任务 2 · 截图① | 乌龟走完三点后的轨迹（三角形折线） |
| `02_运行日志.png` | 任务 2 · 截图② | 完整日志：收齐 3 球 → 依次到达 → 误差汇总 |
| `02_node_list.png` | 任务 2 · 截图③ | `ros2 node list`：`/ball_publisher` `/turtle_navigator` `/turtlesim` |
| `02_误差汇总.png` | 任务 2 · 截图④ | 三个点的最终误差 + 平均/最大误差 |

> 所有截图均在 2026-11-01 13:18 ~ 13:59 之间实拍，带姓名拼音 `zhuwenxuan` 与实时时间戳。
> `02_三个球轨迹` / `02_运行日志` / `02_误差汇总` 三张是把同一张全屏截图按区域裁开的，所以时间戳完全一致（13:49:50），这是同一时刻的三个视角，不是三次运行。
