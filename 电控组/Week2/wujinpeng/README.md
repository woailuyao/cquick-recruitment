# 第 2 周考核 · 吴锦鹏

- 姓名拼音：wujinpeng
- 姓名：吴锦鹏
- 完成时间：2026-11-01
- 运行环境：Ubuntu 22.04 + ROS2 Humble（VirtualBox 虚拟机）

---

## 一、三项任务完成情况

### 任务 1 · `BallPos` 消息 + 发布/订阅　（40 分）

- 包名：`ball_pos`（`ament_cmake` 类型）
- 话题名：`ball_pos`
- 发布频率：默认 2 Hz，可用 `ros2 param` 在线改成 5 Hz（加分项）
- 消息字段：`int32 id` / `float32 x` / `float32 y`
- 做了什么：
  用 `ament_cmake` 建包（自定义消息走不了 `ament_python`），在 `msg/BallPos.msg` 里写三个字段。
  发布者用定时器每 0.5 s 触发一次，一个周期里把 **球1 (2.0, 3.0) → 球2 (5.5, 8.0) → 球3 (9.0, 4.0)** 依次发出去，同时打印自己发了什么；
  订阅者订阅同名话题 `ball_pos`，收到后把 `id / x / y` 打出来。
- 验证结果：
  - `ros2 interface show ball_pos/msg/BallPos` → `int32 id` / `float32 x` / `float32 y`
  - `ros2 topic info /ball_pos` → `Publisher count: 1`、`Subscription count: 1`
  - 订阅端持续收到，序号从第 1 条一路涨到几百条不断
- 加分项（+3）：`freq` 做成 ROS2 参数。`ros2 param set /ball_publisher freq 5.0` 返回 `Set parameter successful`，订阅端刷新速度立刻从 0.5 s 一条变成 0.2 s 一条，**不用改代码也不用重新编译**。

截图：`01_interface_show.png` / `01_发布订阅分屏.png` / `01_话题信息.png` / `01_改频率.png`

### 任务 2 · 三个球的目标点导航　（35 分）

- 实现路线：**闭环**（订阅 `/turtle1/pose` 拿乌龟实时位姿）+ 日志打印每个点实际误差（加分项 +5）
- 到达情况：

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.12, 3.01) | 0.116 |
| 2 | (5.5, 8.0) | (5.43, 7.91) | 0.110 |
| 3 | (9.0, 4.0) | (8.93, 4.09) | 0.111 |

三个点误差全部 ≤ 0.5，顺序 1 → 2 → 3，全程无人工按键，一次跑完。

- 关键参数：最大线速度 `1.6` m/s、最大角速度 `2.0` rad/s、到点阈值 `0.12`、距离比例系数 `1.4`、角度比例系数 `2.2`、对准阈值 `0.25` rad
- 思路：
  1. 收齐 3 个球再动 —— 用字典按 `id` 存坐标，长度到 3 才开始（没收到球之前只等，不发速度）
  2. 每 0.05 s（20 Hz）算一次：`dist = hypot(dx, dy)`，`err = atan2(dy, dx) − theta`
  3. `|err| > 0.25` → 原地转（`linear.x = 0`），先对准再走
  4. 对准了 → `linear.x = 1.4 × dist`（上限 1.6），**越近越慢**，所以不会冲过头
  5. `dist < 0.12` 就认为到点，停车、打日志、切下一个
- 为什么这样调：
  一开始只按固定速度 1.6 m/s 直冲，到点附近在目标周围画圈，误差在 0.6~0.9 之间晃。
  后来加了两条：**距离越近速度越小**（`1.4 × dist`），以及**方位差大于 0.25 就停车原地转**。
  改完误差直接从 0.6+ 掉到 0.11 左右。这就是「速度控制要收敛」的意思。

截图：`02_三个球轨迹.png` / `02_运行日志.png` / `02_节点列表.png`

### 任务 3 · 串口与总线舵机（学习笔记）　（25 分）

- 详细笔记见：`03_串口与总线舵机笔记.md`
- 回环测试：**未做**（手头没有 CH340，计划下周拿到舵机后补做）

---

## 二、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人场景 |
|---|---|---|---|
| **话题 Topic** | 发布 / 订阅，多对多 | 单向、持续流、不管对方在不在 | 视觉持续广播球位置；战术持续广播速度指令 |
| **服务 Service** | 请求 / 响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切到防守模式」 |
| **动作 Action** | 目标 / 反馈 / 结果 | 长任务、能报进度、可中途取消 | 「走到点 (1,1)」→ 过程报进度，可取消 |

**我实际用到的服务调用**：任务 2 开始前的

```bash
ros2 service call /clear std_srvs/srv/Empty
```

它必须是**服务**而不是话题，理由很直白：我要的是「现在立刻把画布擦干净，并且告诉我擦完了」——
- 它只发生**一次**，不是每秒都发（话题是持续流）；
- 它**有回应**，终端会打出 `std_srvs.srv.Empty_Response()`，证明服务端真的执行了（话题发出去没人回应）；
- 它需要**明确的服务端**在跑（turtlesim）。如果 turtlesim 没开，这条命令会一直 `waiting for service to become available`，这也是我踩过的坑之一。

反过来，任务 1 的球坐标用的就是**话题**：发布者每 0.5 s 发一次，根本不管有没有人订阅，订阅者晚点启动也能收到后面的数据。两种方式解决的是完全不同的需求。

---

## 三、差速三公式

- `v = (vL + vR) / 2` —— 两轮速度的**平均值**，就是车体中点往前跑多快。左右轮一样快（vL = vR）时车走直线；两轮都是 0 就停着。
- `ω = (vR − vL) / L` —— 两轮速度的**差**除以轮距 L，是车体自转多快。右轮比左轮快（vR > vL），差为正，车就往左转；两轮一样快，差为 0，不转。
- `r = v / ω` —— 转弯半径。同样速度下 ω 越大，半径越小，弯越急；**ω = 0 时半径无穷大**，物理意义就是走直线（所以不能拿它当除数直接算，要先判断）。

对应到 `Twist` 上：
| 想要 | 给什么 |
|---|---|
| 直行 | `linear.x = v`，`angular.z = 0` |
| 原地转 | `linear.x = 0`，`angular.z ≠ 0` |
| 边走边转（画弧） | 两个都非 0 |

我任务 2 里「先原地对准再走」用的就是第二行：`|err| > 0.25` 时把 `linear.x` 设成 0，只给 `angular.z`。

---

## 四、踩过的坑 & 怎么解决的

1. **`ros2 run ball_pos ball_publisher` 报 `No executable found`，但 `colcon build` 明明成功了**
   → 查 `ls ~/ros2_ws/install/ball_pos/lib/ball_pos/`，发现装进去的文件叫 `ball_publisher.py`，**带 `.py` 后缀**。
   → 原因：`install(PROGRAMS scripts/ball_publisher.py DESTINATION lib/ball_pos)` 会原样保留文件名，`ros2 run` 是严格按名字找的。
   → 解决：改成 `install(PROGRAMS scripts/ball_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME ball_publisher)`，重新 `colcon build`，命令才变成 `ros2 run ball_pos ball_publisher`。

2. **共享文件夹 `/media/sf_recruit` 报 `Permission denied`**
   → 先试 `sudo mount -t vboxsf` 没用，后来才发现共享文件夹其实是挂上的，只是账号不在 `vboxsf` 组里。
   → 解决：`sudo usermod -aG vboxsf $USER` 然后 `newgrp vboxsf`，不用重启就生效了。

3. **想开两个终端左右分屏，`Super+←`、`xdotool` 全部失效**
   → `xdotool getactivewindow` 报 `XGetWindowProperty[_NET_ACTIVE_WINDOW] failed`，说明 Ubuntu 用的是 **Wayland**，X11 系的窗口工具一个都看不见窗口。
   → 解决：改用 `tmux`，`Ctrl+B` 然后 `%` 在**一个窗口里**开出左右两个终端面板，视觉上就是标准分屏，还不用折腾窗口管理器。

4. **虚拟机开机卡在 Ubuntu logo 转圈五分钟**
   → 观察画面一直在动（不是彻底死机），但就是不进系统。想起刚配的共享文件夹路径里有中文（`电控组`），VirtualBox 自动挂载遇到非 ASCII 路径容易卡住。
   → 解决：把共享文件夹改成指向纯英文路径的上一级目录，重启后 2 分钟正常进桌面。

5. **`sudo apt install` 报 `sudo: no password was provided`**
   → 不是密码错，是之前 15 分钟的免密缓存过期了。
   → 解决：老老实实手动输一次密码（Linux 输密码**屏幕上什么都不显示**，连星号都没有，这一点第一次很容易以为键盘坏了）。

---

## 五、给下周自己的提醒

**跑任何节点之前，先 `source ~/ros2_ws/install/setup.bash`，新开一个终端就要 source 一次**——
这一周我在这上面浪费的时间比写代码还多，「明明编译过了却 `ModuleNotFoundError` / `No executable found`」八成都是它。
