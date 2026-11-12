# 第 2 周提交说明 · 游梁思远

## 一、基本信息

- 姓名拼音：youliangsiyuan
- 姓名：游梁思远
- 完成时间：2026-11-01
- 运行环境：Ubuntu 22.04 + ROS2 Humble（VirtualBox 虚拟机）
- 提交目录：`电控组/Week2/youliangsiyuan/`

---

## 二、三项任务完成情况

### 任务 1 · `BallPos` 自定义消息 + 发布/订阅

- 包名：`cquick_ball`（`ament_cmake`）
- 消息定义 `msg/BallPos.msg`：

  ```
  int32 id
  float32 x
  float32 y
  ```

- 话题名：`ball_pos`
- 发布频率：2 Hz（实测两条日志时间戳相差约 0.50 s）

**做了什么**

`scripts/ball_pub.py` 里用一个 2 Hz 的定时器，每个周期把 `BALLS` 表里的三个球依次发一遍（id 1/2/3，坐标写死），每发一条打一行日志。
`scripts/ball_sub.py` 订阅同一个话题，回调里把 `id / x / y` 打出来。两个节点用 `ros2 run` 分别启动。

**验证结果**

- `ros2 interface show cquick_ball/msg/BallPos` 输出三行字段（见 `01_interface_show.png`）
- `ros2 topic info /ball_pos` → `Publisher count: 1` / `Subscription count: 1`（见 `01_话题信息.png`）
- 两终端同时运行，订阅端每 0.5 s 收到 3 条（见 `01_发布订阅分屏.png`）
- 改频率后订阅端明显变快（见 `01_改频率对比.png`：2 Hz 时间隔约 0.53 s，5 Hz 时间隔约 0.20 s）

**改频率这件事我是怎么做的**

一开始想用 `ros2 param set`，但参数回调要依赖 `rcl_interfaces`，我在这上面卡了一会儿（见第五节第 6 条），为了不打断主线，最后是**直接改代码里的 `self.freq`、重新 `colcon build` 再跑**。所以这一项我**没做 `ros2 param` 加分**，只做"改一处、说得清现象"。

### 任务 2 · 三个球的目标点导航（闭环）

- 实现路线：**闭环**。订阅 `ball_pos` 拿目标，订阅 `/turtle1/pose` 拿自己的 (x, y, θ)，发布 `/turtle1/cmd_vel` 控制乌龟。
- 脚本位置：`src/cquick_ball/scripts/ball_nav.py`（和发布、订阅脚本放在同一个包的 `scripts/` 目录里，`ros2 run cquick_ball ball_nav.py` 直接跑）
- 控制频率：20 Hz
- 顺序：`TARGETS = [1, 2, 3]`，写死
- 到点后打印实际坐标和误差，三点跑完打印误差汇总

**到达情况**

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.12, 3.08) | 0.15 |
| 2 | (5.5, 8.0) | (5.41, 7.88) | 0.15 |
| 3 | (9.0, 4.0) | (8.90, 4.11) | 0.15 |

全程无人工按键，一次跑完。截图：`02_三个球轨迹.png`、`02_运行日志.png`、`02_节点列表.png`、`02_误差汇总.png`。

**关键参数**

```python
MAX_LINEAR = 0.5      # 最大线速度
MAX_ANGULAR = 1.5     # 最大角速度
ANGLE_TOL = 0.5       # 方位差小于它才允许前进
SLOW_DIST = 0.8       # 距离小于它开始减速
ARRIVE_DIST = 0.15    # 距离小于它认为到点
CTRL_HZ = 20.0        # 控制频率
```

**为什么这样调**

1. 第一次跑的时候 `ARRIVE_DIST = 0.3`，三个点的误差**都正好是 0.30**——因为判据是"距离 ≤ 0.3 就算到"，一进圈就停，误差全卡在阈值上。虽然合格，但我想留余量（怕评审复跑偏一点就超 0.5），于是改成 0.15，重跑后三个误差**都变成 0.15**。
2. 线速度一开始就定得比较小（0.5），先保证"能稳稳到点"；配合 `SLOW_DIST = 0.8` 的减速，实测三个点都没有冲过头、也没有来回震荡，误差都是 0.15，所以就没再往上加。
3. `ANGLE_TOL = 0.5` 是"方位差超过约 28° 就先原地转"的门槛。我理解它调小会让乌龟走走停停、调大则会一开始就斜着往前冲、走弧线——这一条我还没实测过。

**我理解的三条控制思路**

- 先算 `dist = hypot(dx, dy)`（直线距离）和 `heading_err = wrap(atan2(dy,dx) − θ)`（目标方向和我朝向差多少）；
- 方位差大就先原地转（`linear.x = 0`，`angular.z` 按比例给），对上了再往前走；
- 前进时速度跟距离挂钩（`0.6 * dist`，限幅到 `[0.05, MAX_LINEAR]`），进入 `SLOW_DIST` 后再乘减速系数，做到"越近越慢"。

### 任务 3 · 串口基础与总线舵机（学习笔记）

- 详细笔记见：`03_串口与总线舵机笔记.md`
- 回环测试：**已做**。自己买了 CH340 模块，把 TXD 和 RXD 用杜邦线短接，在 Ubuntu 里用 Python + pyserial 发 16 字节 `#001P1500T1000!`（115200，8-N-1），收到的内容和发出去的完全一致，程序判断 `一致: True`。截图见 `03_回环测试.png`。
- 做的时候还顺带发现两件事：① Ubuntu 的 brltty 服务会抢占 CH340，导致 `/dev/ttyUSB0` 一插上就消失；② `import serial` 需要装 `python3-serial`。两条都写进笔记第二节了。
- 我自己想明白的一个点：**单口回环时改波特率仍然能通**，因为收发用的是同一个串口设置；要造出"波特率不一致"，必须有两个独立的串口。

---

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人场景 |
|---|---|---|---|
| 话题 Topic | 发布/订阅，多对多 | 单向、持续流、不管对方在不在（发了没人听也不报错） | 视觉持续广播球位置；战术持续广播速度指令 |
| 服务 Service | 请求/响应，一对一 | 同步，调一次答一次，必须等结果 | 「现在比分多少」「清空画布」「切到防守模式」 |
| 动作 Action | 目标/反馈/结果 | 长任务、过程中报进度、可以中途取消 | 「走到点 (1,1)」→ 边走边报进度，走歪了能取消 |

**我实际用到的服务调用**：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

任务 2 开始前用它清屏，这样乌龟留下的轨迹才是这一次的三段线。**它必须是服务而不是话题**：我只想让它执行一次并确认执行完了，发一次请求、收到一次回应就结束，不需要持续广播。

> 我的两个节点（`ball_pub` / `ball_nav`）之间用的是**话题**：发布端一直在发，不管有没有人听；订阅端起晚了也能马上接上——这就是话题"多对多、单向、持续"的特点。

---

## 四、差速三公式

设左右轮线速度 `vL`、`vR`，轮距 `L`：

- `v = (vL + vR) / 2`：车体前进速度是两轮的平均。两轮同速就是直行。
- `ω = (vR − vL) / L`：两轮速度差越大转得越快。右轮比左轮快（差为正）就左转，这就是 `angular.z` 为正表示左转的原因。
- `r = v / ω`：转弯半径。ω 越小半径越大；ω = 0 时半径无穷大，就是走直线。

写进 `Twist` 就是：`linear.x` 对应 `v`，`angular.z` 对应 `ω`。

---

## 五、踩过的坑 & 怎么解决的

**1. `package.xml` 里漏了 `member_of_group`，编译报 rosidl 错**

- 现象：`Packages installing interfaces must include '<member_of_group>rosidl_interface_packages</member_of_group>' in their package.xml`，报错位置却指向 `CMakeLists.txt:7`，看着像 CMake 的问题。
- 怎么查：`grep -n member_of_group package.xml`，没有输出——根本没写。
- 解决：`ros2 pkg create` 生成的 `package.xml` 不会自带接口相关标签，要自己补 `<build_depend>rosidl_default_generators</build_depend>`、`<exec_depend>rosidl_default_runtime</exec_depend>` 和 `<member_of_group>rosidl_interface_packages</member_of_group>`，而且必须直接写在 `<package>` 下，不能塞进 `<export>`。

**2. 用 `cat > package.xml <<'EOF'` 粘贴，文件头坏了**

- 现象：`xml.parsers.expat.ExpatError: syntax error: line 1, column 0`——第 1 行第 0 列就错，说明文件第一个字符不是 `<`。
- 怎么查：`head -3 package.xml | cat -A`，能看到行首有不可见字符（BOM `M-oM-;M-?` 或多出来的内容）。
- 解决：删掉重建，用 `nano` 手打一份最小版，保存后先验证 `python3 -c "import xml.dom.minidom as m; m.parse('package.xml'); print('XML OK')"`，再编译。

**3. 把 shebang 当注释删了，`ros2 run` 报 `Exec format error`**

- 现象：`OSError: [Errno 8] Exec format error: '.../ball_pub.py'`。
- 原因：`#!/usr/bin/env python3` 对 Python 是注释，但对内核是"用哪个解释器执行我"的指令，必须在**第 1 行第 1 个字符**。`ros2 run` 是直接 exec 这个文件，所以不能少。
- 解决：`sed -i '1i #!/usr/bin/env python3' scripts/ball_pub.py`，`head -1` 确认后重新 `colcon build`。

**4. 新脚本没登记进 `CMakeLists.txt`，`ros2 run` 报 `No executable found`（踩了两次）**

- 现象：`ros2 run cquick_ball ball_sub.py` → `No executable found`，但文件明明在 `scripts/` 下。
- 怎么查：`ros2 pkg executables cquick_ball` 只列已安装的，缺的就是没装；`ls ~/ros2_ws/install/cquick_ball/lib/cquick_ball/` 里也看不到。
- 原因：`install(PROGRAMS ...)` 是白名单，没写进去的文件不会被复制到 `install/`。
- 解决：在 `install(PROGRAMS)` 里补一行文件名，然后**重新 `colcon build`**。

**5. 改了 `.py` 没重新 `colcon build`，现象是"改了没效果"（踩了三次）**

- 现象：把 `ARRIVE_DIST` 从 0.3 改成 0.15，重跑后三个误差还是 **0.30**，坐标也和上一次一模一样。
- 原因：`ros2 run` 跑的是 `install/` 里的**副本**，不是 `src/` 里的源文件；只有 `colcon build` 才会复制一次。
- 怎么查：

  ```bash
  grep -n "ARRIVE_DIST" ~/ros2_ws/src/cquick_ball/scripts/ball_nav.py
  grep -n "ARRIVE_DIST" ~/ros2_ws/install/cquick_ball/lib/cquick_ball/ball_nav.py
  ```

  两边不一致就是忘了 build（`diff` 一下更快）。
- 解决：固定成「改代码 → `colcon build` → 再跑」。

**6. `Ctrl+C` 退出时报 `rcl_shutdown already called`**

- 现象：`rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the given context`，退出时打一大段 Traceback。
- 原因：按 Ctrl+C 时 rclpy 的信号处理器已经关掉一次 context，我在 `finally` 里又调了一次 `rclpy.shutdown()`。
- 解决：

  ```python
  finally:
      node.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()
  ```

**7. 新终端里 `ros2 run` 报 `Package 'cquick_ball' not found`**

- 现象：重启虚拟机后，同样的命令报找不到包，`cd` 到哪个目录都没用。
- 原因：`colcon build` 只是装到 `~/ros2_ws/install/`，要让 `ros2` 认识它得 source 一次；环境变量只对当前终端有效，新终端/重启后是空的。
- 解决：把两行写进 `~/.bashrc`，以后每个终端自动加载：

  ```bash
  source /opt/ros/humble/setup.bash
  if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi
  ```

  顺序不能反：先 ROS 本体，再自己的工作空间。

**8. `colcon build --packages-select` 报 `unrecognized arguments`**

- 现象：`colcon: error: unrecognized arguments: --packages-select cquick_ball`。
- 原因：这个参数来自 `colcon-package-selection` 扩展，本机装的是精简版 colcon。
- 解决：`sudo apt install -y python3-colcon-common-extensions`；只编译一个包时直接 `colcon build` 也能过。

**9. CH340 插上后 `/dev/ttyUSB0` 一出现就消失**

- 现象：`ls /dev/ttyUSB*` 说没有这个文件；`sudo dmesg | tail` 里看到 `ch341-uart converter now attached to ttyUSB0`，紧接着 `ch341-uart ttyUSB0: ... now disconnected`，中间还有一行提到 `brltty`。
- 原因：Ubuntu 自带盲文终端服务 `brltty`，它会把 USB 串口设备当成盲文显示器抢过去，抢不到就把设备释放掉。
- 解决：

  ```bash
  sudo systemctl stop brltty-udev.service
  sudo systemctl mask brltty-udev.service
  ```

  然后拔插一次 USB，`/dev/ttyUSB0` 就能稳定出现。

**10. `import serial` 报 `ModuleNotFoundError`**

- 原因：Python 标准库没有串口模块，需要额外的 `pyserial`。
- 解决：`sudo apt install -y python3-serial`（用 apt 装到系统 python，避免和 pip 装的版本打架）。

---

## 六、给下周自己的提醒

1. **改完代码先 build 再跑**，别急着看结果——这一周它坑了我三次。
2. **新终端先确认 `printenv ROS_DISTRO` 有输出**，没有就是没 source。
3. 写配置时**用编辑器手打**，别把多行内容直接粘进终端，`package.xml` 那次就是粘坏的。
4. 遇到 `No executable found`，先看 `ros2 pkg executables`，再看 `install/` 目录，最后才怀疑代码。
5. 下周拿到舵机，第一件事是读 ID（`#001PID!`）和上力（`#001PULR!`），然后再写角度；回环测试这周已经做通了，下周直接接舵机。
