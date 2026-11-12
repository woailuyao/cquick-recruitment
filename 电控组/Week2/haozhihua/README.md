# 第 2 周提交说明 · 郝志华

## 一、基本信息

- 姓名拼音：haozhihua
- 姓名：郝志华
- 完成时间：2026-10-30（整理提交）
- 运行环境：Ubuntu 22.04 + ROS2 Humble

## 二、三项任务完成情况

### 任务 1 · `BallPos` + 发布/订阅

- 包名：`ball_comm`（`ament_cmake` 类型，自定义消息必须走 CMake）
- 话题名：`ball_pos`
- 发布频率：默认 2 Hz，可通过 `ros2 param` 调到 5 Hz，不用改代码
- 消息字段：`int32 id` / `float32 x` / `float32 y`
- 做了什么：定时器每 0.5 s 触发一次 `tick()`，每次发 1 条，三个球按 id 1→2→3 轮流发送（`self.i % 3`），1.5 s 轮完一轮，坐标用细则 1.2 的固定值。订阅端在回调里把 `id` 存进字典按 id 去重，打印「收到球 N: (x.xx, y.yy) 已收 N/3」。
- 验证结果：
  - `ros2 interface show ball_comm/msg/BallPos` → 输出 `int32 id` / `float32 x` / `float32 y`
  - `ros2 topic info /ball_pos` → `Publisher count: 1` / `Subscription count: 1`

改参数的观察

```bash
ros2 run ball_comm talker_ball --ros-args -p rate_hz:=5.0
```

- 改法：`rate_hz` 在节点里用 `declare_parameter('rate_hz', 2.0)` 声明，定时器周期由 `1.0 / hz` 算得频率
- 观察：订阅端刷屏速度明显加快，发布端日志同样加快
- 这一项是加分项 +3（用 ros2 param 改频率，不用改代码），证据见 `01_频率参数.png`

### 任务 2 · 三个球的目标点导航

- 实现路线：闭环（订阅 `/turtle1/pose` 拿实时位姿，每 20 Hz 重算一次的）
- 到达情况：

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.13, 3.06) | 0.144 |
| 2 | (5.5, 8.0) | (5.39, 7.90) | 0.147 |
| 3 | (9.0, 4.0) | (8.93, 4.12) | 0.145 |

我设的DIST_TOL = 0.15，所以误差都在0.14几。

- 关键参数：

| 参数 | 值 | 含义 |
|---|---|---|
| `MAX_LIN` | 1.2 | 线速度上限（m/s） |
| `MAX_ANG` | 2.0 | 角速度上限（rad/s） |
| `K_LIN` | 1.0 | 线速度比例系数 |
| `K_ANG` | 2.5 | 角速度比例系数 |
| `DIST_TOL` | 0.15 | 到点判定阈值 |
| `ANG_TOL` | 0.06 | 对准的角度阈值 |
| 控制频率 | 20 Hz | 定时器 0.05 s |

- 为什么这样调：

  1. 速度必须跟距离相关，越进越慢：`linear.x = K_LIN * dist`。如果给固定速度，可能会折返那样抖动
  2. 要正负分别限幅： 加限幅`max(-MAX_ANG, min(MAX_ANG, ...))`，而且考虑正负的绝对值什么的 
  3. 先对准再走：`|err| > ANG_TOL` 时 `linear.x = 0` 原地转，对准了才走。

- 闭环 + 日志打印每个点的实际误差，见 `02_运行日志.png`，这一项是加分项 +5

### 任务 3 · 串口与总线舵机（学习笔记，25 分）

- 详细笔记见：`03_串口与总线舵机笔记.md`

- 回环测试：未做，计划下周拿到舵机后统一补做。
- 我还没搞懂的问题：指令的第三项目标位置，AI说和PWM有关，是每50Hz里的发动us数，类似PWM，但是感觉有点抽象，是怎么和位置扯上关系的？我再问AI研究一下

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人典型场景 |
|---|---|---|---|
| **话题 Topic** | 发布 / 订阅，多对多 | 单向、持续流；发发布者不需要回应，订阅者也不回应 | 视觉可能会发布 3 个球的坐标；战术上发布导航指令 |
| **服务 Service** | 请求 / 响应，一对一 | 同步，发出去要等一个应答，一次请求对应一次应答 | 现在谁需要弄什么战术、评估后觉得能不能行、如果不行怎么办，要有回复 |
| **动作 Action** | 目标 / 反馈 / 结果 | 持续进行，过程中有进度监测，还能中途取消 | 比如要走到点 (1,1)→ 一还剩多远，中途发现球动了可以取消 |

实际用到的服务调用：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

话题与服务的区分：清屏是一次性，有应答。如果是话题，就得持续不断地发「清屏」指令，会一直重复清屏，而实际上清一次就够了。反过来，`/ball_pos`是话题，因为球的位置需要一直更新，而且可能有多个节点同时在听，发布者不需要收到回复，而是有限保证信息稳定发出

| 词 | 是什么 | 本周对应 |
|---|---|---|
| **节点 Node** | 一个跑起来的进程，只干一件事 | `ball_talker`（只管发）、`ball_listener`（只管收）、`ball_nav`（只管导航） |
| **话题 Topic** | 一条有名字的数据通道 | `ball_pos`（发布者和订阅者都往这个名字上挂） |
| **消息 Message** | 跑在话题上的数据格式，规定有几个字段、什么类型 | `ball_comm/msg/BallPos`，`int32 id` + `float32 x` + `float32 y` |
| **发布者 Publisher** | 写数据的一方 | `talker_ball.py` 里的 `self.pub` |
| **订阅者 Subscriber** | 读数据的一方 | `listener_ball.py` / `ball_nav.py` 里的 `create_subscription` |


## 四、差速三公式

- **`v = (vL + vR) / 2`** —— 车整体往前跑多快，看两个轮子的平均速度。两轮同速（vL = vR）时是纯直行；一轮正转一轮反转（vR = −vL）时平均为 0，车原地不前进只打转。线速度只反映平移，不反映转向。
- **`ω = (vR − vL) / L`** —— 车转得多快，看速度差，再除以轮距 `L` 归一化。右轮比左轮快就是向左（逆时针）转，所以 ROS 里 `angular.z` 正值 = 左转。`L` 越大，同速度差转得越慢。
- **`r = v / ω`** —— 转弯半径。`v` 固定时 `ω` 越小弯越大；`ω → 0` 时 `r → ∞`，就是直线。若 `v = 0` 而 `ω ≠ 0` 时 `r = 0`，就自转（任务2的先瞄准）

## 五、踩过的坑和解决方案

**1. `colcon build` 报 `file INSTALL cannot find .../scripts/talker_ball.py`**

现象：CMakeLists 里明明写了 `install(PROGRAMS scripts/talker_ball.py ...)`，一编译就报找不到这个文件。
怎么查：CMake 在配置阶段就要检查 `install()` 里列的文件在不在，不是等到安装才查。我当时先改配置文件、后写代码，文件还不存在
怎么解决：先把三个 `.py` 用 `touch` 建成空文件占位（顺便 `chmod +x`），再往里写代码。

**2. 编译过了，`ros2 run` 却说 `No executable found`**

现象：`colcon build` 全绿，`ros2 run ball_comm talker_ball` 报找不到可执行文件。
怎么查：`ls install/ball_comm/lib/ball_comm/` 一看，装进去的叫 `talker_ball.py`（带那个py后缀），而 `ros2 run` 找的是不带后缀的 `talker_ball`。
怎么解决：`install(PROGRAMS)` 里加 `RENAME talker_ball`，让安装后的名字去掉 `.py`。加完还得 `rm -rf build install` 增量编译不会更新安装目录里已有的旧文件，改了配置也看不见效果。

**3. 加 `--symlink-install` 后报 `Is a directory`**

原因：之前用普通 `colcon build` 生成的是真实文件，symlink 模式想在同样位置建符号链接，冲突了
怎么解决：`rm -rf build install log` 清掉旧的编译产物，再用 symlink 模式重来。两种模式的产物结构不兼容，不能混着用

**4. 把频率改成 5 Hz，输出还是每秒 2 行**

现象：`ros2 run ball_comm talker_ball --ros-args -p rate_hz:=5.0`无效，还是每 0.5 s 一组。
怎么查：`ros2 param list /ball_talker` 只列出 `use_sim_time` 一项，说明代码里的 `declare_parameter` 根本不存在于正在运行的那份文件里。不是改好的那个
根因：改完 `.py` 没重新 `colcon build`，`ros2 run` 跑的是 `install/` 里的旧副本。
怎么解决：保存 → `colcon build --packages-select ball_comm` → `source install/setup.bash` 。改了 `.py` 也必须重编。

**5. `TypeError: '<' not supported between instances of 'dict' and 'int'`**

现象：导航节点一启动就崩，，，提示self.balls不能和后面的作比较
原因：我想判断「三个球收齐了没有」，写成了 `if self.balls < len(ORDER)`——`self.balls` 是字典，**字典没有大小这个序关系**，不能和数字比。
怎么解决：`if len(self.balls) < len(ORDER)`，气笑了

**6. `AssertionError: The 'x' field must be of type 'float'`**

现象：节点跑起来了、球也收齐了，一到发速度就崩，traceback 指向 `cmd.linear.x = 0`。
怎么查：ROS2 的 Python 消息类在字段 setter 里写死了 `assert isinstance(value, float)`。`0` 是 int，C++ 里 `double x = 0;` 会自动转换，Python 没有隐式转换。只学Cpp导致的。
怎么解决：写成 `0.0`。又气笑了

## 六、给下周自己的提醒

> 改完代码先 `colcon build` 再 `source`，别直接开跑

## 七、源码结构

```
src/
└── ball_comm/
    ├── package.xml          # 三行硬依赖：rosidl_default_generators / rosidl_default_runtime / member_of_group
    ├── CMakeLists.txt       # rosidl_generate_interfaces + 三段 install(PROGRAMS ... RENAME ...)
    ├── msg/
    │   └── BallPos.msg      # int32 id / float32 x / float32 y
    └── scripts/
        ├── talker_ball.py
        ├── listener_ball.py # 任务 1 订阅者（字典按 id 去重，打印已收 N/3）
        └── ball_nav.py      # 任务 2 导航节点（闭环，20 Hz，依次 1→2→3）
```



## 八、提交前自查

- [ ] 上传目录里**没有** `build/`、`install/`、`log/`
- [ ] `src/` 下只有 `ball_comm` 一个包，没有 `.pyc`、`__pycache__`
