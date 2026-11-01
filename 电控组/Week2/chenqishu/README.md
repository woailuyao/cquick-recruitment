# 第 2 周考核 · 电控组 · chenqishu

> 三项任务：自定义 `BallPos` 消息并发布 3 个球坐标 → 订阅后让乌龟依次走到 3 个点 → 串口与总线舵机学习笔记。

## 目录

- [一、三项任务做了什么 / 卡在哪 / 怎么解决](#一三项任务做了什么--卡在哪--怎么解决)
- [二、话题 / 服务 / 动作 对比](#二话题--服务--动作-对比)
- [三、差速三公式](#三差速三公式)
- [四、代码结构](#四代码结构)
- [五、一句话给下周的自己](#五一句话给下周的自己)

---

## 一、三项任务做了什么 / 卡在哪 / 怎么解决

> 下面「卡在哪」都是真实踩过的坑。**有意思的是：ROS 的代码几乎一次就跑通了，真正耗掉我两天的是「环境」**——虚拟机、文件权限、输入法、驱动。这也是本周最大的收获。

### 任务 1 · 自定义 BallPos 消息 + 发布/订阅（40 分）

**做了什么**

- 用 `ament_cmake` 建包 `cquick_ball_msgs`（自定义消息必须走 CMake 流程，不能用 `ament_python`）；
- 定义 `msg/BallPos.msg`：`int32 id` / `float32 x` / `float32 y`；
- `package.xml` 补 `rosidl_default_generators`、`rosidl_default_runtime`、`rclpy`、`member_of_group`；
- `CMakeLists.txt` 里 `rosidl_generate_interfaces` 生成消息，`install(PROGRAMS ...)` 把两个脚本装到 `lib/cquick_ball_msgs`；
- `ball_publisher.py`：2 Hz 定时器，每个周期按 id 1→2→3 连发三条（坐标 2.0/3.0、5.5/8.0、9.0/4.0，**一个都没改**）；
- `ball_subscriber.py`：订阅 `ball_pos`，打印 `id / x / y`；
- 发布频率做成了 ROS 参数 `publish_rate`，`ros2 param set` 就能从 2 Hz 改到 5 Hz，**不用改代码、不用重编译**（加分项）。

**卡在哪 / 怎么解决**

1. **从 Windows 打包的文件解压出来，目录权限是 `000`**。`cp` 直接报「对 … 调用 stat 失败：权限不够」，一开始我以为文件损坏了。用 `ls -ld` 看才发现目录权限是坏的，`chmod -R u+rwX ~/chenqishu` 之后立刻正常。
2. **`~/ros2_ws` 目录「凭空消失」**。诡异的地方是：`ls -ld ~/ros2_ws/src` 明明能列出两个包，紧接着 `cd ~/ros2_ws` 却报「没有那个文件或目录」。
   最后解决方式是不再分步执行，改用一条 `&&` 链把「建目录 → 拷包 → 进目录 → 编译」一次走完：
   `cd ~ && mkdir -p ros2_ws/src && cp -r ... && cd ros2_ws && colcon build ...`，一次成功。这一步让我理解了「中间状态被打断」有多坑。
3. **`colcon build` 必须在工作空间根目录执行**，而且要 `--packages-select` 指明包名，否则它会扫全盘找包。
4. **打不出中文**：虚拟机里没装中文输入法，`cd ~/下载` 这种路径根本敲不出来。后来改成「下载到主目录 `~/`」，全程英文路径；实在要复制命令，就用虚拟机里的浏览器打开本地网页再复制。

### 任务 2 · 依次走到三个球坐标（35 分）

**做了什么**（走的是**路线 A 闭环**，加分项）

- 订阅 `/ball_pos`，用字典按 `id` 存坐标，**收齐 3 个才开始动**（长度到 3 才触发）；
- 订阅 `/turtle1/pose` 拿实时 `(x, y, θ)`，20 Hz 控制循环；
- 每个周期算距离 `dist` 和方位角差 `angle_err`：
  - `|angle_err| > 0.35 rad` → `linear.x = 0`，只给 `angular.z`，**原地转对准**；
  - 对准后 → `linear.x = 0.9 × dist`（封顶 0.7），`angular.z = 2.5 × angle_err`，**越近越慢**；
  - `dist < 0.12` → 发全 0 的 `Twist` 停下，用当前实际位姿复算误差、打日志、切下一个点；
- 严格按 1 → 2 → 3，全程无人工按键；最后打印三个点的误差汇总。

**实际结果**：一次跑完，约 28 秒，三个点误差 **0.115 / 0.116 / 0.117**（要求 ≤ 0.5）。

**卡在哪 / 怎么解决**

1. **控制律本身一次就跑通了**，所以这部分没花多少时间。设计时想清楚了两件事：`Twist` 是速度不是位移，所以必须「越近越慢」；差速底盘没有横向速度，所以必须「先对准再走」。
2. **真正卡住的是虚拟机反复卡死**（桌面冻住、鼠标不动），两天里发生了两次。查出来是 **Wayland + 显卡显存只有 16 MB + 没有 3D 加速**的组合。解决：
   - VirtualBox 设置里把显存从 16 MB 调到 **128 MB**；
   - 在 `/etc/gdm3/custom.conf` 里加 `WaylandEnable=false`，强制用 Xorg（`echo $XDG_SESSION_TYPE` 确认变成 `x11`）。
3. **VirtualBox 增强功能装不上**，报 `gcc: not found`，装了 gcc 之后又报 `gcc-12: not found`。原因是内核 6.8 是用 gcc-12 编译的，必须装**同版本**的编译器，`sudo apt install gcc-12` 之后才装上。这个功能装好后才有剪贴板共享，效率提升很大。
4. **截图重拍了几次**：第一次拍完发现细则要求「`colcon build` 无报错 + 双终端同跑」，我只拍了后半截；「改频率」那张拍成了话题频率而不是订阅端输出。返工两次才对。**教训：截图前先逐条读要求。**

### 任务 3 · 串口 + 总线舵机笔记（25 分）

见 [`03_串口与总线舵机笔记.md`](./03_串口与总线舵机笔记.md)。

**做了什么**：整理串口四参数、半双工/共地/电平三个硬约束、波特率不一致的采样原理，逐字拆解 `#001P1500T1000!`，并自己写出两条指令。

**卡在哪**：虚拟机里没有中文输入法，笔记只能在 Windows 上写好再贴进去。另外波特率那段一开始只想到「速度不同所以乱码」，后来才补上真正的机制——串口没有时钟线、起始位只同步一次、误差逐位累积超过半个位宽就采到隔壁位。

**状态**：本周**没做回环测试**（没有 CH340、也没去实验室领舵机），笔记里写明了计划下周拿到舵机后补做 + 预期现象。不扣分，下周补。

---

## 二、话题 / 服务 / 动作 对比

| 通信方式 | 模型 | 特点 | 足球机器人典型场景 |
|---|---|---|---|
| **话题 Topic** | 发布/订阅，多对多 | 单向、持续流、不管对方在不在 | 视觉持续广播球位置；战术持续广播速度指令 |
| **服务 Service** | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切到防守模式」 |
| **动作 Action** | 目标/反馈/结果 | 长任务、报进度、可取消 | 「走到点 (1,1)」→ 过程报进度，可中途取消 |

**任务 1 用的是话题**：发布者一直发，不管有没有人听；订阅者后启动也照样能接上（只收新的）。两个节点靠 **话题名 + 消息类型**一致对上号。

**任务 2 里我实际用到的服务调用：**

```bash
ros2 service call /clear std_srvs/srv/Empty
```

这是 turtlesim 提供的清屏服务，开跑前清掉画布，轨迹才干净。
它和话题最直观的差别就在这条命令上：**它必须有回应**（返回一个空的 `Empty_Response`），**且只调一次就结束**；而话题是「发出去就不管了，没有回执」，两边根本不需要同时在线。

---

## 三、差速三公式

设左右轮速 `vL`、`vR`，轮距（两轮中心距）`L`：

| 公式 | 含义 |
|---|---|
| `v = (vL + vR) / 2` | 两轮**平均**速度 = 车体中心前进的线速度 |
| `ω = (vR − vL) / L` | 左右轮速**差** ÷ 轮距 = 转向角速度；右轮比左轮快则 `ω > 0`，向左转 |
| `r = v / ω` | 转弯半径。`ω → 0` 时 `r → ∞`，也就是走直线；`ω` 越大弯越急 |

**反向求解**（运动控制节点真正要干的事）——已知想要的 `v`、`ω`，求两轮转速：

```
vR = v + ω·L/2
vL = v − ω·L/2
```

这就是「战术节点发 Twist → 运动控制节点换算成左右轮转速 → 串口发给下位机」那一环。

**对应到 `Twist` 的用法：**

| 想要 | 怎么给 |
|---|---|
| 直行 | `linear.x = v`，`angular.z = 0` |
| 原地转 | `linear.x = 0`，`angular.z ≠ 0` |
| 边走边转（画弧） | 两者都非 0 |

---

## 四、代码结构

```
src/
├── cquick_ball_msgs/              任务1：自定义消息 + 发布/订阅
│   ├── msg/BallPos.msg            int32 id / float32 x / float32 y
│   ├── scripts/ball_publisher.py  2 Hz 连发 3 个球，频率可用 ros2 param 调
│   ├── scripts/ball_subscriber.py 订阅 ball_pos 并打印
│   ├── package.xml
│   └── CMakeLists.txt
└── cquick_turtle_nav/             任务2：闭环导航
    ├── cquick_turtle_nav/ball_chaser.py
    ├── resource/cquick_turtle_nav
    ├── setup.py / setup.cfg
    └── package.xml
```

**复现命令：**

```bash
cd ~/ros2_ws
colcon build --packages-select cquick_ball_msgs cquick_turtle_nav
source install/setup.bash

# 任务1
ros2 run cquick_ball_msgs ball_publisher.py
ros2 run cquick_ball_msgs ball_subscriber.py

# 任务2（三个终端）
ros2 run turtlesim turtlesim_node
ros2 run cquick_ball_msgs ball_publisher.py
ros2 run cquick_turtle_nav ball_chaser
```

---

## 五、一句话给下周的自己

> **先把「source 了没」和「话题名对不对」查一遍再怀疑代码**——这周 80% 的「跑不起来」都是这两件事，剩下 20% 才是真的逻辑问题。
