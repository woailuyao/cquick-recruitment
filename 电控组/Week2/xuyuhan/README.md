# 第 2 周考核 · xuyuhan

- 姓名拼音：xuyuhan
- 完成日期：2026-10-31
- 运行环境：Ubuntu 22.04.5 LTS / ROS2 Humble / VMware Workstation 虚拟机
- 工作空间：`~/ros2_ws`

## 交付物

| 文件 | 内容 |
| --- | --- |
| `01_interface_show.png` | `ros2 interface show ball_pos_pkg/msg/BallPos` 输出 |
| `01_发布订阅分屏.png` | 发布端与订阅端同时运行 |
| `01_话题信息.png` | `ros2 topic info /ball_pos`（Publisher 1 / Subscription 1） |
| `01_改频率.png` | `ros2 param set` 改变发布频率后的输出变化 |
| `02_三个球轨迹.png` | 乌龟依次走完三个坐标的轨迹 |
| `02_运行日志.png` | 到达球 1/2/3 的日志与误差 |
| `03_串口与总线舵机笔记.md` | 任务 3 学习笔记 |
| `src/ball_pos_pkg/` | 任务 1 的消息定义与发布/订阅节点、任务 2 的导航节点 |

所有截图在截取前都在终端执行了：

```bash
echo "姓名: xuyuhan"; date "+%Y-%m-%d %H:%M:%S"
```

因此每张图都带有姓名与实时时间，可自证为本人操作。

---

## 任务 1：自定义消息 BallPos + 发布/订阅

### 1.1 做了什么

1. 用 `ament_cmake` 建包 `ball_pos_pkg`，并创建 `msg/`、`scripts/` 目录；
2. 定义消息 `msg/BallPos.msg`，三个字段：

   ```
   int32 id
   float32 x
   float32 y
   ```

3. 写发布者 `ball_publisher.py`：以 2 Hz 定时触发，**每个周期把 3 个球依次发一遍**
   （坐标固定为 `(1, 2.0, 3.0)`、`(2, 5.5, 8.0)`、`(3, 9.0, 4.0)`），并打印自己发出的内容；
4. 写订阅者 `ball_subscriber.py`：订阅 `ball_pos`，回调里打印 `id / x / y`，并用字典按 `id`
   记录已收到的球；
5. 编译后用 `ros2 interface show` 验证消息类型，用 `ros2 topic info` 验证一发布一订阅。

### 1.2 关键配置

`package.xml` 中除了依赖，必须声明本包属于接口包分组：

```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```

`CMakeLists.txt` 中需要三处：

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPos.msg"
)

install(PROGRAMS scripts/ball_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_publisher      # 不加 RENAME 会装成 ball_publisher.py，ros2 run 找不到
)

ament_export_dependencies(rosidl_default_runtime)
```

### 1.3 遇到的问题与解决

| 问题 | 原因 | 解决 |
| --- | --- | --- |
| `BallPos.msg` 只有两行、最后一行没有换行 | 编辑器保存时漏了最后一行 | 用 `printf 'int32 id\nfloat32 x\nfloat32 y\n' > msg/BallPos.msg` 重写，再用 `cat -A` 确认每行以 `$` 结尾 |
| `ros2 run ball_pos_pkg ball_publisher` 报 `No executable found` | `install(PROGRAMS)` 按源文件名安装，可执行文件实际叫 `ball_publisher.py` | 用 `RENAME ball_publisher` 改名；也可先用带 `.py` 的名字验证逻辑 |
| 改了 `CMakeLists.txt` 但 `colcon build` 1.8 秒就“完成”，行为没变 | 旧构建目录没有重新配置 | `rm -rf build/ball_pos_pkg install/ball_pos_pkg` 后重新 build，再 `source install/setup.bash` |
| 编译过了，运行却 `ModuleNotFoundError: ball_pos_pkg` | 当前终端没 source 工作空间 | `source ~/ros2_ws/install/setup.bash`，并在**同一个终端**里运行 |
| `ros2 interface show` 找不到包 | 同上 | 同上 |

### 1.4 三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人里的典型场景 |
| --- | --- | --- | --- |
| **话题 Topic** | 发布/订阅，多对多 | 单向、持续流、发送方不关心有没有人接收、不等待回应 | 视觉持续广播球的位置；战术持续广播速度指令。本任务就是话题 |
| **服务 Service** | 请求/响应，一对一 | 同步、有返回值、调一次答一次，调用方会阻塞等待结果 | 「清空场地」「切换防守模式」「查询当前比分」 |
| **动作 Action** | 目标/反馈/结果 | 长任务、过程中持续反馈进度、可以中途取消 | 「走到点 (1,1)」——过程中报进度，走一半可以取消 |

**我在任务 2 里实际用到的服务调用**：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

这条命令清空乌龟画布。它和话题的差别很直观：**它必须有回应，而且调用一次就结束**；
而 `ball_pos` 话题是一直在发，没人听也照发。

---

## 任务 2：订阅球坐标，让乌龟依次走过去

### 2.1 实现思路（闭环跟随）

节点 `ball_nav` 同时订阅两条话题：`ball_pos`（球在哪）和 `/turtle1/pose`（我在哪），
再向 `/turtle1/cmd_vel` 发布速度指令。控制循环 20 Hz：

1. 用字典按 `id` 收集坐标，**收齐 3 个球才开始动**（否则只等待）；
2. 对当前目标算 `dx = tx - x`、`dy = ty - y`，得到距离 `dist` 和方位角 `atan2(dy, dx)`；
3. 角度差 `err` 先折算到 `[-π, π]`（不折算会多转一整圈）；
4. `|err|` 大于对准阈值 → **原地转**（`linear.x = 0`，`angular.z = k·err`）；
5. 对准之后 → **边走边微调**，线速度取 `min(max_linear, 1.2·dist)`，**越近越慢**；
6. `dist < dist_tol` 判定到点，打印实际坐标与误差，切下一个球；全部走完就发零速度停车。

**为什么“越近越慢”**：`Twist` 发布的是**速度**不是位移。速度恒定就会在 20 Hz 的间隔里冲出阈值，
表现为到点附近来回振荡。

### 2.2 实测结果

三个球按 1 → 2 → 3 依次到达，全程没有人工按键：

| 球 | 目标坐标 | 实际到达 | 误差 |
| --- | --- | --- | --- |
| 1 | (2.00, 3.00) | (2.24, 3.16) | **0.289** |
| 2 | (5.50, 8.00) | (5.33, 7.77) | **0.284** |
| 3 | (9.00, 4.00) | (8.81, 4.21) | **0.286** |

三点误差都小于 0.5，满足判定要求。到点阈值 `dist_tol` 设为 0.30，所以误差都落在这个量级；
想更准可以把它调到 0.20。

### 2.3 遇到的问题

| 问题 | 原因 | 解决 |
| --- | --- | --- |
| 按 `Ctrl+C` 退出时报 `RCLError: failed to shutdown: rcl_shutdown already called` | rclpy 的 SIGINT 处理器已经关闭了上下文，`finally` 里又调了一次 `rclpy.shutdown()` | 改成 `if rclpy.ok(): rclpy.shutdown()` |
| 改完代码启动报 `IndentationError: unindent does not match any outer indentation level` | 编辑时 Tab 与空格混用 | VSCode 执行 `Convert Indentation to Spaces`；编译前先用 `python3 -m py_compile` 查语法 |
| 一开始找不到 `ball_nav` 可执行文件 | 只 `install(PROGRAMS)` 了前两个脚本，没加第三个 | 补 `install(PROGRAMS scripts/ball_nav.py ... RENAME ball_nav)` 后重新 build |

---

## 任务 3：串口与总线舵机

本周不发舵机，这一项只交学习笔记，正文见 `03_串口与总线舵机笔记.md`，包含：

1. 串口四参数（波特率 / 数据位 / 停止位 / 校验位）与 8-N-1；
2. **回环测试**：CH340 模块把 TX 与 RX 短接，用串口助手（COM9、9600、8-N-1）发送
   `xuyuhan`、`123456`、`徐宇涵`，接收区原样返回；并记录了这条测试的边界——它只能证明
   链路和驱动正常，**证明不了波特率设置正确**，因为收发是同一个设备；
3. 波特率不一致导致采样点漂移、位边界读错的原理；
4. `#001P1500T1000!` 的七段逐字拆解，以及自己推导的两条指令
   （`#001P1500T1500!`、`#003PULK!`）；
5. 一个自己还没搞懂的问题：同一根总线上两个舵机同时回数据会怎样，协议靠什么避免。

依据互斥，笔记里没有整段复制手册原文，都是自己的话。

---

## 差速三公式

乌龟是两轮差速底盘，左右轮速 `vL`、`vR`，轮距 `L`：

- **线速度**：`v = (vL + vR) / 2` —— 两轮平均，决定前进多快；
- **角速度**：`ω = (vR − vL) / L` —— 两轮差速，决定转多快；
- **转弯半径**：`r = v / ω` —— ω 越小半径越大，ω = 0 走直线。

对应到 `Twist`：

| 想要 | 该给什么 |
| --- | --- |
| 直行 | `linear.x = v`，`angular.z = 0` |
| 原地转 | `linear.x = 0`，`angular.z ≠ 0` |
| 边走边转（画弧） | 两者都非 0 |

任务 2 的控制律正是基于这组关系：转弯阶段只用 `angular.z`，对准后用 `linear.x` 逼近，
再用小幅 `angular.z` 修正偏差。

---

## 给下周自己的提醒

**先 PULR 再写角度，结束一定 PULK。** 顺序错了，要么舵机纹丝不动让你怀疑人生，
要么它一直顶着力发热把它烧了。
