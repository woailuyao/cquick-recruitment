# 第 2 周考核 · 徐靖轩（xujingxuan）

- 姓名拼音：xujingxuan
- GitHub：EricXu-jingxuan
- 完成日期：2026-10-30 ~ 2026-10-31
- 环境：Ubuntu 22.04.5 LTS（VMware 虚拟机）+ ROS2 Humble

## 目录说明

```
Week2/xujingxuan/
├── 01_interface_show.png     任务1：消息字段
├── 01_话题信息.png           任务1：topic info（Pub 1 / Sub 1）
├── 01_发布订阅分屏.png       任务1：两终端同时运行
├── 01_改频率对比.png         任务1：改频率后速率变化
├── 02_三个球轨迹.png         任务2：三球轨迹
├── 02_运行日志.png           任务2：三点到达日志
├── 02_节点列表.png           任务2：node list
├── 02_误差汇总.png           任务2：三点误差
├── README.md
└── src/ball_pos_pkg/         源码（package.xml / CMakeLists.txt / msg / scripts）
```

---

## 任务一：自定义 BallPos 消息 + 发布 3 个球坐标（40 分）

### 做了什么

`ball_pos_pkg`（`ament_cmake`），消息 `BallPos` 三个字段，发布者 2 Hz 定时器每周期连发 3 条，
订阅者收到打印。

| 文件 | 作用 |
|---|---|
| `msg/BallPos.msg` | `int32 id` / `float32 x` / `float32 y` |
| `package.xml` | rosidl 三件套 + `member_of_group` + `rclpy`/`geometry_msgs`/`turtlesim` 依赖 |
| `CMakeLists.txt` | `rosidl_generate_interfaces` + `install(PROGRAMS)` |
| `scripts/ball_publisher.py` | 2 Hz 定时器，频率做成 `freq` 参数 |
| `scripts/ball_subscriber.py` | 订阅 `ball_pos`，打印 id/x/y |
| `scripts/nav_to_balls.py` | 任务二的导航节点 |

坐标按细则写死：球 1 (2.0, 3.0)、球 2 (5.5, 8.0)、球 3 (9.0, 4.0)。

### 实测

| 验收项 | 结果 |
|---|---|
| `ros2 interface show ball_pos_pkg/msg/BallPos` | `int32 id` / `float32 x` / `float32 y` |
| `colcon build --packages-select ball_pos_pkg` | `1 package finished` |
| `ros2 topic info /ball_pos` | `Publisher count: 1` / `Subscription count: 1` |
| 双终端运行 | 发布端连续打印 `发布 球1/2/3`，订阅端同步 `收到 球1/2/3` |
| 默认 2 Hz | `ros2 topic hz` → **average rate ≈ 6.99**（2 Hz × 3 条） |
| `-p freq:=5.0` | `ros2 topic hz` → **average rate ≈ 15.97**（5 Hz × 3 条） |

### 三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人里的典型场景 |
|---|---|---|---|
| **话题 Topic** | 发布/订阅，多对多 | 单向、持续流，不关心对方在不在 | 视觉持续广播球坐标（本任务的 `ball_pos`）；决策持续广播速度指令（`cmd_vel`） |
| **服务 Service** | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切防守模式」；任务二开跑前清空画布就是用服务 |
| **动作 Action** | 目标/反馈/结果 | 长任务、能报进度、可取消 | 「走到点 (9,4)」→ 过程持续报进度，走错了能中途取消 |

一句话记法：**话题是广播，服务是打电话，动作是下单后能查物流还能取消。**

**任务二里实际用到的服务调用**（清空轨迹，让截图里的轨迹干净）：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

### 五个词是什么

- **节点**：一个独立进程，干一件事（`ball_publisher` 只管发，`ball_subscriber` 只管收）
- **话题**：按名字建立的一根管道 `/ball_pos`
- **消息**：管道里跑的数据格式 `BallPos`
- **发布者 / 订阅者**：往管道里写的一端、从管道里读的一端
- **两个节点靠什么对上号**：**话题名 + 消息类型**，两者都一致才连得上

---

## 任务二：订阅球坐标，依次走过去（35 分 + 闭环 5 分）

### 思路

`nav_to_balls.py` 同时订阅 `/ball_pos`（拿目标）和 `/turtle1/pose`（回读自身位姿），
每 0.1 秒跑一次控制：

1. **收齐 3 个球再动**：用字典按 `id` 存坐标（发布顺序不保证，用列表会和球号错位）
2. **算两个量**：距离 `d`、方位角差 `e = atan2(dy,dx) − θ`，并归一化到 `[−π, π]`
3. **三个分支**：
   - `|e| > 0.25` → 原地转，`linear.x = 0`（先对准再走，否则会画大弧）
   - `d > 0.15` → 边走边微调，`linear.x = min(1.2, 1.5·d)`（越近越慢，否则冲过头）
   - `d ≤ 0.15` → 发一条零速度刹车（不发 0 乌龟会一直按最后指令跑），打日志，切下一个点

### 实测（一次跑完，无人工按键，闭环生效）

| 目标 | 实际坐标 | 误差 |
|---|---|---|
| 球 1 (2.0, 3.0) | (2.108, 3.071) | **0.129** |
| 球 2 (5.5, 8.0) | (5.419, 7.890) | **0.137** |
| 球 3 (9.0, 4.0) | (8.907, 4.107) | **0.142** |

三个点误差全部远小于 0.5。轨迹是干净的三段折线，无画弧、无出界。

### 差速三公式

```
v = (vL + vR) / 2       线速度 = 左右轮速的平均
ω = (vR − vL) / L       角速度 = 左右轮速之差 ÷ 轮距
r = v / ω               转弯半径（ω 越小弯越大；ω = 0 走直线）
```

对应到 `Twist`：`linear.x` 走直线、`angular.z` 原地转、两个都给就是画弧。

---

## 遇到的问题与解决

1. **`package.xml` 被我粘贴成两份，XML 非法。**
   报错 `junk after document element: line 22`。原因是用 nano 时把新内容贴到了文件开头，
   老内容还在下面。改成 `cat > package.xml <<'EOF' … EOF` 整份覆盖才解决。
   **教训**：验证 XML 不能只 `grep` 数标签（我数到 3 个以为对了），要用
   `python3 -c "import xml.etree.ElementTree as ET; ET.parse('package.xml')"` 查结构。

2. **`ros2 run` 报 `No executable found`。**
   两个原因都踩过：脚本没有执行权限（`chmod +x`），以及 `CMakeLists.txt` 里漏了
   `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})`。另外可执行名**带 `.py`**。

3. **定时器参数是周期不是频率。**
   `create_timer(0.5, ...)` 是 0.5 秒一次，不是 0.5 Hz。写反了乌龟会几秒才动一下。

4. **`ros2 param set` 在运行期改不了发布频率。**
   参数只在 `__init__` 读一次、定时器只建一次。运行中 `param set` 参数值变了但频率不变，
   必须用启动参数 `--ros-args -p freq:=5.0`。

5. **方位角差没归一化，乌龟原地转圈。**
   `atan2` 直接相减可能得到 +350°，折回 ±π 才对：`e = atan2(sin(e), cos(e))`。

6. **到点不停、冲过界。**
   turtlesim 会一直按最后一条 `cmd_vel` 运动，到点必须发一条全 0 的 `Twist` 刹车。

7. **`ros2 topic info` / `ros2 node list` 偶发返回空或 `Unknown topic`。**
   是 ros2 daemon 缓存，`pkill -f _ros2_daemon` 杀掉后重试即可（`ros2 daemon stop` 不管用）。

---

## 任务三：串口与总线舵机（本周未完成）

本周时间不够，任务三的笔记尚未完成。诚实说明：**没做，不交**。

---

## 给下周的自己

以前觉得「控制」就是我把指令发出去；写完三点导航才明白，真正起作用的是每 0.1 秒把
`pose` 读回来、按差值修一点——不读回来的指令是盲发，发得再准也在赌。

下周接舵机同理：**先想清楚怎么把位置读回来（`PRAD`），再谈怎么发指令。**
