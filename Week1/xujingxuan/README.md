# Week1 环境搭建与 ROS2 入门 — 徐靖轩（xujingxuan）

## 1. 环境信息

| 项目 | 实测值 |
| --- | --- |
| 宿主机 | Windows 11，VMware Workstation 16 |
| 虚拟机 | Ubuntu 22.04.5 LTS（x86_64），内存 4 GB / 2 核 / 30 GB |
| ROS 版本 | Humble（`printenv ROS_DISTRO` → `humble`，`/opt/ros/humble` 存在） |
| 安装方式 | 鱼香 ROS 一键安装脚本：`wget http://fishros.com/install -O fishros && . fishros`，选择 humble + 桌面版（desktop）+ rosdep |
| 虚拟机增强 | `open-vm-tools` / `open-vm-tools-desktop` 2:12.3.5 |

## 2. 任务 1：环境自检（30 分）

| 截图 | 内容 | 说明 |
| --- | --- | --- |
| `01_环境自检_1.png` | Ubuntu 22.04 桌面 | 进桌面后截取 |
| `01_环境自检_2.png` | `printenv ROS_DISTRO` → `humble` | 证明 ROS2 版本正确 |
| `01_环境自检_3.png` | `ros2 --help` 输出 | 证明 ros2 命令可用（输出 `usage: ros2 [-h] ...`） |

每条截图前均执行：

```bash
echo "姓名: xujingxuan"; date "+%Y-%m-%d %H:%M:%S"
```

## 3. 任务 2：turtlesim 小乌龟（40 分）

操作步骤：

```bash
# 终端 A
ros2 run turtlesim turtlesim_node
# 终端 B
ros2 run turtlesim turtle_teleop_key
```

终端 B 聚焦后用方向键 ↑↓←→ 控制乌龟，画出明显轨迹。

| 截图 | 内容 |
| --- | --- |
| `02_小乌龟.png` | 乌龟窗口 + 运动轨迹 |
| `02_节点列表.png` | `ros2 node list` → `/turtlesim`、`/teleop_turtle` |
| `02_话题列表.png` | `ros2 topic list` → `/turtle1/cmd_vel` 等 |

## 4. 节点与话题的理解（加分项 +10）

**实测数据**（`ros2 topic echo /turtle1/cmd_vel` 时按下 ↑ 抓到的真实报文）：

```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

**我的理解：**

1. **节点（Node）** 是一个独立运行的进程，负责一件事。`turtlesim_node` 负责画乌龟和算位置，`turtle_teleop_key` 负责读键盘。两个节点的代码互相不知道对方存在，各跑各的进程 —— `ros2 node list` 里能看到它们是两个独立条目。关掉键盘节点后，乌龟窗口依然在，只是不动了，说明节点之间确实是解耦的。
2. **话题（Topic）** 是节点之间传数据的管道，走"发布—订阅"模型：`/turtle1/cmd_vel` 上，`turtle_teleop_key` 是发布方，`turtlesim_node` 是订阅方。我按下 ↑ 的瞬间，键盘节点往这个话题发一条 `geometry_msgs/msg/Twist` 消息（`linear.x = 2.0`），乌龟节点收到后开始前进。
3. 只要往这个话题发**同样格式**的数据，谁发都行。我用命令直接发布验证过：

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}"
```

不碰键盘，乌龟照样动 —— 这就是话题通信"只认数据格式、不认是谁发的"的特点。

（`ros2 topic echo /turtle1/pose` 显示乌龟位置从初始 x=5.54, y=5.54 移动到了 x=7.41, y=6.35，轨迹确实产生了。）

## 5. 下周预习（加分 +5）：自己写的 publisher / subscriber

写了两个最简节点（`talker.py` / `listener.py`，同目录下）：

- **发布方** `my_talker`：以 0.5 秒一次向 `/my_chatter`（`std_msgs/String`）发 `hello ros2 #n`，同时向 `/turtle1/cmd_vel` 发 `linear.x=1.0, angular.z=0.8`；
- **订阅方** `my_listener`：同时订阅 `/my_chatter` 和 `/turtle1/pose`，把收到的数据打印出来。

**实测输出**（节选）：

```
[发布] /turtle1/cmd_vel  linear.x=1.0 angular.z=0.8 | /my_chatter: hello ros2 #0
...
[发布] 结束，共发布 12 条

[订阅] /my_chatter  收到: hello ros2 #0
[订阅] /my_chatter  收到: hello ros2 #1
[订阅] /turtle1/pose  x=7.41 y=6.35 theta=1.76   ← 运动前
[订阅] /turtle1/pose  x=6.08 y=4.85 theta=-0.09  ← 被我自己写的节点驱动后
```

理解：发布方只管往某个话题发固定格式的消息，它不知道有没有人在听；订阅方只管订阅话题并处理消息，也不知道是谁发的。双方靠**话题名 + 消息类型**对上，这就是 pub/sub 解耦的意义。我自己写的节点能驱动 turtlesim 动起来，正说明这一点。

**串口预习（第 2 周用）**：UART 是一种串行通信接口，TX 发、RX 收、GND 共地，收发双方必须事先约定**波特率**（每秒传多少个 bit，常见 9600 / 115200），两端波特率不一致就会收到乱码。

## 6. 任务 3：Fork + Pull Request（30 分）

- Fork 仓库后 clone **自己账号下的仓库**（否则 push 会 403）；
- 在 `Week1/xujingxuan/` 下放入截图与本文档；
- 提交并 Push，发起 Pull Request，最后在 PR 里 @ 一位学长评审。

**本次完成情况（实测）**：已 Fork 到 `EricXu-jingxuan/cquick-recruitment`，推送提交 `5371490`（10 个文件，仅新增、无删除），并已发起 PR —— **PR #31**（`woailuyao:main` ← `EricXu-jingxuan:main`，状态 open，可合并）。

## 7. 踩坑记录

1. `. fishros` 不能写成 `sh fishros` —— 用 `sh` 执行不会把环境配进当前 shell。
2. **手敲 Linux 命令极易丢空格**：`ros2--help`、`head-20`、`ros2 topic' list` 都报 `command not found`。教训：命令一律复制粘贴，终端里粘贴是 `Ctrl+Shift+V` 或右键。
3. `cmd_vel` 最后一个字符是**字母 l**（velocity），敲成数字 `1` 会报"话题不存在"。
4. `/turtle1/cmd_vel` 没有数据时，`ros2 topic echo` 会一直刷 `Message is dropped` —— 不是报错，按下方向键就会出真实数据。
5. 装系统时不要点"Ubuntu 24.04.5 LTS Upgrade" —— 升到 24.04 后 ROS2 Humble 装不了（Humble 只支持 22.04）。
6. 主机与虚拟机之间复制粘贴失效，是因为 `open-vm-tools` 没生效；装完 `open-vm-tools open-vm-tools-desktop` 并重启后恢复正常（终端里粘贴是 `Ctrl+Shift+V`，不是 `Ctrl+V`，后者会打出 `^V`）。
