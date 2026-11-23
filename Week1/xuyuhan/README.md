# 第 1 周考核 · xuyuhan

- 姓名拼音：xuyuhan
- 完成日期：2026-10-25
- 运行环境：Ubuntu 22.04.5 LTS（VMware Workstation Pro 虚拟机）
- ROS 版本：ROS2 Humble

## 交付物

| 文件 | 内容 |
| --- | --- |
| `01_环境自检_1_桌面.png` | Ubuntu 22.04 桌面 |
| `01_环境自检_2_ROS_DISTRO.png` | `printenv ROS_DISTRO` 输出 `humble` |
| `01_环境自检_3_ros2_help.png` | `ros2 --help` 输出 |
| `02_小乌龟.png` | turtlesim 窗口，乌龟已移动并留下轨迹 |
| `02_节点列表.png` | `ros2 node list` 输出 |
| `02_话题列表.png` | `ros2 topic list` 输出 |

每张截图截取前都在同一终端执行了下面这条命令，因此图中同时包含姓名与实时时间，可自证为本人操作：

```bash
echo "姓名: xuyuhan"; date "+%Y-%m-%d %H:%M:%S"
```

---

## 任务 1：系统安装与 ROS2 环境自检

### 做了什么

1. 在 VMware Workstation Pro 中新建虚拟机，分配 4 GB 内存、2 核 CPU、30 GB 磁盘；
2. 安装 Ubuntu 22.04.5 LTS 桌面版，键盘布局选 English (US)，时区 Asia/Shanghai；
3. 装 `open-vm-tools` / `open-vm-tools-desktop`，解决分辨率自适应与时间同步；
4. 用鱼香 ROS 脚本安装 ROS2 Humble：

   ```bash
   wget http://fishros.com/install -O fishros && . fishros
   ```

   依次选择：`1` 一键安装 → 换源 → `humble(ROS2)` → 桌面版 → 安装 rosdep 选 `y`。

### 自检结果

```bash
$ printenv ROS_DISTRO
humble

$ ros2 --help
usage: ros2 [-h] [--use-python-default-warnings] ...
```

两项均正常，环境可用。

### 遇到的问题与解决

| 问题 | 原因 | 解决 |
| --- | --- | --- |
| 首次开机弹出「Ubuntu 24.04.5 LTS Upgrade Available」 | 系统默认对新版本做升级提示 | ROS2 Humble 只支持 22.04，选 **Not Upgrade**，并把 `/etc/update-manager/release-upgrades` 的 `Prompt` 改为 `never` |
| 虚拟机内键盘输入无响应 | VMware 未把输入定向到虚拟机 | 先用鼠标点击终端窗口内部，或用 `Ctrl+G` 把键盘交给虚拟机 |
| 每次启动 turtlesim 出现 `Ignoring XDG_SESSION_TYPE=wayland` 警告 | Qt 在 Wayland 会话下的提示 | 不影响运行；`export QT_QPA_PLATFORM=xcb` 可消除 |
| 图片无法从虚拟机复制到主机 | 文件跨主机传输依赖 open-vm-tools，Wayland 下不稳定 | 改为在主机侧用 `Win+Shift+S` 截取 VMware 窗口画面，直接落到 Windows |

---

## 任务 2：ROS2 小乌龟 —— 节点与话题

### 启动方式

两个终端分别执行：

```bash
# 终端 A
ros2 run turtlesim turtlesim_node

# 终端 B
ros2 run turtlesim turtle_teleop_key
```

启动后在终端 B 点一下鼠标让窗口获得焦点，方向键即可控制乌龟移动。

### 我的观察

```bash
$ ros2 node list
/teleop_turtle
/turtlesim

$ ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

$ ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

按方向键时执行 `ros2 topic echo /turtle1/cmd_vel`，可以看到 `linear.x`（前进/后退）和
`angular.z`（转向）的数值实时变化；`ros2 node info /turtlesim` 则显示它订阅 `cmd_vel`、
发布 `pose`。

### 加分项：我对「节点」和「话题」的理解

**节点（node）** 是一个独立运行、职责单一的进程。`turtlesim_node` 只负责画乌龟、更新位置；
`turtle_teleop_key` 只负责读键盘。它们之间没有互相调用，关掉任意一个，另一个都不会崩溃，
只是失去对应功能——这正是「一个节点做一件事」的体现。

**话题（topic）** 是一条带名字的消息通道，采用发布/订阅模型。发布者只管把消息放进通道，
不关心谁在听；订阅者只管从通道取消息，不关心谁发的。双方互不认识，因此可以一对多、
多对一，而且是异步的。

**两者的关系**：`turtle_teleop_key` 把按键翻译成速度指令（消息类型
`geometry_msgs/msg/Twist`，`linear.x` 控制线速度、`angular.z` 控制角速度），发布到
`/turtle1/cmd_vel`；`turtlesim_node` 订阅这条话题，收到就驱动乌龟移动；同时它把自己的位姿
发布到 `/turtle1/pose`。**节点是「谁在干活」，话题是「它们怎么连起来」**——这就是 ROS2 的
基本结构。

用 `ros2 topic info` 能看到 `Publisher count: 1, Subscription count: 1`，正好印证了这个
「一发布一订阅」的连接关系，而不是靠猜。

### 遇到的问题

- 刚启动时按方向键乌龟不动：原因是键盘焦点不在 `turtle_teleop_key` 所在的终端上，
  点一下该窗口后恢复正常；
- 轨迹想重新画：用 `ros2 service call /clear std_srvs/srv/Empty` 清屏，再控制乌龟画图形。

---

## 任务 3：Git 提交

### 流程

1. 在 GitHub 上 Fork 主仓库到自己的账号；
2. 把**自己 Fork 出来的**仓库 clone 到本地（不是主仓库）；
3. 新建 `Week1/xuyuhan/` 目录，放入 6 张截图和本文件；
4. `git add .` → `git commit -m "Week1: 完成环境搭建与 turtlesim 节点/话题验证"` → `git push`；
5. 在 Fork 仓库页面发起 Pull Request，目标为主仓库 `main` 分支。

### 踩过的坑

1. **直接 clone 主仓库**：push 时没有权限。改回自己 Fork 的地址后正常；
2. **没点 Fork 就 clone**：报 `ERROR: Repository not found`，重新 Fork 后解决；
3. **直连 GitHub 克隆时 `Recv failure: Connection was reset`**：改用 SSH over 443
   （`HostName ssh.github.com` / `Port 443`）后稳定；
4. 中文文件名在 `git status` 里显示为八进制转义：`git config --global core.quotepath false` 解决。

### 感想

以前只用过 `git clone` 下载别人的代码，这次第一次完整走完 Fork → commit → push → PR。
最大的感受是「权限边界」很清楚：主仓库只能通过 PR 修改，自己 Fork 的仓库才能直接写。
另外分支、远程地址这些概念，只有真的推一次、被拒一次，才算真懂。
