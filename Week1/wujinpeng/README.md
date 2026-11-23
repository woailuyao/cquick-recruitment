# 第 1 周考核 · 吴锦鹏

- 姓名：吴锦鹏（wujinpeng）
- 学号 / 联系方式：学号：20264232
- 完成日期：2026-10-21

## 任务 1：环境自检

方案 A · 虚拟机：VirtualBox 7.2.22 + Ubuntu 22.04.5 LTS（分配 4 核 / 6 GB 内存 / 30 GB 虚拟磁盘）。

ROS2 用鱼香一键脚本安装：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

选项路径：一键安装 ROS → 更换系统源并清理第三方源 → 自动测速选源 → **中科大镜像** → `humble(ROS2)` → **桌面版**。

自检结果：

```bash
$ printenv ROS_DISTRO
humble

$ ros2 --help
usage: ros2 [-h] [--use-python-default-buffering]
        Call `ros2 <command> -h` for more detailed usage. ...
ros2 is an extensible command-line tool for ROS 2.
```

### 踩过的坑

**1. `ros2 --help` 报 command not found，但 `ros2` 单独敲却是好的**

一开始以为是没装好，反复重装。后来发现报错是这样：

```
ros2 --help: command not found
```

注意报错里**把 `ros2 --help` 整串当成了一个命令名**。真正没装好的话应该是 `ros2: command not found`，冒号前只有 `ros2` 两个字。

原因是**输入法停在中文状态**，敲出来的是全角空格，bash 不认它当分隔符，于是整串被当成一个命令去找。

解决办法：敲命令前按 `Shift` 切到英文输入法。

**辨认方法**：看 `command not found` 冒号前面那串有没有空格——有空格就是全角空格问题，不是环境问题。

**2. `passwd` 输密码时屏幕上什么都不显示**

Linux 输密码是不回显的，连星号都没有，光标也不动。第一次遇到以为键盘坏了，反复敲导致输错。

正确做法：盲打密码，直接回车，不要因为没反应就重复输入。

**3. 装增强功能时报 `gcc-12: not found`**

内核模块编译失败，补装 `gcc-12` 后重新执行 `VBoxLinuxAdditions.run` 解决：

```bash
sudo apt install -y build-essential gcc-12 linux-headers-generic
```

## 任务 2：小乌龟

```bash
ros2 run turtlesim turtlesim_node      # 终端 A：乌龟窗口
ros2 run turtlesim turtle_teleop_key   # 终端 B：键盘控制
```

用方向键控制乌龟画出了一条折线轨迹。

### 对「节点」和「话题」的理解

- **节点（node）**：一个独立干活的程序进程，每个节点只负责一件事。`turtlesim_node` 负责画乌龟和计算位置，`turtle_teleop_key` 负责读键盘。
- **话题（topic）**：节点之间传数据的管道。发布者往管道里写，订阅者从管道里读，双方不需要知道对方是谁、在哪。

这两个节点是典型的发布/订阅关系：`turtle_teleop_key` 把速度指令发布到 `/turtle1/cmd_vel`，
`turtlesim_node` 订阅这个话题，拿到速度后更新乌龟位置并重绘。

验证：

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
```

这种设计的好处是解耦：我可以把键盘换成手柄、换成自己写的程序，只要往 `/turtle1/cmd_vel`
发同样格式的消息，乌龟照样动，`turtlesim_node` 一行代码都不用改。

## 任务 3：Git 提交

理解：Fork是复制一份到自己仓库，然后PR是发起请求，将我的改动合并于主仓库  
坑：push 时报 Connection was reset，一开始不知道是网络问题
