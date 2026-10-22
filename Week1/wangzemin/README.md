# 第 1 周考核 · wangzemin

- 姓名：王泽民
- 姓名拼音：wangzemin
- 学号 / 联系方式：—
- 完成日期：2026-10-21

## 任务 1：系统安装与 ROS2 环境自检

我用的是 VMware 虚拟机，先装好 Ubuntu 22.04，再用鱼香 ROS 一键脚本装 ROS2 Humble：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

脚本里依次选了「1 一键安装 ROS」、换源、`humble(ROS2)`、桌面版，rosdep 选 `y`，然后等它自己装完。

装完在终端执行 `printenv ROS_DISTRO`，输出是 `humble`；再执行 `ros2 --help`，能正常打出帮助信息和一堆子命令（topic、node、service……），说明环境装好了。

截图前按防抄袭要求，先在同一个终端敲了 `echo "姓名:wangzemin"; date "+%Y-%m-%d %H:%M:%S"`，所以截图里能同时看到姓名拼音和当前时间。

## 任务 2：小乌龟

一个终端跑 `ros2 run turtlesim turtlesim_node`，弹出小乌龟窗口；再开一个终端跑 `ros2 run turtlesim turtle_teleop_key`，用方向键就能让小乌龟动起来，图里是小乌龟走出来的星形轨迹。

这两个概念我是这么理解的：

- **节点（node）**：一个正在运行的程序，只负责一件小事。`turtlesim_node` 负责画小乌龟、算它的位置；`turtle_teleop_key` 负责读键盘按键。`ros2 node list` 里能看到 `/turtlesim` 和 `/teleop_turtle` 两个，说明它们是两个各自独立跑的程序。
- **话题（topic）**：一条带名字的数据通道，节点之间靠它传消息。往里发消息的是发布者，从里面收消息的是订阅者，两边不用知道对方是谁、在不在同一台电脑上，只要认准同名话题就行。
- **两者的关系**：`turtle_teleop_key` 把按键翻译成速度指令，发布到 `/turtle1/cmd_vel`；`turtlesim_node` 订阅这个话题，收到速度就把小乌龟动起来。反过来，`turtlesim_node` 会往 `/turtle1/pose` 发乌龟的位置和朝向，往 `/turtle1/color_sensor` 发画笔颜色，这些在 `ros2 topic list` 里都能看到。

所以两个节点是互相独立的：把 teleop 关掉，小乌龟窗口还在，只是没人给它发指令了，它就停着不动。这就是"节点 + 话题"的松耦合——谁都不直接调谁，全靠话题传消息。

## 任务 3：Git 提交

第一次完整走 Fork + PR 流程，步骤大概是这样：

1. 在考核仓库页面点右上角 Fork，把它复制到自己的账号下；
2. clone 自己 Fork 出来的仓库（不是 clone 主仓库）；
3. 在 `Week1/` 里新建个人文件夹 `wangzemin/`，按「序号_内容」命名放好截图和 README；
4. `git add .` → `git commit -m "..."` → `git push`，推到自己 Fork 的仓库；
5. 在自己 Fork 的页面点 Contribute → Open Pull Request，目标选主仓库的 main 分支。

踩过的坑和记住的几点：

- **别 clone 错仓库**：一开始容易顺手 clone 主仓库，那样 push 会没权限，正确做法是 clone 自己账号下那份 Fork；
- **主仓库只能靠 PR 进**：直接往主仓库 push 一定失败，这不是权限坏了，本来就是这么设计的；
- **PR 交完还能改**：后面发现要改，本地改完再 commit + push，原来的 PR 会自动更新，不用重开一个。
