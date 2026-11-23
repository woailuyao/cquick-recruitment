# 第 1 周考核 · 秦雅丽

- 姓名：秦雅丽（qinyali）
- 学号 / 联系方式：待补充
- 完成日期：2026-10-25（截图）／2026-10-26（提交）
- 环境：Ubuntu 22.04（VMware 虚拟机）＋ ROS2 Humble

## 任务 1：环境自检

按鱼香 ROS 的一键脚本（`wget http://fishros.com/install -O fishros && . fishros`）安装：
换国内源 → 选 `humble(ROS2)` → 桌面版 → 安装 rosdep。

装完后在终端自检：

```bash
printenv ROS_DISTRO     # 输出 humble
ros2 --help             # 能看到 ros2 的子命令列表
```

遇到的问题：一开始把命令写成了 `ros2 run turtlesim`（漏掉可执行文件名），
终端提示 `the following arguments are required: executable_name`，补上
`turtlesim_node` 后正常启动。另外 Wayland 下会提示
`Ignoring XDG_SESSION_TYPE=wayland on Gnome`，属于警告，不影响运行。

截图（含姓名 + 实时时间戳）：

| 文件 | 内容 |
| --- | --- |
| `01_环境自检.png` | `printenv ROS_DISTRO` 输出 `humble`，以及 `ros2` 帮助/子命令输出 |

## 任务 2：小乌龟

两个终端分别启动：

```bash
ros2 run turtlesim turtlesim_node        # 终端 1：小乌龟窗口
ros2 run turtlesim turtle_teleop_key     # 终端 2：键盘控制
```

用方向键让乌龟移动，并用 `ros2 node list` / `ros2 topic list` 查看当前运行的
节点与话题。

**我对“节点”和“话题”的理解（加分项）**：

- **节点（Node）**：一个独立运行、各干一件事的程序。`turtlesim_node` 负责画乌龟、
  算运动，`turtle_teleop_key` 负责读键盘——它们是两个互不依赖的进程，各自启停都不影响对方。
- **话题（Topic）**：节点之间传数据的“频道”，有名字（如 `/turtle1/cmd_vel`），
  发的一方叫发布者（publisher），收的一方叫订阅者（subscriber），双方不需要知道对方存在。
- **关系**：`turtle_teleop_key` 把方向键转换成速度指令，发布到 `/turtle1/cmd_vel`；
  `turtlesim_node` 订阅这个话题，收到速度后积分出新的位姿，同时把位姿发布到 `/turtle1/pose`。
  这种“发布/订阅 + 话题解耦”的方式，就是后面做传感器、底盘、上位机通信的基础。

截图（含姓名 + 实时时间戳）：

| 文件 | 内容 |
| --- | --- |
| `02_小乌龟.png` | 乌龟已离开初始位置并画出轨迹，同时开着 `turtle_teleop_key` 终端 |
| `02_节点列表.png` | `ros2 node list` 输出 `/turtlesim` |
| `02_话题列表.png` | `ros2 topic list` 输出 `/turtle1/cmd_vel`、`/turtle1/pose` 等 |
| `02_节点与话题列表_原始截图.png` | 上面两张清单的完整原始截图（含桌面、时间戳、姓名），便于核对未被裁剪篡改 |

## 任务 3：Git 提交

第一次完整走 Fork + Pull Request 流程：

1. 在 GitHub 上 Fork `woailuyao/cquick-recruitment` 到自己账号；
2. `git clone` 自己 Fork 出来的仓库（不是主仓库）；
3. 在 `Week1/qinyali/` 下放截图和本 README；
4. `git add .` → `git commit -m "feat(week1): 提交秦雅丽第1周环境搭建与turtlesim任务"` → `git push`；
5. 在自己的 Fork 页面点 `Contribute → Open Pull Request`，目标为主仓库 `main`。

踩过的坑：GitHub 网页访问受网络/IP 影响，push 和开 PR 有延迟；学会了先确认
`git remote -v` 指向的是自己的 Fork，避免误推主仓库。
