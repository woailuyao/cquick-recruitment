# 第 1 周考核 · pansiqi

- 姓名：pansiqi（Brain-Pan）
- 学号 / 联系方式：3068497314@qq.com
- 完成日期：2026-10-25

## 任务 1：环境自检

**环境**：VMware Workstation 虚拟机，Ubuntu 22.04.5 LTS，分配 4 核 CPU / 4GB 内存 / 30GB 磁盘，网络为 NAT 模式。

**安装过程**：在 VMware 中导入 Ubuntu 22.04 镜像并完成系统初始化后，安装 ROS2 Humble：先将 apt 源换成国内镜像以加快下载，再添加 ROS2 软件源，安装桌面版 `ros-humble-desktop`（包含 RViz2、Gazebo），随后执行 `rosdep init` / `rosdep update` 解决依赖，最后把 `source /opt/ros/humble/setup.bash` 写入 `~/.bashrc`，使每个新终端自动加载 ROS2 环境。

**自检结果**：

- `printenv ROS_DISTRO` → `humble`，版本正确；
- `ros2 --help` 能正常列出全部子命令（action / bag / node / topic / run 等），无报错。

**截图**：

- `01_环境自检.png`：Ubuntu 桌面，终端中打印姓名与实时时间；
- `01_ROS版本自检.png`：`printenv ROS_DISTRO` 输出 humble；
- `01_ros2_help.png`：`ros2 --help` 的输出。

## 任务 2：小乌龟

启动 `turtlesim_node` 后出现小乌龟窗口；另开一个终端运行 `turtle_teleop_key`，把键盘焦点放在该终端后，用方向键控制乌龟前进与转向，画布上留下了移动轨迹。

**我对“节点”和“话题”的理解**：

- **节点（Node）**：ROS2 中独立运行的程序单元，通常一个节点只承担一件事。`turtlesim_node` 负责模拟小乌龟世界（维护位置、速度，并在画布上绘制轨迹），`turtle_teleop_key` 负责读取键盘输入。它们是两个相互独立的进程，彼此并不知道对方存在。
- **话题（Topic）**：节点之间传递数据的通道，有名字、有固定的消息类型，采用“发布 / 订阅”方式通信——一方把消息发布到话题，另一方订阅该话题来接收消息。
- **两者的关系**：`turtle_teleop_key` 把方向键翻译成 `geometry_msgs/msg/Twist` 消息（`linear.x` 线速度、`angular.z` 角速度），发布到 `/turtle1/cmd_vel` 话题；`turtlesim_node` 订阅这个话题，收到消息后按速度更新乌龟的位置和姿态。整个过程中两个节点没有直接调用，只通过话题交换消息，因此是解耦的——换一个发布者（例如自己写程序往同一话题发指令），乌龟同样会动。

**验证**：`ros2 node list` 显示 `/teleop_turtle` 与 `/turtlesim` 两个节点；`ros2 topic list` 显示 `/turtle1/cmd_vel`、`/turtle1/pose`、`/turtle1/color_sensor` 等话题。

**截图**：

- `02_小乌龟.png`：乌龟移动后的窗口（画布上留有轨迹）；
- `02_节点列表.png`：`ros2 node list` 输出两个节点；
- `02_话题列表.png`：`ros2 topic list` 输出，含 `/turtle1/cmd_vel`。

## 任务 3：Git 提交

第一次完整走 Fork + Pull Request 流程：

1. 在 GitHub 网页把考核仓库 Fork 到自己账号；
2. `git clone` 自己 Fork 出来的仓库（不是主仓库）；
3. 在仓库中新建 `Week1/pansiqi/` 目录，放入截图与本文说明；
4. `git add .` → `git commit` → `git push`，把改动推送到自己的 Fork；
5. 在自己的仓库页面发起 Pull Request，把改动合并请求发给主仓库原来的 main 分支。

**体会**：Fork + PR 的意义在于——即使没有主仓库的写权限，也能参与协作：改动先进入自己的副本，再通过 PR 请求合并，评审通过后才进入主仓库。相比直接 push，多了一层“先审后合”的保护，避免了未经检查的改动直接污染主仓库。
