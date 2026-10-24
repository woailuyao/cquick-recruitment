# 第 1 周考核 · <张诚成>

- 姓名：<张诚成>
  <zhangchengcheng>
- 学号 / 联系方式：<20266304 15583361832>
- 完成日期：<20261024>
- 环境：<VMware / VirtualBox / 双系统 / WSL2>，Ubuntu 22.04 + ROS2 Humble


## 任务 1：环境自检

### 我做了什么

- 用 <VMware / VirtualBox / 双系统> 装了 Ubuntu 22.04，分配 <4GB 内存 / 2 核 CPU / 30GB 磁盘>；
- 用鱼香 ROS 一键脚本装了 ROS2 Humble：`wget http://fishros.com/install -O fishros && . fishros`；
- 菜单里依次选：1（一键安装 ROS）→ 换源 → humble → 桌面版 → rosdep 选 y。

### 自检结果

- `printenv ROS_DISTRO` 输出：`<humble>`
- `ros2 --help` ：<能正常输出帮助信息 / 报了什么错>

### 踩过的坑

<1.不知道打开Ubuntu的密码，发现ai帮我设置了>


截图：`01_环境自检.png`（Ubuntu 桌面）、`01_ROS版本.png`（printenv）、`01_ros2帮助.png`（ros2 --help）

## 任务 2：小乌龟

### 我怎么跑通的

```bash
# 终端 1
ros2 run turtlesim turtlesim_node
# 终端 2
ros2 run turtlesim turtle_teleop_key
```

然后用方向键控制小乌龟移动，画出了轨迹。

### 我理解的「节点」和「话题」

- **节点**：<是一个用来执行一个动作的综合性模块，同时有着发出，收集信息的功能>
- **话题**：<节点之间针对某一合作性动作的约定，两者在此约定下交换信息并同时执行自己的原有功能>
- **两者关系**：<节点是不同的功能模块，各自负责自己所设计的动作，话题则让两个节点在统一的规则之下，能够互相
联系，利用对方的功能形成统一的整体。>

### 命令证据

- `ros2 node list` 看到：`/turtlesim`、`/teleop_turtle`
- `ros2 topic list` 看到：`/turtle1/cmd_vel`、`/turtle1/pose` 等

截图：`02_小乌龟.png`、`02_节点列表.png`、`02_话题列表.png`

## 任务 3：Git 提交

### 走通的流程

1. 在 GitHub 网页 Fork 考核仓库到自己账号；
2. `git clone` **自己 Fork 出来**的那个仓库（URL 里是我的用户名）；
3. 在 `Week1/<拼音>/` 下放入截图和本文件；
4. `git add` → `git commit` → `git push` 到自己的 Fork；
5. 网页点 Contribute → Open pull request，base 选 `woailuyao/cquick-recruitment` 的 `main`。

### 踩过的坑

<1.一直找不到相应的加速器·，2登录github之后不会用，发现要看所适配的b站视频>

## 自证说明

- 每张关键截图里都执行了 `echo "姓名: <拼音>"; date "+%Y-%m-%d %H:%M:%S"`，姓名与时间戳和命令输出在同一屏；
- 截图里的姓名拼音与本文件夹名一致。
