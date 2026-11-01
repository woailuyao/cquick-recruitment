# 第 1 周考核 · zhuwenxuan

- 姓名：zhuwenxuan
- 学号 / 联系方式：20266268（重庆大学）· 19123100156
- 完成日期：2026-10-26
- 运行环境：**WSL2 + Ubuntu 22.04.5 LTS + ROS2 Humble**（依据考核细则 FAQ Q1，截图已注明）

---

## 任务 1：环境安装与 ROS2 环境自检

### 最终结果

```bash
$ printenv ROS_DISTRO
humble

$ ros2 --help
usage: ros2 [-h] [--use-python-default-buffering] ...
ros2 is an extensible command-line tool for ROS 2.

$ dpkg -l | grep ros-humble-desktop
ros-humble-desktop   0.10.0-1jammy.20260916.003705
```

### 过程与踩坑（这部分花了我最多时间）

我的电脑是 Windows 11 家庭版，之前从没装过 Linux，**连 WSL 功能都没启用**。整个过程踩了四个坑：

**坑 1：微软商店下载只有 20–30 KB/s**

按常规做法执行 `wsl --install -d Ubuntu-22.04`，下载卡在 10% 一个小时没动。
用脚本测速确认：从微软 CDN 和 GitHub 直连拉大文件都是 **0.03 MB/s** 左右，而 351 MB 的安装包按这个速度要 3 小时，没法等。

**坑 2：商店路线走不通，改用离线方案**

放弃 `wsl --install`，改成三步手工：

1. 用管理员权限执行 `dism` 启用 `Microsoft-Windows-Subsystem-Linux` 和 `VirtualMachinePlatform`
2. 从国内 GitHub 加速代理（`ghfast.top`）下载 `wsl.3.0.3.0.x64.msi`（351 MB）直接安装
3. 从 Ubuntu 官方源下载 `ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz`（325 MB），用 `wsl --import` 导入

这样完全绕开了商店，下载速度从 30 KB/s 提到可以接受的范围，整个过程 10 分钟以内完成。

**坑 3：网上流传的鱼香镜像源是假的**

按教程把 ROS2 apt 源配成 `https://mirror.fishros.com/ros2/ubuntu`，`apt-get update` 却一直失败。
排查发现这个地址**返回的是一段 HTML 网页，不是 apt 仓库索引**——服务器返回 200 状态码，所以表面上看"能访问"，实际内容完全不对。同样的问题也出现在 `mirror.fishros.com/ros.key`。

**换成阿里云的 `https://mirrors.aliyun.com/ros2/ubuntu` 后正常**，实测索引里有 7972 个包，`ros-humble-desktop`、`ros-humble-turtlesim`、`ros-humble-rviz2` 都在。

**坑 4：ros2 命令报一大坨 Python 异常**

装好后执行 `ros2 node list`，抛出一长串 traceback，结尾是：

```
http.client.RemoteDisconnected: Remote end closed connection without response
```

原因：`ros2` 命令默认会先连接一个后台守护进程（`_ros2_daemon`）来查询节点信息，这个进程卡死了。用 `pkill -9 -f _ros2_daemon` 杀掉重启可以修复；紧急情况下加 `--no-daemon` 参数可以直接绕过：

```bash
ros2 node list --no-daemon
```

### 收获

1. 装环境前先**测速**，别盲信教程里的默认源；
2. HTTP 返回 200 **不等于**内容是对的，要用 `file` 或看头几个字节验证；
3. Windows 上装 Linux 环境，`wsl --install` 不是唯一解，`--import` + 官方 rootfs 更可控。

---

## 任务 2：ROS2 小乌龟 —— 节点与话题

### 操作

三个终端分别执行：

```bash
# 终端 1：小乌龟本体
ros2 run turtlesim turtlesim_node

# 终端 2：键盘控制
ros2 run turtlesim turtle_teleop_key

# 终端 3：查看节点和话题
ros2 node list
ros2 topic list
```

### 我对「节点」和「话题」的理解

**节点（node）**：一个独立运行的小程序，只干一件事。
`turtlesim_node` 负责"画出乌龟并计算它的位置"，`turtle_teleop_key` 只负责"把键盘按键翻译成速度指令"。
它们各自是独立的进程，谁也不调用谁的函数——删掉其中一个，另一个照常运行，只是配合不起来了。

**话题（topic）**：一条**有名字**的数据通道，用来传消息。
发的一方叫 publisher，收的一方叫 subscriber，双方**互相不知道对方是谁**，只知道话题名。

**它们的关系**：节点通过话题解耦。

放到小乌龟这个例子里：

```
turtle_teleop_key ──发布──▶ /turtle1/cmd_vel ──订阅──▶ turtlesim_node
      (节点)                  (话题，速度指令)              (节点)

turtlesim_node ────发布──▶ /turtle1/pose ────订阅──▶ 谁想看谁订阅
                             (话题，乌龟位姿)
```

我按方向键 → teleop 节点把"前进/转向"打包成 `Twist` 消息发到 `/turtle1/cmd_vel` → turtlesim 节点订阅到这个话题，读取速度，更新乌龟坐标并重绘 → 同时把新坐标发布到 `/turtle1/pose`。

**为什么这样设计**：如果以后要换成手柄控制，不用改 turtlesim 一行代码，只要再写一个节点往 `/turtle1/cmd_vel` 发消息就行。这就是"松耦合"的好处——第 2 周的发布/订阅，本质上就是在学怎么自己写这两端。

### 一个小细节

`ros2 node list` 里显示的是 `/teleop_turtle` 而不是 `/turtle_teleop_key`。
**这是正常的**：`turtle_teleop_key` 是**可执行文件名**（`ros2 run` 后面填的），`teleop_turtle` 是它启动后注册的**节点名**。ROS2 里这两个名字本来就可以不同，`ros2 node list` 列出的是节点名。

---

## 第一次走 Fork + PR 流程，最大的感受是：**主仓库是真的推不进去**。

我一开始下意识想 clone 主仓库再改，但实际你只有自己 Fork 出来的仓库有写权限，
主仓库只能通过 PR 进入 —— 这是有意设计的，不是权限出错。

另一个卡点是网络：`github.com` 主站在国内时通时不通。我用脚本测了 8 个
GitHub 边缘 IP，发现同一个域名有的能连、有的 20 秒超时。最后是手动改 `hosts`
指向一个能通的 IP 才打开了网页。

至于提交本身，网页上传比想象中简单：Fork → 进入 `Week1` → Upload files →
拖文件夹 → 写 commit message → 开 PR。但有个细节容易错：
**上传页面显示的路径是相对「当前所在目录」的**，如果没先进 `Week1` 就上传，
文件会落到仓库根目录，路径就不合规了 —— 我第一次就传错了。

如果以后再交，我会直接用命令行 `git clone` / `add` / `commit` / `push`，
因为命令行能看清每一步到底改了什么，网页上传更像是「盲交」。

> 待补：走完 Fork + PR 流程后填写。
