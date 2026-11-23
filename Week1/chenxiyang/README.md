# 第 1 周考核 · 陈锡洋（chenxiyang）

- 姓名：陈锡洋（chenxiyang）
- 联系方式：860727656@qq.com
- 完成日期：2026-10-25

## 任务 1：环境自检

用 VMware Workstation 装了 Ubuntu 22.04，再用鱼香 ROS 的一键脚本装 ROS2 Humble 桌面版。
自检结果：`printenv ROS_DISTRO` 输出 `humble`，`ros2 --help` 能正常显示帮助信息。

遇到的问题：
1.看的第一个视频给的是20.4，白下载了半天，幸好及时发现。
2.第一次装ros2自检 提示 command not found重开虚拟机又安装了一次才成功

## 任务 2：小乌龟

两个终端分别跑 `ros2 run turtlesim turtlesim_node` 和 `ros2 run turtlesim turtle_teleop_key`，
用方向键控制乌龟，让它画出了轨迹。

- `ros2 node list` 看到 `/turtlesim`、`/teleop_turtle` 两个节点
- `ros2 topic list` 看到 `/turtle1/cmd_vel`、`/turtle1/pose` 等话题

### 我对“节点”和“话题”的理解
“节点”就是终端里运行的一段“程序”，而话题就是节点之间的通信通道，使得节点间既能相互联系又不会干扰另外一个节点的整体代码。


## 任务 3：Git 提交

第一次完整走 Fork + PR：

1. 网页上 Fork 主仓库到自己账号（chenxiyang3945）
2. `git clone` **自己的 fork**（不是主仓库）
3. 新建 `Week1/chenxiyang/`，放入 4 张截图和本说明
4. `git add` → `git commit` → `git push`
5. 向主仓库发起 Pull Request

**踩过的坑：**

1. 一开始的截图是 Windows 上用 QQ 截的，还把 node list / topic list 和小乌龟合在一张图里。
   规范要 4 个独立文件，评审按文件名找图，合成一张等于“缺 2 张”，后来按规范重截了一遍。
2. `git clone` 报 `fatal: ... GnuTLS recv error (-54): Error in the pull function`。
   一开始以为是命令写错，其实是虚拟机到 GitHub 的连接被打断，属网络问题，调整网络后重试通过


