# 第 1 周考核 · xiabiling

- 姓名：xiabiling
- 完成日期：2026-10-20

## 任务 1：环境自检
我使用了 VMware 虚拟机安装了 Ubuntu 22.04。在安装 ROS2 时踩了不少坑：
1. 一开始把鱼香脚本的 `-O`（大写字母O）错打成了数字 `-0`，导致 wget 报错“无效的选项”。
2. 后来执行时，漏掉了 `. fishros` 之间的空格，导致系统提示“未找到命令”。
3. 成功运行脚本后，我选择了替换系统源和 ROS 源，并安装了 Humble 桌面版。
最后，敲 `printenv ROS_DISTRO` 成功输出 humble，环境配置成功。

## 任务 2：小乌龟
成功运行了 turtlesim_node 和 turtle_teleop_key。
**我对节点和话题的理解：**
`turtlesim_node` 和 `turtle_teleop_key` 就像是两台独立的对讲机（这就是**节点**）。它们互相看不见对方，但它们约定好了一个公共频道，叫作 `/turtle1/cmd_vel`（这就是**话题**）。
当我按下方向键时，键盘控制程序（teleop）就会对着这个频道喊：“往前冲！”；小乌龟程序一直在这个频道里听着，听到指令就往前走。它们俩通过这个频道（话题）实现了配合。

*踩坑补充*：我在终端敲 `ros2 node list` 时，一开始只看到了 `/turtlesim`，没有看到第二个节点。后来我通过 `ros2 topic info /turtle1/cmd_vel` 查看到 Publisher 和 Subscriber 的数量都是 1，证明了通信是正常的，也明白了新版 Humble 里 teleop 的注册机制可能不太一样。

## 任务 3：Git 提交
第一次走 Fork+PR 流程。因为虚拟机里的网络访问 GitHub 不稳定，所以改在 Windows 里操作。学会了如何生成 Token，以及如何用 `git add`、`git commit` 和 `git push` 把本地文件推送到自己的仓库。感觉终于学会了规范的代码提交。