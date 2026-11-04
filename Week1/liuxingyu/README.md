# 第 1 周考核 · 刘星语

- 姓名：刘星语
- 学号：20264513
- 完成日期：2026-10-22

## 任务 1：环境自检

在 VMware 中安装 Ubuntu 22.04，用鱼香 ROS 一键脚本安装 ROS2 Humble 桌面版。

踩坑：VMware 官网注册时 QQ 邮箱收不到验证码，换 Foxmail 邮箱才收到，之后仍需审核，遂改从 B 站获取安装包；创建虚拟机时看到 "Erase disk and install Ubuntu" 误以为会清空整个磁盘，询问 AI 后确认只针对虚拟硬盘，继续完成安装。

## 任务 2：小乌龟

我的理解：节点是可执行文件运行起来的实例，负责一个独立的模块化功能；节点之间通过话题（topic）通信，publisher 发布数据、subscriber 接收，是单向的。`turtle_teleop_key` 发布速度消息到 `/turtle1/cmd_vel`，`turtlesim` 订阅该话题后移动。

## 任务 3：Git 提交

第一次完整走 Fork → Clone 自己的仓库 → Commit → Push → Pull Request 流程，过程较繁琐，在 AI 帮助下完成，加深了对 Git 协作的理解。
