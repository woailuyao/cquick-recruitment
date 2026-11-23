# 第 1 周考核 · 姓名

- 姓名：杨荟立
- 完成日期：2026-10-25

## 任务 1：环境自检
我在VirtualBox和VMware中启动Ubuntu 22.04.5 LTS，安装并检查 ROS2 Humble。‘printenv ROS_DISTRO’输出为 ‘humble’，‘ros2 --help’ 可以正常执行。
遇到的问题：因为最初用VMware下载时注册broadcom账号是接收不到验证码，采用了VirtualBox安装虚拟机，后来将qq邮箱换成学校邮箱成功注册，推测可能是qq邮箱将海外服务器的发信拦截

## 任务 2：小乌龟
节点：如turtlesim_node（画乌龟、让乌龟动）和turtle_teleop_key（读键盘）是两个独立的程序，各自运行，负责某件事

话题：节点之间靠话题（如：/turtle1/cmd_vel）这个“通道”通信。键盘节点发布（publish）“移动”指令到这个话题，乌龟节点订阅（subscribe）这个话题，收到指令后让乌龟动，只能由键盘单向发送。

节点之间互不影响，但能通过话题通信

## 任务 3：Git 提交
完全不了解Fork、Branch、Commit、Push 和 Pull Request的意思，不知道在哪打开网页，对全英文的界面无从下手，后来挨个问ai，在ai的帮助下理解了每个步骤的意义和用途，一步步将零散的知识点串联成完整的流程
