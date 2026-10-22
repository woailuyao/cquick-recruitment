# 第1周考核 - wangxuanhe
姓名：王煊赫
完成日期：2026-10-22

## 任务1：环境自检
完成Ubuntu22.04与ROS2 Humble安装配置，使用ros2 --version校验版本。
踩坑：新开终端找不到ros2命令；
解决：把 source /opt/ros/humble/setup.bash 写入 ~/.bashrc，自动加载环境变量。

## 任务2：小乌龟
节点是ROS2里独立运行的程序，turtlesim_node用来显示乌龟窗口，turtle_teleop_key接收键盘按键。
话题是节点之间传递数据的通道。键盘节点往 /turtle1/cmd_vel 发布速度消息，乌龟节点订阅这个话题，接收指令移动。

## 任务3：Git提交
学习Fork + PR提交流程。
踩坑：作业文件必须放在Week1/wangxuanhe目录下，不能直接推送到主仓库。