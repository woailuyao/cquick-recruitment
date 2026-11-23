# 第1周考核 · 鲍鑫剑
- 姓名：鲍鑫剑
- 完成日期：2026-10-25

## 任务1：环境自检
成功在Ubuntu22.04虚拟机安装ROS2 Humble，使用printenv ROS_DISTRO确认版本为humble，环境正常。

## 任务2：小乌龟turtlesim
启动turtlesim节点，通过teleop键盘方向键控制小乌龟移动，画出运动轨迹，理解ROS2节点之间通信、话题通信的基础概念。
我的理解：
turtlesim 是 ROS2 经典入门案例，演示了话题通信。
一共两个节点：
 
1.  turtlesim_node ：乌龟界面节点，负责画面渲染，订阅速度话题，控制小乌龟移动、留下轨迹。

2.  turtle_teleop_key ：键盘控制节点，读取方向按键，不断向话题  /turtle1/cmd_vel  发布速度指令。
 
两个节点不需要直接互相调用，一个发布消息、一个订阅消息，这就是ROS话题通信的特点，实现节点之间的松耦合通信
## 任务3：Git提交踩坑记录
1. 一开始访问GitHub超时，借助Watt Toolkit才成功打开仓库
2. 需要先Fork原仓库，只能在自己Fork后的仓库内新建文件夹上传文件
3. 文件目录必须放在Week1/baoxinjian下面，不能直接丢在Week1根目录
