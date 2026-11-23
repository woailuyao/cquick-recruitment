# ROS2 Humble 现场指令

在 Ubuntu 主终端正常启动任务一或任务二，复制当前运行的 frames.jsonl 路径。第二终端执行对应一条（替换路径）：

```bash
CQ=/home/hu_yuhan/Cquick/Week3-4
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch "$CQ/lab_launch_20261119/ros/cquick.launch.py" mode:=ball follow:=true ball_log:=/path/to/frames.jsonl
# 或：
ros2 launch "$CQ/lab_launch_20261119/ros/cquick.launch.py" mode:=pose follow:=true pose_log:=/path/to/frames.jsonl
```

同 ROS 环境用 `ros2 topic echo /soccer/ball` 或 `ros2 topic echo /localization/field_pose`。节点不打开相机；现场顺序启动，联合离线允许使用两个不占相机的文件输入。球为 camera_optical 原图像素，定位为 field_map 下相机光心，位置 m、朝向 deg/rad，PnP 残差 px。没有载体外参。无效帧清空，实时跟随长期无新记录发布 invalid。离线启动及 DDS 话题收发结果见 verification。现场真实输入仍待验证。
