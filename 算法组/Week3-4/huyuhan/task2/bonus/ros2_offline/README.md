# ROS2 双话题离线回放验证

## 1. 实验目的

验证 CQuick 算法系统的球检测与场地定位数据
能否通过 ROS2 双话题进行传输，并正确处理有效、
无效和空检测结果。

## 2. 环境

- Ubuntu 22.04.5 LTS
- ROS2 Humble
- Python 3.10
- std_msgs/msg/String

## 3. ROS2 话题

| Topic | 内容 |
|---|---|
| /soccer/ball | 球检测结果 |
| /localization/field_pose | 场地定位结果 |

两个节点读取同一份历史 JSONL 数据，
分别完成消息转换和发布。

## 4. 实验方法

1. 启动 ROS2 环境。
2. 启动双话题订阅审计程序。
3. 使用 cquick.launch.py 启动两个发布节点。
4. 设置 follow=false，执行历史数据离线回放。
5. 统计消息数量、状态分布和 JSON 解析错误。
6. 对照原始 JSONL 重新统计，核验结果。

## 5. 实验结果

原始数据总计 2230 帧。

### 球检测

- 接收消息：2230
- 球检测帧：658
- 检测实例：659
- 空检测消息：1572
- JSON 解析错误：0

### 场地定位

- 接收消息：2230
- 有效定位：631
- 无效定位：1599

定位状态：
- ok：631
- insufficient_markers：1435
- no_consistent_match：150
- ambiguous：13
- camera_outside_measured_half：1

有效定位的匹配标志点数量：
- 3 点：532
- 4 点：99

### 数据一致性

原始 JSONL 统计与 ROS2 订阅端的
消息总量及状态分布一致。

本实验验证的是数量和统计分布一致，
没有完成逐条消息序号的 exactly-once 核验。

## 6. 复现方法

终端一：

source /opt/ros/humble/setup.bash

CQ="$HOME/Cquick/Week3-4"
LIVE="$CQ/site_review_20261119/automatic_demo/runs/live_2603112118_20261122_105055_241733/frames.jsonl"

source /opt/ros/humble/setup.bash

ros2 launch "$CQ/lab_launch_20261119/ros/cquick.launch.py" \
mode:=both ball_log:="$LIVE" pose_log:="$LIVE" follow:=false

终端二先运行审计程序：

source /opt/ros/humble/setup.bash

python3 cquick_ros_replay_audit_v2.py

注意：审计程序必须先启动，再启动发布节点。

## 7. 实验边界

本实验使用历史 JSONL 数据，不使用实时摄像头。

因此不能据此宣称已完成实时相机 ROS2 集成、
硬件实时闭环或端到端实时延迟验收。

## 8. 文件说明

- scripts/：审计程序
- logs/：审计日志与数据哈希
