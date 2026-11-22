# src/ ROS2 包骨架（细则 2.11 / 666 行 +2 分）

## 状态：⚠️ 未在实机验证

开发机是 **Windows 11，没装 ROS2 Humble**，因此这两个包**从未在 ROS2 环境跑通过**。
接口、消息字段、launch 文件按细则 2.11 与 676~694 行的建议目录结构写好，
**20 号下午到实验室 Ubuntu 机上按下面步骤验证一次**。README 里也如实标注了这一点，
不要声称「已跑通」。

## 目录

```text
src/
├── vision_interfaces/          # 接口包（ament_cmake + rosidl）
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       ├── BallDetection.msg  # /soccer/ball
│       └── FieldPose.msg      # /localization/field_pose
└── field_localization/       # 视觉定位包（ament_python）
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/field_localization
    ├── launch/demo.launch.py
    └── field_localization/
        ├── __init__.py
        ├── ball_detector_node.py   # 相机 → YOLO → /soccer/ball + /soccer/image_raw
        └── field_pose_node.py      # 图像 → 标志匹配 → /localization/field_pose
```

单 USB 相机只能被一个进程打开，所以**只有 `ball_detector_node` 开相机**，
并把原始帧发到 `/soccer/image_raw`；`field_pose_node` 订阅该话题，不再重复开相机。

## 话题与字段（与细则 2.11 表格一致）

| 话题 | 类型 | 关键字段 |
|---|---|---|
| `/soccer/ball` | `vision_interfaces/msg/BallDetection` | `header`、`detected`、`conf`、`x1..y2`、`center_u/v`（像素） |
| `/soccer/image_raw` | `sensor_msgs/Image` | bgr8，`frame_id=camera_frame` |
| `/localization/field_pose` | `vision_interfaces/msg/FieldPose` | `x/y/yaw_camera_map`（m / **度**）、`x/y/yaw_carrier_map`、`valid`、`marker_count`、`residual`(px)、`status` |

坐标约定：C0 地图，原点=场地中心，+X 指向唯一球门，+Y 为面向 +X 时的左侧，
单位米，yaw 逆时针为正且**单位是度**。载体为独立移动测试架，
以相机光心为参考原点，故 `x_carrier_map = x_camera_map`，输出 `camera_frame`。

`status` 取值：`OK` / `INSUFFICIENT_MARKERS`（<4 点）/
`AMBIGUOUS`（镜像或被 h 先验拒绝）/ `MARKER_MODEL_NOT_TRAINED` / `NO_IMAGE_YET`。
**定位失败时 `valid=false` 且坐标置 0，绝不伪造有效位姿。**

## 实验室验证步骤（20 号下午）

```bash
# 1. 把 week3/src 两个包拷进 colcon 工作区，并让 task2/ 与 config/ 可被 import
cd ~/ros2_ws/src
ln -s /path/to/week3/src/vision_interfaces .
ln -s /path/to/week3/src/field_localization .
# field_pose_node 用 Path(__file__).parents[3] 找 week3 根，
# 所以 week3 目录需在可访问路径；必要时设 PYTHONPATH=/path/to/week3

# 2. 编译
cd ~/ros2_ws
colcon build --packages-select vision_interfaces field_localization
source install/setup.bash

# 3. 起节点（相机接好、/dev/video0 确认）
ros2 launch field_localization demo.launch.py ball_model:=/path/to/best_train_v4.pt marker_model:=/path/to/markers_best.pt

# 4. 另开终端核对话题输出
ros2 topic echo /soccer/ball
ros2 topic echo /localization/field_pose
ros2 topic hz /localization/field_pose
```

## 依赖

`rclpy` / `vision_interfaces` / `std_msgs` / `sensor_msgs` / `cv_bridge` /
`python3-opencv` / `python3-numpy` / `ultralytics` / `torch`（后两者用项目 `.venv`
或系统 Python，注意 ROS2 Humble 默认 Python 3.10 与开发机 3.13 不同，
**现场按实验室环境装依赖，不要直接照搬 Windows .venv**）。

## 与录屏工具的分工

`tools/record_demo.py` 负责细则要求的**左右分屏连续录屏**（左 RGB / 右地图 +
状态字段），不依赖 ROS2；本 `src/` 包负责**ROS2 节点与话题输出**的验收项。
两者可同时跑，但注意相机占用：录屏工具会自己开相机，
若同时起 `ball_detector_node` 会冲突 —— 现场二选一，或先录屏再起 ROS2 节点。
