# 第 3–4 周考核：足球检测 + 单目场地自定位

- 姓名：Chenxiefan
- 完成日期：2026-11-22
- 摄像头：单目 USB RGB 相机（0bda:5844），固定焦距、固定对焦
- 接口 / 设备：主机 MJPEG 桥，http://192.168.56.1:8080/video
- 分辨率 / 帧率：1280×720 / 相机 30 fps，任务一检测约 6.4 fps，任务二定位约 8 fps
- 镜头：固定焦距，未记录具体毫米数
- 运行环境：Ubuntu 22.04，Python 3.10，PyTorch 2.14，Ultralytics 8.4，ROS 2 Humble
- GPU：无，CPU 运行

本目录是《算法组第 3，4 周招新考核》的完整实现，包含：

- 任务一：足球检测数据采集、YOLO 训练、实时检测、指标评估；
- 任务二：场地 L/T/X 标志检测、相机内外参、地面投影、与 `field.yaml` 匹配、二维位姿输出；
- ROS2 接口、离线评估脚本、演示视频生成脚本。

运行环境：Ubuntu 22.04 虚拟机（VMware），Python 3.10，OpenCV 5.0，PyTorch 2.14 CPU，Ultralytics 8.4，ROS 2 Humble。

本目录只提交轻量工程文件：原始视频、抽帧图片、录屏原文件、`runs/`、模型权重均未上传。

## 一、相机与场地配置

| 文件 | 内容 |
|---|---|
| `config/camera.yaml` | 1280×720，fx=fy=1574.7278，cx=639.5，cy=359.5，畸变 [0.220516, -1.528214, 0.014901, -0.004193, 0]，RMS 1.015 px |
| `config/camera_extrinsics.yaml` | 高度 0.663 m，俯仰 13.0° 向下，滚转 0°，yaw 0°，原点在镜头正下方 |
| `config/field.yaml` | 15 个标志点，原点中心圈，半场 3.975 × 5.955 m |
| `config/field_geometry.yaml` | 场地尺寸、原点、轴向说明 |

坐标约定：地图原点在中心圈圆心（中线中点），+X 指向唯一球门，+Y 为面向 +X 时的左侧，yaw 从 +X 逆时针为正；场地是半场，x 在 0~3.975，y 在 -2.9775~+2.9775。载体 +X 向前、+Y 向左、+Z 向上；相机 +X 向右、+Y 向下、+Z 向前。

相机倒装，采集时由主机端 MJPEG 桥 `--rotate 180` 翻转，标定参数仍然有效。

## 二、任务一：足球检测

### 2.1 数据

| 项目 | 数量 |
|---|---|
| 原始视频 | 6 段（video1~video6，共约 285 秒） |
| 训练集 | 646 张（402 张有球 + 244 张负样本） |
| 验证集 | 62 张（39 张有球 + 23 张负样本） |
| 划分方式 | 按视频划分，video1/2/3/6 训练，video4/5 验证，无同段视频泄漏 |

负样本包含空场地、矿泉水瓶、白色器材、场地线拐角、场地标志帧，以及后续补充训练中发现的难负样本（半圆、白线、亮斑）。

### 2.2 训练

| 项目 | 设置 |
|---|---|
| 模型 | YOLOv8n（官方预训练权重初始化） |
| 轮数 | 50 轮 |
| 输入尺寸 | 640 |
| batch | 8 |
| 设备 | CPU（无 GPU） |
| 耗时 | 约 2 小时 |

训练数据扩充到 646 张后从头训练，得到最终权重 `models/ball_best.pt`；上一版权重保留为 `models/ball_best_v3.pt`。

### 2.3 验证集指标

| 模型 | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| v2（50 轮基线） | 0.928 | 1.000 | 0.984 | 0.908 |
| v3（加入难负样本） | 0.925 | 1.000 | 0.993 | 0.902 |
| v4（扩充数据后重训，最终） | 1.000 | 0.967 | 0.994 | 0.940 |

验证集 62 张、39 个正样本，指标存在一定波动。

### 2.4 可视化与失败案例

验证集预测图见 `runs/detect/runs/val_pred_v2/`，备用素材整段结果见 `data/output/ball_demo_v4.mp4`。

观察到的失败情况：

- 极端运动模糊（拖影超过球体本身）的帧仍可能漏检；
- 画面边缘只露出很小一部分的白色物体可能产生低置信误检；
- 中心圈半圆、白线拐角、胶带、手机在加入难负样本后误检大大减少。

实时检测命令：

```bash
python3 scripts/detect_ball.py --model models/ball_best.pt --device 0
python3 scripts/detect_ball.py --model models/ball_best.pt --source video:data/raw/video31.mp4 --save
```

输出包括检测框、类别、置信度、框中心像素坐标、FPS 和单帧推理耗时。

### 2.5 ONNX 导出与推理对比

把 ball_best.pt 导出成 ONNX（imgsz 640，opset 12），在同一段测试视频 video31 上取 60 帧对比，置信度 0.4：

- PyTorch 平均 65.5 ms，15.3 fps；
- ONNX 平均 49.9 ms，20.0 fps；
- 两边检测数量 60/60 帧一致，匹配框平均 IoU 0.929。

导出命令：yolo export model=models/ball_best.pt format=onnx imgsz=640 opset=12

## 三、任务二：单目场地自定位

### 3.1 标志类别

现场实际存在的标志为 L 型、T 型、X 型三种白线交点。点球点没有物理标记，只有中线，因此不参与训练，类别固定为：

```text
LCross
TCross
XCross
```

### 3.2 数据

| 项目 | 数量 |
|---|---|
| 原始视频 | 12 段（video11、12、14~19、110~113），后续补采 211~215、501~503 |
| 候选帧 | 847 张（去模糊、去重复后） |
| 训练集 | 557 张（含 501~503 补采的 120 张） |
| 验证集 | 54 张（video15 + video17，三类齐全） |
| 划分方式 | 按视频划分，训练与验证无重叠 |

标注要求：框住标志可见的白色线条，使交点位于框中心附近；框内保留每条线 30~50 像素，供后续几何细化使用。

### 3.3 训练与指标

模型 YOLOv8n，类别 3，CPU 训练。

640 输入、50 轮，最佳第 22 轮：

| 类别 | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| LCross | 0.676 | 0.917 | 0.949 | 0.459 |
| TCross | 1.000 | 0.569 | 0.739 | 0.342 |
| XCross | 0.944 | 0.992 | 0.986 | 0.473 |
| 全部 | 0.873 | 0.826 | 0.892 | 0.425 |

960 输入、30 轮：

| 类别 | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| LCross | 1.000 | 0.942 | 0.972 | 0.581 |
| TCross | 0.806 | 0.848 | 0.882 | 0.324 |
| XCross | 0.980 | 0.972 | 0.994 | 0.478 |

提高输入分辨率后，LCross 和 TCross 的召回明显改善。

2026-11-22 补采 501~503 共 120 帧，在 960 基线上继续微调 20 轮：

| 类别 | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|
| LCross | 0.816 | 0.958 | 0.965 | 0.431 |
| TCross | 0.765 | 0.727 | 0.779 | 0.324 |
| XCross | 0.873 | 0.745 | 0.899 | 0.414 |
| 全部 | 0.818 | 0.810 | 0.881 | 0.390 |

验证集指标低于 960 基线，但补采视角下的真实视频稳定性得到提升：video211 有效帧从 1245 提升到 1265（共 1266），video213 从 671 提升到 733（共 734）。最终采用微调权重 `models/marker_best.pt`，基线保留为 `models/marker_best_baseline.pt`。

501~503 的标注重点是右上角 LCross 和容易混成 TCross 的负样本。

### 3.4 定位链路

```text
YOLO 检测 L/T/X
→ refine_landmark 在框内用霍夫直线求交点，得到 landmark_pixel
→ 去畸变 + 内参得到相机射线
→ 与地面平面 Z=0 求交，得到标志在载体坐标系下的地面坐标
→ solve_pose 与 field.yaml 中同类标志做多候选匹配
→ 输出 x_camera_map / y_camera_map / yaw_camera_map / valid / marker_count / residual / status
```

`solve_pose` 的处理：

- 对观测点两两组合生成候选位姿；
- 按类别做最近邻分配，最小二乘精化；
- 按内点数和残差排序；
- 当次优解与最优解内点数接近、残差接近但位姿差异大时，输出 `status=ambiguous`、`valid=false`；
- 标志少于 3 个时输出 `insufficient_markers`。

提升稳定性方面的尝试：

- `MarkerTracker`：按检测框中心做帧间关联，EMA 平滑标志像素位置，轨迹需连续出现 2 帧才确认；
- 同类重复框去重：IoU > 0.35 或中心重合且尺寸接近时只保留置信度最高的；
- 轨迹合并：同类别、框中心距离 < 30 px 的轨迹只留一条；
- 细化带先验：交点候选离上一帧 15 px 内才接受，否则保持；旧位置跑出当前框时改用框内候选；
- 位姿平滑：有效位姿做 EMA，失效或跳变时重置；
- 位姿先验消歧：首帧要求至少 4 个标志建立锚点，之后三点解用上一帧位姿选择一致的候选。

### 3.5 测试位姿

外参：高度 0.663 m；俯角 13.0°，滚转 0°，yaw 0°。

真值文件 `data/test_poses_final.csv`：

| 编号 | 视频 | 相机位置 | 坐标 (m) | 朝向 | yaw（CCW） |
|---|---|---|---|---|---|
| P1 | video211 | 左侧 XCross | (0.000, 0.720) | 左上角 LCross | 30.0° |
| P2 | video213 | 场地中心 | (0.000, 0.000) | 左上角 LCross | 36.9° |
| P3 | video221 | 右侧 XCross | (0.000, -0.720) | 球门线右端 TCross | 342.75° |

测试结果（完整视频，开启多帧跟踪与位姿先验）：

| 编号 | 有效帧 | 估计位姿 | 位置误差 | yaw 误差 |
|---|---|---|---|---|
| P1 | 1265 / 1266 | (0.11, 0.62, 31.9°) | 0.15 m | 1.9° |
| P2 | 733 / 734 | (0.02, -0.01, 37.5°) | 0.02 m | 0.6° |
| P3 | 635 / 636 | (0.10, -0.79, 343.0°) | 0.12 m | 0.2° |

三个不同位置、三个不同朝向，全部满足 0.5 m / 10°。

补充说明：

- video212 与 video211 是同一位置、同一朝向，属于重复素材；
- video214、video215 画面里最多只有 2~3 个标志，无法唯一解位姿，未采用；
- 211 的 yaw 在 CCW 约定下为正约 30°；若按顺时针约定会写成 -30°，两者是同一个方向；
- 221 的 yaw 为 342.75°（即 -17.25°）；
- 三点解存在对称歧义，正式录屏时应尽量让画面里出现 4 个以上标志。

## 四、复现命令

### 权重文件

权重不放进仓库，两个文件的 SHA256：

| 文件 | SHA256 |
|---|---|
| models/ball_best.pt | e6ec2aaaa8f5b82bf065e16b272ec3fccea83fef7cf18fd413e923d540bb04a6 |
| models/marker_best.pt | b73e7ae0a1b926902520c475a83d5dfb808b76a4c124b9bb0cf93790733e6d2f |

获取方式（GitHub Release）：

- https://github.com/chengdousheshou/cquick-recruitment/releases/download/week3-4-weights/ball_best.pt
- https://github.com/chengdousheshou/cquick-recruitment/releases/download/week3-4-weights/marker_best.pt

下载后放到 `models/` 下即可，文件名保持不变。

```bash
cd 算法组/Week3-4/chenxiefan

# 球检测实时
python3 scripts/detect_ball.py --model models/ball_best.pt --device 0

# 场地定位（图片 / 视频 / 摄像头）
python3 scripts/locate_field.py --model models/marker_best.pt \
    --source image:test.jpg --csv out/pose.csv
python3 scripts/locate_field.py --model models/marker_best.pt \
    --source video:data/raw/video213.mp4 --imgsz 960 --csv out/pose213.csv --no-show

# 标签检查
python3 scripts/check_labels.py --data dataset/ball --classes 1
python3 scripts/check_labels.py --data dataset/marker --classes 3

# 位姿误差评估
python3 scripts/eval_pose.py --truth data/test_poses_final.csv --pred out/pose.csv
```

训练命令：

```bash
yolo detect train model=yolov8n.pt data=dataset/ball/data.yaml \
    epochs=50 imgsz=640 batch=8 amp=False project=runs name=ball

yolo detect train model=yolov8n.pt data=dataset/marker/data.yaml \
    epochs=50 imgsz=640 batch=8 amp=False project=runs name=marker

yolo detect train model=yolov8n.pt data=dataset/marker/data.yaml \
    epochs=30 imgsz=960 batch=4 amp=False project=runs name=marker960
```

## 五、ROS2 接口

```bash
source /opt/ros/humble/setup.bash
cd 算法组/Week3-4/chenxiefan
colcon build --base-paths src \
    --build-base build_soccer --install-base install_soccer \
    --packages-select soccer_interfaces soccer_vision
source install_soccer/setup.bash

ros2 run soccer_vision ball_detector
ros2 run soccer_vision field_locator

ros2 launch soccer_vision dual.launch.py
```

话题：

- `/soccer/ball`：`BallDetection`，含检测框、中心像素、置信度；
- `/localization/field_pose`：`FieldPose`，含 x/y/yaw、valid、marker_count、residual、status。

## 六、B 站演示录屏

任务一：https://www.bilibili.com/video/BV1nSyGzYExN

任务二：https://www.bilibili.com/video/BV1vyyLzaEZj

两段都是连续实时相机输入，没有剪辑。任务一 4 分 18 秒，片尾有 5 秒信息补充；任务二 4 分 13 秒，片尾接了一段 ROS2 节点与话题输出。

逐项确认：

- 是否包含连续实时相机输入：是
- 是否展示足球检测框、置信度和 FPS：是
- 是否展示 L/T/X 标志识别与几何特征点：是
- 是否采用左 RGB / 右地图的左右分屏：是
- 是否展示 field.yaml 匹配和地图俯视图：是
- 是否在右侧地图标出相机光心位置与朝向箭头：是
- 是否展示 x_camera_map / y_camera_map / yaw_camera_map：是
- 是否展示至少 3 个不同位置 / 朝向：是
- 是否展示 marker_count / residual / valid / status：是
- GitHub 本目录是否已提交该 B 站链接：是

## 七、C0 现场测试

待现场测试后填写。

| 测试点 | 参考位置 / 朝向 | 估计位置 / 朝向 | 位置误差 | 航向误差 |
|---|---|---|---|---|
| P1 | | | | |
| P2 | | | | |
| P3 | | | | |

## 八、问题与解决过程

问题一：静止时标志中心抖动，偶尔同一个标志出现两个绿框。

先量了连续帧的中心跳变：平均 11.3 像素，90 分位 38 像素，最大 60 像素。检测框本身是稳定的，跳变来自框内霍夫交点在不同候选之间切换；重复框则是 NMS 没有压掉同一个标志的第二个框。

处理方式：跟踪按检测框中心关联，中心做 EMA；同类重复框按 IoU 和中心距离去重；细化交点时以上一帧位置为约束，超过 15 像素的候选不采用；中心距离小于 30 像素的同类轨迹合并。

结果：跳变平均降到 1.4 像素，90 分位 5.8 像素，最大 14.8 像素，超过 20 像素的帧为 0。

问题二：长时间站在同一位置，坐标会缓慢漂移；画面只有 3 个标志时，偶尔锁到旋转约 30 度的错误解。

处理方式：锚点不再每帧更新，只有位置变化超过 0.15 米或 5 度才换；位姿增加死区和保持锁，连续 4 帧超过阈值才允许更新；还没有锚点且只有 3 个标志时，直接输出 ambiguous，不硬给位姿。

结果：video213 静止段的位置范围从 0.287 米降到 0.140 米；211、213、221 三个位姿的位置误差分别为 0.15、0.02、0.12 米。

问题三：录屏时画面突然变卡。

排查发现主机上起了两个相机桥进程，同时抢同一个 USB 相机。杀掉多余进程，只留一个，读取恢复 30 fps。

## 九、已知限制

- 720P 画面偏软，边缘畸变校正精度一般；
- 由于支架结构需要将相机倒装，依赖主机端相机桥做 180° 翻转；
- 中心圈弧线在个别角度仍会被误检为足球，属于残余误检；
- 点球点没有物理标记，未训练 `PenaltyPoint` 类别，地图中保留该坐标但不参与匹配；
- 场地胶带位置可能产生小变动，`field.yaml` 为 2026-11-24 精测版本，与 2026-11-21 版本相比尺寸差在 1 厘米量级；
- 三点解存在对称歧义，画面里少于 4 个标志时可能输出 `ambiguous` 或锁到错误解；
- ROS2 `field_locator` 已接入在线检测与匹配，但受标志检测限制，远景下仍可能 `insufficient_markers`。

## 十、待办

- [x] 补采 501~503 并微调 960 标志模型；
- [x] 匹配阶段加入置信度过滤、虚拟中心点、更严格的残差与歧义约束；
- [x] 加入多帧跟踪、重复框去重、细化先验和位姿先验；
- [x] ROS2 `field_locator` 接入在线检测与匹配；
- [x] 录制任务一、任务二的连续实时演示，B 站链接见上；
- [ ] 现场复测 0.5 m / 10° 指标。
